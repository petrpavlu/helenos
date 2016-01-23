/*
 * Copyright (c) 2015 Petr Pavlu
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @addtogroup aarch64boot
 * @{
 */
/** @file
 * @brief Bootstrap.
 */

#include <typedefs.h>
#include <align.h>
#include <arch/_components.h>
#include <arch/arch.h>
#include <arch/asm.h>
#include <arch/boot.h>
#include <arch/main.h>
#include <arch/regutils.h>
#include <errno.h>
#include <inflate.h>
#include <macros.h>
#include <memstr.h>
#include <printf.h>
#include <putchar.h>
#include <str.h>
#include <version.h>

static efi_system_table_t *efi_system_table;

/** Ensure the visibility of updates to instructions in the given range.
 *
 * @param addr Address of the first instruction.
 * @param size Size of the instruction block (in bytes).
 */
static void ensure_visibility(void *addr, size_t size)
{
	for (uintptr_t a = (uintptr_t) addr; a < (uintptr_t) addr + size;
	    a += 4)
		asm volatile (
			/* Clean to Point of Unification to make the new
			 * instruction visible to the instruction cache. */
			"dc cvau, %[a]\n"
			/* Ensure completion on all PEs. */
			"dsb ish\n"
			/* Ensure instruction cache/branch predictor discards
			 * stale data. */
			"ic ivau, %[a]\n"
			/* Ensure completion on all PEs. */
			"dsb ish\n"
			/* Synchronize context on this PE. */
			"isb\n"
			: : [a] "r" (a) : "memory"
		);
}

/** Translate given UEFI memory type to the bootinfo memory type.
 *
 * @param type UEFI memory type.
 */
static memtype_t get_memtype(uint32_t type)
{
	switch (type)
	{
	case EFI_RESERVED:
	case EFI_RUNTIME_SERVICES_CODE:
	case EFI_RUNTIME_SERVICES_DATA:
	case EFI_UNUSABLE_MEMORY:
	case EFI_ACPI_MEMORY_NVS:
	case EFI_MEMORY_MAPPED_IO:
	case EFI_MEMORY_MAPPED_IO_PORT_SPACE:
	case EFI_PAL_CODE:
		return MEMTYPE_UNUSABLE;
	case EFI_LOADER_CODE:
	case EFI_LOADER_DATA:
	case EFI_BOOT_SERVICES_CODE:
	case EFI_BOOT_SERVICES_DATA:
	case EFI_CONVENTIONAL_MEMORY:
	case EFI_PERSISTENT_MEMORY:
		return MEMTYPE_AVAILABLE;
	case EFI_ACPI_RECLAIM_MEMORY:
		return MEMTYPE_ACPI_RECLAIM;
	}

	return MEMTYPE_UNUSABLE;
}

/** Send a byte to the UEFI console output.
 *
 * @param byte Byte to send.
 */
static void scons_sendb(uint8_t byte)
{
	int16_t out[2] = {byte, '\0'};
	efi_system_table->cons_out->output_string(efi_system_table->cons_out,
	    out);
}

/** Display a character.
 *
 * @param ch Character to display.
 */
void putchar(wchar_t ch)
{
	if (ch == '\n')
		scons_sendb('\r');

	if (ascii_check(ch))
		scons_sendb((uint8_t) ch);
	else
		scons_sendb(U_SPECIAL);
}

efi_status_t bootstrap(void *efi_handle_in,
    efi_system_table_t *efi_system_table_in, void *load_address)
{
	efi_status_t status;
	uintptr_t current_el;
	uint64_t memmap = 0;
	sysarg_t memmap_size;
	sysarg_t memmap_key;
	sysarg_t memmap_descriptor_size;
	uint32_t memmap_descriptor_version;
	uint64_t alloc_addr = 0;
	sysarg_t alloc_pages = 0;

	/*
	 * Bootinfo structure is dynamically allocated in the AArch64 port. It
	 * is placed directly after the inflated components. This assures that
	 * if the kernel identity maps the first gigabyte of the main memory in
	 * the kernel/upper address space then it can access the bootinfo
	 * because the inflated components and bootinfo can always fit in this
	 * area.
	 */
	bootinfo_t *bootinfo;

	efi_system_table = efi_system_table_in;

	version_print();

	printf("Boot data: %p -> %p\n", get_bdata_start(), get_bdata_end());
	printf("\nMemory statistics\n");
	printf(" %p|%p: loader\n", load_address, load_address);
	printf(" %p|%p: UEFI system table\n", efi_system_table_in,
	    efi_system_table_in);

	component_t *components = get_components();
	for (size_t i = 0; i < COMPONENTS; i++) {
		printf(" %p|%p: %s image (%zu/%zu bytes)\n",
		    components[i].start, components[i].start,
		    components[i].name, components[i].inflated,
		    components[i].size);
	}

	/* Validate the exception level. */
	current_el = CurrentEL_read();
	if (current_el != CURRENT_EL_EL1) {
		printf("Error: Unexpected CurrentEL value %0#18" PRIx64 ".\n",
		    current_el);
		status = EFI_UNSUPPORTED;
		goto fail;
	}

	/* Obtain memory map. */
	status = efi_get_memory_map(efi_system_table, &memmap_size,
	    (efi_v1_memdesc_t **) &memmap, &memmap_key, &memmap_descriptor_size,
	    &memmap_descriptor_version);
	if (status != EFI_SUCCESS) {
		printf("Error: Unable to obtain initial memory map, status "
		    "code: %" PRIx64 ".\n", status);
		goto fail;
	}

	/* Find start of usable RAM. */
	uint64_t memory_base = (uint64_t) -1;
	for (sysarg_t i = 0; i < memmap_size / memmap_descriptor_size; i++) {
		efi_v1_memdesc_t *desc = (void *) memmap +
		    (i * memmap_descriptor_size);
		if (get_memtype(desc->type) != MEMTYPE_AVAILABLE ||
		    !(desc->attribute & EFI_MEMORY_WB))
			continue;

		if (desc->phys_start < memory_base)
			memory_base = desc->phys_start;
	}

	/* Deallocate memory holding the map. */
	efi_system_table->boot_services->free_pool((void *) memmap);
	memmap = 0;

	if (memory_base == (uint64_t) -1) {
		printf("Error: Memory map does not contain any usable RAM.\n");
		status = EFI_UNSUPPORTED;
		goto fail;
	}

	/*
	 * Check that everything is aligned on a 4kB boundary and the kernel can
	 * be placed by the inflate code at a correct address.
	 */

	/* Statically check PAGE_SIZE and BOOT_OFFSET. */
#if PAGE_SIZE != 4096
#error Unsupported PAGE_SIZE
#endif
#if !IS_ALIGNED(BOOT_OFFSET, PAGE_SIZE)
#error Unsupported BOOT_OFFSET
#endif
	/*
	 * Dynamically check the memory base. The condition should be always
	 * true because UEFI guarantees each physical/virtual address in the
	 * memory map is aligned on a 4kB boundary.
	 */
	if (!IS_ALIGNED(memory_base, PAGE_SIZE)) {
		printf("Error: Start of usable RAM (%p) is not aligned on a "
		    "4kB boundary.\n", (void *) memory_base);
		status = EFI_UNSUPPORTED;
		goto fail;
	}

	/*
	 * Calculate where the components (including the kernel) will get
	 * placed.
	 */
	uint64_t inflated_base = memory_base + BOOT_OFFSET;
	printf(" %p|%p: kernel entry point\n", (void *) inflated_base,
	    (void *) inflated_base);

	/*
	 * Determine where components should be placed and how much memory is
	 * needed for them.
	 */
	void *dest[COMPONENTS];
	uint64_t top = inflated_base;
	size_t cnt = 0;
	for (size_t i = 0; i < COMPONENTS; i++) {
		dest[i] = (void *) top;
		top += components[i].inflated;
		top = ALIGN_UP(top, PAGE_SIZE);
		cnt++;
	}

	/* Allocate memory for the inflated components and for the bootinfo. */
	alloc_pages = (ALIGN_UP(top, EFI_PAGE_SIZE) -
	    ALIGN_DOWN(inflated_base, EFI_PAGE_SIZE)) / EFI_PAGE_SIZE +
	    ALIGN_UP(sizeof(*bootinfo), EFI_PAGE_SIZE) / EFI_PAGE_SIZE;
	alloc_addr = inflated_base;
	status = efi_system_table->boot_services->allocate_pages(
	    EFI_ALLOCATE_ADDRESS, EFI_LOADER_CODE, alloc_pages, &alloc_addr);
	if (status != EFI_SUCCESS) {
		printf("Error: Unable to allocate memory for inflated "
		    "components and bootinfo, status code: %" PRIx64 ".\n",
		    status);
		goto fail;
	}

	bootinfo = (void *) alloc_addr + (alloc_pages - 1) * EFI_PAGE_SIZE;
	printf(" %p|%p: boot info structure\n", bootinfo, bootinfo);

	memset(bootinfo, 0, sizeof(*bootinfo));
	bootinfo->taskmap.cnt = cnt;

	/*
	 * Statically check that information about all components can be
	 * recorded in the bootinfo.
	 */
#if COMPONENTS >= TASKMAP_MAX_RECORDS
#error TASKMAP_MAX_RECORDS too small
#endif

	printf("\nInflating components ... ");

	for (size_t i = cnt; i > 0; i--) {
		printf("%s ", components[i - 1].name);

		int err = inflate(components[i - 1].start,
		    components[i - 1].size, dest[i - 1],
		    components[i - 1].inflated);
		if (err != EOK) {
			printf("\n%s: Inflating error %d\n",
			    components[i - 1].name, err);
			status = EFI_LOAD_ERROR;
			goto fail;
		}
		/* Ensure visibility of the component. */
		ensure_visibility(dest[i - 1], components[i - 1].inflated);

		/* Store information about the component in the bootinfo. */
		if (i > 0) {
			bootinfo->taskmap.tasks[i - 1].addr =
			    components[i - 1].start;
			bootinfo->taskmap.tasks[i - 1].size =
			    components[i - 1].inflated;

			str_cpy(bootinfo->taskmap.tasks[i - 1].name,
			    BOOTINFO_TASK_NAME_BUFLEN, components[i - 1].name);
		}
	}

	printf(".\n");

	/* Get final memory map. */
	status = efi_get_memory_map(efi_system_table, &memmap_size,
	    (efi_v1_memdesc_t **) &memmap, &memmap_key, &memmap_descriptor_size,
	    &memmap_descriptor_version);
	if (status != EFI_SUCCESS) {
		printf("Error: Unable to obtain final memory map, status code: "
		    "%" PRIx64 ".\n", status);
		goto fail;
	}

	/* Convert the UEFI memory map to the bootinfo representation. */
	cnt = 0;
	memtype_t current_type = MEMTYPE_UNUSABLE;
	void *current_start = 0;
	size_t current_size = 0;
	sysarg_t memmap_items_count = memmap_size / memmap_descriptor_size;
	for (sysarg_t i = 0; i < memmap_items_count; i++) {
		efi_v1_memdesc_t *desc = (void *) memmap +
		    (i * memmap_descriptor_size);

		/* Get type of the new area. */
		memtype_t type;
		if (!(desc->attribute & EFI_MEMORY_WB))
			type = MEMTYPE_UNUSABLE;
		else
			type = get_memtype(desc->type);

		/* Try to merge the new area with the previous one. */
		if (type == current_type &&
		    (uint64_t)current_start + current_size == desc->phys_start) {
			current_size += desc->pages + EFI_PAGE_SIZE;
			if (i != memmap_items_count - 1)
				continue;
		}

		/* Record the previous area. */
		if (current_type != MEMTYPE_UNUSABLE) {
			if (cnt >= MEMMAP_MAX_RECORDS) {
				printf("Error: Too many usable memory "
				    "areas.\n");
				status = EFI_UNSUPPORTED;
				goto fail;
			}
			bootinfo->memmap.zones[cnt].type = current_type;
			bootinfo->memmap.zones[cnt].start = current_start;
			bootinfo->memmap.zones[cnt].size = current_size;
			cnt++;
		}

		/* Remember the new area. */
		current_type = type;
		current_start = (void *) desc->phys_start;
		current_size = desc->pages * EFI_PAGE_SIZE;
	}
	bootinfo->memmap.cnt = cnt;

	printf("Booting the kernel...\n");

	/* Exit boot services. This is a point of no return. */
	efi_system_table->boot_services->exit_boot_services(efi_handle_in,
	    memmap_key);

	jump_to_kernel((void *) inflated_base, bootinfo);

fail:
	if (memmap != 0)
		efi_system_table->boot_services->free_pool((void *) memmap);

	if (alloc_addr != 0)
		efi_system_table->boot_services->free_pages(alloc_addr,
		    alloc_pages);

	return status;
}

/** @}
 */
