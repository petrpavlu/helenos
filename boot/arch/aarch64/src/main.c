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
#include <errno.h>
#include <inflate.h>
#include <macros.h>
#include <printf.h>
#include <putchar.h>
#include <str.h>
#include <version.h>

extern void *bdata_start;
extern void *bdata_end;

static efi_system_table_t *efi_system_table;
static bootinfo_t bootinfo;

/** Ensure the visibility of updates to instructions in the given range.
 *
 * @param addr Address of the first instruction.
 * @param size Size of the instruction block (in bytes).
 */
static inline void ensure_visibility(void *addr, size_t size)
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
	sysarg_t memory_map_size;
	efi_v1_memdesc_t *memory_map = NULL;
	sysarg_t map_key;
	sysarg_t descriptor_size;
	uint32_t descriptor_version;
	uint64_t alloc_addr = 0;
	sysarg_t alloc_pages = 0;

	efi_system_table = efi_system_table_in;

	version_print();

	printf("Boot data: %p -> %p\n", get_bdata_start(), get_bdata_end());
	printf("\nMemory statistics\n");
	printf(" %p|%p: loader\n", load_address, load_address);
	printf(" %p|%p: boot info structure\n", &bootinfo, &bootinfo);
	printf(" %p|%p: UEFI system table\n", efi_system_table_in,
	    efi_system_table_in);

	component_t *components = get_components();
	for (size_t i = 0; i < COMPONENTS; i++) {
		printf(" %p|%p: %s image (%zu/%zu bytes)\n",
		    components[i].start, components[i].start,
		    components[i].name, components[i].inflated,
		    components[i].size);
	}

	/* Obtain memory map. */
	status = efi_get_memory_map(efi_system_table, &memory_map_size,
	    &memory_map, &map_key, &descriptor_size, &descriptor_version);
	if (status != EFI_SUCCESS) {
		printf("Error: Unable to obtain memory map, status code: "
		    "%lx.\n", status);
		goto fail;
	}

	/* Find start of usable RAM. */
	bool memory_base_found = false;
	uint64_t memory_base;
	for (sysarg_t i = 0; i < memory_map_size / descriptor_size; i++) {
		efi_v1_memdesc_t *desc = (void *) memory_map +
		    (i * descriptor_size);
		if (!(desc->attribute & EFI_MEMORY_WB))
			continue;

		if (!memory_base_found) {
			memory_base = desc->phys_start;
			memory_base_found = true;
		} else if (desc->phys_start < memory_base)
			memory_base = desc->phys_start;
	}

	/* Deallocate memory holding the map. */
	efi_system_table->boot_services->free_pool(memory_map);
	memory_map = NULL;

	if (!memory_base_found) {
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
	bootinfo.cnt = 0;
	for (size_t i = 0; i < min(COMPONENTS, TASKMAP_MAX_RECORDS); i++) {
		if (i > 0) {
			bootinfo.tasks[bootinfo.cnt].addr = (void *) top;
			bootinfo.tasks[bootinfo.cnt].size =
			    components[i].inflated;

			str_cpy(bootinfo.tasks[bootinfo.cnt].name,
			    BOOTINFO_TASK_NAME_BUFLEN, components[i].name);

			bootinfo.cnt++;
		}

		dest[i] = (void *) top;
		top += components[i].inflated;
		top = ALIGN_UP(top, PAGE_SIZE);
		cnt++;
	}

	/* Allocate memory for the inflated components. */
	alloc_pages = (ALIGN_UP(top, EFI_PAGE_SIZE) -
	    ALIGN_DOWN(inflated_base, EFI_PAGE_SIZE)) / EFI_PAGE_SIZE;
	alloc_addr = inflated_base;
	status = efi_system_table->boot_services->allocate_pages(
	    EFI_ALLOCATE_ADDRESS, EFI_LOADER_CODE, alloc_pages, &alloc_addr);
	if (status != EFI_SUCCESS) {
		printf("Error: Unable to allocate memory for inflated "
		    "components, status code: %lx.\n", status);
		goto fail;
	}

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
	}

	printf(".\n");

	/* TODO */

	printf("Booting the kernel...\n");
	jump_to_kernel((void *) inflated_base, &bootinfo);

fail:
	if (memory_map != NULL)
		efi_system_table->boot_services->free_pool(memory_map);

	if (alloc_addr != 0)
		efi_system_table->boot_services->free_pages(alloc_addr,
		    alloc_pages);

	return status;
}

/** @}
 */
