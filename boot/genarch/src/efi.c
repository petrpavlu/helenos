/*
 * Copyright (c) 2011 Jakub Jermar
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

#include <genarch/efi.h>
#include <arch/types.h>
#include <typedefs.h>
#include <align.h>

void *efi_vendor_table_find(efi_system_table_t *st, efi_guid_t guid)
{
	sysarg_t i;

	for (i = 0; i < st->conf_table_entries; i++)
		if ((st->conf_table[i].guid.low == guid.low) &&
		    (st->conf_table[i].guid.high == guid.high))
			return st->conf_table[i].table;

	return NULL;
}

efi_status_t efi_get_memory_map(efi_system_table_t *st,
    efi_allocate_type_t allocate_type, efi_memory_type_t memory_type,
    efi_v1_memdesc_t **memory_map, sysarg_t *memory_map_size, sysarg_t *map_key,
    sysarg_t *descriptor_size, uint32_t *descriptor_version)
{
	efi_status_t status;
	sysarg_t pages = 1;

	while (1) {
		/* Allocate space for the memory map. */
		status = st->boot_services->allocate_pages(allocate_type,
		    memory_type, pages, (uint64_t *) memory_map);
		if (status != EFI_SUCCESS)
			return status;

		/* Try to obtain the map. */
		*memory_map_size = pages * EFI_PAGE_SIZE;
		status = st->boot_services->get_memory_map(memory_map_size,
		    *memory_map, map_key, descriptor_size, descriptor_version);
		if (status == EFI_SUCCESS)
			return status;

		/* An error occurred, release the allocated memory. */
		st->boot_services->free_pages((uint64_t) *memory_map, pages);
		if (status != EFI_BUFFER_TOO_SMALL)
			return status;

		/* Try with a bigger buffer. */
		pages = ALIGN_UP(*memory_map_size, EFI_PAGE_SIZE) /
		    EFI_PAGE_SIZE;
	}
}
