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
 * @brief Definitions of basic types like #uintptr_t.
 */

#ifndef BOOT_aarch64_TYPES_H
#define BOOT_aarch64_TYPES_H

#include <arch/common.h>

#define TASKMAP_MAX_RECORDS        32
#define BOOTINFO_TASK_NAME_BUFLEN  32

typedef uint64_t size_t;
typedef uint64_t uintptr_t;
typedef uint64_t sysarg_t;

typedef uint64_t pfn_t;

typedef struct {
	/** Address where the task was placed. */
	void *addr;
	/** Size of the task's binary. */
	size_t size;
	/** Task name. */
	char name[BOOTINFO_TASK_NAME_BUFLEN];
} task_t;

typedef struct {
	/** Number of boot tasks. */
	size_t cnt;
	/** Boot task data. */
	task_t tasks[TASKMAP_MAX_RECORDS];

	/** UEFI memory map. */
	uint64_t memory_map;
	/* Total size of the UEFI memory map in bytes. */
	sysarg_t memory_map_size;
	/* Size of one entry in the UEFI memory map. */
	sysarg_t map_descriptor_size;
} bootinfo_t;

#endif

/** @}
 */