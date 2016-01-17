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

/** @addtogroup aarch64
 * @{
 */
/** @file
 * @brief Various AArch64-specific macros.
 */

#ifndef KERN_aarch64_ARCH_H_
#define KERN_aarch64_ARCH_H_

#include <config.h>
#include <typedefs.h>

#define BOOTINFO_TASK_NAME_BUFLEN  32
#define TASKMAP_MAX_RECORDS        32
#define MEMMAP_MAX_RECORDS        128

/** Task structure.
 *
 * Must be in sync with the task structure used by the boot loader.
 */
typedef struct {
	void *addr;
	size_t size;
	char name[BOOTINFO_TASK_NAME_BUFLEN];
} utask_t;

/** Task map structure.
 *
 * Must be in sync with the taskmap structure used by the boot loader.
 */
typedef struct {
	size_t cnt;
	utask_t tasks[TASKMAP_MAX_RECORDS];
} taskmap_t;

/** Memory zone types.
 *
 * Must be in sync with the memtype enum used by the boot loader.
 */
typedef enum {
	MEMTYPE_UNUSABLE,
	MEMTYPE_AVAILABLE,
	MEMTYPE_ACPI_RECLAIM
} memtype_t;

/** Memory area.
 *
 * Must be in sync with the memzone structure used by the boot loader.
 */
typedef struct {
	memtype_t type;
	void *start;
	size_t size;
} memzone_t;

/** System memory map.
 *
 * Must be in sync with the memmap structure used by the boot loader.
 */
typedef struct {
	size_t cnt;
	memzone_t zones[MEMMAP_MAX_RECORDS];
} memmap_t;

/** Bootinfo structure.
 *
 * Must be in sync with the bootinfo structure used by the boot loader.
 */
typedef struct {
	taskmap_t taskmap;
	memmap_t memmap;
} bootinfo_t;

extern void arch_pre_main(void *entry, bootinfo_t *bootinfo);

#endif

/** @}
 */
