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
 * @brief AArch64 architecture specific functions.
 */

#include <abi/errno.h>
#include <arch.h>
#include <ddi/irq.h>
#include <interrupt.h>
#include <proc/scheduler.h>
#include <syscall/syscall.h>
#include <sysinfo/sysinfo.h>
#include <userspace.h>

/** Physical memory map received from the bootcode. */
memmap_t memmap;

/** Perform AArch64 specific initialization before main_bsp() is called. */
void arch_pre_main(void *entry __attribute__((unused)), bootinfo_t *bootinfo)
{
	/* Copy init task info. */
	init.cnt = min3(bootinfo->taskmap.cnt, TASKMAP_MAX_RECORDS,
	    CONFIG_INIT_TASKS);

	size_t i;
	for (i = 0; i < init.cnt; i++) {
		init.tasks[i].paddr = KA2PA(bootinfo->taskmap.tasks[i].addr);
		init.tasks[i].size = bootinfo->taskmap.tasks[i].size;
		str_cpy(init.tasks[i].name, CONFIG_TASK_NAME_BUFLEN,
		    bootinfo->taskmap.tasks[i].name);
	}

	/* Copy physical memory map. */
	memmap.cnt = min(bootinfo->memmap.cnt, MEMMAP_MAX_RECORDS);
	for (i = 0; i < memmap.cnt; i++) {
		memmap.zones[i].type = bootinfo->memmap.zones[i].type;
		memmap.zones[i].start = bootinfo->memmap.zones[i].start;
		memmap.zones[i].size = bootinfo->memmap.zones[i].size;
	}
}

/** Perform AArch64 specific tasks needed before the memory management is
 * initialized.
 */
void arch_pre_mm_init(void)
{
	/* REVISIT */
}

/** Perform AArch64 specific tasks needed before the memory management is
 * initialized.
 */
void arch_post_mm_init(void)
{
	if (config.cpu_active != 1)
		return;

	/* Initialize IRQ routing. */
	irq_init(0, 0);

	/* Merge all memory zones to 1 big zone. */
	zone_merge_all();

	/* REVISIT */
}

/** Perform AArch64 specific tasks needed before the multiprocessing is
 * initialized.
 */
void arch_pre_smp_init(void)
{
}

/** Perform AArch64 specific tasks needed after the multiprocessing is
 * initialized.
 */
void arch_post_smp_init(void)
{
	/* Currently the only supported platform for AArch64 is 'arm'. */
	static const char *platform = "arm";

	sysinfo_set_item_data("platform", NULL, (void *) platform,
	    str_size(platform));

	/* REVISIT */
}

/** Calibrate delay loop. */
void calibrate_delay_loop(void)
{
	/* REVISIT */
}

/** Perform AArch64 specific tasks needed after cpu is initialized. */
void arch_post_cpu_init(void)
{
	/* REVISIT */
}

/** Set thread-local storage pointer. Not used on AArch64. */
sysarg_t sys_tls_set(uintptr_t addr)
{
	return EOK;
}

/** Change processor mode.
 *
 * @param kernel_uarg Userspace settings (entry point, stack, ...).
 */
void userspace(uspace_arg_t *kernel_uarg)
{
	/* REVISIT */

	while (1)
		;
}

/** Perform AArch64 specific tasks needed before the new task is run. */
void before_task_runs_arch(void)
{
	/* REVISIT */
}

/** Perform AArch64 specific tasks needed before the new thread is scheduled.
 */
void before_thread_runs_arch(void)
{
	/* REVISIT */
}

/** Perform AArch64 specific tasks before a thread stops running. */
void after_thread_ran_arch(void)
{
	/* REVISIT */
}

/** Reboot the system. */
void arch_reboot(void)
{
	/* Not implemented. */
	/* REVISIT */
	while (true)
		;
}

/** Construct function pointer.
 *
 * @param fptr   Function pointer structure.
 * @param addr   Function address.
 * @param caller Calling function address.
 *
 * @return Address of the function pointer.
 */
void *arch_construct_function(fncptr_t *fptr, void *addr, void *caller)
{
	return addr;
}

/** Perform AArch64 specific tasks to initialize IRQ processing. */
void irq_initialize_arch(irq_t *irq __attribute__((unused)))
{
}

/** @}
 */
