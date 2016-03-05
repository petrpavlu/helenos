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

/** @addtogroup arm64
 * @{
 */
/** @file
 * @brief ARM64 architecture specific functions.
 */

#include <abi/errno.h>
#include <arch.h>
#include <arch/exception.h>
#include <arch/machine_func.h>
#include <ddi/irq.h>
#include <interrupt.h>
#include <proc/scheduler.h>
#include <syscall/syscall.h>
#include <sysinfo/sysinfo.h>
#include <userspace.h>

/** Perform ARM64 specific initialization before main_bsp() is called. */
void arch_pre_main(void *entry __attribute__((unused)), bootinfo_t *bootinfo)
{
	/* Copy init task info. */
	init.cnt = min3(bootinfo->taskmap.cnt, TASKMAP_MAX_RECORDS,
	    CONFIG_INIT_TASKS);

	size_t i;
	for (i = 0; i < init.cnt; i++) {
		init.tasks[i].paddr =
		    (uintptr_t) bootinfo->taskmap.tasks[i].addr;
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

	/* Initialize machine_ops pointer. */
	machine_ops_init();
}

/** Perform ARM64 specific tasks needed before the memory management is
 * initialized.
 */
void arch_pre_mm_init(void)
{
	if (config.cpu_active != 1)
		return;

	/* Initialize exception dispatch table. */
	exception_init();
}

/** Perform ARM64 specific tasks needed before the memory management is
 * initialized.
 */
void arch_post_mm_init(void)
{
	if (config.cpu_active != 1)
		return;

	/* Initialize IRQ routing. */
	irq_init(16, 16);

	/* Merge all memory zones to 1 big zone. */
	zone_merge_all();

	/* Initialize output device. */
	machine_output_init();
}

/** Perform ARM64 specific tasks needed before the multiprocessing is
 * initialized.
 */
void arch_pre_smp_init(void)
{
}

/** Perform ARM64 specific tasks needed after the multiprocessing is
 * initialized.
 */
void arch_post_smp_init(void)
{
	/* Set platform name. */
	const char *platform = machine_get_platform_name();

	sysinfo_set_item_data("platform", NULL, (void *) platform,
	    str_size(platform));

	/* Initialize input device. */
	machine_input_init();
}

/** Calibrate delay loop. */
void calibrate_delay_loop(void)
{
	/* REVISIT */
}

/** Perform ARM64 specific tasks needed after cpu is initialized. */
void arch_post_cpu_init(void)
{
	/* REVISIT */
}

/** Set thread-local storage pointer. Not used on ARM64. */
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
	/* Prepare return to EL0. */
	SPSR_EL1_write((SPSR_EL1_read() & ~SPSR_MODE_MASK) |
	    SPSR_MODE_ARM64_EL0T);

	/* Set program entry. */
	ELR_EL1_write((uintptr_t) kernel_uarg->uspace_entry);

	/* Set user stack. */
	SP_EL0_write(((uintptr_t) kernel_uarg->uspace_stack +
	    kernel_uarg->uspace_stack_size));

	/* Clear Thread ID register. */
	TPIDR_EL0_write(0);

	asm volatile (
		/*
		 * Clear all general-purpose registers, except x0 that holds an
		 * argument for the user space.
		 */
		"mov x0, %[uspace_uarg]\n"
		"mov x1, #0\n"
		"mov x2, #0\n"
		"mov x3, #0\n"
		"mov x4, #0\n"
		"mov x5, #0\n"
		"mov x6, #0\n"
		"mov x7, #0\n"
		"mov x8, #0\n"
		"mov x9, #0\n"
		"mov x10, #0\n"
		"mov x11, #0\n"
		"mov x12, #0\n"
		"mov x13, #0\n"
		"mov x14, #0\n"
		"mov x15, #0\n"
		"mov x16, #0\n"
		"mov x17, #0\n"
		"mov x18, #0\n"
		"mov x19, #0\n"
		"mov x20, #0\n"
		"mov x21, #0\n"
		"mov x22, #0\n"
		"mov x23, #0\n"
		"mov x24, #0\n"
		"mov x25, #0\n"
		"mov x26, #0\n"
		"mov x27, #0\n"
		"mov x28, #0\n"
		"mov x29, #0\n"
		"mov x30, #0\n"
		"eret\n"
		:: [uspace_uarg] "r" (kernel_uarg->uspace_uarg)
	);

	/* Unreachable. */
	while (1)
		;
}

/** Perform ARM64 specific tasks needed before the new task is run. */
void before_task_runs_arch(void)
{
	/* REVISIT */
}

/** Perform ARM64 specific tasks needed before the new thread is scheduled.
 */
void before_thread_runs_arch(void)
{
	/* REVISIT */
}

/** Perform ARM64 specific tasks before a thread stops running. */
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

/** Perform ARM64 specific tasks to initialize IRQ processing. */
void irq_initialize_arch(irq_t *irq __attribute__((unused)))
{
}

/** @}
 */
