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
 * @brief Memory barriers.
 */

#ifndef KERN_arm64_BARRIER_H_
#define KERN_arm64_BARRIER_H_

#ifdef KERNEL
#include <typedefs.h>
#else
#include <sys/types.h>
#endif

#define CS_ENTER_BARRIER()  asm volatile ("" ::: "memory")
#define CS_LEAVE_BARRIER()  asm volatile ("" ::: "memory")

/* TODO It is unneccessary to use full-system barriers in most cases where
 * these macros are used. Provide a better granularity. */
/** Full system data synchronization barrier. */
#define memory_barrier()  asm volatile ("dsb sy" ::: "memory")
/** Full system data synchronization barrier for reads. */
#define read_barrier()    asm volatile ("dsb ld" ::: "memory")
/** Full system data synchronization barrier for writes. */
#define write_barrier()   asm volatile ("dsb st" ::: "memory")
/** Instruction synchronization barrier. */
#define inst_barrier()    asm volatile ("isb" ::: "memory")

#ifdef KERNEL

/** Ensure visibility of instruction update for a multiprocessor.
 *
 * @param addr Address of the instruction.
 */
#define smc_coherence(addr) \
do { \
	asm volatile ( \
		/* Clean to Point of Unification to make the new instruction
		 * visible to the instruction cache. */ \
		"dc cvau, %[a]\n" \
		/* Ensure completion on all PEs. */ \
		"dsb ish\n" \
		/* Ensure instruction cache/branch predictor discards stale
		 * data. */ \
		"ic ivau, %[a]\n" \
		/* Ensure completion on all PEs. */ \
		"dsb ish\n" \
		/* Synchronize context on this PE. */ \
		"isb\n" \
		: : [a] "r" (addr) : "memory" \
	); \
} while (0)

/** Ensure visibility of instruction updates for a multiprocessor.
 *
 * @param addr Address of the first instruction.
 * @param size Size of the instruction block (in bytes).
 */
#define smc_coherence_block(addr, size) \
do { \
	for (uintptr_t a = (uintptr_t) addr; a < (uintptr_t) addr + size; \
	    a += 4) \
		smc_coherence(a); \
} while (0)

#endif /* KERNEL */

#endif

/** @}
 */
