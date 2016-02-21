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

#include <arch/exception.h>
#include <arch/istate_struct.h>

.text

.global memcpy_from_uspace
.global memcpy_to_uspace
.global memcpy_from_uspace_failover_address
.global memcpy_to_uspace_failover_address
.global early_putchar
.global exc_vector

memcpy_from_uspace:
memcpy_to_uspace:
	/* Completely un-optimized memcpy(). */
	/*
	 * TODO Improve this code when HelenOS uses newer binutils which have a
	 * fixed GNU assembler.
	 */

	cbz x2, 2f
	mov x3, x0
1:
	ldrb w4, [x3], #1
	strb w4, [x1], #1
	subs x2, x2, #1
	b.ne 1b

2:
	ret

memcpy_from_uspace_failover_address:
memcpy_to_uspace_failover_address:
	mov x0, #0
	ret

/* REVISIT */
early_putchar:
	ret

/* Exception vector. */
.macro handler i
handler_\i:
	/*
	 * Initial code for each handler, at maximum 128 bytes (32
	 * instructions).
	 */

	/* Static checks for the following code. */
#if ISTATE_OFFSET_X1 + ISTATE_SIZE_X1 != ISTATE_OFFSET_X2
#error x1 and x2 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X3 + ISTATE_SIZE_X3 != ISTATE_OFFSET_X4
#error x3 and x4 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X5 + ISTATE_SIZE_X5 != ISTATE_OFFSET_X6
#error x5 and x6 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X7 + ISTATE_SIZE_X7 != ISTATE_OFFSET_X8
#error x7 and x8 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X9 + ISTATE_SIZE_X9 != ISTATE_OFFSET_X10
#error x9 and x10 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X11 + ISTATE_SIZE_X11 != ISTATE_OFFSET_X12
#error x11 and x12 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X13 + ISTATE_SIZE_X13 != ISTATE_OFFSET_X14
#error x13 and x14 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X15 + ISTATE_SIZE_X15 != ISTATE_OFFSET_X16
#error x15 and x16 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X17 + ISTATE_SIZE_X17 != ISTATE_OFFSET_X18
#error x17 and x18 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X19 + ISTATE_SIZE_X19 != ISTATE_OFFSET_X20
#error x19 and x20 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X21 + ISTATE_SIZE_X21 != ISTATE_OFFSET_X22
#error x21 and x22 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X23 + ISTATE_SIZE_X23 != ISTATE_OFFSET_X24
#error x23 and x24 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X25 + ISTATE_SIZE_X25 != ISTATE_OFFSET_X26
#error x25 and x26 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X27 + ISTATE_SIZE_X27 != ISTATE_OFFSET_X28
#error x27 and x28 are not successive in istate_t
#endif
#if ISTATE_OFFSET_X29 + ISTATE_SIZE_X29 != ISTATE_OFFSET_X30
#error x29 and x30 are not successive in istate_t
#endif
#if ISTATE_OFFSET_PC + ISTATE_SIZE_PC != ISTATE_OFFSET_X0
#error pc and x0 are not successive in istate_t
#endif
#if ISTATE_OFFSET_SPSR + ISTATE_SIZE_SPSR != ISTATE_OFFSET_SP
#error spsr and sp are not successive in istate_t
#endif

	/* Save current state. */
	.word 0xd10443ff /* sub sp, sp, #ISTATE_SIZE */ /* 0x00 */
	stp x1, x2, [sp, #ISTATE_OFFSET_X1]     /* 0x04 */
	stp x3, x4, [sp, #ISTATE_OFFSET_X3]     /* 0x08 */
	stp x5, x6, [sp, #ISTATE_OFFSET_X5]     /* 0x0c */
	stp x7, x8, [sp, #ISTATE_OFFSET_X7]     /* 0x10 */
	stp x9, x10, [sp, #ISTATE_OFFSET_X9]    /* 0x14 */
	stp x11, x12, [sp, #ISTATE_OFFSET_X11]  /* 0x18 */
	stp x13, x14, [sp, #ISTATE_OFFSET_X13]  /* 0x1c */
	stp x15, x16, [sp, #ISTATE_OFFSET_X15]  /* 0x20 */
	stp x17, x18, [sp, #ISTATE_OFFSET_X17]  /* 0x24 */
	stp x19, x20, [sp, #ISTATE_OFFSET_X19]  /* 0x28 */
	stp x21, x22, [sp, #ISTATE_OFFSET_X21]  /* 0x2c */
	stp x23, x24, [sp, #ISTATE_OFFSET_X23]  /* 0x30 */
	stp x25, x26, [sp, #ISTATE_OFFSET_X25]  /* 0x34 */
	stp x27, x28, [sp, #ISTATE_OFFSET_X27]  /* 0x38 */
	stp x29, x30, [sp, #ISTATE_OFFSET_X29]  /* 0x3c */

	mrs x1, elr_el1                         /* 0x40 */
	stp x1, x0, [sp, #ISTATE_OFFSET_PC]     /* 0x44 */

	mrs x0, spsr_el1                        /* 0x48 */
	mrs x1, sp_el0                          /* 0x4c */
	stp x0, x1, [sp, #ISTATE_OFFSET_SPSR]   /* 0x50 */

	mov x0, #\i                             /* 0x54 */
	mov x1, sp                              /* 0x58 */
	bl exc_dispatch                         /* 0x5c */

	/* Restore previous state. */
	ldp x0, x1, [sp, #ISTATE_OFFSET_SPSR]   /* 0x60 */
	msr spsr_el1, x0                        /* 0x64 */
	msr sp_el0, x1                          /* 0x68 */

	ldp x1, x0, [sp, #ISTATE_OFFSET_PC]     /* 0x6c */
	msr elr_el1, x1                         /* 0x70 */

	ldp x1, x2, [sp, #ISTATE_OFFSET_X1]     /* 0x74 */
	ldp x3, x4, [sp, #ISTATE_OFFSET_X3]     /* 0x78 */
	b exc_restore_end                       /* 0x7c */
.endm

exc_restore_end:
	/* Restore remaining registers and return from the exception handler. */
	ldp x5, x6, [sp, #ISTATE_OFFSET_X5]
	ldp x7, x8, [sp, #ISTATE_OFFSET_X7]
	ldp x9, x10, [sp, #ISTATE_OFFSET_X9]
	ldp x11, x12, [sp, #ISTATE_OFFSET_X11]
	ldp x13, x14, [sp, #ISTATE_OFFSET_X13]
	ldp x15, x16, [sp, #ISTATE_OFFSET_X15]
	ldp x17, x18, [sp, #ISTATE_OFFSET_X17]
	ldp x19, x20, [sp, #ISTATE_OFFSET_X19]
	ldp x21, x22, [sp, #ISTATE_OFFSET_X21]
	ldp x23, x24, [sp, #ISTATE_OFFSET_X23]
	ldp x25, x26, [sp, #ISTATE_OFFSET_X25]
	ldp x27, x28, [sp, #ISTATE_OFFSET_X27]
	ldp x29, x30, [sp, #ISTATE_OFFSET_X29]
	.word 0x910443ff /* add sp, sp, #ISTATE_SIZE */
	eret

.align 11
exc_vector:
.org exc_vector + 0x000
	handler EXC_CURRENT_EL_SP_SEL0_SYNCH
.org exc_vector + 0x080
	handler EXC_CURRENT_EL_SP_SEL0_IRQ
.org exc_vector + 0x100
	handler EXC_CURRENT_EL_SP_SEL0_FIQ
.org exc_vector + 0x180
	handler EXC_CURRENT_EL_SP_SEL0_SERROR
.org exc_vector + 0x200
	handler EXC_CURRENT_EL_SP_SELX_SYNCH
.org exc_vector + 0x280
	handler EXC_CURRENT_EL_SP_SELX_IRQ
.org exc_vector + 0x300
	handler EXC_CURRENT_EL_SP_SELX_FIQ
.org exc_vector + 0x380
	handler EXC_CURRENT_EL_SP_SELX_SERROR
.org exc_vector + 0x400
	handler EXC_LOWER_EL_AARCH64_SYNCH
.org exc_vector + 0x480
	handler EXC_LOWER_EL_AARCH64_IRQ
.org exc_vector + 0x500
	handler EXC_LOWER_EL_AARCH64_FIQ
.org exc_vector + 0x580
	handler EXC_LOWER_EL_AARCH64_SERROR
.org exc_vector + 0x600
	handler EXC_LOWER_EL_AARCH32_SYNCH
.org exc_vector + 0x680
	handler EXC_LOWER_EL_AARCH32_IRQ
.org exc_vector + 0x700
	handler EXC_LOWER_EL_AARCH32_FIQ
.org exc_vector + 0x780
	handler EXC_LOWER_EL_AARCH32_SERROR
.org exc_vector + 0x800
