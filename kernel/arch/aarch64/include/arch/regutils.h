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
 * @brief Utilities for convenient manipulation with ARM registers.
 */

#ifndef KERN_aarch64_REGUTILS_H_
#define KERN_aarch64_REGUTILS_H_

#ifndef __ASM__
#if defined(KERNEL) || defined(BOOT)
#include <typedefs.h>
#else
#include <sys/types.h>
#endif

#define SPECIAL_REG_GEN_READ(name) \
	static inline uintptr_t name##_read(void) \
	{ \
		uintptr_t res; \
		asm volatile ( \
			"mrs %[res], " #name \
			: [res] "=r" (res) \
		); \
		return res; \
	}

#define SPECIAL_REG_GEN_WRITE(name) \
	static inline void name##_write(uintptr_t regn) \
	{ \
		asm volatile ( \
			"msr " #name ", %[regn]" \
			:: [regn] "r" (regn) \
		); \
	}

#else /* __ASM__ */

#define SPECIAL_REG_GEN_READ(name)
#define SPECIAL_REG_GEN_WRITE(name)

#endif /* __ASM__*/

SPECIAL_REG_GEN_WRITE(TTBR0_EL1)
#define TTBR0_EL1_ASID_SHIFT  48

SPECIAL_REG_GEN_READ(DAIF)
SPECIAL_REG_GEN_WRITE(DAIF)
#define DAIF_IRQ_BIT_SHIFT  7  /* I flag */
#define DAIF_IRQ_BIT  (1 << DAIF_IRQ_BIT_SHIFT)

#define SPSR_MODE_MASK  0x1f
#define SPSR_MODE_AARCH64_EL0T  0x00  /* AArch64, Exception Level 0, SP_EL0 */

/* TLBI VAE1IS and TLBI ASIDE1IS parameter. */
#define TLBI_ASID_SHIFT  48

#endif

/** @}
 */
