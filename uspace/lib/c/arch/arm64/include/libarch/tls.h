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

/** @addtogroup libcarm64
 * @{
 */
/** @file
 * @brief Thread-local storage.
 */

#ifndef LIBC_arm64_TLS_H_
#define LIBC_arm64_TLS_H_

#include <stdint.h>

#define CONFIG_TLS_VARIANT_1

/** Offsets for accessing thread-local variables are shifted 16 bytes higher.
 */
#define ARM64_TP_OFFSET  (-16)

/** TCB (Thread Control Block) struct.
 *
 * TLS starts just after this struct.
 */
typedef struct {
	/** Fibril data. */
	void *fibril_data;
} tcb_t;

/** Sets TLS address to the TPIDR_EL0 register.
 *
 * @param tcb TCB (TLS starts behind).
 */
static inline void __tcb_set(tcb_t *tcb)
{
	uint8_t *tls = (uint8_t *) tcb;
	tls += sizeof(tcb_t) + ARM64_TP_OFFSET;
	asm volatile (
		"msr tpidr_el0, %[tls]"
		: : [tls] "r" (tls)
	);
}

/** Returns TCB address.
 *
 * @return TCB address (starts before TLS which address is stored in the
 * TPIDR_EL0 register).
 */
static inline tcb_t *__tcb_get(void)
{
	uint8_t *ret;
	asm volatile (
		"mrs %[tls], tpidr_el0"
		: [tls] "=r" (ret)
	);
	return (tcb_t *) (ret - ARM64_TP_OFFSET - sizeof(tcb_t));
}

#endif

/** @}
 */
