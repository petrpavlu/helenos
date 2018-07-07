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

/** @addtogroup arm64boot
 * @{
 */
/** @file
 * @brief Relocate support for mkarray entries.
 */

#ifndef BOOT_arm64_RELOCATE_H
#define BOOT_arm64_RELOCATE_H

#define RELOC_MKARRAY_ENTRY(label, index, name, symbol) \
do { \
	asm volatile ( \
		".global " #symbol "\n" \
		"adr x0, " #label "s + %[off]\n" \
		"adr x1, 1f\n" \
		"adrp x2, " #symbol "\n" \
		"add x2, x2, #:lo12:" #symbol "\n" \
		"stp x1, x2, [x0], #16\n" \
		"b 2f\n" \
		"1:\n" \
		".asciz " #name "\n" \
		".align 2\n" \
		"2:\n" \
		: : [off] "i" (index * sizeof(label##_t)) : "x0", "x1", "x2" \
	); \
} while (0)

#define RETURN_MKARRAY_ENTRIES(label) \
do { \
	label##_t *res; \
	asm volatile ( \
		".global " #label "s\n" \
		"adr %[res], " #label "s\n" \
		: [res] "=r" (res) \
	); \
	return res; \
} while (0)

#endif

/** @}
 */
