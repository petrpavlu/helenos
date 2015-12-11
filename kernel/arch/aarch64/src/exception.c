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
 * @brief Exception handlers and exception initialization routines.
 */

#include <interrupt.h>
#include <print.h>

/** Print #istate_t structure content.
 *
 * @param istate Structure to be printed.
 */
void istate_decode(istate_t *istate)
{
	printf("x0 =%0#18" PRIx64 "\tx1 =%0#18" PRIx64 "\t"
	    "x2 =%0#18" PRIx64 "\tx3 =%0#18" PRIx64 "\n",
	    istate->x0, istate->x1, istate->x2, istate->x3);
	printf("x4 =%0#18" PRIx64 "\tx5 =%0#18" PRIx64 "\t"
	    "x6 =%0#18" PRIx64 "\tx7 =%0#18" PRIx64 "\n",
	    istate->x4, istate->x5, istate->x6, istate->x7);
	printf("x8 =%0#18" PRIx64 "\tx9 =%0#18" PRIx64 "\t"
	    "x10=%0#18" PRIx64 "\tx11=%0#18" PRIx64 "\n",
	    istate->x8, istate->x9, istate->x10, istate->x11);
	printf("x12=%0#18" PRIx64 "\tx13=%0#18" PRIx64 "\t"
	    "x14=%0#18" PRIx64 "\tx15=%0#18" PRIx64 "\n",
	    istate->x12, istate->x13, istate->x14, istate->x15);
	printf("x16=%0#18" PRIx64 "\tx17=%0#18" PRIx64 "\t"
	    "x18=%0#18" PRIx64 "\tx19=%0#18" PRIx64 "\n",
	    istate->x16, istate->x17, istate->x18, istate->x19);
	printf("x20=%0#18" PRIx64 "\tx21=%0#18" PRIx64 "\t"
	    "x22=%0#18" PRIx64 "\tx23=%0#18" PRIx64 "\n",
	    istate->x20, istate->x21, istate->x22, istate->x23);
	printf("x24=%0#18" PRIx64 "\tx25=%0#18" PRIx64 "\t"
	    "x26=%0#18" PRIx64 "\tx27=%0#18" PRIx64 "\n",
	    istate->x24, istate->x25, istate->x26, istate->x27);
	printf("x28=%0#18" PRIx64 "\tx29=%0#18" PRIx64 "\t"
	    "x30=%0#18" PRIx64 "\n", istate->x28, istate->x29, istate->x30);
	printf("sp =%0#18" PRIx64 "\tpc =%0#18" PRIx64 "\t"
	    "spsr=%0#18" PRIx64 "\n", istate->sp, istate->pc, istate->spsr);
}

/** @}
 */
