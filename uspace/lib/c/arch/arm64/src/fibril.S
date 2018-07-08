#
# Copyright (c) 2015 Petr Pavlu
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# - Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# - The name of the author may not be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#include <abi/asmtool.h>

.text

FUNCTION_BEGIN(__setjmp)
	# Save callee-saved registers into context_t pointed by x0.
	mov x1, sp
	stp x1, x30, [x0], #16
	stp x19, x20, [x0], #16
	stp x21, x22, [x0], #16
	stp x23, x24, [x0], #16
	stp x25, x26, [x0], #16
	stp x27, x28, [x0], #16
	# Save the last register and TLS.
	mrs x1, tpidr_el0
	stp x29, x1, [x0]

	# Return 0.
	mov x0, #0
	ret
FUNCTION_END(__setjmp)

FUNCTION_BEGIN(__longjmp)
	# Restore callee-saved registers from context_t pointed by x0.
	ldp x2, x30, [x0], #16
	mov sp, x2
	ldp x19, x20, [x0], #16
	ldp x21, x22, [x0], #16
	ldp x23, x24, [x0], #16
	ldp x25, x26, [x0], #16
	ldp x27, x28, [x0], #16
	# Restore the last register and TLS.
	ldp x29, x2, [x0]
	msr tpidr_el0, x2

	# Return the second argument.
	mov x0, x1
	ret
FUNCTION_END(__longjmp)
