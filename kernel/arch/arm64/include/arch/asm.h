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
 * @brief Declarations of functions implemented in assembly.
 */

#ifndef KERN_arm64_ASM_H_
#define KERN_arm64_ASM_H_

#include <config.h>
#include <trace.h>

extern char exc_vector;

/** CPU specific way to sleep cpu. */
NO_TRACE static inline void cpu_sleep(void)
{
	asm volatile ("wfe");
}

/** Return base address of current stack.
 *
 * Return the base address of the current stack.
 * The stack is assumed to be STACK_SIZE bytes long.
 * The stack must start on page boundary.
 */
NO_TRACE static inline uintptr_t get_stack_base(void)
{
	uintptr_t v;

	asm volatile (
	    "mov %[v], sp\n"
	    "and %[v], %[v], %[size]\n"
	    : [v] "=&r" (v)
	    : [size] "r" (~((uint64_t) STACK_SIZE - 1))
	);

	return v;
}

NO_TRACE static inline void asm_delay_loop(uint32_t usec)
{
	/* REVISIT */
}

/** Halts CPU. */
NO_TRACE static inline __attribute__((noreturn)) void cpu_halt(void)
{
	/* REVISIT */
	while (true)
		;
}

/** Output byte to port.
 *
 * @param port Port to write to.
 * @param val  Value to write.
 */
NO_TRACE static inline void pio_write_8(ioport8_t *port, uint8_t val)
{
	*port = val;
}

/** Output half-word to port.
 *
 * @param port Port to write to.
 * @param val  Value to write.
 */
NO_TRACE static inline void pio_write_16(ioport16_t *port, uint16_t val)
{
	*port = val;
}

/** Output word to port.
 *
 * @param port Port to write to.
 * @param val  Value to write.
 */
NO_TRACE static inline void pio_write_32(ioport32_t *port, uint32_t val)
{
	*port = val;
}

/** Get byte from port.
 *
 * @param port Port to read from.
 * @return Value read.
 */
NO_TRACE static inline uint8_t pio_read_8(const ioport8_t *port)
{
	return *port;
}

/** Get word from port.
 *
 * @param port Port to read from.
 * @return Value read.
 */
NO_TRACE static inline uint16_t pio_read_16(const ioport16_t *port)
{
	return *port;
}

/** Get double word from port.
 *
 * @param port Port to read from.
 * @return Value read.
 */
NO_TRACE static inline uint32_t pio_read_32(const ioport32_t *port)
{
	return *port;
}

#endif

/** @}
 */
