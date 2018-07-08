/*
 * Copyright (c) 2016 Petr Pavlu
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

/** @file
 */

#include <align.h>
#include <as.h>
#include <ddi.h>
#include <errno.h>
#include <macros.h>
#include <stdint.h>
#include <sysinfo.h>
#include <stdio.h>
#include "../ctl/serial.h"
#include "pl011.h"

/** Simplified PrimeCell UART (PL011) register map. */
typedef struct {
	/** UART data register. */
	ioport32_t data;
	/** Opaque registers. */
	PADD32[5];
	/** Flag register. */
	const ioport32_t flag;
	/** Transmit FIFO full. */
#define PL011_UART_FLAG_TXFF_FLAG  (1 << 5)

	/** Opaque registers. */
	PADD32[1017];
} pl011_uart_regs_t;

typedef struct {
	pl011_uart_regs_t *regs;
} pl011_t;

static pl011_t pl011;

static void pl011_sendb(uint8_t byte)
{
	/* Wait for space to become available in the TX FIFO. */
	while (pio_read_32(&pl011.regs->flag) & PL011_UART_FLAG_TXFF_FLAG)
		;

	pio_write_32(&pl011.regs->data, byte);
}

static void pl011_putchar(wchar_t ch)
{
	if (ascii_check(ch))
		pl011_sendb(ch);
	else
		pl011_sendb('?');
}

static void pl011_control_puts(const char *str)
{
	while (*str != '\0')
		pl011_sendb(*str++);
}

static void pl011_flush(void)
{
	// TODO
}

int pl011_init(void)
{
	sysarg_t present;
	int rc = sysinfo_get_value("fb", &present);
	if (rc != EOK)
		present = false;

	printf("pl011_init(), present=%lx\n", present);
	if (!present)
		return ENOENT;

	sysarg_t kind;
	rc = sysinfo_get_value("fb.kind", &kind);
	if (rc != EOK)
		kind = (sysarg_t) -1;

	printf("pl011_init(), kind=%lx\n", kind);
	if (kind != 7)
		return EINVAL;

	sysarg_t paddr;
	rc = sysinfo_get_value("fb.address.physical", &paddr);
	if (rc != EOK)
		return rc;

	printf("pl011_init(), paddr=%lx\n", paddr);
	pl011.regs = AS_AREA_ANY;

	rc = physmem_map(paddr,
	    ALIGN_UP(sizeof(pl011_uart_regs_t), PAGE_SIZE) >> PAGE_WIDTH,
	    AS_AREA_READ | AS_AREA_WRITE, (void **) &pl011.regs);
	if (rc != EOK)
		return rc;

	printf("pl011_init(), serial_init\n");
	return serial_init(pl011_putchar, pl011_control_puts, pl011_flush);
}

/** @}
 */
