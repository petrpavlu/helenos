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

/** @addtogroup arm64virt
 * @{
 */
/** @file
 * @brief QEMU virt platform driver.
 */

#include <arch/mach/virt/virt.h>
#include <console/console.h>
#include <genarch/drivers/pl011/pl011.h>
#include <genarch/srln/srln.h>

#define VIRT_UART_IRQ  1
#define VIRT_UART  0x09000000

static void virt_output_init(void);
static void virt_input_init(void);
static const char *virt_get_platform_name(void);

struct {
	pl011_uart_t uart;
} virt;

struct arm_machine_ops virt_machine_ops = {
	virt_output_init,
	virt_input_init,
	virt_get_platform_name
};

static void virt_output_init(void)
{
	if (pl011_uart_init(&virt.uart, VIRT_UART_IRQ, VIRT_UART))
		stdout_wire(&virt.uart.outdev);
}

static void virt_input_init(void)
{
	/* REVISIT */
	srln_instance_t *srln_instance = srln_init();
	if (srln_instance) {
		indev_t *sink = stdin_wire();
		indev_t *srln = srln_wire(srln_instance, sink);

		pl011_uart_input_wire(&virt.uart, srln);
	}
}

const char *virt_get_platform_name(void)
{
	return "virt";
}

/** @}
 */
