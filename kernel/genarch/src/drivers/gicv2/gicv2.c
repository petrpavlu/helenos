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

/** @addtogroup genarch
 * @{
 */
/**
 * @file
 * @brief ARM Generic Interrupt Controller, Architecture version 2.0.
 *
 * This IRQC is present on the QEMU virt platform for ARM.
 */

#include <genarch/drivers/gicv2/gicv2.h>
#include <arch/asm.h>

/** Initialize GICv2 interrupt controller.
 *
 * @param irqc Instance structure.
 * @param distr Distributor registers.
 * @param cpui CPU interface registers.
 */
void gicv2_init(gicv2_t *irqc, gicv2_distr_regs_t *distr,
    gicv2_cpui_regs_t *cpui)
{
	irqc->distr = distr;
	irqc->cpui = cpui;

	/* Get maximum number of interrupts. */
	uint32_t typer = pio_read_32(&distr->typer);
	irqc->num_interrupts = (((typer & GICV2D_TYPER_ITLINESNUMBER_MASK) >>
	    GICV2D_TYPER_ITLINESNUMBER_SHIFT) + 1) * 32;

	/* Disable all interrupts. */
	for (unsigned int i = 0; i < irqc->num_interrupts / 8; i++)
		pio_write_32(&distr->icenabler[i], 0xffffffff);

	/* Enable interrupts for all priority levels. */
	pio_write_32(&cpui->pmr, 0xff);

	/* Enable signaling of interrupts. */
	pio_write_32(&cpui->ctlr, GICV2C_CTLR_ENABLE_FLAG);
	pio_write_32(&distr->ctlr, GICV2D_CTLR_ENABLE_FLAG);
}

/** @}
 */
