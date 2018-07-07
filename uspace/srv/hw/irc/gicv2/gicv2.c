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

/** @addtogroup gicv2
 * @{
 */
/** @file
 * @brief ARM Generic Interrupt Controller, Architecture version 2.0.
 *
 * This IRQ controller is present on the QEMU virt platform for ARM.
 */

#include <async.h>
#include <bitops.h>
#include <ddi.h>
#include <errno.h>
#include <io/log.h>
#include <ipc/irc.h>
#include <ipc/services.h>
#include <macros.h>
#include <ns.h>
#include <stdint.h>
#include <stdio.h>
#include <str.h>
#include <sysinfo.h>

#define NAME  "gicv2"

/** GICv2 distributor register map. */
typedef struct {
	/** Distributor control register. */
	ioport32_t ctlr;
#define GICV2D_CTLR_ENABLE_FLAG  0x1

	/** Interrupt controller type register. */
	const ioport32_t typer;
#define GICV2D_TYPER_IT_LINES_NUMBER_SHIFT  0
#define GICV2D_TYPER_IT_LINES_NUMBER_MASK \
	(0x1f << GICV2D_TYPER_IT_LINES_NUMBER_SHIFT)

	/** Distributor implementer identification register. */
	const ioport32_t iidr;
	/** Reserved. */
	PADD32[5];
	/** Implementation defined registers. */
	ioport32_t impl[8];
	/** Reserved. */
	PADD32[16];
	/** Interrupt group registers. */
	ioport32_t igroupr[32];
	/** Interrupt set-enable registers. */
	ioport32_t isenabler[32];
	/** Interrupt clear-enable registers. */
	ioport32_t icenabler[32];
	/** Interrupt set-pending registers. */
	ioport32_t ispendr[32];
	/** Interrupt clear-pending registers. */
	ioport32_t icpendr[32];
	/** GICv2 interrupt set-active registers. */
	ioport32_t isactiver[32];
	/** Interrupt clear-active registers. */
	ioport32_t icactiver[32];
	/** Interrupt priority registers. */
	ioport32_t ipriorityr[255];
	/** Reserved. */
	PADD32;
	/** Interrupt processor target registers. First 8 words are read-only.
	 */
	ioport32_t itargetsr[255];
	/** Reserved. */
	PADD32;
	/** Interrupt configuration registers. */
	ioport32_t icfgr[64];
	/** Implementation defined registers. */
	ioport32_t impl2[64];
	/** Non-secure access control registers. */
	ioport32_t nsacr[64];
	/** Software generated interrupt register. */
	ioport32_t sgir;
	/** Reserved. */
	PADD32[3];
	/** SGI clear-pending registers. */
	ioport32_t cpendsgir[4];
	/** SGI set-pending registers. */
	ioport32_t spendsgir[4];
	/** Reserved. */
	PADD32[40];
	/** Implementation defined identification registers. */
	const ioport32_t impl3[12];
} gicv2_distr_regs_t;

/* GICv2 CPU interface register map. */
typedef struct {
	/** CPU interface control register. */
	ioport32_t ctlr;
#define GICV2C_CTLR_ENABLE_FLAG  0x1

	/** Interrupt priority mask register. */
	ioport32_t pmr;
	/** Binary point register. */
	ioport32_t bpr;
	/** Interrupt acknowledge register. */
	const ioport32_t iar;
#define GICV2C_IAR_INTERRUPT_ID_SHIFT  0
#define GICV2C_IAR_INTERRUPT_ID_MASK \
	(0x3ff << GICV2C_IAR_INTERRUPT_ID_SHIFT)
#define GICV2C_IAR_CPUID_SHIFT  10
#define GICV2C_IAR_CPUID_MASK \
	(0x7 << GICV2C_IAR_CPUID_SHIFT)

	/** End of interrupt register. */
	ioport32_t eoir;
	/** Running priority register. */
	const ioport32_t rpr;
	/** Highest priority pending interrupt register. */
	const ioport32_t hppir;
	/** Aliased binary point register. */
	ioport32_t abpr;
	/** Aliased interrupt acknowledge register. */
	const ioport32_t aiar;
	/** Aliased end of interrupt register. */
	ioport32_t aeoir;
	/** Aliased highest priority pending interrupt register. */
	const ioport32_t ahppir;
	/** Reserved. */
	PADD32[5];
	/** Implementation defined registers. */
	ioport32_t impl[36];
	/** Active priorities registers. */
	ioport32_t apr[4];
	/** Non-secure active priorities registers. */
	ioport32_t nsapr[4];
	/** Reserved. */
	PADD32[3];
	/** CPU interface identification register. */
	const ioport32_t iidr;
	/** Unallocated. */
	PADD32[960];
	/** Deactivate interrupt register. */
	ioport32_t dir;
} gicv2_cpui_regs_t;

/** GICv2 driver-specific device data. */
typedef struct {
	gicv2_distr_regs_t *distr;
	gicv2_cpui_regs_t *cpui;
	unsigned inum_total;
} gicv2_t;

static gicv2_t gicv2;

/** Enable specific interrupt. */
static int gicv2_enable_irq(sysarg_t irq)
{
	if (irq > gicv2.inum_total)
		return EINVAL;

	log_msg(LOG_DEFAULT, LVL_NOTE, "Enable interrupt '%" PRIun "'.", irq);

	pio_write_32(&gicv2.distr->isenabler[irq / 32],
	    BIT_V(uint32_t, irq % 32));
	return EOK;
}

/** Handle one connection to GICv2.
 *
 * @param iid   Hash of the request that opened the connection.
 * @param icall Call data of the request that opened the connection.
 * @param arg	Local argument.
 */
static void icpic_connection(ipc_callid_t iid, ipc_call_t *icall, void *arg)
{
	ipc_callid_t callid;
	ipc_call_t call;

	/* Answer the first IPC_M_CONNECT_ME_TO call. */
	async_answer_0(iid, EOK);

	while (true) {
		callid = async_get_call(&call);

		if (!IPC_GET_IMETHOD(call)) {
			/* The other side has hung up. */
			async_answer_0(callid, EOK);
			return;
		}

		switch (IPC_GET_IMETHOD(call)) {
		case IRC_ENABLE_INTERRUPT:
			async_answer_0(callid,
			    gicv2_enable_irq(IPC_GET_ARG1(call)));
			break;
		case IRC_CLEAR_INTERRUPT:
			/* Noop. */
			async_answer_0(callid, EOK);
			break;
		default:
			async_answer_0(callid, EINVAL);
			break;
		}
	}
}

static int gicv2_init(void)
{
	sysarg_t present;
	int rc = sysinfo_get_value("gicv2", &present);
	if (rc != EOK)
		present = 0;

	/* GICv2 not found. */
	if (!present) {
		log_msg(LOG_DEFAULT, LVL_ERROR, "Device not present.");
		return ENOENT;
	}

	/* Obtain addresses of distributor and CPU interface registers. */
	sysarg_t distr;
	rc = sysinfo_get_value("gicv2.distr.address.physical", &distr);
	if (rc != EOK) {
		log_msg(LOG_DEFAULT, LVL_ERROR, "Error getting physical "
		    "address of the distributor registers.");
		return rc;
	}

	sysarg_t cpui;
	rc = sysinfo_get_value("gicv2.cpui.address.physical", &cpui);
	if (rc != EOK) {
		log_msg(LOG_DEFAULT, LVL_ERROR, "Error getting physical "
		    "address of the CPU interface registers.");
		return rc;
	}

	/* Enable physical IO to access the registers. */
	void *distr_virt = NULL;
	void *cpui_virt = NULL;
	rc = pio_enable((void *) distr, sizeof(gicv2_distr_regs_t),
	    &distr_virt);
	if (rc != EOK) {
		log_msg(LOG_DEFAULT, LVL_ERROR, "Error enabling PIO for "
		    "distributor registers.");
		goto error;
	}
	rc = pio_enable((void *) cpui, sizeof(gicv2_cpui_regs_t), &cpui_virt);
	if (rc != EOK) {
		log_msg(LOG_DEFAULT, LVL_ERROR, "Error enabling PIO for CPU "
		    "interface registers.");
		goto error;
	}

	gicv2.distr = distr_virt;
	gicv2.cpui = cpui_virt;

	/* Get maximum number of interrupts. */
	uint32_t typer = pio_read_32(&gicv2.distr->typer);
	gicv2.inum_total = (((typer & GICV2D_TYPER_IT_LINES_NUMBER_MASK) >>
	    GICV2D_TYPER_IT_LINES_NUMBER_SHIFT) + 1) * 32;

	async_set_fallback_port_handler(icpic_connection, NULL);

	/* Register itself as an IRC service. */
	rc = ENOENT; /*service_register(SERVICE_IRC); FIXME */
	if (rc != EOK) {
		log_msg(LOG_DEFAULT, LVL_ERROR, "Failed registering itself as "
		    "an IRC service.");
		goto error;
	}

	return EOK;

error:
	if (distr_virt != NULL)
		pio_disable(distr_virt, sizeof(gicv2_distr_regs_t));
	if (cpui_virt != NULL)
		pio_disable(cpui_virt, sizeof(gicv2_cpui_regs_t));
	return rc;
}

int main(int argc, char **argv)
{
	int rc;

	printf(NAME ": HelenOS GICv2 interrupt controller driver\n");

	rc = log_init(NAME);
	if (rc != EOK) {
		printf(NAME ": Error connecting logging service.\n");
		return 1;
	}

	if (gicv2_init() != EOK)
		return -1;

	log_msg(LOG_DEFAULT, LVL_NOTE, NAME ": Accepting connections.");
	task_retval(0);
	async_manager();

	/* Not reached. */
	return 0;
}

/**
 * @}
 */
