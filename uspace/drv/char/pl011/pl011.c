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

/** @addtogroup pl011
 * @{
 */

/** @file
 * @brief ARM PrimeCell PL011 UART driver.
 */

#include <bitops.h>
#include <ddf/driver.h>
#include <ddf/interrupt.h>
#include <ddf/log.h>
#include <device/hw_res_parsed.h>
#include <errno.h>
#include <io/chardev_srv.h>
#include <irc.h>
#include <macros.h>
#include <stdio.h>
#include <str_error.h>

#define NAME "pl011"
#define NAMESPACE "char"

/** PL011 register map. */
typedef struct {
	/** UART data register. */
	ioport32_t data;
	union {
		/** Receive status register (same values that are in upper bits
		 * of data register).
		 */
		const ioport32_t status;
		/** Error clear register (writing anything clears all errors).
		 */
		ioport32_t error_clear;
	};
	/** Reserved. */
	PADD32[4];
	/** Flag register. */
	const ioport32_t flag;
	/** Transmit FIFO full. */
#define PL011_UART_FLAG_TXFF_FLAG  (1 << 5)

	/** Reserved. */
	PADD32;
	/** IrDA low-power counter register. */
	ioport32_t irda_low_power;
	/** Integer baud rate register. */
	ioport32_t int_baud_divisor;
	/** Fractional baud rate register. */
	ioport32_t fract_baud_divisor;
	/** Line control register. */
	ioport32_t line_control_high;
	/** Control register. */
	ioport32_t control;
	/** Interrupt FIFO level select register. */
	ioport32_t interrupt_fifo;
	/** Interrupt mask set/clear register. */
	ioport32_t interrupt_mask;
	/** Raw interrupt status register (pending interrupts before applying
	 * the mask).
	 */
	const ioport32_t raw_interrupt_status;
	/** Masked interrupt status register (pending interrupts after applying
	 * the mask).
	 */
	const ioport32_t masked_interrupt_status;
	/** Interrupt clear register (write 1s to clear pending interrupts). */
	ioport32_t interrupt_clear;

	/** Interrupt indicating a change in the nUARTRI modem status. */
#define PL011_UART_INTERRUPT_RIM_FLAG   (1 << 0)
	/** Interrupt indicating a change in the nUARTCTS modem status. */
#define PL011_UART_INTERRUPT_CTSM_FLAG  (1 << 1)
	/** Interrupt indicating a change in the nUARTDCD modem status. */
#define PL011_UART_INTERRUPT_DCDM_FLAG  (1 << 2)
	/** Interrupt indicating a change in the nUARTDSR modem status. */
#define PL011_UART_INTERRUPT_DSRM_FLAG  (1 << 3)
	/** The receive interrupt. */
#define PL011_UART_INTERRUPT_RX_FLAG    (1 << 4)
	/** The transmit interrupt. */
#define PL011_UART_INTERRUPT_TX_FLAG    (1 << 5)
	/** The receive timeout interrupt.  */
#define PL011_UART_INTERRUPT_RT_FLAG    (1 << 6)
	/** Interrupt indicating an overrun error. */
#define PL011_UART_INTERRUPT_FE_FLAG    (1 << 7)
	/** Interrupt indicating a break in the reception. */
#define PL011_UART_INTERRUPT_PE_FLAG    (1 << 8)
	/** Interrupt indicating a parity error in the received character. */
#define PL011_UART_INTERRUPT_BE_FLAG    (1 << 9)
	/** Interrupt indicating a framing error in the received character. */
#define PL011_UART_INTERRUPT_OE_FLAG    (1 << 10)
	/** All interrupt mask. */
#define PL011_UART_INTERRUPT_ALL        0x3ff

	/** DMA control register. */
	ioport32_t dma_control;
	/** Reserved. */
	PADD32[13];
	/** Reserved for test purposes. */
	PADD32[4];
	/** Reserved. */
	PADD32[976];
	/** Reserved for future ID expansion. */
	PADD32[4];
	/** UARTPeriphID0 register. */
	const ioport32_t periph_id0;
	/** UARTPeriphID1 register. */
	const ioport32_t periph_id1;
	/** UARTPeriphID2 register. */
	const ioport32_t periph_id2;
	/** UARTPeriphID3 register. */
	const ioport32_t periph_id3;
	/** UARTPCellID0 register. */
	const ioport32_t cell_id0;
	/** UARTPCellID1 register. */
	const ioport32_t cell_id1;
	/** UARTPCellID2 register. */
	const ioport32_t cell_id2;
	/** UARTPCellID3 register. */
	const ioport32_t cell_id3;
} pl011_uart_regs_t;

static int pl011_dev_add(ddf_dev_t *);
static int pl011_fun_online(ddf_fun_t *);
static int pl011_fun_offline(ddf_fun_t *);
static void pl011_char_conn(ipc_callid_t, ipc_call_t *, void *);
static int pl011_read(chardev_srv_t *, void *, size_t);
static int pl011_write(chardev_srv_t *, const void *, size_t);

/** PL011 driver operations. */
static driver_ops_t driver_ops = {
	.dev_add = pl011_dev_add,
	.fun_online = pl011_fun_online,
	.fun_offline = pl011_fun_offline
};

/** PL011 driver. */
static driver_t pl011_driver = {
	.name = NAME,
	.driver_ops = &driver_ops
};

/** PL011 character interface. */
static chardev_ops_t pl011_chardev_ops = {
	.read = pl011_read,
	.write = pl011_write
};

/** PL011 driver-specific device data. */
typedef struct {
	async_sess_t *parent_sess;
	ddf_dev_t *dev;
	char *name;

	ddf_fun_t *fun;
	chardev_srvs_t cds;

	pl011_uart_regs_t *regs;
	int irq;
	uint8_t buffer[64];
	size_t buf_rp;
	size_t buf_wp;
	fibril_condvar_t buf_cv;
	fibril_mutex_t buf_lock;
} pl011_t;

/** Address range accessed by the PL011 interrupt pseudo-code. */
static const irq_pio_range_t pl011_ranges[] = {
	{
		.base = 0,
		.size = sizeof(pl011_uart_regs_t)
	}
};

/** PL011 interrupt pseudo-code instructions. */
static const irq_cmd_t pl011_cmds[] = {
	{
		/* Read masked_interrupt_status. */
		.cmd = CMD_PIO_READ_32,
		.addr = NULL,
		.dstarg = 1
	},
	{
		.cmd = CMD_AND,
		.value = PL011_UART_INTERRUPT_RX_FLAG |
		    PL011_UART_INTERRUPT_RT_FLAG,
		.srcarg = 1,
		.dstarg = 3
	},
	{
		.cmd = CMD_PREDICATE,
		.value = 1,
		.srcarg = 3
	},
	{
		/* Read data. */
		.cmd = CMD_PIO_READ_32,
		.addr = NULL,
		.dstarg = 2
	},
	{
		.cmd = CMD_ACCEPT
	}
};

/** Process an interrupt from a PL011 device.
 *
 * @param dev The PL011 serial port device.
 */
static void pl011_interrupt(ipc_callid_t iid, ipc_call_t *call, ddf_dev_t *dev)
{
	const char *name = ddf_dev_get_name(dev);
	uint32_t intrs = IPC_GET_ARG1(*call);
	uint8_t byte = IPC_GET_ARG2(*call);

	ddf_msg(LVL_DEBUG, "pl011_dev_interrupt(), device=%s, intrs=%#x, "
	    "byte=%#x.", name, intrs, byte);

	if ((intrs & (PL011_UART_INTERRUPT_RX_FLAG |
	    PL011_UART_INTERRUPT_RT_FLAG)) == 0) {
		/* TODO */
		return;
	}

	pl011_t *pl011 = (pl011_t *) ddf_dev_data_get(dev);

	fibril_mutex_lock(&pl011->buf_lock);

	size_t nidx = (pl011->buf_wp + 1) % sizeof(pl011->buffer);
	if (nidx == pl011->buf_rp) {
		/* Buffer overrun. */
		ddf_msg(LVL_WARN, "Buffer overrun on '%s'. Byte '%#x' "
		    "discarded.", name, byte);
		fibril_mutex_unlock(&pl011->buf_lock);
		return;
	}

	pl011->buffer[pl011->buf_wp] = byte;
	pl011->buf_wp = nidx;
	ddf_msg(LVL_DEBUG2, "Byte '%#x' saved to the buffer of '%s'.", byte,
	    name);
	fibril_condvar_broadcast(&pl011->buf_cv);

	fibril_mutex_unlock(&pl011->buf_lock);
}

/** Initialize a PL011 device.
 *
 * @param pl011 The PL011 serial port device data.
 *
 * @return Error code.
 */
static int pl011_init(pl011_t *pl011)
{
	const char *name = ddf_dev_get_name(pl011->dev);
	hw_res_list_parsed_t res;
	int rc;
	bool pio_enabled = false;
	bool interrupt_handler_registered = false;

	const size_t range_count = sizeof(pl011_ranges) /
	    sizeof(irq_pio_range_t);
	irq_pio_range_t ranges[range_count];
	const size_t cmd_count = sizeof(pl011_cmds) / sizeof(irq_cmd_t);
	irq_cmd_t cmds[cmd_count];

	fibril_mutex_initialize(&pl011->buf_lock);
	fibril_condvar_initialize(&pl011->buf_cv);
	pl011->buf_rp = pl011->buf_wp = 0;

	/* Get the information about the device. */
	pl011->parent_sess = ddf_dev_parent_sess_create(pl011->dev);
	if (pl011->parent_sess == NULL) {
		ddf_msg(LVL_ERROR, "Failed connecting parent driver of device "
		    "'%s'.", name);
		rc = ENOENT;
		goto error;
	}

	hw_res_list_parsed_init(&res);
	rc = hw_res_get_list_parsed(pl011->parent_sess, &res, 0);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed getting resource list for device "
		    "'%s': %s.", name, str_error(rc));
		goto error;
	}

	/* Validate the obtained information. */
	if (res.mem_ranges.count != 1) {
		ddf_msg(LVL_ERROR, "Expected exactly one memory range for "
		    "device '%s' but received '%zu'.", name,
		    res.mem_ranges.count);
		rc = EINVAL;
		goto error;
	}

	size_t iosize = RNGSZ(res.mem_ranges.ranges[0]);
	if (iosize < sizeof(pl011_uart_regs_t)) {
		ddf_msg(LVL_ERROR, "Expected memory range of size at least "
		    "'%zu' bytes for device '%s' but got '%zu' bytes.",
		    sizeof(pl011_uart_regs_t), name, iosize);
		rc = EINVAL;
		goto error;
	}

	if (res.irqs.count != 1) {
		ddf_msg(LVL_ERROR, "Expected exactly one IRQ for device '%s' "
		    "but received '%zu'.", name, res.irqs.count);
		rc = EINVAL;
		goto error;
	}

	/* Enable IO to the device. */
	void *iobase = RNGABSPTR(res.mem_ranges.ranges[0]);
	void *regs;
	rc = pio_enable(iobase, sizeof(pl011_uart_regs_t), &regs);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Error enabling PIO for device '%s': %s.",
		    name, str_error(rc));
		goto error;
	}
	pio_enabled = true;

	pl011->regs = regs;

	/* Register an interrupt handler. */
	memcpy(ranges, pl011_ranges, sizeof(pl011_ranges));
	memcpy(cmds, pl011_cmds, sizeof(pl011_cmds));

	ranges[0].base = (sysarg_t) iobase;
	pl011_uart_regs_t *regsphys = iobase;
	cmds[0].addr = (void *) &regsphys->masked_interrupt_status;
	cmds[3].addr = (void *) &regsphys->data;

	irq_code_t irq_code = {
		.rangecount = range_count,
		.ranges = ranges,
		.cmdcount = cmd_count,
		.cmds = cmds
	};

	rc = register_interrupt_handler(pl011->dev, res.irqs.irqs[0],
	    pl011_interrupt, &irq_code);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed registering interrupt handler for "
		    "device '%s' and IRQ '%d': %s.", name, res.irqs.irqs[0],
		    str_error(rc));
		goto error;
	}
	interrupt_handler_registered = true;

	/* Enable the interrupt. */
	rc = irc_enable_interrupt(res.irqs.irqs[0]);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed enabling interrupt '%d' for device "
		    "'%s': %s.", res.irqs.irqs[0], name, str_error(rc));
		goto error;
	}

	pl011->irq = res.irqs.irqs[0];

	return EOK;

error:
	if (interrupt_handler_registered)
		unregister_interrupt_handler(pl011->dev, res.irqs.irqs[0]);
	if (pio_enabled)
		pio_disable(pl011->regs, sizeof(pl011_uart_regs_t));
	return rc;
}

/** Deinitialize a PL011 device.
 *
 * @param pl011 The PL011 serial port device data.
 *
 * @return Error code.
 */
static void pl011_uninit(pl011_t *pl011)
{
	irc_disable_interrupt(pl011->irq);
	unregister_interrupt_handler(pl011->dev, pl011->irq);
	pio_disable(pl011->regs, sizeof(pl011_uart_regs_t));
}

/** Read data from a PL011 port.
 *
 * @param srv Connection-specific data.
 * @param data Output data buffer.
 * @param size Size of the output data buffer.
 *
 * @return Bytes read.
 */
static int pl011_read(chardev_srv_t *srv, void *buffer, size_t size)
{
	pl011_t *pl011 = (pl011_t *) srv->srvs->sarg;
	uint8_t *bp = buffer;
	size_t left = size;

	fibril_mutex_lock(&pl011->buf_lock);

	while (left > 0) {
		while (pl011->buf_rp == pl011->buf_wp)
			fibril_condvar_wait(&pl011->buf_cv, &pl011->buf_lock);
		*bp++ = pl011->buffer[pl011->buf_rp];
		--left;
		pl011->buf_rp = (pl011->buf_rp + 1) % sizeof(pl011->buffer);
	}

	fibril_mutex_unlock(&pl011->buf_lock);

	return size;
}

/** Write data to a PL011 port.
 *
 * @param srv Connection-specific data.
 * @param data Data to write.
 * @param size Size of the data.
 *
 * @return Bytes written.
 */
static int pl011_write(chardev_srv_t *srv, const void *data, size_t size)
{
	pl011_t *pl011 = (pl011_t *) srv->srvs->sarg;
	uint8_t *dp = (uint8_t *) data;

	for (size_t i = 0; i < size; i++) {
		/* Wait until it is safe to write to the device. */
		while (pio_read_32(&pl011->regs->flag) &
		    PL011_UART_FLAG_TXFF_FLAG)
			;

		pio_write_32(&pl011->regs->data, dp[i]);
	}

	return size;
}

/** Handle a new connection to a PL011 device.
 *
 * @param iid The IPC call ID.
 * @param icall IPC request.
 * @param arg A PL011 function.
 */
static void pl011_char_conn(ipc_callid_t iid, ipc_call_t *icall, void *arg)
{
	pl011_t *pl011 = (pl011_t *) ddf_dev_data_get(ddf_fun_get_dev(
	    (ddf_fun_t *) arg));

	chardev_conn(iid, icall, &pl011->cds);
}

/** Initialize a new device instance of the PL011 driver.
 *
 * @param dev The PL011 serial port device.
 *
 * @return Error code.
 */
static int pl011_dev_add(ddf_dev_t *dev)
{
	const char *name = ddf_dev_get_name(dev);
	ddf_fun_t *fun = NULL;
	int rc;
	bool pl011_initialized = false;

	ddf_msg(LVL_DEBUG, "pl011_dev_add(), device=%s.", name);

	/* Allocate soft state. */
	pl011_t *pl011 = ddf_dev_data_alloc(dev, sizeof(pl011_t));
	if (pl011 == NULL) {
		ddf_msg(LVL_ERROR, "Failed allocating soft state for device"
		    "'%s'.", name);
		rc = ENOMEM;
		goto error;
	}

	pl011->dev = dev;

	/* Inititialize the device and access to it. */
	rc = pl011_init(pl011);
	if (rc != EOK)
		goto error;
	pl011_initialized = true;

	/* Set up the chardev interface. */
	chardev_srvs_init(&pl011->cds);
	pl011->cds.ops = &pl011_chardev_ops;
	pl011->cds.sarg = pl011;

	/* Create and set up a function. */
	fun = ddf_fun_create(dev, fun_inner, "a");
	if (fun == NULL) {
		ddf_msg(LVL_ERROR, "Failed creating function 'a' for device "
		    "'%s'.", name);
		rc = ENOMEM;
		goto error;
	}

	pl011->fun = fun;
	ddf_fun_set_conn_handler(fun, pl011_char_conn);

	rc = ddf_fun_add_match_id(fun, "char/sttykbd", 10);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed adding match ID 'char/sttykbd' to "
		    "function 'a' for device '%s': %s.", name, str_error(rc));
		goto error;
	}

	/* Make the function visible. */
	rc = ddf_fun_bind(fun);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed binding function 'a' of device "
		    "'%s': %s.", name, str_error(rc));
		goto error;
	}

	ddf_msg(LVL_NOTE, "Controlling '%s' (%" PRIun ").", name,
	    ddf_dev_get_handle(dev));
	return EOK;

error:
	if (pl011_initialized)
		pl011_uninit(pl011);
	if (fun != NULL)
		ddf_fun_destroy(fun);
	return rc;
}

/** Online a PL011 function.
 *
 * @param fun The PL011 function to online.
 *
 * @return Error code.
 */
static int pl011_fun_online(ddf_fun_t *fun)
{
	ddf_msg(LVL_DEBUG, "pl011_fun_online(), device=%s, fun=%s.",
	    ddf_dev_get_name(ddf_fun_get_dev(fun)), ddf_fun_get_name(fun));
	return ddf_fun_online(fun);
}

/** Offline a PL011 function.
 *
 * @param fun The PL011 function to offline.
 *
 * @return Error code.
 */
static int pl011_fun_offline(ddf_fun_t *fun)
{
	ddf_msg(LVL_DEBUG, "pl011_fun_offline(), device=%s, fun=%s.",
	    ddf_dev_get_name(ddf_fun_get_dev(fun)), ddf_fun_get_name(fun));
	return ddf_fun_offline(fun);
}

int main(int argc, char *argv[])
{
	printf("%s: HelenOS PL011 serial device driver\n", NAME);
	int rc = ddf_log_init(NAME);
	if (rc != EOK) {
		printf("%s: Error connecting logging service.\n", NAME);
		return 1;
	}

	return ddf_driver_main(&pl011_driver);
}

/**
 * @}
 */
