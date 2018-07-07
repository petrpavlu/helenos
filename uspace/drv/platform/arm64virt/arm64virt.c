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

/**
 * @defgroup arm64virt ARM64 QEMU virt platform driver.
 * @brief HelenOS ARM64 QEMU virt platform driver.
 * @{
 */

/** @file
 */

#include <assert.h>
#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>

#include <ddf/driver.h>
#include <ddf/log.h>
#include <ops/hw_res.h>
#include <ops/pio_window.h>

#define NAME "arm64virt"

#define ARM64VIRT_UART_IRQ  33
#define ARM64VIRT_UART_MEMBASE  0x09000000
#define ARM64VIRT_UART_MEMSIZE  0x00001000

typedef struct arm64virt_fun {
	hw_resource_list_t hw_resources;
} arm64virt_fun_t;

static int arm64virt_dev_add(ddf_dev_t *);

static driver_ops_t arm64virt_ops = {
	.dev_add = &arm64virt_dev_add
};

static driver_t arm64virt_driver = {
	.name = NAME,
	.driver_ops = &arm64virt_ops
};

static hw_resource_t arm64virt_uart_res[] = {
	{
		.type = MEM_RANGE,
		.res.mem_range = {
			.address = ARM64VIRT_UART_MEMBASE,
			.size = ARM64VIRT_UART_MEMSIZE,
			.relative = false,
			.endianness = LITTLE_ENDIAN
		}
	},
	{
		.type = INTERRUPT,
		.res.interrupt = {
			.irq = ARM64VIRT_UART_IRQ
		}
	}
};

static pio_window_t arm64virt_pio_window = {
	.mem = {
		.base = 0,
		.size = -1
	}
};

static arm64virt_fun_t arm64virt_uart_fun_proto = {
	.hw_resources = {
		sizeof(arm64virt_uart_res) / sizeof(arm64virt_uart_res[0]),
		arm64virt_uart_res
	},
};

/** Obtain function soft-state from DDF function node. */
static arm64virt_fun_t *arm64virt_fun(ddf_fun_t *fnode)
{
	return ddf_fun_data_get(fnode);
}

static hw_resource_list_t *arm64virt_get_resources(ddf_fun_t *fnode)
{
	arm64virt_fun_t *fun = arm64virt_fun(fnode);

	assert(fun != NULL);
	return &fun->hw_resources;
}

static int arm64virt_enable_interrupt(ddf_fun_t *fun, int irq)
{
	/* TODO */
	return false;
}

static pio_window_t *arm64virt_get_pio_window(ddf_fun_t *fnode)
{
	return &arm64virt_pio_window;
}

static hw_res_ops_t arm64virt_hw_res_ops = {
	.get_resource_list = &arm64virt_get_resources,
	.enable_interrupt = &arm64virt_enable_interrupt,
};

static pio_window_ops_t arm64virt_pio_window_ops = {
	.get_pio_window = &arm64virt_get_pio_window
};

static ddf_dev_ops_t arm64virt_fun_ops = {
	.interfaces = {
		[HW_RES_DEV_IFACE] = &arm64virt_hw_res_ops,
		[PIO_WINDOW_DEV_IFACE] = &arm64virt_pio_window_ops
	}
};

static int arm64virt_add_fun(ddf_dev_t *dev, const char *name,
    const char *str_match_id, arm64virt_fun_t *fun_proto)
{
	ddf_msg(LVL_NOTE, "Adding function '%s'.", name);

	ddf_fun_t *fnode = NULL;
	int rc;

	/* Create new device. */
	fnode = ddf_fun_create(dev, fun_inner, name);
	if (fnode == NULL) {
		ddf_msg(LVL_ERROR, "Error creating function '%s'.", name);
		rc = ENOMEM;
		goto error;
	}

	arm64virt_fun_t *fun = ddf_fun_data_alloc(fnode,
	    sizeof(arm64virt_fun_t));
	*fun = *fun_proto;

	/* Add match ID. */
	rc = ddf_fun_add_match_id(fnode, str_match_id, 100);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Error adding match ID to function '%s'.",
		    name);
		goto error;
	}

	/* Set provided operations to the device. */
	ddf_fun_set_ops(fnode, &arm64virt_fun_ops);

	/* Register function. */
	rc = ddf_fun_bind(fnode);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed binding function '%s'.", name);
		goto error;
	}

	return EOK;

error:
	if (fnode != NULL)
		ddf_fun_destroy(fnode);

	return rc;
}

static int arm64virt_add_functions(ddf_dev_t *dev)
{
	int rc;

	rc = arm64virt_add_fun(dev, "uart", "arm/pl011",
	    &arm64virt_uart_fun_proto);
	if (rc != EOK)
		return rc;

	return EOK;
}

/** Add device. */
static int arm64virt_dev_add(ddf_dev_t *dev)
{
	ddf_msg(LVL_NOTE, "arm64virt_dev_add(), device=%s.",
	    ddf_dev_get_name(dev));

	/* Register functions. */
	if (arm64virt_add_functions(dev))
		ddf_msg(LVL_ERROR, "Failed to add functions for ARM64 QEMU "
		    "virt platform.");

	return EOK;
}

int main(int argc, char *argv[])
{
	int rc;

	printf(NAME ": HelenOS ARM64 QEMU virt platform driver\n");

	rc = ddf_log_init(NAME);
	if (rc != EOK) {
		printf(NAME ": Failed connecting logging service.");
		return 1;
	}

	return ddf_driver_main(&arm64virt_driver);
}

/**
 * @}
 */
