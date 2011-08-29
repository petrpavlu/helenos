/*
 * Copyright (c) 2010 Lenka Trochtova
 * Copyright (c) 2011 Jiri Svoboda
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
 * @defgroup libdrv generic device driver support.
 * @brief HelenOS generic device driver support.
 * @{
 */

/** @file
 */

#include <assert.h>
#include <ipc/services.h>
#include <ipc/ns.h>
#include <async.h>
#include <stdio.h>
#include <errno.h>
#include <bool.h>
#include <fibril_synch.h>
#include <stdlib.h>
#include <str.h>
#include <str_error.h>
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <devman.h>

#include <ipc/driver.h>

#include "dev_iface.h"
#include "ddf/driver.h"
#include "ddf/interrupt.h"

/** Driver structure */
static driver_t *driver;

/** Devices */
LIST_INITIALIZE(devices);
FIBRIL_MUTEX_INITIALIZE(devices_mutex);

/** Functions */
LIST_INITIALIZE(functions);
FIBRIL_MUTEX_INITIALIZE(functions_mutex);

/** Interrupts */
static interrupt_context_list_t interrupt_contexts;

static irq_cmd_t default_cmds[] = {
	{
		.cmd = CMD_ACCEPT
	}
};

static irq_code_t default_pseudocode = {
	sizeof(default_cmds) / sizeof(irq_cmd_t),
	default_cmds
};

static ddf_dev_t *create_device(void);
static void delete_device(ddf_dev_t *);
static void dev_add_ref(ddf_dev_t *);
static void dev_del_ref(ddf_dev_t *);
static void fun_add_ref(ddf_fun_t *);
static void fun_del_ref(ddf_fun_t *);
static remote_handler_t *function_get_default_handler(ddf_fun_t *);
static void *function_get_ops(ddf_fun_t *, dev_inferface_idx_t);

static void driver_irq_handler(ipc_callid_t iid, ipc_call_t *icall)
{
	int id = (int)IPC_GET_IMETHOD(*icall);
	interrupt_context_t *ctx;
	
	ctx = find_interrupt_context_by_id(&interrupt_contexts, id);
	if (ctx != NULL && ctx->handler != NULL)
		(*ctx->handler)(ctx->dev, iid, icall);
}

interrupt_context_t *create_interrupt_context(void)
{
	interrupt_context_t *ctx;
	
	ctx = (interrupt_context_t *) malloc(sizeof(interrupt_context_t));
	if (ctx != NULL)
		memset(ctx, 0, sizeof(interrupt_context_t));
	
	return ctx;
}

void delete_interrupt_context(interrupt_context_t *ctx)
{
	if (ctx != NULL)
		free(ctx);
}

void init_interrupt_context_list(interrupt_context_list_t *list)
{
	memset(list, 0, sizeof(interrupt_context_list_t));
	fibril_mutex_initialize(&list->mutex);
	list_initialize(&list->contexts);
}

void
add_interrupt_context(interrupt_context_list_t *list, interrupt_context_t *ctx)
{
	fibril_mutex_lock(&list->mutex);
	ctx->id = list->curr_id++;
	list_append(&ctx->link, &list->contexts);
	fibril_mutex_unlock(&list->mutex);
}

void remove_interrupt_context(interrupt_context_list_t *list,
    interrupt_context_t *ctx)
{
	fibril_mutex_lock(&list->mutex);
	list_remove(&ctx->link);
	fibril_mutex_unlock(&list->mutex);
}

interrupt_context_t *
find_interrupt_context_by_id(interrupt_context_list_t *list, int id)
{
	interrupt_context_t *ctx;
	
	fibril_mutex_lock(&list->mutex);
	
	list_foreach(list->contexts, link) {
		ctx = list_get_instance(link, interrupt_context_t, link);
		if (ctx->id == id) {
			fibril_mutex_unlock(&list->mutex);
			return ctx;
		}
	}
	
	fibril_mutex_unlock(&list->mutex);
	return NULL;
}

interrupt_context_t *
find_interrupt_context(interrupt_context_list_t *list, ddf_dev_t *dev, int irq)
{
	interrupt_context_t *ctx;
	
	fibril_mutex_lock(&list->mutex);
	
	list_foreach(list->contexts, link) {
		ctx = list_get_instance(link, interrupt_context_t, link);
		if (ctx->irq == irq && ctx->dev == dev) {
			fibril_mutex_unlock(&list->mutex);
			return ctx;
		}
	}
	
	fibril_mutex_unlock(&list->mutex);
	return NULL;
}


int
register_interrupt_handler(ddf_dev_t *dev, int irq, interrupt_handler_t *handler,
    irq_code_t *pseudocode)
{
	interrupt_context_t *ctx = create_interrupt_context();
	
	ctx->dev = dev;
	ctx->irq = irq;
	ctx->handler = handler;
	
	add_interrupt_context(&interrupt_contexts, ctx);
	
	if (pseudocode == NULL)
		pseudocode = &default_pseudocode;
	
	int res = register_irq(irq, dev->handle, ctx->id, pseudocode);
	if (res != EOK) {
		remove_interrupt_context(&interrupt_contexts, ctx);
		delete_interrupt_context(ctx);
	}

	return res;
}

int unregister_interrupt_handler(ddf_dev_t *dev, int irq)
{
	interrupt_context_t *ctx = find_interrupt_context(&interrupt_contexts,
	    dev, irq);
	int res = unregister_irq(irq, dev->handle);
	
	if (ctx != NULL) {
		remove_interrupt_context(&interrupt_contexts, ctx);
		delete_interrupt_context(ctx);
	}
	
	return res;
}

static void add_to_functions_list(ddf_fun_t *fun)
{
	fibril_mutex_lock(&functions_mutex);
	list_append(&fun->link, &functions);
	fibril_mutex_unlock(&functions_mutex);
}

static void remove_from_functions_list(ddf_fun_t *fun)
{
	fibril_mutex_lock(&functions_mutex);
	list_remove(&fun->link);
	fibril_mutex_unlock(&functions_mutex);
}

static ddf_dev_t *driver_get_device(devman_handle_t handle)
{
	ddf_dev_t *dev = NULL;
	
	assert(fibril_mutex_is_locked(&devices_mutex));
	
	list_foreach(devices, link) {
		dev = list_get_instance(link, ddf_dev_t, link);
		if (dev->handle == handle)
			return dev;
	}
	
	return NULL;
}

static ddf_fun_t *driver_get_function(devman_handle_t handle)
{
	ddf_fun_t *fun = NULL;
	
	assert(fibril_mutex_is_locked(&functions_mutex));
	
	list_foreach(functions, link) {
		fun = list_get_instance(link, ddf_fun_t, link);
		if (fun->handle == handle)
			return fun;
	}
	
	return NULL;
}

static void driver_dev_add(ipc_callid_t iid, ipc_call_t *icall)
{
	char *dev_name = NULL;
	int res;
	
	devman_handle_t dev_handle = IPC_GET_ARG1(*icall);
    	devman_handle_t parent_fun_handle = IPC_GET_ARG2(*icall);
	
	ddf_dev_t *dev = create_device();

	/* Add one reference that will be dropped by driver_dev_remove() */
	dev_add_ref(dev);
	dev->handle = dev_handle;

	async_data_write_accept((void **) &dev_name, true, 0, 0, 0, 0);
	dev->name = dev_name;

	/*
	 * Currently not used, parent fun handle is stored in context
	 * of the connection to the parent device driver.
	 */
	(void) parent_fun_handle;
	
	res = driver->driver_ops->add_device(dev);
	
	if (res != EOK) {
		dev_del_ref(dev);
		async_answer_0(iid, res);
		return;
	}
	
	fibril_mutex_lock(&devices_mutex);
	list_append(&dev->link, &devices);
	fibril_mutex_unlock(&devices_mutex);
	
	async_answer_0(iid, res);
}

static void driver_dev_remove(ipc_callid_t iid, ipc_call_t *icall)
{
	devman_handle_t devh;
	ddf_dev_t *dev;
	int rc;
	
	printf("libdrv: driver_dev_offline()\n");
	devh = IPC_GET_ARG1(*icall);
	
	fibril_mutex_lock(&devices_mutex);
	dev = driver_get_device(devh);
	dev_add_ref(dev);
	fibril_mutex_unlock(&devices_mutex);
	
	if (dev == NULL) {
		async_answer_0(iid, ENOENT);
		return;
	}
	
	if (driver->driver_ops->dev_remove != NULL)
		rc = driver->driver_ops->dev_remove(dev);
	else
		rc = ENOTSUP;
	
	if (rc == EOK)
		dev_del_ref(dev);
	
	async_answer_0(iid, (sysarg_t) rc);
}

static void driver_fun_online(ipc_callid_t iid, ipc_call_t *icall)
{
	devman_handle_t funh;
	ddf_fun_t *fun;
	int rc;
	
	funh = IPC_GET_ARG1(*icall);
	
	/*
	 * Look the function up. Bump reference count so that
	 * the function continues to exist until we return
	 * from the driver.
	 */
	fibril_mutex_lock(&functions_mutex);
	
	fun = driver_get_function(funh);
	if (fun != NULL)
		fun_add_ref(fun);
	
	fibril_mutex_unlock(&functions_mutex);
	
	if (fun == NULL) {
		async_answer_0(iid, ENOENT);
		return;
	}
	
	/* Call driver entry point */
	if (driver->driver_ops->fun_online != NULL)
		rc = driver->driver_ops->fun_online(fun);
	else
		rc = ENOTSUP;
	
	fun_del_ref(fun);
	
	async_answer_0(iid, (sysarg_t) rc);
}

static void driver_fun_offline(ipc_callid_t iid, ipc_call_t *icall)
{
	devman_handle_t funh;
	ddf_fun_t *fun;
	int rc;
	
	funh = IPC_GET_ARG1(*icall);
	
	/*
	 * Look the function up. Bump reference count so that
	 * the function continues to exist until we return
	 * from the driver.
	 */
	fibril_mutex_lock(&functions_mutex);
	
	fun = driver_get_function(funh);
	if (fun != NULL)
		fun_add_ref(fun);
	
	fibril_mutex_unlock(&functions_mutex);
	
	if (fun == NULL) {
		async_answer_0(iid, ENOENT);
		return;
	}
	
	/* Call driver entry point */
	if (driver->driver_ops->fun_offline != NULL)
		rc = driver->driver_ops->fun_offline(fun);
	else
		rc = ENOTSUP;
	
	async_answer_0(iid, (sysarg_t) rc);
}

static void driver_connection_devman(ipc_callid_t iid, ipc_call_t *icall)
{
	/* Accept connection */
	async_answer_0(iid, EOK);
	
	while (true) {
		ipc_call_t call;
		ipc_callid_t callid = async_get_call(&call);
		
		if (!IPC_GET_IMETHOD(call))
			break;
		
		switch (IPC_GET_IMETHOD(call)) {
		case DRIVER_DEV_ADD:
			driver_dev_add(callid, &call);
			break;
		case DRIVER_DEV_REMOVE:
			driver_dev_remove(callid, &call);
			break;
		case DRIVER_FUN_ONLINE:
			driver_fun_online(callid, &call);
			break;
		case DRIVER_FUN_OFFLINE:
			driver_fun_offline(callid, &call);
			break;
		default:
			async_answer_0(callid, ENOTSUP);
		}
	}
}

/** Generic client connection handler both for applications and drivers.
 *
 * @param drv True for driver client, false for other clients
 *            (applications, services, etc.).
 *
 */
static void driver_connection_gen(ipc_callid_t iid, ipc_call_t *icall, bool drv)
{
	/*
	 * Answer the first IPC_M_CONNECT_ME_TO call and remember the handle of
	 * the device to which the client connected.
	 */
	devman_handle_t handle = IPC_GET_ARG2(*icall);

	fibril_mutex_lock(&functions_mutex);
	ddf_fun_t *fun = driver_get_function(handle);
	fibril_mutex_unlock(&functions_mutex);
	/* XXX Need a lock on fun */
	
	if (fun == NULL) {
		printf("%s: driver_connection_gen error - no function with handle"
		    " %" PRIun " was found.\n", driver->name, handle);
		async_answer_0(iid, ENOENT);
		return;
	}
	
	if (fun->conn_handler != NULL) {
		/* Driver has a custom connection handler. */
		(*fun->conn_handler)(iid, icall, (void *)fun);
		return;
	}
	
	/*
	 * TODO - if the client is not a driver, check whether it is allowed to
	 * use the device.
	 */
	
	int ret = EOK;
	/* Open device function */
	if (fun->ops != NULL && fun->ops->open != NULL)
		ret = (*fun->ops->open)(fun);
	
	async_answer_0(iid, ret);
	if (ret != EOK)
		return;
	
	while (true) {
		ipc_callid_t callid;
		ipc_call_t call;
		callid = async_get_call(&call);
		sysarg_t method = IPC_GET_IMETHOD(call);
		
		if (!method) {
			/* Close device function */
			if (fun->ops != NULL && fun->ops->close != NULL)
				(*fun->ops->close)(fun);
			async_answer_0(callid, EOK);
			return;
		}
		
		/* Convert ipc interface id to interface index */
		
		int iface_idx = DEV_IFACE_IDX(method);
		
		if (!is_valid_iface_idx(iface_idx)) {
			remote_handler_t *default_handler =
			    function_get_default_handler(fun);
			if (default_handler != NULL) {
				(*default_handler)(fun, callid, &call);
				continue;
			}
			
			/*
			 * Function has no such interface and
			 * default handler is not provided.
			 */
			printf("%s: driver_connection_gen error - "
			    "invalid interface id %d.",
			    driver->name, iface_idx);
			async_answer_0(callid, ENOTSUP);
			continue;
		}
		
		/* Calling one of the function's interfaces */
		
		/* Get the interface ops structure. */
		void *ops = function_get_ops(fun, iface_idx);
		if (ops == NULL) {
			printf("%s: driver_connection_gen error - ", driver->name);
			printf("Function with handle %" PRIun " has no interface "
			    "with id %d.\n", handle, iface_idx);
			async_answer_0(callid, ENOTSUP);
			continue;
		}
		
		/*
		 * Get the corresponding interface for remote request
		 * handling ("remote interface").
		 */
		remote_iface_t *rem_iface = get_remote_iface(iface_idx);
		assert(rem_iface != NULL);
		
		/* get the method of the remote interface */
		sysarg_t iface_method_idx = IPC_GET_ARG1(call);
		remote_iface_func_ptr_t iface_method_ptr =
		    get_remote_method(rem_iface, iface_method_idx);
		if (iface_method_ptr == NULL) {
			/* The interface has not such method */
			printf("%s: driver_connection_gen error - "
			    "invalid interface method.", driver->name);
			async_answer_0(callid, ENOTSUP);
			continue;
		}
		
		/*
		 * Call the remote interface's method, which will
		 * receive parameters from the remote client and it will
		 * pass it to the corresponding local interface method
		 * associated with the function by its driver.
		 */
		(*iface_method_ptr)(fun, ops, callid, &call);
	}
}

static void driver_connection_driver(ipc_callid_t iid, ipc_call_t *icall)
{
	driver_connection_gen(iid, icall, true);
}

static void driver_connection_client(ipc_callid_t iid, ipc_call_t *icall)
{
	driver_connection_gen(iid, icall, false);
}

/** Function for handling connections to device driver. */
static void driver_connection(ipc_callid_t iid, ipc_call_t *icall, void *arg)
{
	sysarg_t conn_type;

	if (iid == 0) {
		/* Callback connection from devman */
		/* XXX Use separate handler for this type of connection */
		conn_type = DRIVER_DEVMAN;
	} else {
		conn_type = IPC_GET_ARG1(*icall);
	}

	/* Select interface */
	switch (conn_type) {
	case DRIVER_DEVMAN:
		/* Handle request from device manager */
		driver_connection_devman(iid, icall);
		break;
	case DRIVER_DRIVER:
		/* Handle request from drivers of child devices */
		driver_connection_driver(iid, icall);
		break;
	case DRIVER_CLIENT:
		/* Handle request from client applications */
		driver_connection_client(iid, icall);
		break;
	default:
		/* No such interface */
		async_answer_0(iid, ENOENT);
	}
}

/** Create new device structure.
 *
 * @return		The device structure.
 */
static ddf_dev_t *create_device(void)
{
	ddf_dev_t *dev;

	dev = calloc(1, sizeof(ddf_dev_t));
	if (dev == NULL)
		return NULL;

	return dev;
}

/** Create new function structure.
 *
 * @return		The device structure.
 */
static ddf_fun_t *create_function(void)
{
	ddf_fun_t *fun;

	fun = calloc(1, sizeof(ddf_fun_t));
	if (fun == NULL)
		return NULL;

	init_match_ids(&fun->match_ids);
	link_initialize(&fun->link);

	return fun;
}

/** Delete device structure.
 *
 * @param dev		The device structure.
 */
static void delete_device(ddf_dev_t *dev)
{
	if (dev->driver_data != NULL)
		free(dev->driver_data);
	free(dev);
}

/** Delete function structure.
 *
 * @param dev		The device structure.
 */
static void delete_function(ddf_fun_t *fun)
{
	clean_match_ids(&fun->match_ids);
	if (fun->driver_data != NULL)
		free(fun->driver_data);
	if (fun->name != NULL)
		free(fun->name);
	free(fun);
}

/** Increase device reference count. */
static void dev_add_ref(ddf_dev_t *dev)
{
	atomic_inc(&dev->refcnt);
}

/** Decrease device reference count.
 *
 * Free the device structure if the reference count drops to zero.
 */
static void dev_del_ref(ddf_dev_t *dev)
{
	if (atomic_predec(&dev->refcnt) == 0)
		delete_device(dev);
}

/** Increase function reference count.
 *
 * This also increases reference count on the device. The device structure
 * will thus not be deallocated while there are some associated function
 * structures.
 */
static void fun_add_ref(ddf_fun_t *fun)
{
	dev_add_ref(fun->dev);
	atomic_inc(&fun->refcnt);
}

/** Decrease function reference count.
 *
 * Free the function structure if the reference count drops to zero.
 */
static void fun_del_ref(ddf_fun_t *fun)
{
	ddf_dev_t *dev = fun->dev;

	if (atomic_predec(&fun->refcnt) == 0)
		delete_function(fun);

	dev_del_ref(dev);
}

/** Allocate driver-specific device data. */
extern void *ddf_dev_data_alloc(ddf_dev_t *dev, size_t size)
{
	void *data;

	assert(dev->driver_data == NULL);

	data = calloc(1, size);
	if (data == NULL)
		return NULL;

	dev->driver_data = data;
	return data;
}

/** Create a DDF function node.
 *
 * Create a DDF function (in memory). Both child devices and external clients
 * communicate with a device via its functions.
 *
 * The created function node is fully formed, but only exists in the memory
 * of the client task. In order to be visible to the system, the function
 * must be bound using ddf_fun_bind().
 *
 * This function should only fail if there is not enough free memory.
 * Specifically, this function succeeds even if @a dev already has
 * a (bound) function with the same name.
 *
 * Type: A function of type fun_inner indicates that DDF should attempt
 * to attach child devices to the function. fun_exposed means that
 * the function should be exported to external clients (applications).
 *
 * @param dev		Device to which we are adding function
 * @param ftype		Type of function (fun_inner or fun_exposed)
 * @param name		Name of function
 *
 * @return		New function or @c NULL if memory is not available
 */
ddf_fun_t *ddf_fun_create(ddf_dev_t *dev, fun_type_t ftype, const char *name)
{
	ddf_fun_t *fun;

	fun = create_function();
	if (fun == NULL)
		return NULL;

	/* Add one reference that will be dropped by ddf_fun_destroy() */
	fun->dev = dev;
	fun_add_ref(fun);

	fun->bound = false;
	fun->ftype = ftype;

	fun->name = str_dup(name);
	if (fun->name == NULL) {
		delete_function(fun);
		return NULL;
	}

	return fun;
}

/** Allocate driver-specific function data. */
extern void *ddf_fun_data_alloc(ddf_fun_t *fun, size_t size)
{
	void *data;

	assert(fun->bound == false);
	assert(fun->driver_data == NULL);

	data = calloc(1, size);
	if (data == NULL)
		return NULL;

	fun->driver_data = data;
	return data;
}

/** Destroy DDF function node.
 *
 * Destroy a function previously created with ddf_fun_create(). The function
 * must not be bound.
 *
 * @param fun		Function to destroy
 */
void ddf_fun_destroy(ddf_fun_t *fun)
{
	assert(fun->bound == false);

	/*
	 * Drop the reference added by ddf_fun_create(). This will deallocate
	 * the function as soon as all other references are dropped (i.e.
	 * as soon control leaves all driver entry points called in context
	 * of this function.
	 */
	fun_del_ref(fun);
}

static void *function_get_ops(ddf_fun_t *fun, dev_inferface_idx_t idx)
{
	assert(is_valid_iface_idx(idx));
	if (fun->ops == NULL)
		return NULL;
	return fun->ops->interfaces[idx];
}

/** Bind a function node.
 *
 * Bind the specified function to the system. This effectively makes
 * the function visible to the system (uploads it to the server).
 *
 * This function can fail for several reasons. Specifically,
 * it will fail if the device already has a bound function of
 * the same name.
 *
 * @param fun		Function to bind
 * @return		EOK on success or negative error code
 */
int ddf_fun_bind(ddf_fun_t *fun)
{
	assert(fun->bound == false);
	assert(fun->name != NULL);
	
	int res;
	
	add_to_functions_list(fun);
	res = devman_add_function(fun->name, fun->ftype, &fun->match_ids,
	    fun->dev->handle, &fun->handle);
	if (res != EOK) {
		remove_from_functions_list(fun);
		return res;
	}
	
	fun->bound = true;
	return res;
}

/** Unbind a function node.
 *
 * Unbind the specified function from the system. This effectively makes
 * the function invisible to the system.
 *
 * @param fun		Function to unbind
 * @return		EOK on success or negative error code
 */
int ddf_fun_unbind(ddf_fun_t *fun)
{
	int res;
	
	assert(fun->bound == true);
	
	res = devman_remove_function(fun->handle);
	if (res != EOK)
		return res;

	remove_from_functions_list(fun);
	
	fun->bound = false;
	return EOK;
}

/** Online function.
 *
 * @param fun		Function to online
 * @return		EOK on success or negative error code
 */
int ddf_fun_online(ddf_fun_t *fun)
{
	int res;
	
	assert(fun->bound == true);
	
	res = devman_drv_fun_online(fun->handle);
	if (res != EOK)
		return res;
	
	return EOK;
}

/** Offline function.
 *
 * @param fun		Function to offline
 * @return		EOK on success or negative error code
 */
int ddf_fun_offline(ddf_fun_t *fun)
{
	int res;
	
	assert(fun->bound == true);
	
	res = devman_drv_fun_offline(fun->handle);
	if (res != EOK)
		return res;
	
	return EOK;
}

/** Add single match ID to inner function.
 *
 * Construct and add a single match ID to the specified function.
 * Cannot be called when the function node is bound.
 *
 * @param fun			Function
 * @param match_id_str		Match string
 * @param match_score		Match score
 * @return			EOK on success, ENOMEM if out of memory.
 */
int ddf_fun_add_match_id(ddf_fun_t *fun, const char *match_id_str,
    int match_score)
{
	match_id_t *match_id;
	
	assert(fun->bound == false);
	assert(fun->ftype == fun_inner);
	
	match_id = create_match_id();
	if (match_id == NULL)
		return ENOMEM;
	
	match_id->id = str_dup(match_id_str);
	match_id->score = 90;
	
	add_match_id(&fun->match_ids, match_id);
	return EOK;
}

/** Get default handler for client requests */
static remote_handler_t *function_get_default_handler(ddf_fun_t *fun)
{
	if (fun->ops == NULL)
		return NULL;
	return fun->ops->default_handler;
}

/** Add exposed function to category.
 *
 * Must only be called when the function is bound.
 */
int ddf_fun_add_to_category(ddf_fun_t *fun, const char *cat_name)
{
	assert(fun->bound == true);
	assert(fun->ftype == fun_exposed);
	
	return devman_add_device_to_category(fun->handle, cat_name);
}

int ddf_driver_main(driver_t *drv)
{
	int rc;

	/*
	 * Remember the driver structure - driver_ops will be called by generic
	 * handler for incoming connections.
	 */
	driver = drv;
	
	/* Initialize the list of interrupt contexts. */
	init_interrupt_context_list(&interrupt_contexts);
	
	/* Set generic interrupt handler. */
	async_set_interrupt_received(driver_irq_handler);
	
	/*
	 * Register driver with device manager using generic handler for
	 * incoming connections.
	 */
	rc = devman_driver_register(driver->name, driver_connection);
	if (rc != EOK) {
		printf("Error: Failed to register driver with device manager "
		    "(%s).\n", (rc == EEXISTS) ? "driver already started" :
		    str_error(rc));
		
		return 1;
	}
	
	/* Return success from the task since server has started. */
	rc = task_retval(0);
	if (rc != EOK)
		return 1;

	async_manager();
	
	/* Never reached. */
	return 0;
}

/**
 * @}
 */
