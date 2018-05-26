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

/** @addtogroup drvkbd
 * @{
 */
/** @file
 * @brief Serial TTY-like keyboard driver.
 */

#include <abi/ipc/methods.h>
#include <ddf/driver.h>
#include <ddf/log.h>
#include <errno.h>
#include <io/chardev.h>
#include <io/kbd_event.h>
#include <io/keycode.h>
#include <ipc/kbdev.h>
#include <macros.h>
#include <stdio.h>
#include <str_error.h>

#define NAME "sttykbd"

/** Scancode table entry. */
typedef struct {
	keycode_t code;
	keymod_t mod;
} sttykbd_map_item_t;

/** Scancode table for serial console keyboard.
 *
 * Note: The decoder that uses this table works correctly only if the US layout
 * is used. This is because the output codes represent keyboard keys and so can
 * be a subject to an additional translation by the keyboard layout. The input
 * is then correct only if the second (layout) translation cancels out the first
 * (this table) translation.
 *
 * It should be made possible in the future to provide already lowered key
 * representation to input readers to avoid this problem.
 */
static const sttykbd_map_item_t sttykbd_map[] = {
	[0x01] = {KC_A, KM_CTRL},
	[0x02] = {KC_B, KM_CTRL},
	[0x03] = {KC_C, KM_CTRL},
	[0x04] = {KC_D, KM_CTRL},
	[0x05] = {KC_E, KM_CTRL},
	[0x06] = {KC_F, KM_CTRL},
	[0x07] = {KC_G, KM_CTRL},
	[0x08] = {KC_BACKSPACE, 0},
	[0x09] = {KC_TAB, 0},
	[0x0a] = {KC_ENTER, 0},
	[0x0b] = {KC_K, KM_CTRL},
	[0x0c] = {KC_L, KM_CTRL},
	[0x0d] = {KC_ENTER, 0},
	[0x0e] = {KC_N, KM_CTRL},
	[0x0f] = {KC_O, KM_CTRL},
	[0x10] = {KC_P, KM_CTRL},
	[0x11] = {KC_Q, KM_CTRL},
	[0x12] = {KC_R, KM_CTRL},
	[0x13] = {KC_S, KM_CTRL},
	[0x14] = {KC_T, KM_CTRL},
	[0x15] = {KC_U, KM_CTRL},
	[0x16] = {KC_V, KM_CTRL},
	[0x17] = {KC_W, KM_CTRL},
	[0x18] = {KC_X, KM_CTRL},
	[0x19] = {KC_Y, KM_CTRL},
	[0x1a] = {KC_Z, KM_CTRL},

	[0x20] = {KC_SPACE, 0},
	[0x21] = {KC_1, KM_SHIFT},
	[0x22] = {KC_QUOTE, KM_SHIFT},
	[0x23] = {KC_3, KM_SHIFT},
	[0x24] = {KC_4, KM_SHIFT},
	[0x25] = {KC_5, KM_SHIFT},
	[0x26] = {KC_7, KM_SHIFT},
	[0x27] = {KC_QUOTE, 0},
	[0x28] = {KC_9, KM_SHIFT},
	[0x29] = {KC_0, KM_SHIFT},
	[0x2a] = {KC_8, KM_SHIFT},
	[0x2b] = {KC_EQUALS, KM_SHIFT},
	[0x2c] = {KC_COMMA, 0},
	[0x2d] = {KC_MINUS, 0},
	[0x2e] = {KC_PERIOD, 0},
	[0x2f] = {KC_SLASH, 0},

	[0x30] = {KC_0, 0},
	[0x31] = {KC_1, 0},
	[0x32] = {KC_2, 0},
	[0x33] = {KC_3, 0},
	[0x34] = {KC_4, 0},
	[0x35] = {KC_5, 0},
	[0x36] = {KC_6, 0},
	[0x37] = {KC_7, 0},
	[0x38] = {KC_8, 0},
	[0x39] = {KC_9, 0},

	[0x3a] = {KC_SEMICOLON, KM_SHIFT},
	[0x3b] = {KC_SEMICOLON, 0},
	[0x3c] = {KC_COMMA, KM_SHIFT},
	[0x3d] = {KC_EQUALS, 0},
	[0x3e] = {KC_PERIOD, KM_SHIFT},
	[0x3f] = {KC_SLASH, KM_SHIFT},
	[0x40] = {KC_2, KM_SHIFT},

	[0x41] = {KC_A, KM_SHIFT},
	[0x42] = {KC_B, KM_SHIFT},
	[0x43] = {KC_C, KM_SHIFT},
	[0x44] = {KC_D, KM_SHIFT},
	[0x45] = {KC_E, KM_SHIFT},
	[0x46] = {KC_F, KM_SHIFT},
	[0x47] = {KC_G, KM_SHIFT},
	[0x48] = {KC_H, KM_SHIFT},
	[0x49] = {KC_I, KM_SHIFT},
	[0x4a] = {KC_J, KM_SHIFT},
	[0x4b] = {KC_K, KM_SHIFT},
	[0x4c] = {KC_L, KM_SHIFT},
	[0x4d] = {KC_M, KM_SHIFT},
	[0x4e] = {KC_N, KM_SHIFT},
	[0x4f] = {KC_O, KM_SHIFT},
	[0x50] = {KC_P, KM_SHIFT},
	[0x51] = {KC_Q, KM_SHIFT},
	[0x52] = {KC_R, KM_SHIFT},
	[0x53] = {KC_S, KM_SHIFT},
	[0x54] = {KC_T, KM_SHIFT},
	[0x55] = {KC_U, KM_SHIFT},
	[0x56] = {KC_V, KM_SHIFT},
	[0x57] = {KC_W, KM_SHIFT},
	[0x58] = {KC_X, KM_SHIFT},
	[0x59] = {KC_Y, KM_SHIFT},
	[0x5a] = {KC_Z, KM_SHIFT},

	[0x5b] = {KC_LBRACKET, 0},
	[0x5c] = {KC_BACKSLASH, 0},
	[0x5d] = {KC_RBRACKET, 0},
	[0x5e] = {KC_6, KM_SHIFT},
	[0x5f] = {KC_MINUS, KM_SHIFT},
	[0x60] = {KC_BACKTICK, 0},

	[0x61] = {KC_A, 0},
	[0x62] = {KC_B, 0},
	[0x63] = {KC_C, 0},
	[0x64] = {KC_D, 0},
	[0x65] = {KC_E, 0},
	[0x66] = {KC_F, 0},
	[0x67] = {KC_G, 0},
	[0x68] = {KC_H, 0},
	[0x69] = {KC_I, 0},
	[0x6a] = {KC_J, 0},
	[0x6b] = {KC_K, 0},
	[0x6c] = {KC_L, 0},
	[0x6d] = {KC_M, 0},
	[0x6e] = {KC_N, 0},
	[0x6f] = {KC_O, 0},
	[0x70] = {KC_P, 0},
	[0x71] = {KC_Q, 0},
	[0x72] = {KC_R, 0},
	[0x73] = {KC_S, 0},
	[0x74] = {KC_T, 0},
	[0x75] = {KC_U, 0},
	[0x76] = {KC_V, 0},
	[0x77] = {KC_W, 0},
	[0x78] = {KC_X, 0},
	[0x79] = {KC_Y, 0},
	[0x7a] = {KC_Z, 0},

	[0x7b] = {KC_LBRACKET, KM_SHIFT},
	[0x7c] = {KC_BACKSLASH, KM_SHIFT},
	[0x7d] = {KC_RBRACKET, KM_SHIFT},
	[0x7e] = {KC_BACKTICK, KM_SHIFT},
	[0x7f] = {KC_BACKSPACE, 0}
};

/** Correspondence table entry. */
typedef struct {
	keymod_t mod;
	keycode_t code;
} sttykbd_mods_item_t;

/** Correspondence between modifers and the modifier keycodes. */
static const sttykbd_mods_item_t sttykbd_mods[] = {
	{KM_LSHIFT, KC_LSHIFT},
	{KM_LCTRL, KC_LCTRL},
};

static int sttykbd_add(ddf_dev_t *);
static void default_connection_handler(ddf_fun_t *, ipc_callid_t, ipc_call_t *);

/** Serial line keyboard driver operations. */
static driver_ops_t sttykbd_driver_ops = {
	.dev_add = sttykbd_add,
};

/** Serial line keyboard driver structure. */
static driver_t sttykbd_driver = {
	.name = NAME,
	.driver_ops = &sttykbd_driver_ops
};

/** Keyboard function operations. */
static ddf_dev_ops_t sttykbd_ops = {
	.default_handler = default_connection_handler
};

/** Serial line keyboard driver-specific device data. */
typedef struct {
	ddf_fun_t *fun;             /**< Keyboard function. */
	async_sess_t *client_sess;  /**< Callback connection to client. */
	fid_t polling_fibril;       /**< Fibril retrieving the data. */
} sttykbd_t;

static void push_event(async_sess_t *sess, kbd_event_type_t type,
    unsigned int key)
{
	async_exch_t *exch = async_exchange_begin(sess);
	async_msg_4(exch, KBDEV_EVENT, type, key, 0, 0);
	async_exchange_end(exch);
}

/** Get data and parse scancodes.
 *
 * @param arg Pointer to xt_kbd_t structure.
 *
 * @return EIO on error.
 *
 */
static int polling(void *arg)
{
	assert(arg != NULL);

	ddf_dev_t *dev = arg;
	sttykbd_t *kbd = ddf_dev_data_get(dev);
	async_sess_t *parent_sess = ddf_dev_parent_sess_get(dev);
	const char *name = ddf_dev_get_name(dev);

	assert(kbd != NULL);
	assert(parent_sess != NULL);

	/* Start an exchange with the parent session. */
	async_exch_t *parent_exch = async_exchange_begin(parent_sess);
	if (parent_exch == NULL) {
		ddf_msg(LVL_ERROR, "Failed to start session exchange with the "
		    "parent session for device '%s'.", name);
		return ENOMEM;
	}

	while (true) {
		uint8_t code = 0;
		ssize_t size = chardev_read(parent_exch, &code, 1);
		if (size != 1) {
			ddf_msg(LVL_ERROR, "Failed to read a character from "
			    "the parent of device '%s'.", name);
			return EIO;
		}

		/* TODO ANSI escape code processing. */

		const sttykbd_map_item_t *key =
		    code < ARRAY_SIZE(sttykbd_map) ? &sttykbd_map[code] : NULL;

		if (key == NULL) {
			ddf_msg(LVL_WARN, "Unknown scancode '%#x' received by "
			    "'%s'.", code, name);
			continue;
		}

		/* Simulate the stroke. */
		size_t i;
		for (i = 0; i < ARRAY_SIZE(sttykbd_mods); i++)
			if (key->mod & sttykbd_mods[i].mod)
				push_event(kbd->client_sess, KEY_PRESS,
				    sttykbd_mods[i].code);

		push_event(kbd->client_sess, KEY_PRESS, key->code);
		push_event(kbd->client_sess, KEY_RELEASE, key->code);

		for (i = 0; i < ARRAY_SIZE(sttykbd_mods); i++)
			if (key->mod & sttykbd_mods[i].mod)
				push_event(kbd->client_sess, KEY_RELEASE,
				    sttykbd_mods[i].code);
	}
}

/** Default handler for IPC methods not handled by DDF.
 *
 * @param fun     Device function handling the call.
 * @param icallid Call id.
 * @param icall   Call data.
 */
static void default_connection_handler(ddf_fun_t *fun, ipc_callid_t icallid,
    ipc_call_t *icall)
{
	sysarg_t method = IPC_GET_IMETHOD(*icall);
	ddf_dev_t *dev = ddf_fun_get_dev(fun);
	const char *name = ddf_dev_get_name(dev);
	sttykbd_t *kbd = ddf_dev_data_get(dev);

	switch (method) {
	/*
	 * This might be ugly but async_callback_receive_start makes no
	 * difference for incorrect call and malloc failure.
	 */
	case IPC_M_CONNECT_TO_ME: {
		async_sess_t *sess =
		    async_callback_receive_start(EXCHANGE_SERIALIZE, icall);

		/* Probably ENOMEM error, try again. */
		if (sess == NULL) {
			ddf_msg(LVL_WARN, "Failed creating callback session "
			    "for device '%s'.", name);
			async_answer_0(icallid, EAGAIN);
			break;
		}

		if (kbd->client_sess == NULL) {
			kbd->client_sess = sess;
			ddf_msg(LVL_DEBUG, "Set client session for device "
			    "'%s'.", name);
			async_answer_0(icallid, EOK);
		} else {
			ddf_msg(LVL_ERROR, "Client session already set for "
			    "device '%s'.", name);
			async_answer_0(icallid, ELIMIT);
		}

		break;
	}
	default:
		ddf_msg(LVL_ERROR, "Unknown method '%" PRIun "' "
		    "for device '%s'.", method, name);
		async_answer_0(icallid, EINVAL);
		break;
	}
}

/** The dev_add callback method of the serial line keyboard driver.
 *
 * Initialize the newly added device.
 *
 * @param dev The serial line keyboard device.
 */
static int sttykbd_add(ddf_dev_t *dev)
{
	const char *name = ddf_dev_get_name(dev);
	async_sess_t *parent_sess;
	ddf_fun_t *fun = NULL;
	bool need_fun_unbind = false;
	int rc;

	/* Allocate soft state. */
	sttykbd_t *kbd = ddf_dev_data_alloc(dev, sizeof(sttykbd_t));
	if (kbd == NULL) {
		ddf_msg(LVL_ERROR, "Failed allocating soft state for device"
		    "'%s'.", name);
		return ENOMEM;
	}

	/* Connect to the parent. */
	parent_sess = ddf_dev_parent_sess_create(dev);
	if (parent_sess == NULL) {
		ddf_msg(LVL_ERROR, "Failed connecting parent driver of device "
		    "'%s'.", name);
		rc = ENOENT;
		goto error;
	}

	/* Create and set up a function. */
	fun = ddf_fun_create(dev, fun_exposed, "kbd");
	if (fun == NULL) {
		ddf_msg(LVL_ERROR, "Failed creating function 'kbd' for device "
		    "'%s'.", name);
		rc = ENOMEM;
		goto error;
	}

	kbd->fun = fun;
	ddf_fun_set_ops(fun, &sttykbd_ops);

	/* Make the function visible. */
	rc = ddf_fun_bind(fun);
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed binding function 'kbd' of device "
		    "'%s': %s.", name, str_error(rc));
		goto error;
	}
	need_fun_unbind = true;

	/* Add the function to the keyboard category. */
	rc = ddf_fun_add_to_category(fun, "keyboard");
	if (rc != EOK) {
		ddf_msg(LVL_ERROR, "Failed adding function 'kbd' of device "
		    "'%s' to category 'keyboard': %s.", name, str_error(rc));
		goto error;
	}

	/* Create a polling fibril */
	kbd->polling_fibril = fibril_create(polling, dev);
	if (kbd->polling_fibril == 0) {
		ddf_msg(LVL_ERROR, "Failed creating polling fibril for device "
		    "'%s'.", name);
		rc = ENOMEM;
		goto error;
	}

	fibril_add_ready(kbd->polling_fibril);

	ddf_msg(LVL_NOTE, "Controlling '%s' (%" PRIun ").", name,
	    ddf_dev_get_handle(dev));
	return EOK;

error:
	if (need_fun_unbind)
		ddf_fun_unbind(fun);
	if (fun != NULL)
		ddf_fun_destroy(fun);
	return rc;
}

int main(int argc, char *argv[])
{
	printf(NAME ": HelenOS STTY keyboard driver.\n");
	int rc = ddf_log_init(NAME);
	if (rc != EOK) {
		printf("%s: Error connecting logging service.\n", NAME);
		return 1;
	}
	return ddf_driver_main(&sttykbd_driver);
}

/**
 * @}
 */
