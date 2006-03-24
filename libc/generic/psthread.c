/*
 * Copyright (C) 2006 Ondrej Palkovsky
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

#include <libadt/list.h>
#include <psthread.h>
#include <malloc.h>
#include <unistd.h>
#include <thread.h>
#include <stdio.h>
#include <kernel/arch/faddr.h>

static LIST_INITIALIZE(ready_list);

static void psthread_exit(void) __attribute__ ((noinline));
static void psthread_main(void);

/** Function to preempt to other pseudo thread without adding
 * currently running pseudo thread to ready_list.
 */
void psthread_exit(void)
{
	psthread_data_t *pt;

	if (list_empty(&ready_list)) {
		/* Wait on IPC queue etc... */
		printf("Cannot exit!!!\n");
		_exit(0);
	}
	pt = list_get_instance(ready_list.next, psthread_data_t, link);
	list_remove(&pt->link);
	context_restore(&pt->ctx);
}

/** Function that is called on entry to new uspace thread */
void psthread_main(void)
{
	psthread_data_t *pt = __tls_get();
	pt->retval = pt->func(pt->arg);

	pt->finished = 1;
	if (pt->waiter)
		list_append(&pt->waiter->link, &ready_list);

	psthread_exit();
}

/** Schedule next userspace pseudo thread.
 *
 * @return 0 if there is no ready pseudo thread, 1 otherwise.
 */
int psthread_schedule_next(void)
{
	psthread_data_t *pt;

	if (list_empty(&ready_list))
		return 0;

	pt = __tls_get();
	if (!context_save(&pt->ctx))
		return 1;
	
	list_append(&pt->link, &ready_list);
	pt = list_get_instance(ready_list.next, psthread_data_t, link);
	list_remove(&pt->link);

	context_restore(&pt->ctx);
}

/** Wait for uspace pseudo thread to finish.
 *
 * @param psthrid Pseudo thread to wait for.
 *
 * @return Value returned by the finished thread.
 */
int psthread_join(pstid_t psthrid)
{
	volatile psthread_data_t *pt, *mypt;
	volatile int retval;

	/* Handle psthrid = Kernel address -> it is wait for call */

	pt = (psthread_data_t *) psthrid;

	if (!pt->finished) {
		mypt = __tls_get();
		if (context_save(&((psthread_data_t *) mypt)->ctx)) {
			pt->waiter = (psthread_data_t *) mypt;
			psthread_exit();
		}
	}
	retval = pt->retval;

	free(pt->stack);
	__free_tls((psthread_data_t *) pt);

	return retval;
}

/**
 * Create a userspace thread and append it to ready list.
 *
 * @param func Pseudo thread function.
 * @param arg Argument to pass to func.
 *
 * @return 0 on failure, TLS of the new pseudo thread.
 */
pstid_t psthread_create(int (*func)(void *), void *arg)
{
	psthread_data_t *pt;

	pt = __make_tls();
	pt->stack = (char *) malloc(getpagesize());

	if (!pt->stack) {
		return 0;
	}

	pt->arg= arg;
	pt->func = func;
	pt->finished = 0;
	pt->waiter = NULL;

	context_save(&pt->ctx);
	context_set(&pt->ctx, FADDR(psthread_main), pt->stack, getpagesize(), pt);

	list_append(&pt->link, &ready_list);

	return (pstid_t )pt;
}
