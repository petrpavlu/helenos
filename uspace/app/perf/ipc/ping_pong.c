/*
 * Copyright (c) 2009 Jiri Svoboda
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ipc_test.h>
#include <async.h>
#include <errno.h>
#include "../perf.h"

#define MIN_DURATION_SECS  10
#define NUM_SAMPLES 10

static errno_t ping_pong_measure(ipc_test_t *test, uint64_t niter,
    uint64_t *rduration)
{
	struct timespec start;
	uint64_t count;

	getuptime(&start);

	for (count = 0; count < niter; count++) {
		errno_t retval = ipc_test_ping(test);

		if (retval != EOK) {
			printf("Error sending ping message.\n");
			return EIO;
		}
	}

	struct timespec now;
	getuptime(&now);

	*rduration = ts_sub_diff(&now, &start) / 1000;
	return EOK;
}

static void ping_pong_report(uint64_t niter, uint64_t duration)
{
	printf("Completed %" PRIu64 " round trips in %" PRIu64 " us",
	    niter, duration);

	if (duration > 0) {
		printf(", %" PRIu64 " rt/s.\n", niter * 1000 * 1000 / duration);
	} else {
		printf(".\n");
	}
}

const char *bench_ping_pong(void)
{
	errno_t rc;
	uint64_t duration;
	uint64_t dsmp[NUM_SAMPLES];
	ipc_test_t *test;
	const char *msg;

	rc = ipc_test_create(&test);
	if (rc != EOK)
		return "Failed contacting IPC test server.";

	printf("Warm up and determine work size...\n");

	struct timespec start;
	getuptime(&start);

	uint64_t niter = 1;

	while (true) {
		rc = ping_pong_measure(test, niter, &duration);
		if (rc != EOK) {
			msg = "Failed.";
			goto error;
		}

		ping_pong_report(niter, duration);

		if (duration >= MIN_DURATION_SECS * 1000000)
			break;

		niter *= 2;
	}

	printf("Measure %d samples...\n", NUM_SAMPLES);

	int i;

	for (i = 0; i < NUM_SAMPLES; i++) {
		rc = ping_pong_measure(test, niter, &dsmp[i]);
		if (rc != EOK) {
			msg = "Failed.";
			goto error;
		}

		ping_pong_report(niter, dsmp[i]);
	}

	double sum = 0.0;

	for (i = 0; i < NUM_SAMPLES; i++)
		sum += (double)niter / ((double)dsmp[i] / 1000000.0l);

	double avg = sum / NUM_SAMPLES;

	double qd = 0.0;
	double d;
	for (i = 0; i < NUM_SAMPLES; i++) {
		d = (double)niter / ((double)dsmp[i] / 1000000.0l) - avg;
		qd += d * d;
	}

	double stddev = qd / (NUM_SAMPLES - 1); // XXX sqrt

	printf("Average: %.0f rt/s Std.dev^2: %.0f rt/s Samples: %d\n",
	    avg, stddev, NUM_SAMPLES);

	ipc_test_destroy(test);
	return NULL;
error:
	ipc_test_destroy(test);
	return msg;
}
