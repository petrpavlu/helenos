/*
 * Copyright (c) 2015 Petr Pavlu
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

/** @addtogroup aarch64mm
 * @{
 */
/** @file
 * @brief Frame related functions.
 */

#include <arch/mm/frame.h>
#include <mm/frame.h>
#include <config.h>
#include <align.h>
#include <macros.h>

static void frame_common_arch_init(bool low)
{
	/* REVISIT */
	uintptr_t base;
	size_t size;

	base = ALIGN_UP(0, FRAME_SIZE);
	size = ALIGN_DOWN(0xf0000000, FRAME_SIZE);

	if (!frame_adjust_zone_bounds(low, &base, &size))
		return;

	if (low) {
		zone_create(ADDR2PFN(base), SIZE2FRAMES(size),
		    0,
		    ZONE_AVAILABLE | ZONE_LOWMEM);
	} else {
		pfn_t conf = zone_external_conf_alloc(SIZE2FRAMES(size));
		if (conf != 0)
			zone_create(ADDR2PFN(base), SIZE2FRAMES(size), conf,
			    ZONE_AVAILABLE | ZONE_HIGHMEM);
	}
}

/** Create low memory zones. */
void frame_low_arch_init(void)
{
	if (config.cpu_active > 1)
		return;

	frame_common_arch_init(true);
}

/** Create high memory zones. */
void frame_high_arch_init(void)
{
	if (config.cpu_active > 1)
		return;

	frame_common_arch_init(false);
}

/** @}
 */
