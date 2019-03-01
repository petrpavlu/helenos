/*
 * Copyright (c) 2018 Jiří Zárevúcky
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

#ifndef BOOT_PAYLOAD_H_
#define BOOT_PAYLOAD_H_

#include <arch/asm.h>
#include <arch/types.h>
#include <stddef.h>
#include <stdint.h>

#ifndef loader_start_arch
extern uint8_t loader_start[];
#define loader_start_arch() loader_start
#endif

#ifndef loader_end_arch
extern uint8_t loader_end[];
#define loader_end_arch() loader_end
#endif

#ifndef payload_start_arch
extern uint8_t payload_start[];
#define payload_start_arch() payload_start
#endif

#ifndef payload_end_arch
extern uint8_t payload_end[];
#define payload_end_arch() payload_end
#endif

size_t payload_uncompressed_size(void);
void extract_payload(taskmap_t *, uint8_t *, uint8_t *, uintptr_t,
    void (*)(void *, size_t));

#endif
