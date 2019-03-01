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

/** @addtogroup arm64boot
 * @{
 */
/** @file
 * @brief Functions implemented in assembly.
 */

#ifndef BOOT_arm64_ASM_H
#define BOOT_arm64_ASM_H

extern void *get_loader_start(void);
extern void *get_loader_end(void);
extern void *get_payload_start(void);
extern void *get_payload_end(void);

#define loader_start_arch() get_loader_start()
#define loader_end_arch() get_loader_end()
#define payload_start_arch() get_payload_start()
#define payload_end_arch() get_payload_end()

/** Jump to the kernel entry point.
 *
 * @param entry    Kernel entry point.
 * @param bootinfo Structure holding information about loaded tasks.
 */
extern void jump_to_kernel(void *entry, void *bootinfo)
    __attribute__((noreturn));

#endif

/** @}
 */
