/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Eduard Vintila <eduard.vintila47@gmail.com>
 *
 * TODO: Copyright notice
 *
 * EXTRACT_BYTE, CPU_TO_FDT32 and FDT_MAGIC macros are taken from libfdt:
 * Copyright (C) 2006 David Gibson, IBM Corporation.
 * Copyright 2012 Kim Phillips, Freescale Semiconductor.
 *
 * memmove is taken from nolibc:
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <uk/config.h>
#include <uk/arch/limits.h>
#include <uk/arch/types.h>
#include <uk/plat/common/sections.h>
#include <uk/essentials.h>

#define _EXTRACT_BYTE(x, n)	((__u32)((__u8 *)(x))[n])
#define _CPU_TO_FDT32(x) ((_EXTRACT_BYTE(x, 0) << 24) | (_EXTRACT_BYTE(x, 1) << 16) | \
			 (_EXTRACT_BYTE(x, 2) << 8) | _EXTRACT_BYTE(x, 3))
#define _FDT_MAGIC	0xd00dfeed

extern void _setup_pagetables(void*);
extern void _start_mmu(void);


/* Placeholder until we port nolibc */
void *__memmove(void *dst, const void *src, unsigned int len)
{
	__u8 *d = dst;
	const __u8 *s = src;

	if (src > dst) {
		for (; len > 0; --len)
			*(d++) = *(s++);
	} else {
		s += len - 1;
		d += len - 1;

		for (; len > 0; --len)
			*(d--) = *(s--);
	}

	return dst;
}

void _libkvmplat_start(void *opaque, void *dtb_pointer)
{
    __u32 dtb_magic, dtb_size;
    void *pagetables_start_addr;

    dtb_magic = _CPU_TO_FDT32(dtb_pointer);
    if (dtb_magic != _FDT_MAGIC)
        return; /* invalid DTB, UK_CRASH? */

    dtb_size = _CPU_TO_FDT32((__u32 *) dtb_pointer + 1);

    /* Move the DTB at the end of the kernel image */
    __memmove((void *) __END, dtb_pointer, dtb_size);

    /* Setup the page tables at the end of the DTB */
    pagetables_start_addr = (void *) ALIGN_UP(__END + dtb_size, __PAGE_SIZE);
    _setup_pagetables(pagetables_start_addr);

	_start_mmu();
}
