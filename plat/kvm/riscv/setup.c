/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Eduard Vintila <eduard.vintila47@gmail.com>
 *
 * TODO: Copyright notice
 *
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
#include <uk/plat/bootstrap.h>
#include <uk/print.h>
#include <kvm/config.h>
#include <libfdt.h>
#include <uart/ns16550.h>
#include <string.h>

extern void _setup_pagetables(void*);
extern void _start_mmu(void);

struct kvmplat_config _libkvmplat_cfg = { 0 };

/* Placeholder until we implement hart stop/reset */
void ukplat_terminate(enum ukplat_gstate request __unused)
{

}

static void _init_dtb(void *dtb_pointer)
{
	int ret, dtb_size;

	if ((ret = fdt_check_header(dtb_pointer)))
		UK_CRASH("Invalid DTB: %s\n", fdt_strerror(ret));

	uk_pr_info("Found device tree at: %p\n", dtb_pointer);

	/* Move the DTB at the end of the kernel image */
	dtb_size = fdt_totalsize(dtb_pointer);
	_libkvmplat_cfg.dtb = memmove((void *) __END, dtb_pointer, dtb_size);
}


void _libkvmplat_start(void *opaque __unused, void *dtb_pointer)
{
    void *pagetables_start_addr;

	_init_dtb(dtb_pointer);

    /* Setup the page tables at the end of the DTB and start the MMU */
	/* TODO: Using the DTB, find how much memory is available before creating the page tables */
    pagetables_start_addr = (void *) ALIGN_UP(__END + fdt_totalsize(_libkvmplat_cfg.dtb), __PAGE_SIZE);
    _setup_pagetables(pagetables_start_addr);
	_start_mmu();

	/* Setup the NS16550 serial UART */
	ns16550_console_init(_libkvmplat_cfg.dtb);

	uk_pr_debug("Hello?\n");
}
