/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <wei.chen@arm.com>
 *			Eduard Vintila <eduard.vintila47@gmail.com>
 *
 * Copyright (c) 2018, Arm Ltd. All rights reserved.
 * Copyright (c) 2022, University of Bucharest. All rights reserved.
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
#include <riscv/sbi.h>
#include <uk/plat/time.h>
#include <kvm/intctrl.h>
#include <stdio.h>

extern __u64 _setup_pagetables(void *);
extern void _start_mmu(void);
extern void _init_traps(void);

struct kvmplat_config _libkvmplat_cfg = {0};

#define MAX_CMDLINE_SIZE 1024
static char cmdline[MAX_CMDLINE_SIZE];
static const char *appname = CONFIG_UK_NAME;

extern void _libkvmplat_newstack(uint64_t stack_start, void (*tramp)(void *),
				 void *arg);

static void _init_dtb(void *dtb_pointer)
{
	int ret, dtb_size;

	if ((ret = fdt_check_header(dtb_pointer)))
		UK_CRASH("Invalid DTB: %s\n", fdt_strerror(ret));

	uk_pr_info("Found device tree at: %p\n", dtb_pointer);

	/* Move the DTB at the end of the kernel image */
	dtb_size = fdt_totalsize(dtb_pointer);
	_libkvmplat_cfg.dtb = memmove((void *)__END, dtb_pointer, dtb_size);
}

static void _dtb_get_cmdline(char *cmdline, size_t maxlen)
{
	int fdtchosen, len;
	const char *fdtcmdline;

	fdtchosen = fdt_path_offset(_libkvmplat_cfg.dtb, "/chosen");
	if (fdtchosen < 0)
		goto enocmdl;
	fdtcmdline =
	    fdt_getprop(_libkvmplat_cfg.dtb, fdtchosen, "bootargs", &len);
	if (!fdtcmdline || (len <= 0))
		goto enocmdl;

	if (likely(maxlen >= (unsigned int)len))
		maxlen = len;
	else
		uk_pr_err("Command line too long, truncated\n");

	strncpy(cmdline, fdtcmdline, maxlen);
	/* ensure null termination */
	cmdline[maxlen - 1] = '\0';

	uk_pr_info("Command line: %s\n", cmdline);
	return;

enocmdl:
	uk_pr_info("No command line found\n");
}

static void _init_dtb_mem(void)
{
	int fdt_mem, prop_len = 0, prop_min_len;
	int naddr, nsize;
	const __u64 *regs;
	__u64 mem_base, mem_size, max_addr;

	/* search for assigned VM memory in DTB */
	if (fdt_num_mem_rsv(_libkvmplat_cfg.dtb) != 0)
		uk_pr_warn("Reserved memory is not supported\n");

	fdt_mem = fdt_node_offset_by_prop_value(
	    _libkvmplat_cfg.dtb, -1, "device_type", "memory", sizeof("memory"));
	if (fdt_mem < 0) {
		uk_pr_warn("No memory found in DTB\n");
		return;
	}

	naddr = fdt_address_cells(_libkvmplat_cfg.dtb, fdt_mem);
	if (naddr < 0 || naddr >= FDT_MAX_NCELLS)
		UK_CRASH("Could not find proper address cells!\n");

	nsize = fdt_size_cells(_libkvmplat_cfg.dtb, fdt_mem);
	if (nsize < 0 || nsize >= FDT_MAX_NCELLS)
		UK_CRASH("Could not find proper size cells!\n");

	/*
	 * QEMU will always provide us at least one bank of memory.
	 * unikraft will use the first bank for the time-being.
	 */
	regs = fdt_getprop(_libkvmplat_cfg.dtb, fdt_mem, "reg", &prop_len);

	/*
	 * The property must contain at least the start address
	 * and size, each of which is 8-bytes.
	 */
	prop_min_len = (int)sizeof(fdt32_t) * (naddr + nsize);
	if (regs == NULL || prop_len < prop_min_len)
		UK_CRASH("Bad 'reg' property: %p %d\n", regs, prop_len);

	/* If we have more than one memory bank, give a warning messasge */
	if (prop_len > prop_min_len)
		uk_pr_warn("Currently, we support only one memory bank!\n");

	mem_base = fdt64_to_cpu(regs[0]);
	mem_size = fdt64_to_cpu(regs[1]);
	if (mem_base > __TEXT)
		UK_CRASH("Fatal: Image outside of RAM\n");

	max_addr = mem_base + mem_size;
	uk_pr_info("Memory base: 0x%lx, size: 0x%lx, max_addr: 0x%lx\n",
		   mem_base, mem_size, max_addr);

	if (_libkvmplat_cfg.pagetable.end > max_addr)
		UK_CRASH("Not enough memory for storing the pagetables\n");

	_libkvmplat_cfg.bstack.end = ALIGN_DOWN(max_addr, __STACK_ALIGN_SIZE);
	_libkvmplat_cfg.bstack.len = ALIGN_UP(__STACK_SIZE, __STACK_ALIGN_SIZE);
	_libkvmplat_cfg.bstack.start =
	    _libkvmplat_cfg.bstack.end - _libkvmplat_cfg.bstack.len;

	_libkvmplat_cfg.heap.start = _libkvmplat_cfg.pagetable.end;
	_libkvmplat_cfg.heap.end = _libkvmplat_cfg.bstack.start;
	_libkvmplat_cfg.heap.len =
	    _libkvmplat_cfg.heap.end - _libkvmplat_cfg.heap.start;

	if (_libkvmplat_cfg.heap.start > _libkvmplat_cfg.heap.end)
		UK_CRASH("Not enough memory, giving up...\n");
}

static void _libkvmplat_entry2(void *arg __attribute__((unused)))
{
	ukplat_entry_argp(DECONST(char *, appname), (char *)cmdline,
			  strlen(cmdline));
}

void _libkvmplat_start(void *opaque __unused, void *dtb_pointer)
{
	_init_dtb(dtb_pointer);

	/* Setup the page tables at the end of the DTB and start the MMU */
	_libkvmplat_cfg.pagetable.start =
	    ALIGN_UP(__END + fdt_totalsize(_libkvmplat_cfg.dtb), __PAGE_SIZE);
	_libkvmplat_cfg.pagetable.len =
	    _setup_pagetables((void *)_libkvmplat_cfg.pagetable.start);
	_libkvmplat_cfg.pagetable.end =
	    _libkvmplat_cfg.pagetable.start + _libkvmplat_cfg.pagetable.len;
	_start_mmu();

	/* Setup the NS16550 serial UART */
	ns16550_console_init(_libkvmplat_cfg.dtb);

	uk_pr_info("Entering from KVM (riscv64)...\n");

	/* Get command line arguments from DTB */
	_dtb_get_cmdline(cmdline, sizeof(cmdline));

	_init_dtb_mem();

	_init_traps();

	intctrl_init();

	uk_pr_info("pagetable start: %p\n",
		   (void *)_libkvmplat_cfg.pagetable.start);
	uk_pr_info("     heap start: %p\n", (void *)_libkvmplat_cfg.heap.start);
	uk_pr_info("      stack top: %p\n",
		   (void *)_libkvmplat_cfg.bstack.start);

	_libkvmplat_newstack((uint64_t)_libkvmplat_cfg.bstack.end,
			     _libkvmplat_entry2, NULL);
}
