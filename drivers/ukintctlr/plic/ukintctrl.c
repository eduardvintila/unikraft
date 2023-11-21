/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Eduard Vintilă <eduard.vintila47@gmail.com>
 *
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
#include <riscv/cpu_defs.h>
#include <riscv/cpu.h>
#include <uk/assert.h>
#include <uk/intctlr.h>
#include <uk/intctlr/plic.h>

struct uk_intctlr_desc intctlr;

int uk_intctlr_probe(void)
{
	int rc = -ENODEV;
	struct uk_intctlr_driver_ops *ops;

#if CONFIG_LIBUKINTCTLR_PLIC
	rc = plic_probe(&ops);
	if (rc < 0)
		UK_CRASH("Interrupt controller not found, crashing...\n");
	intctlr.name = "RISC-V PLIC";
	intctlr.ops = ops;
#endif
	_csr_set(CSR_SIE, SIP_SEIP); /* Enable external interrupts */

	return uk_intctlr_register(&intctlr);
}
