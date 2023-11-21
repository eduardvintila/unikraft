/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Eduard Vintila <eduard.vintila47@gmail.com>
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
#include <libfdt.h>
#include <uk/arch/types.h>
#include <uk/print.h>
#include <uk/assert.h>
#include <uk/plat/common/irq.h>
#include <uk/ofw/fdt.h>
#include <uk/intctlr.h>
#include <riscv/cpu.h>
#include <riscv/irq.h>

static struct uk_intctlr_driver_ops plic_ops = {
	.configure_irq = plic_configure_ireq
	.fdt_xlat = plic_fdt_xlat,
	.mask_irq = plic_mask_irq,
	.unmask_irq = plic_unmask_irq,
};

static __paddr_t plic_mmio_base;

/* PLIC memory map */
#define PLIC_PRIORITIES_OFFSET 0
#define PLIC_PENDING_OFFSET 0x1000
#define PLIC_ENABLE_OFFSET 0x2000
#define PLIC_PRIORITY_THRESHOLDS_OFFSET 0x200000
#define PLIC_CLAIM_OFFSET 0x200004
#define PLIC_COMPLETE_OFFSET 0x200004

/* Interrupts are handled on a per-context basis.
 * Context 1 is represented by hart 0 (the boot hart) running in S-mode.
 * Context 0 is hart 0 running in M-mode (not useful to us, only the SBI
 * firmware runs in M-mode)
 *
 * Hence, odd-numbered contexts 1,3,5,7 etc. correspond to harts 0,1,2,3 etc.
 * running in S-mode. For the moment, we use only the boot hart to handle
 * interrupts.
 *
 * One must take into account the above context numbering scheme if they wish to
 * directly route PLIC interrupts to other harts as well.
 */
#define PLIC_BOOT_CTX 1

#define PLIC_BOOT_CTX_ENABLE_OFFSET (PLIC_ENABLE_OFFSET + 0x80 * PLIC_BOOT_CTX)

#define PLIC_BOOT_CTX_PRIORITY_THRESHOLDS_OFFSET                               \
	(PLIC_PRIORITY_THRESHOLDS_OFFSET + 0x1000 * PLIC_BOOT_CTX)

#define PLIC_BOOT_CTX_CLAIM_OFFSET (PLIC_CLAIM_OFFSET + 0x1000 * PLIC_BOOT_CTX)

#define PLIC_BOOT_CTX_COMPLETE_OFFSET                                          \
	(PLIC_COMPLETE_OFFSET + 0x1000 * PLIC_BOOT_CTX)

#define PLIC_REG(r) ((__u32 *)(plic_mmio_base + (r)))
#define PLIC_REG_READ(r) (ioreg_read32(PLIC_REG(r)))
#define PLIC_REG_WRITE(r, x) (ioreg_write32(PLIC_REG(r), x))

/* Retrieve the base address of the memory mapped PLIC */
static int plic_probe(void)
{
	struct ukplat_bootinfo *bi = ukplat_bootinfo_get();
	int plic_node, rc;
	__u64 size;
	void *fdt;

	UK_ASSERT(bi);
	fdt = (void *)bi->dtb;

	if ((plic_node =
		 fdt_node_offset_by_compatible(fdt, -1, "sifive,plic-1.0.0"))
		< 0
	    && (plic_node =
		    fdt_node_offset_by_compatible(fdt, -1, "riscv,plic0"))
		   < 0)
		return -1;

	rc = fdt_get_address(fdt, plic_node, 0, &plic_mmio_base, &size);
	if (rc < 0)
		return -1;

	uk_pr_info("Found RISC-V PLIC at %p\n", (void *)plic_mmio_base);
	return 0;
}

static void plic_enable_irq(unsigned int irq)
{
	__u32 irq_word_offset = irq >> 3; /* (irq / 32) * sizeof(__u32) */
	__u8 irq_bit_offset = irq & 31; /* irq % 32 */
	__u32 mask = 1 << irq_bit_offset;
	__u32 word = PLIC_REG_READ(PLIC_BOOT_CTX_ENABLE_OFFSET + irq_word_offset);

	word |= mask;
	PLIC_REG_WRITE(PLIC_BOOT_CTX_ENABLE_OFFSET + irq_word_offset, word);
}

static void plic_disable_irq(unsigned int irq)
{
	__u32 irq_word_offset = irq >> 3; /* (irq / 32) * sizeof(__u32) */
	__u8 irq_bit_offset = irq & 31; /* irq % 32 */
	__u32 mask = ~(1 << irq_bit_offset);
	__u32 word = PLIC_REG_READ(PLIC_BOOT_CTX_ENABLE_OFFSET + irq_word_offset);

	word &= mask;
	PLIC_REG_WRITE(PLIC_BOOT_CTX_ENABLE_OFFSET + irq_word_offset, word);
}

static void plic_set_priority(unsigned int irq, __u32 priority)
{
	PLIC_REG_WRITE(PLIC_PRIORITIES_OFFSET + irq * sizeof(__u32), priority);
}

static void plic_set_priority_threshold(__u32 threshold)
{
	PLIC_REG_WRITE(PLIC_BOOT_CTX_PRIORITY_THRESHOLDS_OFFSET, threshold);
}

static unsigned int plic_claim(void)
{
	return PLIC_REG_READ(PLIC_BOOT_CTX_CLAIM_OFFSET);
}

static void plic_complete(unsigned int irq)
{
	PLIC_REG_WRITE(PLIC_BOOT_CTX_COMPLETE_OFFSET, irq);
}

void plic_handle_irq(struct __regs *regs)
{
	unsigned int irq = plic_claim();

	/* Run the handler as long as there is an interrupt pending */
	while (irq) {
		if (unlikely(irq >= __MAX_IRQ))
			UK_CRASH("Invalid IRQ, crashing...\n");

		uk_intctlr_irq_handle(regs, irq);
		plic_complete(irq);

		irq = plic_claim();
	}
}

static void plic_unmask_irq(unsigned int irq)
{
	/*
	 * The RISC-V PLIC spec specifies that global interrupt source 0 doesn't
	 * exist. We use IRQ 0 as an internal convention for timer interrupts.
	 * Those are manipulated through Control Status Registers, not the PLIC,
	 * hence timer interrupts are not treated as external interrupts.
	 */
	if (irq)
		plic_enable_irq(irq), plic_set_priority(irq, 1);
	else
		/*
		 * Sets the enable supervisor timer interrupt bit.
		 * A timer interrupt actually fires only when a timer event has
		 * been scheduled via SBI, which in turn uses machine mode
		 * specific CSRs (such as mtimecmp) to program a timer alarm.
		 */
		_csr_set(CSR_SIE, SIP_STIP);
}

static void plic_mask_irq(unsigned int irq)
{
	if (irq)
		plic_disable_irq(irq);
	else
		/* Disable timer interrupts */
		_csr_clear(CSR_SIE, SIP_STIP);
}

static int plic_configure_irq(struct uk_intctlr_irq *irq __unused)
{
	return 0;
}

static int plic_fdt_xlat(struct uk_intctlr_irq *irq __unused)
{
	// TODO:
	return 0;
}

int plic_init(struct uk_intctlr_driver_ops **ops)
{
	int rc = plic_probe();
	if (rc < 0)
		return -1;

	*ops = &plic_ops;
	plic_set_priority_threshold(0);

	return 0;
}
