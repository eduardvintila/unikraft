/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Eduard Vintila <eduard.vintila47@gmail.com>
 *
 * TODO: Copyright notice
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
#include <riscv/cpu.h>
#include <uk/print.h>

/* PLIC memory map */
#define PLIC_PRIORITIES_OFFSET   0
#define PLIC_PENDING_OFFSET      0x1000
#define PLIC_ENABLE_OFFSET       0x2000
#define PLIC_PRIORITY_THRESHOLDS_OFFSET 0x200000
#define PLIC_CLAIM_OFFSET       0x200004
#define PLIC_COMPLETE_OFFSET    0x200004

/* Interrupts are handled on a per-context basis.
 * Context 1 is represented by hart 0 (the boot hart) running in S-mode.
 * Context 0 is hart 0 running in M-mode (not useful to us, only the SBI firmware runs in M-mode)
 *
 * Hence, odd-numbered contexts 1,3,5,7 etc. correspond to harts 0,1,2,3 etc. running in S-mode.
 * For the moment, we use only the boot hart to handle interrupts.
 *
 * One must take into account the above context numbering scheme if they wish to directly
 * route PLIC interrupts to other harts aswell.
 */
#define PLIC_BOOT_CTX 1

#define PLIC_BOOT_CTX_ENABLE_OFFSET \
    (PLIC_ENABLE_OFFSET + 0x80 * PLIC_BOOT_CTX)

#define PLIC_BOOT_CTX_PRIORITY_THRESHOLDS_OFFSET \
    (PLIC_PRIORITY_THRESHOLDS_OFFSET + 0x1000 * PLIC_BOOT_CTX)

#define PLIC_BOOT_CTX_CLAIM_OFFSET \
    (PLIC_CLAIM_OFFSET + 0x1000 * PLIC_BOOT_CTX)

#define PLIC_BOOT_CTX_COMPLETE_OFFSET \
    (PLIC_COMPLETE_OFFSET + 0x1000 * PLIC_BOOT_CTX)

static __paddr_t plic_mmio_base;

#define PLIC_REG(r) ((__u32 *) (plic_mmio_base + (r)))
#define PLIC_REG_READ(r) (ioreg_read32(PLIC_REG(r)))
#define PLIC_REG_WRITE(r, x) (ioreg_write32(PLIC_REG(r), x))

/* Retrieve the base address of the memory mapped PLIC */
static __paddr_t _dtb_get_plic_base(void *dtb)
{
    int plic_node;
    int naddr;
	const void *regs;
    __paddr_t base_addr;

    if ((plic_node = fdt_node_offset_by_compatible(dtb, -1, "sifive,plic-1.0.0")) < 0 &&
	    (plic_node = fdt_node_offset_by_compatible(dtb, -1, "riscv,plic0")) < 0)
        return (__paddr_t) NULL;

    naddr = fdt_address_cells(dtb, plic_node);
    regs = fdt_getprop(dtb, plic_node, "reg", NULL);

    if (naddr == 1)
        base_addr = (__paddr_t) fdt32_to_cpu(*(__u32 *)regs);
    else if (naddr == 2)
        base_addr = (__paddr_t) fdt64_to_cpu(*(__u64 *)regs);
    else
        return (__paddr_t) NULL;

    uk_pr_info("Found RISC-V PLIC at %p\n", (void *) base_addr);
    return base_addr;
}

void plic_enable_irq(unsigned int irq)
{
    __u32 word_offset = irq / 32;
    __u8 bit_offset = irq % 32;
    __u32 mask = 1 << bit_offset;
    __u32 word = PLIC_REG_READ(PLIC_BOOT_CTX_ENABLE_OFFSET + word_offset);

    word |= mask;
    PLIC_REG_WRITE(PLIC_BOOT_CTX_ENABLE_OFFSET + word_offset, word);
}

void plic_disable_irq(unsigned int irq)
{
    __u32 word_offset = irq / 32;
    __u8 bit_offset = irq % 32;
    __u32 mask = ~(1 << bit_offset);
    __u32 word = PLIC_REG_READ(PLIC_BOOT_CTX_ENABLE_OFFSET + word_offset);

    word &= mask;
    PLIC_REG_WRITE(PLIC_BOOT_CTX_ENABLE_OFFSET + word_offset, word);
}

void plic_set_priority(unsigned int irq, __u32 priority)
{
    PLIC_REG_WRITE(PLIC_PRIORITIES_OFFSET + irq * 4, priority);
}

void plic_set_priority_threshold(__u32 threshold)
{
    PLIC_REG_WRITE(PLIC_BOOT_CTX_PRIORITY_THRESHOLDS_OFFSET, threshold);
}

unsigned int plic_claim(void)
{
    return PLIC_REG_READ(PLIC_BOOT_CTX_CLAIM_OFFSET);
}

void plic_complete(unsigned int irq)
{
    return PLIC_REG_WRITE(PLIC_BOOT_CTX_COMPLETE_OFFSET, irq);
}

void plic_handle_irq(void)
{
    // TODO
}

int init_plic(void *dtb)
{
    plic_mmio_base = _dtb_get_plic_base(dtb);
    if (!plic_mmio_base)
        return -1;

    return 0;
}
