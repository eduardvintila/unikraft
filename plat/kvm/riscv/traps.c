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

#include <uk/config.h>
#include <uk/arch/limits.h>
#include <uk/arch/types.h>
#include <uk/essentials.h>
#include <uk/assert.h>
#include <uk/print.h>
#include <kvm/config.h>
#include <kvm/irq.h>
#include <riscv/cpu_defs.h>
#include <riscv/cpu.h>
#include <riscv/plic.h>

extern void __trap_handler(void);

void do_unknown_exception(struct __regs *regs, unsigned long scause)
{
    uk_pr_crit("Unknown exception, scause: %lu, pc: %lx, sp: %lx, fp: %lx\n",
        scause, regs->pc, regs->sp, regs->s[0]);

    /* TODO: dump regs and mem */
    UK_CRASH("Crashing...\n");
}

void do_page_fault(struct __regs *regs, unsigned long scause __unused)
{
    unsigned long stval = _csr_read(CSR_STVAL);
    uk_pr_crit("Page fault at address 0x%lx, stval: 0x%lx, sp: 0x%lx, "
     "fp: 0x%lx\n", regs->pc, stval, regs->sp, regs->s[0]);

    /* TODO: dump regs and mem */
    UK_CRASH("Crashing...\n");
}

void _trap_handler(struct __regs *regs)
{
    unsigned long scause = _csr_read(CSR_SCAUSE);
    if (scause & CAUSE_INTERRUPT){
        if (scause == (CAUSE_INTERRUPT | IRQ_S_EXT))
            plic_handle_irq();
        else if (scause == (CAUSE_INTERRUPT | IRQ_S_TIMER))
            /* Timer interrupts are not routed through the PLIC, so call _ukplat_irq_handle directly. */
            _ukplat_irq_handle(0);
        else
            /* Software interrupt */
            do_unknown_exception(regs, scause);
    } else {
        if (scause == CAUSE_LOAD_PAGE_FAULT || scause == CAUSE_STORE_PAGE_FAULT || scause == CAUSE_FETCH_PAGE_FAULT)
            do_page_fault(regs, scause);
        else
            do_unknown_exception(regs, scause);
    }
    /* uintptr_t sepc = _csr_read(CSR_SEPC);
    sepc += 4;
    _csr_write(CSR_SEPC, sepc); */
}

void _init_traps(void)
{
    uintptr_t handler = (uintptr_t) &__trap_handler;
    uk_pr_info("sscratch: 0x%lx\n", _csr_read(CSR_SSCRATCH));
    uk_pr_info("stvec: 0x%lx\n", _csr_read(CSR_STVEC));
    uk_pr_info("sip: 0x%lx\n", _csr_read(CSR_SIP));
    uk_pr_info("sie: 0x%lx\n", _csr_read(CSR_SIE));
    uk_pr_info("sstatus: 0x%lx\n", _csr_read(CSR_SSTATUS));
    _csr_write(CSR_STVEC, handler);
    uk_pr_info("stvec: 0x%lx\n", _csr_read(CSR_STVEC));
}
