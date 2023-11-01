/* SPDX-License-Identifier: BSD-3-Clause */
/* Copyright (c) 2023, Unikraft GmbH and The Unikraft Authors.
 * Licensed under the BSD-3-Clause License (the "License").
 * You may not use this file except in compliance with the License.
 */

#include <arm/arm64/irq.h>
#include <string.h>
#include <uk/process.h>

void uk_syscall_regs_popall(void);

void clone_setup_child_ctx(struct uk_syscall_regs *pusr,
				  struct uk_thread *child, __uptr sp)
{
	__uptr auxsp_pos = child->auxsp;
	struct uk_syscall_regs *cusr;

	/* Create a child context whose stack pointer is that of the auxiliary
	 * stack, minus the parent's `struct uk_syscall_regs` saved on the
	 * auxiliary stack that we will have to first patch now and then pop off
	 */

	/* Make room for child's `struct uk_syscall_regs` and copy them */
	auxsp_pos = ALIGN_DOWN(auxsp_pos, UK_SYSCALL_REGS_END_ALIGN);
	auxsp_pos -= UK_SYSCALL_REGS_SIZE;
	memcpy((void *)auxsp_pos, (void *)pusr, UK_SYSCALL_REGS_SIZE);

	/* Now patch the child's return registers */
	cusr = (struct uk_syscall_regs *)auxsp_pos;

	/* Child must see x0 as 0 */
	cusr->regs.x[0] = 0x0;

	/* Make sure we have interrupts enabled, as this is supposedly a normal
	 * userspace thread - the other flags don't really matter since the
	 * first thing the child does is compare x0 to 0x0.
	 */
	cusr->regs.spsr_el1 &= ~PSR_I;

	/* Make sure we do return to what the child is expected to
	 * have as an instruction pointer as well as a stack pointer.
	 */
	cusr->regs.elr_el1 = pusr->regs.lr;
	cusr->regs.sp = sp;

	/* Use parent's user land TPIDR_EL0 if clone did not have SETTLS */
	if (!child->tlsp)
		cusr->ulctx.tpidr_el0 = pusr->ulctx.tpidr_el0;
	else
		cusr->ulctx.tpidr_el0 = child->tlsp;

	ukarch_ctx_init(&child->ctx,
			auxsp_pos,
			0,
			(__uptr)&uk_syscall_regs_popall);
}
