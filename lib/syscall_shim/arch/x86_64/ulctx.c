/* SPDX-License-Identifier: BSD-3-Clause */
/* Copyright (c) 2023, Unikraft GmbH and The Unikraft Authors.
 * Licensed under the BSD-3-Clause License (the "License").
 * You may not use this file except in compliance with the License.
 */
#include <uk/arch/lcpu.h>
#include <uk/assert.h>
#include <uk/plat/common/cpu.h>
#include <uk/thread.h>
#include <x86/gsbase.h>
#include <uk/syscall.h>

void ukarch_ulctx_switchoff(struct ukarch_ulctx *ulctx)
{
	UK_ASSERT(ulctx);
	UK_ASSERT(lcpu_get_current());

	/* This can only be called from Unikraft ctx in bincompat mode.
	 * Therefore, X86_MSR_GS_BASE holds the current `struct lcpu` and
	 * X86_MSR_KERNEL_GS_BASE contains the app-saved gs_base.
	 */
	ulctx->gs_base = rdkgsbase();

#if CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS
	ukarch_ulctx_switchoff_tls(ulctx);
#endif /* CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS */
}

void ukarch_ulctx_switchon(struct ukarch_ulctx *ulctx)
{
	UK_ASSERT(ulctx);
	UK_ASSERT(lcpu_get_current());

#if CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS
	ukarch_ulctx_switchon_tls(ulctx);
#endif /* CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS */

	/* This can only be called from Unikraft ctx in bincompat mode.
	 * Therefore, X86_MSR_GS_BASE holds the current `struct lcpu` and
	 * X86_MSR_KERNEL_GS_BASE contains the app-saved gs_base.
	 */
	wrgsbase((__uptr)lcpu_get_current());
	wrkgsbase(ulctx->gs_base);
}

#if CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS
__uptr ukarch_ulctx_get_tlsp(struct ukarch_ulctx *ulctx)
{
	UK_ASSERT(ulctx);

	return ulctx->fs_base;
}

void ukarch_ulctx_set_tlsp(struct ukarch_ulctx *ulctx, __uptr tlsp)
{
	UK_ASSERT(ulctx);

	uk_pr_debug("System call updated userland TLS pointer register to %p (before: %p)\n",
		    (void *)ulctx->fs_base, (void *)tlsp);

	ulctx->fs_base = tlsp;
}

void ukarch_ulctx_switchoff_tls(struct ukarch_ulctx *ulctx)
{
	struct uk_thread *t = uk_thread_current();

	UK_ASSERT(ulctx);
	UK_ASSERT(t);

	ulctx->fs_base = ukplat_tlsp_get();
	ukplat_tlsp_set(t->uktlsp);
	t->tlsp = t->uktlsp;
}

void ukarch_ulctx_switchon_tls(struct ukarch_ulctx *ulctx)
{
	struct uk_thread *t = uk_thread_current();

	UK_ASSERT(ulctx);
	UK_ASSERT(t);

	ukplat_tlsp_set(ulctx->fs_base);
	t->tlsp = ulctx->fs_base;
}
#endif /* CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS */

__uptr ukarch_ulctx_get_gs_base(struct ukarch_ulctx *ulctx)
{
	UK_ASSERT(ulctx);

	return ulctx->gs_base;
}

void ukarch_ulctx_set_gs_base(struct ukarch_ulctx *ulctx, __uptr gs_base)
{
	UK_ASSERT(ulctx);

	uk_pr_debug("System call updated userland GS_BASE pointer register to %p (before: %p)\n",
		    (void *)ulctx->gs_base, (void *)gs_base);

	ulctx->gs_base = gs_base;
}
