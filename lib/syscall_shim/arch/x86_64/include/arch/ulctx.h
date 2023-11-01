/* SPDX-License-Identifier: BSD-3-Clause */
/* Copyright (c) 2023, Unikraft GmbH and The Unikraft Authors.
 * Licensed under the BSD-3-Clause License (the "License").
 * You may not use this file except in compliance with the License.
 */

#ifndef __UK_SYSCALL_H__
#error Do not include this header directly
#endif

#define UKARCH_ULCTX_SIZE			16

#if !__ASSEMBLY__

#include <uk/essentials.h>

/* Architecture specific userland context */
struct ukarch_ulctx {
	/* The current value of %gs's gs_base register of the application.
	 * On syscall entry, this will be updated to hold the value of
	 * MSR_KERNEL_GS_BASE following a swapgs instruction.
	 */
	__uptr gs_base;

	__uptr fs_base;
};

UK_CTASSERT(sizeof(struct ukarch_ulctx) == UKARCH_ULCTX_SIZE);

#if CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS
__uptr ukarch_ulctx_get_tlsp(struct ukarch_ulctx *ulctx);

void ukarch_ulctx_set_tlsp(struct ukarch_ulctx *ulctx, __uptr tlsp);

void ukarch_ulctx_switchoff_tls(struct ukarch_ulctx *ulctx);

void ukarch_ulctx_switchon_tls(struct ukarch_ulctx *ulctx);
#endif /* CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS */

void ukarch_ulctx_switchoff(struct ukarch_ulctx *ulctx);

void ukarch_ulctx_switchon(struct ukarch_ulctx *ulctx);

__uptr ukarch_ulctx_get_gs_base(struct ukarch_ulctx *ulctx);

void ukarch_ulctx_set_gs_base(struct ukarch_ulctx *ulctx, __uptr gs_base);

#endif /* !__ASSEMBLY__ */
