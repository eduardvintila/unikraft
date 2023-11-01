/* SPDX-License-Identifier: BSD-3-Clause */
/* Copyright (c) 2023, Unikraft GmbH and The Unikraft Authors.
 * Licensed under the BSD-3-Clause License (the "License").
 * You may not use this file except in compliance with the License.
 */

#ifndef __UK_SYSCALL_H__
#error Do not include this header directly
#endif

/* Taken from regmap_linuxabi.h, but defined differently so as not to
 * needlessly contaminate the namespace of source files including this header
 */

#define usr_arg0		x[0]
#define usr_arg1		x[1]
#define usr_arg2		x[2]
#define usr_arg3		x[3]
#define usr_arg4		x[4]
#define usr_arg5		x[5]
