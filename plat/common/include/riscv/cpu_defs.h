/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#ifndef __PLAT_CMN_RISCV_CPU_DEFS_H__
#define __PLAT_CMN_RISCV_CPU_DEFS_H__

#include <uk/asm.h>

#define _UL(x)		(_AC(x, UL))

#define SATP64_MODE				_UL(0xF000000000000000)
#define SATP64_ASID             _UL(0x0FFFF00000000000)
#define SATP64_PPN				_UL(0x00000FFFFFFFFFFF)
#define SATP_MODE_OFF			_UL(0)
#define SATP_MODE_SV39			_UL(8)



#endif
