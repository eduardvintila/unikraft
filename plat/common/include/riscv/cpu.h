/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <wei.chen@arm.com>
 *			Eduard Vintila <eduard.vintila47@gmail.com>
 *
 * Copyright (c) 2018, Arm Ltd. All rights reserved.
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

#ifndef __PLAT_COMMON_RISCV64_CPU_H__
#define __PLAT_COMMON_RISCV64_CPU_H__

#include <uk/essentials.h>
#include <uk/arch/types.h>

/* TODO: Is inline assembly necessary? */

static inline __u8 ioreg_read8(const volatile __u8 *address)
{
	__u8 value;

	asm volatile ("lb %0, 0(%1)" : "=r"(value) : "r"(address));
	return value;
}

static inline __u16 ioreg_read16(const volatile __u16 *address)
{
	__u16 value;

	asm volatile ("lh %0, 0(%1)" : "=r"(value) : "r"(address));
	return value;
}

static inline __u32 ioreg_read32(const volatile __u32 *address)
{
	__u32 value;

	asm volatile ("lw %0, 0(%1)" : "=r"(value) : "r"(address));
	return value;
}

static inline __u64 ioreg_read64(const volatile __u64 *address)
{
	__u64 value;

	asm volatile ("ld %0, 0(%1)" : "=r"(value) : "r"(address));
	return value;
}

static inline void ioreg_write8(const volatile __u8 *address, __u8 value)
{
	asm volatile ("sb %0, 0(%1)" : : "rZ"(value), "r"(address));
}

static inline void ioreg_write16(const volatile __u16 *address,
				 __u16 value)
{
	asm volatile ("sh %0, 0(%1)" : : "rZ"(value), "r"(address));
}

static inline void ioreg_write32(const volatile __u32 *address,
				 __u32 value)
{
	asm volatile ("sw %0, 0(%1)" : : "rZ"(value), "r"(address));
}

static inline void ioreg_write64(const volatile __u64 *address,
				 __u64 value)
{
	asm volatile ("sd %0, 0(%1)" : : "rZ"(value), "r"(address));
}

#endif
