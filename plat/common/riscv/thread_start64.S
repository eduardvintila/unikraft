/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <wei.chen@arm.com>
 *	    Eduard Vintila <eduard.vintila47@gmail.com>
 *
 * Copyright (c) 2018, Arm Ltd. All rights reserved.
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
//#include <uk/arch/ctx.h>
#include <uk/arch/lcpu.h>
#include <uk/asm.h>

/*
 * Thread stack memory layout:
 *
 * |-----------------------------| --> SP
 * |    arguments      		 | In init_sp, we have pushed thread main
 * |-----------------------------| function and arguments to stackfunction
 * |    thread main function     | and arguments to stack.
 * |-----------------------------| --> ctx->sp = SP - function - arguments
 * |    Reversed room for riscv64|
 * |    Callee saved registers:  |
 * |    s1 ~ s11                 |
 * |-----------------------------|
 * |    s0=fp (frame pointer)    |
 * |-----------------------------|
 * |    ...                      |
 * |    ...                      |
 * |-----------------------------|
 * |    thread                   | --> used for uk_thread_current
 * |-----------------------------|
 */

/*
 * This function will jump to thread main function and set up
 * thread exit handler.
 */
ENTRY(asm_thread_starter)
	ld a0, 0(sp) /* Load args to a0 */
	ld a1, 8(sp) /* Load func to a1 */
	addi sp, sp, 16 /* Pop args and func */
	jr a1 /* Jump to thread main func */
	call uk_sched_thread_exit
ENDPROC(asm_thread_starter)

/*
 * a0 = ctx->sp
 * a1 = ctx->ip = asm_thread_starter
 *
 * This function will set the SP to thread's SP and jump to
 * ctx->ip: asm_thread_starter
 */
ENTRY(asm_ctx_start)
	mv sp, a0	/* set SP */
	jr a1		/* jump to asm_thread_starter */
ENDPROC(asm_ctx_start)

/*
 * a0 = prectx
 * a1 = nextctx
 * This function will switch to the next thread context
 */
ENTRY(asm_sw_ctx_switch)

	/* Save callee-saved registers to prevctx's stack */
	addi sp, sp, -13 * REGBYTES
	sd ra, 12*REGBYTES(sp)
	sd s11, 11*REGBYTES(sp)
	sd s10, 10*REGBYTES(sp)
	sd s9, 9*REGBYTES(sp)
	sd s8, 8*REGBYTES(sp)
	sd s7, 7*REGBYTES(sp)
	sd s6, 6*REGBYTES(sp)
	sd s5, 5*REGBYTES(sp)
	sd s4, 4*REGBYTES(sp)
	sd s3, 3*REGBYTES(sp)
	sd s2, 2*REGBYTES(sp)
	sd s1, 1*REGBYTES(sp)
	sd s0, 0*REGBYTES(sp)

	/* Save sp and restore point to previous context */
	la t0, restore_point
	sd t0, OFFSETOF_SW_CTX_IP(a0)
	sd sp, OFFSETOF_SW_CTX_SP(a0)

	/* Restore sp and restore point from next context */
	ld ra, OFFSETOF_SW_CTX_IP(a1)
	ld sp, OFFSETOF_SW_CTX_SP(a1)

	ret
restore_point:
	/* Restore the callee-saved registers */
	ld ra, 12*REGBYTES(sp)
	ld s11, 11*REGBYTES(sp)
	ld s10, 10*REGBYTES(sp)
	ld s9, 9*REGBYTES(sp)
	ld s8, 8*REGBYTES(sp)
	ld s7, 7*REGBYTES(sp)
	ld s6, 6*REGBYTES(sp)
	ld s5, 5*REGBYTES(sp)
	ld s4, 4*REGBYTES(sp)
	ld s3, 3*REGBYTES(sp)
	ld s2, 2*REGBYTES(sp)
	ld s1, 1*REGBYTES(sp)
	ld s0, 0*REGBYTES(sp)
	addi sp, sp, 13 * REGBYTES

	ret
ENDPROC(asm_sw_ctx_switch)
