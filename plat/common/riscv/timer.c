/* SPDX-License-Identifier: ISC */
/*
 * Authors: Dan Williams
 *          Martin Lucina
 *          Ricardo Koller
 *          Costin Lupu <costin.lupu@cs.pub.ro>
 *          Eduard Vintila <eduard.vintila47@gmail.com>
 *
 * Copyright (c) 2015-2017 IBM
 * Copyright (c) 2016-2017 Docker, Inc.
 * Copyright (c) 2018, NEC Europe Ltd., NEC Corporation
 * TODO: Copyright 2022
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice appear
 * in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* Nanoseconds-ticks conversions taken from solo5 tscclock.c */
/*-
 * Copyright (c) 2014, 2015 Antti Kantee.  All Rights Reserved.
 * Copyright (c) 2015 Martin Lucina.  All Rights Reserved.
 * Modified for solo5 by Ricardo Koller <kollerr@us.ibm.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <riscv/time.h>
#include <riscv/cpu_defs.h>
#include <riscv/cpu.h>
#include <riscv/sbi.h>
#include <uk/arch/time.h>
#include <uk/plat/lcpu.h>
#include <uk/print.h>
#include <rtc/rtc.h>
#include <libfdt.h>

/* Frequency at which the time counter CSR is updated */
/* TODO: u32 or u64? */
static __u32 timebase_freq;

/* Multiplier for converting time counter ticks to nanoseconds. (0.32) fixed point. */
static __u64 time_mult;

/* Multiplier for converting nsecs to time counter ticks. (1.32) fixed point. */
static __u32 ticks_mult;

/* Time counter value at boot */
static __u64 boot_ticks;

/* Epoch offset for computing wall time */
static __nsec rtc_epochoffset;


/* Get the current time counter ticks */
static __u64 get_timer_ticks()
{
    return _csr_read(CSR_TIME);
}

/* Convert time counter ticks to nanoseconds */
static __nsec ticks_to_ns(__u64 ticks)
{
    return (__nsec) mul64_32(ticks, time_mult);
}

/* Convert nanoseconds to time counter ticks */
static __u64 ns_to_ticks(__nsec nsec)
{
    return (__u64) mul64_32(nsec, ticks_mult);
}

/* No. of nanoseconds since boot */
__nsec timer_monotonic_clock(void)
{
    return ticks_to_ns(get_timer_ticks() - boot_ticks);
}

__nsec timer_epoch_offset(void)
{
    return rtc_epochoffset;
}

/*
 * Retrieve the time counter frequency from the device tree.
 * Return 0 if it could not be found.
 */
static __u32 _dtb_get_timer_freq(void *dtb)
{
    const __u32 *freq_prop;
    int cpus_offset = fdt_path_offset(dtb, "/cpus");

    if (cpus_offset < 0)
        return 0;

    freq_prop = fdt_getprop(dtb, cpus_offset, "timebase-frequency", NULL);
    if (!freq_prop)
        return 0;

    return fdt32_to_cpu(freq_prop[0]);
}

void timer_cpu_block_until(__nsec until)
{
    __nsec now_ns;
    __u64 delta_ticks, now_ticks, until_ticks;

    UK_ASSERT(ukplat_lcpu_irqs_disabled());

    now_ticks = get_timer_ticks();
    now_ns = ticks_to_ns(now_ticks);
    if (now_ns < until) {
        delta_ticks = ns_to_ticks(until - now_ns);
        until_ticks = ticks_to_ns(now_ticks + delta_ticks);

        /* Set the time alarm through SBI */
        sbi_set_timer(until_ticks);

        /* Halt the hart until it receives an interrupt */
        ukplat_lcpu_halt_irq();
    }
}

/*
 * Initialize the RISC-V timer.
 * This function MUST be called after RTC initialization in order to compute the RTC epoch offset correctly.
 * Returns 0 on success, -1 on failure.
 */
int init_timer(void *dtb)
{
    __nsec rtc_boot;

    timebase_freq = _dtb_get_timer_freq(dtb);
    if (!timebase_freq)
        return -1;

    rtc_boot = rtc_gettimeofday();
    boot_ticks = get_timer_ticks();

    uk_pr_info("RTC timeofday boot: %lu\n", rtc_gettimeofday());
    uk_pr_info("Boot-time ticks: %lu\n", boot_ticks);
    uk_pr_info("Found time counter frequency: %u\n", timebase_freq);

    /*
	 * (0.32) time_mult = UKARCH_NSEC_PER_SEC (32.32) / tsc_freq (32.0)
     */
    time_mult = (UKARCH_NSEC_PER_SEC << 32) / timebase_freq;
    //uk_pr_info("test %lu, time_mult: %u\n", (UKARCH_NSEC_PER_SEC << 32) / timebase_freq, time_mult);

    /*
     *     f = UKARCH_NSEC_PER_SEC / timebase_freq     (0.31) fixed point.
     *     ticks_mult = 1 / f                          (1.32) fixed point.
     */
    ticks_mult = (1ULL << 63) / ((UKARCH_NSEC_PER_SEC << 31) / timebase_freq);


    rtc_epochoffset = rtc_boot - timer_monotonic_clock();
    return 0;
}
