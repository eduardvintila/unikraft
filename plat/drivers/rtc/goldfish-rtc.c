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
#include <rtc/rtc.h>
#include <riscv/cpu.h>
#include <uk/print.h>

#ifdef CONFIG_ARCH_ARM_64
#include <arm/cpu.h>
#elif CONFIG_ARCH_RISCV_64
#include <riscv/cpu.h>
#endif

static __paddr_t rtc_mmio_base;

#define RTC_LOW 0x00
#define RTC_HIGH 0x04

/* Goldfish RTC registers are 32 bits wide */
#define RTC_REG(r) ((__u32 *)(rtc_mmio_base + r))
#define RTC_REG_READ(r) (ioreg_read32(RTC_REG(r)))


/* Retrieve the base address of the memory mapped Goldfish RTC device from the DTB */
static __paddr_t _dtb_get_rtc_base(void *dtb)
{
    int rtc_node;
    int naddr;
	const void *regs;
    __paddr_t base_addr;

    rtc_node = fdt_node_offset_by_compatible(dtb, -1, "google,goldfish-rtc");
    if (rtc_node < 0)
        return (__paddr_t) NULL;

    naddr = fdt_address_cells(dtb, rtc_node);
    regs = fdt_getprop(dtb, rtc_node, "reg", NULL);

    if (naddr == 1)
        base_addr = (__paddr_t) fdt32_to_cpu(*(__u32 *)regs);
    else if (naddr == 2)
        base_addr = (__paddr_t) fdt64_to_cpu(*(__u64 *)regs);
    else
        return (__paddr_t) NULL;

    uk_pr_info("Found goldfish-rtc device at %p\n", (void *) base_addr);
    return base_addr;
}

/* Returns POSIX time if the RTC device has been initialized, 0 otherwise. */
__u64 rtc_gettimeofday(void)
{
    __u64 time = 0;

    if (rtc_mmio_base) {
        __u64 time_low, time_high;
        time_low = RTC_REG_READ(RTC_LOW);
        time_high = RTC_REG_READ(RTC_HIGH);
        time = (time_high << 32) | time_low;
    }

    return time;
}

/* Initialize the Goldfish RTC device. Returns 0 on success, -1 otherwise. */
int init_rtc(void *dtb)
{
    rtc_mmio_base = _dtb_get_rtc_base(dtb);

    if (!rtc_mmio_base)
        return -1;

    return 0;
}
