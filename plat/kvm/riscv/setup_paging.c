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

#include <riscv/cpu_defs.h>
#include <uk/arch/limits.h>
#include <uk/arch/types.h>
#include <uk/plat/common/sections.h>
#include <string.h>

#define PAGETABLE_SIZE __PAGE_SIZE
#define PAGETABLE_ENTRIES _UL(1 << 9)

#define __MEGAPAGE_SIZE _UL(__PAGE_SIZE << 9)

#define PAGE_VALID      _UL(1 << 0)
#define PAGE_INVALID        0
#define PAGE_GLOBAL     _UL(1 << 5)
#define PAGE_READ       _UL(1 << 1)
#define PAGE_WRITE      _UL(1 << 2)
#define PAGE_EXEC       _UL(1 << 3)
#define PAGE_RW         (PAGE_READ | PAGE_WRITE)
#define PAGE_RX         (PAGE_READ | PAGE_EXEC)
#define PAGE_LINK       _UL(0) /* Entry is a link to the next level of page table */
#define PAGE_VALID_LINK (PAGE_VALID | PAGE_LINK)

/* Used in setting x's physical page number (PPN) in a page table entry. *x* must be page-aligned */
#define PAGE_PPN(x)     (((__phys_addr)(x)) >> 2)

/* Obtain the physical address stored in the page table entry *x* */
#define PAGE_PPN_TO_ADDR(x) ((void *)(((__phys_addr)(x) >> 10) << __PAGE_SHIFT))

/* Get x's PPN */
#define PPN(x) ((__phys_addr)(x) >> __PAGE_SHIFT)

#define VPN0(x)         ((((__phys_addr)(x)) >> __PAGE_SHIFT) & _UL(0x1FF))
#define VPN1(x)         ((((__phys_addr)(x)) >> (__PAGE_SHIFT + 9)) & _UL(0x1FF))

#define SET_SATP64_MODE(mode) ((((__u64)(mode)) << 60) & SATP64_MODE)
#define SET_SATP64_ASID(asid) ((((__u64)(asid)) << 44) & SATP64_ASID)
#define SET_SATP64_PPN(ppn)   ((__u64)(ppn) & SATP64_PPN)

#define PLATFORM_MAX_MEM_ADDR _UL(0x8000000000)

/* Root page table */
__pte *_l2_pagetable;

/* Level 1 page table for mapping addresses 0x0080000000 - 0x7FFFFFFFFF */
__pte *_l1_pagetable;

/* Number of level 0 tables */
int _l0_pagetables_cnt;

extern void __start_mmu(__u64 satp);

/*
 * Layout of an Sv39 virtual address, where VPN[i] is an index in the level i page table.
 *
 * Bits 39-63 must be equal to bit 38.
 *
 * Adapted from the RISC-V Privileged Architecture Instruction Set Manual.
 *
 * 38             30 29           21 20           12 11                       0
 * ----------------------------------------------------------------------------
 * |     VPN[2]     |    VPN[1]     |    VPN[0]     |        page offset      |
 * ----------------------------------------------------------------------------
 *         9                9              9                     12
 */


/*
 * Memory layout of the QEMU virt and KVMTOOL RISC-V platforms
 *
 * | 0x0 - 0x7FFFFFFF   | 0x80000000 - 0x801FFFFF | 0x80200000 - 0x7FFFFFFFFF |
 * ----------------------------------------------------------------------------
 * | DEVICES MMIO|PCI-e |       SBI Firmware      |            <1>            |
 * ----------------------------------------------------------------------------
 *
 * <1> TEXT|DATA|BSS|DTB|PAGETABLES|HEAP|BOOTSTACK
 */

/**
 * Get the location of the L0 page table for the given L1 index.
 */
__pte *_get_l0_table(int l1_idx)
{
    __pte *l0_table;

    if ((_l1_pagetable[l1_idx] & PAGE_VALID_LINK) == PAGE_VALID_LINK) {
        /* Entry points to a L0 table */
        l0_table = PAGE_PPN_TO_ADDR(_l1_pagetable[l1_idx]);
    } else {
        /* Allocate a new L0 table */
        l0_table = _l1_pagetable + PAGETABLE_ENTRIES + _l0_pagetables_cnt * PAGETABLE_ENTRIES;
        _l0_pagetables_cnt++;

        /* Link the L1 entry to the newly created L0 table */
        _l1_pagetable[l1_idx] = PAGE_VALID_LINK | PAGE_GLOBAL | PAGE_PPN(l0_table);
    }

    return l0_table;
}

/**
 *  Identity map a region with *mode* permission bits.
 *  Addresses *start* and *end* MUST be page-aligned.
 */
void _map_region(__phys_addr start, __phys_addr end, __u16 mode)
{
    int l1_idx = VPN1(start), l0_idx = VPN0(start);
    __phys_addr addr = start;

    while (addr < end && l1_idx != 0) {
        if (l0_idx == 0 && (end - addr) >= __MEGAPAGE_SIZE) {
            /**
             * l0_idx = 0 means that *addr* is megapage-aligned and we can thus map
             * 2MiB pages using the L1 table.
             */
            _l1_pagetable[l1_idx] = PAGE_VALID | mode | PAGE_GLOBAL | PAGE_PPN(addr);
            addr += __MEGAPAGE_SIZE;
        } else {
            __pte *l0_pagetable = _get_l0_table(l1_idx);
            l0_pagetable[l0_idx] = PAGE_VALID | mode | PAGE_GLOBAL | PAGE_PPN(addr);
            addr += __PAGE_SIZE;
        }

        l0_idx = VPN0(addr);
        l1_idx = VPN1(addr);
    }

    if (l1_idx == 0)
        return; /* L1 table has been filled, region is too big. UK_CRASH? */
}

/**
 * Enable paging.
 */
void _start_mmu(void)
{
    __u64 satp = SET_SATP64_MODE(SATP_MODE_SV39) | SET_SATP64_ASID(0) |
                        SET_SATP64_PPN(PPN(_l2_pagetable));

    __start_mmu(satp);
}

/**
 * Setup a 3-level paging scheme for the Sv39 addressing mode, where *start*
 * points to the location where the page tables will be stored, starting
 * with the root page table L2.
 *
 * We identity map all addresses such that VA=PA. We also map at most 1GiB of space
 * from the start of the kernel image for the moment.
 */
void _setup_pagetables(void *start)
{
    /* memset(__BSS_START, 0, __END - __BSS_START); */

    _l2_pagetable = start;

    /* Place the l1 table at the end of the l2 table */
    _l1_pagetable = _l2_pagetable + PAGETABLE_ENTRIES;

    /* Map 0x0 - 0x7FFFFFFF as read/write */
    _l2_pagetable[0] = PAGE_VALID | PAGE_RW   | PAGE_GLOBAL | PAGE_PPN(_UL(0x0));
    _l2_pagetable[1] = PAGE_VALID | PAGE_RW   | PAGE_GLOBAL | PAGE_PPN(_UL(0x40000000));

    /* Map 0x80000000 - 0x7FFFFFFFFF using the next page table level */
    _l2_pagetable[2] = PAGE_VALID_LINK | PAGE_GLOBAL | PAGE_PPN(_l1_pagetable);

    /* Map the rest of the address space (0x8000000000 - 0xFFFFFFFFFF) as inaccessible */
    memset(_l2_pagetable + 3, PAGE_INVALID, (PAGETABLE_ENTRIES - 3) * sizeof(__pte));

    /* Map the SBI firmware space 0x80000000 - 0x801FFFFF as inaccessible */
    _l1_pagetable[0] = PAGE_INVALID;

    /* Preemptively mark all entries in the L1 table as invalid */
    memset(_l1_pagetable + 1, PAGE_INVALID, (PAGETABLE_ENTRIES - 1) * sizeof(__pte));

    /* Map the text section as read/executable only */
    _map_region(__TEXT, __ETEXT, PAGE_RX);

    /* Map the sections between text and rodata as read/write */
    _map_region(__ETEXT, __RODATA, PAGE_RW);

    /* Map the rodata section and the constructor tables as read-only */
    _map_region(__RODATA, __ECTORS, PAGE_READ);

    /* Map tls, data, bss and other sections as read/write */
    _map_region(__ECTORS, __END, PAGE_RW);

    /* Map the rest of the address space (heap and stack) as read/write */
    _map_region(__END, PLATFORM_MAX_MEM_ADDR, PAGE_RW);

    /* TODO: memset the remaining pte's in L0 to PAGE_INVALID (is it really necessary though?) */
}

