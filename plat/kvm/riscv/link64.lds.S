/* #include <uk/arch/limits.h> */
#include <uk/plat/common/common.lds.h>

/* QEMU RISCV64 virt platform's ram base address */
#define RAM_BASE_ADDR 0x80200000

/* Reserve first 1MB for DTB */
#define DTB_RESERVED_SIZE 0x100000

OUTPUT_ARCH(riscv)
ENTRY(_libkvmplat_entry)

SECTIONS {
	. = RAM_BASE_ADDR;

	/* Place DTB binary at the beginning of the RAM */
	_dtb = .;
	_start_ram_addr = .;
	. = . + DTB_RESERVED_SIZE;

	/* Code */
	_text = .;
	.text :
	{
		*(.text)
		*(.text.*)
	}

	. = ALIGN(__PAGE_SIZE);
	_etext = .;

	EXCEPTION_SECTIONS

	CTORTAB_SECTION

	INITTAB_SECTION

	/* Read-only data */
	. = ALIGN(__PAGE_SIZE);
	_rodata = .;
	.rodata :
	{
		*(.rodata)
		*(.rodata.*)
	}


	_erodata = .;

	/* Constructor tables (read-only) */
	. = ALIGN(0x8);
	_ctors = .;
	.preinit_array : {
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	}

	. = ALIGN(0x8);
	.init_array : {
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
		KEEP (*(.init_array .ctors))
		PROVIDE_HIDDEN (__init_array_end = .);
	}
	_ectors = .;
	. = ALIGN(__PAGE_SIZE);

	TLS_SECTIONS

	/* Read-write data that is initialized explicitly in code */
	_data = .;
	.data :
	{
		*(.data)
		*(.data.*)
	}
	_edata = .;
	. = ALIGN(__PAGE_SIZE);

	__bss_start = .;
	/* Read-write data that is not initialized explicitly in code */
	.bss :
	{
		*(.bss)
		*(.bss.*)
		*(COMMON)
		. = ALIGN(__PAGE_SIZE);
	/*
	 * TODO
	 * We can define the stack and page-table here, and free them once
	 * the boot is done.
	 */
	}

	_end = .;

	.comment       0 : { *(.comment) }
	DEBUG_SYMBOLS

	/DISCARD/ : { *(.note.gnu.build-id) }
}
