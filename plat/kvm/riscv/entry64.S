#include <uk/arch/limits.h>
#include <uk/asm.h>
#include <uk/plat/common/sections.h>
#include <uk/config.h>

.text
ENTRY(_libkvmplat_entry)
	// wfi
	li s0, 0
1:
	addi s0, s0, 5
    j 1b
END(_libkvmplat_entry)

ENTRY(_libkvmplat_newstack)
	wfi
    j _libkvmplat_newstack
END(_libkvmplat_newstack)
