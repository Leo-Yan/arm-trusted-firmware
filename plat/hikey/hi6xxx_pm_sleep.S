#define __SOC_H_FOR_ASM__
#include <pwrctrl_multi_memcfg.h>
//#include "soc_baseaddr_interface.h"

	.global pm_asm_code_begin_flag
	.global pm_asm_code_end_flag
	.global jump_to_kernel
	.global wake_entry

pm_asm_code_begin_flag:

wake_entry:
	nop
	mov x0,0
	msr oslar_el1, x0

	mrs x0, s3_1_c15_c2_0
	bic x0, x0, #0x1E000000
 	orr x0, x0, #0x180000
	orr x0, x0, #0xe000
	msr s3_1_c15_c2_0, x0

	mrs x3, actlr_el3
        orr x3, x3, #(0x1<<5)
        msr actlr_el3, x3

        mrs x3, actlr_el2
        orr x3, x3, #(0x1<<5)
        msr actlr_el2, x3

jump_to_kernel:
	LDR x3,=PWRCTRL_ACPU_ASM_D_ARM_PARA_AD
	mrs	x0, mpidr_el1
	and	x1, x0, #(0xFF)
	and	x0, x0, #(0xFF << 8)
	add	x0, x1, x0, LSR #6
	lsl	x0, x0, #3
	ldr	x4, [x3, x0]
	/*msr elr_el3, x0*/

	mov x0, #0x0
	mov x1, #0x0
	mov x2, #0x0
	mov x3, #0x0
      /*now jump to kernel*/
	br x4
 	/*eret*/

	.ltorg

/*asm end flg*/
pm_asm_code_end_flag:
