/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
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

#ifndef __HI6XXX_DEF_H__
#define __HI6XXX_DEF_H__

#include <platform_def.h> /* for TZROM_SIZE */


/* Firmware Image Package */
#define FIP_IMAGE_NAME			"fip.bin"

/*******************************************************************************

 * hi6xxx memory map related constants
 ******************************************************************************/

#define FLASH0_BASE		0x08000000
#define FLASH0_SIZE		TZROM_SIZE

#define FLASH1_BASE		0x0c000000
#define FLASH1_SIZE		0x04000000

#define PSRAM_BASE		0x14000000
#define PSRAM_SIZE		0x04000000

#define VRAM_BASE		0x18000000
#define VRAM_SIZE		0x02000000

/* Aggregate of all devices in the first GB */
#define DEVICE0_BASE		0x1a000000
#define DEVICE0_SIZE		0x12200000

#define DEVICE1_BASE		0x2f000000
#define DEVICE1_SIZE		0x200000

#define NSRAM_BASE		0x2e000000
#define NSRAM_SIZE		0x10000

#define MBOX_OFF		0x1000

/* Base address where parameters to BL31 are stored */
#define PARAMS_BASE		TZDRAM_BASE

#define DRAM1_BASE		0x00000000ull
#define DRAM1_SIZE		0x80000000ull
#define DRAM1_END		(DRAM1_BASE + DRAM1_SIZE - 1)
#define DRAM1_SEC_SIZE		0x01000000ull

#define HI6XXX_DRAM_BASE	DRAM1_BASE
#define HI6XXX_DRAM_SIZE	DRAM1_SIZE

#define DRAM2_BASE		0x880000000ull
#define DRAM2_SIZE		0x780000000ull
#define DRAM2_END		(DRAM2_BASE + DRAM2_SIZE - 1)

#define PCIE_EXP_BASE		0x40000000
#define TZRNG_BASE		    0x7fe60000
#define TZNVCTR_BASE		0x7fe70000
#define TZROOTKEY_BASE		0x7fe80000

/* Load address of BL33 in the FVP port */
//#define NS_IMAGE_OFFSET		(DRAM1_BASE + 0x8000000) /* DRAM + 128MB */

/*******************************************************************************
 * CCI-400 related constants
 ******************************************************************************/
#define CCI400_BASE			0xF6E90000
#define CCI400_SL_IFACE_CLUSTER0	3
#define CCI400_SL_IFACE_CLUSTER1	4
#define CCI400_SL_IFACE_INDEX(mpidr)	(mpidr & MPIDR_CLUSTER_MASK ? \
					 CCI400_SL_IFACE_CLUSTER1 :   \
					 CCI400_SL_IFACE_CLUSTER0)

/*******************************************************************************
 * GIC-400 & interrupt handling related constants
 ******************************************************************************/
/* Base HI6XXX compatible GIC memory map */
#define BASE_GICR_BASE          0xF6800000
#define BASE_GICD_BASE			0xF6801000
#define BASE_GICC_BASE			0xF6802000
#define BASE_GICH_BASE			0xF6804000
#define BASE_GICV_BASE			0xF6806000

#define IRQ_SEC_SGI_0			8
#define IRQ_SEC_SGI_1			9
#define IRQ_SEC_SGI_2			10
#define IRQ_SEC_SGI_3			11
#define IRQ_SEC_SGI_4			12
#define IRQ_SEC_SGI_5			13
#define IRQ_SEC_SGI_6			14
#define IRQ_SEC_SGI_7			15

/*******************************************************************************
 * PL011 related constants
 ******************************************************************************/
#define PL011_UART0_BASE		0xF8015000

#define HI6XXX_SYSCNT_FREQ      1200000
#endif /* __HI6XXX_DEF_H__ */
