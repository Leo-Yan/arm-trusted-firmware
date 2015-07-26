

/*****************************************************************************
  1 其他头文件包含
*****************************************************************************/
#ifndef __PWRCTRL_MULTI_MEMCFG_H__
#define __PWRCTRL_MULTI_MEMCFG_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif


/*****************************************************************************
  2 宏定义
*****************************************************************************/

/********************************************/
#ifndef __PWRCTRL_MULTI_MEMCFG_ACPU__
#define __PWRCTRL_MULTI_MEMCFG_ACPU__


#define PWRCTRL_ACPU_ASM_SPACE_ADDR         (0xFFF80000)
#define PWRCTRL_ACPU_ASM_SPACE_SIZE         (0x1C00)

#define PWRCTRL_ACPU_ASM_MEM_BASE           (PWRCTRL_ACPU_ASM_SPACE_ADDR)   /*sum size: 8k*/
#define PWRCTRL_ACPU_ASM_MEM_SIZE           (PWRCTRL_ACPU_ASM_SPACE_SIZE)                        /*8k*/
#define PWRCTRL_ACPU_ASM_CODE_BASE          (PWRCTRL_ACPU_ASM_MEM_BASE + 0x200)
#define PWRCTRL_ACPU_ASM_DATA_BASE          (PWRCTRL_ACPU_ASM_MEM_BASE + 0xE00)
#define PWRCTRL_ACPU_ASM_DATA_SIZE          (0xE00)

/********************************* phy addr ***********************************/
#define PWRCTRL_ACPU_ASM_D_C0_ADDR          (PWRCTRL_ACPU_ASM_DATA_BASE)
#define PWRCTRL_ACPU_ASM_D_C0_MMU_PARA_AD   (PWRCTRL_ACPU_ASM_DATA_BASE + 0)    /*size:0x20 - 8*4    :0x20*/
#define PWRCTRL_ACPU_ASM_D_ARM_PARA_AD      (PWRCTRL_ACPU_ASM_DATA_BASE + 0x20) /*size:0xe0 - 56*4   :0x100*/

#define PWRCTRL_ACPU_ASM_D_COMM_ADDR        (PWRCTRL_ACPU_ASM_DATA_BASE + 0x700)/*0x3800*/


#define PWRCTRL_ACPU_REBOOT                 (PWRCTRL_ACPU_ASM_D_COMM_ADDR)                                          /*0x3800 size 0x200*/
#define PWRCTRL_ACPU_REBOOT_SIZE            (0x200)                                                                 /*for exc*/
#define PWRCTRL_ACPU_ASM_SLICE_BAK_ADDR     (PWRCTRL_ACPU_REBOOT + PWRCTRL_ACPU_REBOOT_SIZE)                        /*0x3a00*/
#define PWRCTRL_ACPU_ASM_SLICE_BAK_SIZE     (4)
#define PWRCTRL_ACPU_ASM_DEBUG_FLAG_ADDR    (PWRCTRL_ACPU_ASM_SLICE_BAK_ADDR + PWRCTRL_ACPU_ASM_SLICE_BAK_SIZE)     /*0x3a04*/
#define PWRCTRL_ACPU_ASM_DEBUG_FLAG_SIZE    (4)
#define EXCH_A_CORE_POWRCTRL_CONV_ADDR      (PWRCTRL_ACPU_ASM_DEBUG_FLAG_ADDR + PWRCTRL_ACPU_ASM_DEBUG_FLAG_SIZE)   /*0x3a08*/
#define EXCH_A_CORE_POWRCTRL_CONV_SIZE      (4)


#define MEMORY_AXI_CPU_IDLE_ADDR            (EXCH_A_CORE_POWRCTRL_CONV_ADDR + EXCH_A_CORE_POWRCTRL_CONV_SIZE)
#define MEMORY_AXI_CPU_IDLE_SIZE            (4 + 12 + 16 + 28 + 28 + 16 + 28 + \
12 + 24 + 20 + 64 + 4 + 4 + 4 + 4 + 12 + 4 + 4 + 4 + 4 + 16 + 4 + 0x2BC + 24 + \
20 + 12 + 16)

#define ACPU_CORE_BITS_ADDR                     (MEMORY_AXI_CPU_IDLE_ADDR + MEMORY_AXI_CPU_IDLE_SIZE)
#define ACPU_CORE_BITS_SIZE                     (4)

#define ACPU_CLUSTER_IDLE_ADDR                   (ACPU_CORE_BITS_ADDR + ACPU_CORE_BITS_SIZE)
#define ACPU_CLUSTER_IDLE_SIZE                   (4)

#define ACPU_A53_FLAGS_ADDR                   (ACPU_CLUSTER_IDLE_ADDR + ACPU_CLUSTER_IDLE_SIZE)
#define ACPU_A53_FLAGS_SIZE                   (4)

#define ACPU_POWER_STATE_QOS_ADDR	(ACPU_A53_FLAGS_ADDR+ACPU_A53_FLAGS_SIZE)
#define ACPU_POWER_STATE_QOS_SIZE	(4)

#define ACPU_UNLOCK_CORE_FLAGS_ADDR	(ACPU_POWER_STATE_QOS_ADDR+ACPU_POWER_STATE_QOS_SIZE)
#define ACPU_UNLOCK_CORE_FLAGS_SIZE	(8)

#define ACPU_SUBSYS_POWERDOWN_FLAGS_ADDR                   (ACPU_UNLOCK_CORE_FLAGS_ADDR + ACPU_UNLOCK_CORE_FLAGS_SIZE)
#define ACPU_SUBSYS_POWERDOWN_FLAGS_SIZE                   (4)

#define ACPU_CORE_POWERDOWN_FLAGS_ADDR                   (ACPU_SUBSYS_POWERDOWN_FLAGS_ADDR + ACPU_SUBSYS_POWERDOWN_FLAGS_SIZE)
#define ACPU_CORE_POWERDOWN_FLAGS_SIZE                   (4)

#define ACPU_CLUSTER_POWERDOWN_FLAGS_ADDR                   (ACPU_CORE_POWERDOWN_FLAGS_ADDR + ACPU_CORE_POWERDOWN_FLAGS_SIZE)
#define ACPU_CLUSTER_POWERDOWN_FLAGS_SIZE                   (4)

#define ACPU_ARM64_FLAGA  						      (ACPU_CLUSTER_POWERDOWN_FLAGS_ADDR + ACPU_CLUSTER_POWERDOWN_FLAGS_SIZE)
#define ACPU_ARM64_FLAGA_SIZE                         (4)

#define ACPU_ARM64_FLAGB 						      (ACPU_ARM64_FLAGA + ACPU_ARM64_FLAGA_SIZE)
#define ACPU_ARM64_FLAGB_SIZE                         (4)

#define MCU_EXCEPTION_FLAGS_ADDR                   (ACPU_ARM64_FLAGB + ACPU_ARM64_FLAGB_SIZE)
#define MCU_EXCEPTION_FLAGS_SIZE                   (4)

#define ACPU_MASTER_CORE_STATE_ADDR                   (MCU_EXCEPTION_FLAGS_ADDR + MCU_EXCEPTION_FLAGS_SIZE)
#define ACPU_MASTER_CORE_STATE_SIZE                   (4)

#define PWRCTRL_AXI_RESERVED_ADDR               (ACPU_MASTER_CORE_STATE_ADDR + ACPU_MASTER_CORE_STATE_SIZE)

#if (PWRCTRL_AXI_RESERVED_ADDR >= (PWRCTRL_ACPU_ASM_SPACE_ADDR + PWRCTRL_ACPU_ASM_SPACE_SIZE))
#error acpu_low_power_memory(PWRCTRL_AXI_RESERVED_ADDR) used beyond (PWRCTRL_ACPU_ASM_SPACE_ADDR + PWRCTRL_ACPU_ASM_SPACE_SIZE)
#endif








#endif

















/*****************************************************************************
  3 枚举定义
*****************************************************************************/


/*****************************************************************************
  4 消息头定义
*****************************************************************************/


/*****************************************************************************
  5 消息定义
*****************************************************************************/


/*****************************************************************************
  6 STRUCT定义
*****************************************************************************/


/*****************************************************************************
  7 UNION定义
*****************************************************************************/


/*****************************************************************************
  8 OTHERS定义
*****************************************************************************/


/*****************************************************************************
  9 全局变量声明
*****************************************************************************/


/*****************************************************************************
  10 函数声明
*****************************************************************************/












#ifdef __cplusplus
    #if __cplusplus
        }
    #endif
#endif

#endif /* end of pwrctrl_multi_memcfg.h */
