#ifndef __MDRV_IPC_ENUM_H__
#define __MDRV_IPC_ENUM_H__
#ifdef __cplusplus
extern "C"
{
#endif

typedef enum tagIPC_INT_CORE_E
{
    IPC_CORE_ACPU = 0x0,

    /* !!!!新增元素请添加到最后  */
    IPC_CORE_BUTTOM
}IPC_INT_CORE_E;

typedef enum tagIPC_INT_LEV_E
{
    IPC_MCU_INT_SRC_ACPU0_PD            = 10,   /* bit10, acpu0 power down */
    IPC_MCU_INT_SRC_ACPU1_PD            = 11,   /* bit11, acpu1 power down */
    IPC_MCU_INT_SRC_ACPU2_PD            = 12,   /* bit12, acpu2 power down */
    IPC_MCU_INT_SRC_ACPU3_PD            = 13,   /* bit13, acpu3 power down */
    IPC_MCU_INT_SRC_ACPU_PD             = 16,   /* bit16, acpu power down */
    IPC_MCU_INT_SRC_ACPU4_PD            = 26,   /* bit26, acpu4:cluster1 core0 power down */
    IPC_MCU_INT_SRC_ACPU5_PD            = 27,   /* bit27, acpu5:cluster1 core1 power down */
    IPC_MCU_INT_SRC_ACPU6_PD            = 28,   /* bit28, acpu6:cluster1 core2 power down */
    IPC_MCU_INT_SRC_ACPU7_PD            = 29,   /* bit29, acpu7:cluster1 core3 power down */

    IPC_MCU_INT_SRC_BUTT                = 32,

    /* 仅解决编译问题 */
    IPC_INT_BUTTOM                      = 32
}IPC_INT_LEV_E;

typedef enum tagIPC_SEM_ID_E
{
    IPC_SEM_CPUIDLE = 27,
    IPC_SEM_BUTTOM
}IPC_SEM_ID_E;


#ifdef __cplusplus
}
#endif

#endif
