#define pr_fmt(fmt) "hisi_pm_ipc: " fmt

#include "mdrv_ipc.h"
#include <mmio.h>
#include <pwrctrl_multi_memcfg.h>
#include <hisi_ipc.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//#include "pwrctrl_multi_memcfg.h"    /* config ddr autorefresh in sram_reboot() */

#define HISI_MAX_CPUS		4
#define HISI_MAX_CLUSTERS	2

#define SOC_IPC_S_BASE_ADDR                           (0xF7510000)
#define SOC_IPC_CPU_RAW_INT_ADDR(base, i)             ((base) + (0x400+(0x10*(i))))
#define writel(val, addr)		mmio_write_32((uintptr_t)addr, (uint32_t)val)
#define readl(addr)				mmio_read_32((uintptr_t)addr)

#define BIT(x) (0x1 << (x))


static unsigned int core_ipc_num[HISI_MAX_CLUSTERS][HISI_MAX_CPUS] = {
    {IPC_MCU_INT_SRC_ACPU0_PD,
     IPC_MCU_INT_SRC_ACPU1_PD,
     IPC_MCU_INT_SRC_ACPU2_PD,
     IPC_MCU_INT_SRC_ACPU3_PD,
     },
     {
     IPC_MCU_INT_SRC_ACPU4_PD,
     IPC_MCU_INT_SRC_ACPU5_PD,
     IPC_MCU_INT_SRC_ACPU6_PD,
     IPC_MCU_INT_SRC_ACPU7_PD,
     }
};

unsigned long g_ipc_base_addr;

unsigned int g_CoreNum = 0;

static unsigned long acpu_core_powerdown_flags = 0;
static unsigned long acpu_cluster_powerdown_flags = 0;
extern int hisi_cores_pd_in_cluster_besides_curr(unsigned int cluster, unsigned int core);
#define CLUSTER_CPUS_IDLE_MASK          (0x0F)

#define INTSRC_NUM					32

#define IPC_RegWr(uwAddr, uwValue) (*((volatile unsigned long *)(uwAddr)) = uwValue)
#define IPC_RegRd(uwAddr)          (*((volatile unsigned long *)(uwAddr)))

#define IPC_BASE_ADDR  0xF7510000
#define IPC_HS_CTRL_ADDR(base, j, k)  ((base) + (0x800+(0x100*(j))+(0x8*(k))))
#define IPC_HS_CTRL(j,k)        (IPC_HS_CTRL_ADDR((unsigned long)IPC_BASE_ADDR, j, k))

void IPC_SpinLock (unsigned int u32SignalNum)
{
	unsigned int u32HsCtrl;

	if(u32SignalNum >= INTSRC_NUM)
	{
		return;
	}
	for(;;)
	{
		u32HsCtrl = IPC_RegRd(IPC_HS_CTRL(g_CoreNum, u32SignalNum));
		if (0 == u32HsCtrl)
		{
			break;
		}
	}
}

void IPC_SpinUnLock (unsigned int u32SignalNum)
{
	if(u32SignalNum >= INTSRC_NUM)
	{
		return;
	}
	/*将信号量请求寄存器清0*/
	IPC_RegWr(IPC_HS_CTRL(g_CoreNum, u32SignalNum), 0);
}



static void hisi_ipc_send(unsigned int ipc_num)
{
    unsigned int reg = 0;

    if(0 == g_ipc_base_addr)
    {
        printf("error ipc base is null!!!\n");
        return;
    }

    printf("%s: %x %d\n", __func__, g_ipc_base_addr, ipc_num);

    reg = BIT(ipc_num);
    writel(reg, g_ipc_base_addr);
}

void hisi_ipc_core_suspend(unsigned int core, unsigned int cluster, unsigned int affinity_level)
{
    hisi_ipc_send(core_ipc_num[cluster][core]);
}

void hisi_ipc_sys_suspend(unsigned int core, unsigned int cluster, unsigned int affinity_level)
{
    hisi_ipc_send(IPC_MCU_INT_SRC_ACPU_PD);
}

unsigned int hisi_core_pd_flags_rd()
{
    return readl(acpu_core_powerdown_flags);
}

void hisi_ipc_pm_on_off(unsigned int core, unsigned int cluster, enum pm_mode mode)
{
    unsigned int val = 0;

    if(PM_ON == mode)
    {
        IPC_SpinLock(IPC_SEM_CPUIDLE);

        val = readl(acpu_core_powerdown_flags);
	    val |= (0x01 << (cluster*16 + core * 4));
        writel(val, acpu_core_powerdown_flags);


        IPC_SpinUnLock(IPC_SEM_CPUIDLE);
        hisi_ipc_core_suspend(core, cluster, 0);
    }
    else if(PM_OFF == mode)
    {
        IPC_SpinLock(IPC_SEM_CPUIDLE);

	    val = readl(acpu_core_powerdown_flags);
	    val |= (0x01 << (cluster*16 + core * 4 + 1));
        writel(val, acpu_core_powerdown_flags);

        IPC_SpinUnLock(IPC_SEM_CPUIDLE);
        hisi_ipc_core_suspend(core, cluster, 0);
    }
}

void hisi_ipc_cls_on_off(unsigned int core, unsigned int cluster, enum pm_mode mode)
{

    unsigned int val = 0;
    if(PM_ON == mode)
    {
        IPC_SpinLock(IPC_SEM_CPUIDLE);

    	val = readl(acpu_cluster_powerdown_flags);
    	val |= (0x01 << (cluster * 4));
        writel(val, acpu_cluster_powerdown_flags);

        IPC_SpinUnLock(IPC_SEM_CPUIDLE);
        hisi_ipc_core_suspend(core, cluster, 0);
    }
    else if(PM_OFF == mode)
    {
        IPC_SpinLock(IPC_SEM_CPUIDLE);
	 if(hisi_cores_pd_in_cluster_besides_curr(cluster, core))
        {
    	 val = readl(acpu_cluster_powerdown_flags);
    	 val |= (0x01 << (cluster * 4 + 1));
        writel(val, acpu_cluster_powerdown_flags);
	 }
        IPC_SpinUnLock(IPC_SEM_CPUIDLE);
        hisi_ipc_core_suspend(core, cluster, 0);
    }
}

void hisi_ipc_pm_suspend(unsigned int core, unsigned int cluster, unsigned int affinity_level)
{
    unsigned int val = 0;

    IPC_SpinLock(IPC_SEM_CPUIDLE);

	val = readl(acpu_core_powerdown_flags);
	val |= (0x01 << (cluster*16 + core * 4 + 2));
    writel(val, acpu_core_powerdown_flags);

    IPC_SpinUnLock(IPC_SEM_CPUIDLE);
    hisi_ipc_core_suspend(core, cluster, 0);
}

void hisi_ipc_cls_suspend(unsigned int core, unsigned int cluster, unsigned int affinity_level)
{
	unsigned int val = 0;

    IPC_SpinLock(IPC_SEM_CPUIDLE);
    if(hisi_cores_pd_in_cluster_besides_curr(cluster, core))
    {
	val = readl(acpu_cluster_powerdown_flags);
	val |= (0x01 << (cluster * 4 + 1));
    	writel(val, acpu_cluster_powerdown_flags);
    }
    IPC_SpinUnLock(IPC_SEM_CPUIDLE);

    hisi_ipc_core_suspend(core, cluster, 0);
}

void hisi_ipc_psci_system_off(unsigned int core, unsigned int cluster)
{
	hisi_ipc_sys_suspend(0, 0, 0);
}


int  hisi_ipc_init(void)
{
    g_CoreNum = IPC_CORE_ACPU;

    g_ipc_base_addr = SOC_IPC_CPU_RAW_INT_ADDR(SOC_IPC_S_BASE_ADDR, 2);

    acpu_core_powerdown_flags = ACPU_CORE_POWERDOWN_FLAGS_ADDR;
    acpu_cluster_powerdown_flags = ACPU_CLUSTER_POWERDOWN_FLAGS_ADDR;

    writel(0x8, acpu_core_powerdown_flags);
    writel(0x8, acpu_cluster_powerdown_flags);

	return 0;
}
