/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
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

#include <bakery_lock.h>
#include <mmio.h>
#include <pwrctrl_multi_memcfg.h>
#include "../../hisi_def.h"
#include "hi6xxx_pwrc.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <hisi_ipc.h>

static volatile unsigned int cluster_idle_flag[CLUSTER_NUM];
static volatile unsigned int cores_idle_flag[CLUSTER_NUM];
static void *core_entry_point_base;
static unsigned long g_cci_base;

#define CLUSTER_CPUS_IDLE_MASK          (0x0F)
#define ACPU_SUBSYS_POWERDOWN_FLAG 0xDEAD

unsigned long g_soc_acpu_sctrl_baseaddr = 0;
#define SOC_ACPU_SCTRL_BASE_ADDR (g_soc_acpu_sctrl_baseaddr)

#define writel(val, addr)		mmio_write_32((uintptr_t)addr, (uint32_t)val)
#define readl(addr)				mmio_read_32((uintptr_t)addr)

#define BIT(x) (0x1 << (x))


extern void  pm_asm_code_begin_flag();
extern void  pm_asm_code_end_flag();



//void print_err(const char *fmt, ...)
//{
//	;
//}

#define print_err printf

struct core_entry_point_stru{
	unsigned long core_entry;
};

static void *g_acpu_sc_base_map;
static void *g_acpu_subsys_powerdown_flag_base;


/*
 *return & modify Flag
 */
int hisi_cluster_is_idle(unsigned int cluster)
{
    return !!((cluster_idle_flag[cluster] & CLUSTER_CPUS_IDLE_MASK) == CLUSTER_CPUS_IDLE_MASK);
}

int hisi_cluster_is_powered_on(unsigned int cluster)
{
    return !!!((cluster_idle_flag[cluster] & CLUSTER_CPUS_IDLE_MASK) == CLUSTER_CPUS_IDLE_MASK);
}

int hisi_cluster_allcores_powered_off(unsigned int cluster)
{
    return !!((cluster_idle_flag[cluster] & CLUSTER_CPUS_IDLE_MASK) == CLUSTER_CPUS_IDLE_MASK);
}

int hisi_core_is_powered_up(unsigned int cluster, unsigned int core)
{
    return !!!(cores_idle_flag[cluster] & (0x1 << core));
}

int hisi_cores_pd_in_cluster_besides_curr(unsigned int cluster, unsigned int core)
{
    unsigned int val = 0;
    unsigned core_val = 0;

    val = hisi_core_pd_flags_rd();
    val = val >> (cluster * 16);

    for(int i =0; i < 4; i++)
    {
        if(i != core)
        {
            core_val = (val >> (i * 4)) & 0xF;
            if((core_val & 0x8) && (!(core_val & 0x2) )&& (!(core_val & 0x4)))
            {
                return 0;
            }
        }
   }
   return 1;
}

int hisi_cores_powered_off_besides_curr(unsigned int core)
{
    unsigned int val = 0;
    val = hisi_core_pd_flags_rd();
    return (val == (0x8 << (core * 4)));
}


void hisi_set_cpuidle_flag(unsigned int cluster, unsigned int core, unsigned long arg)
{
    cores_idle_flag[cluster] |= (0x1 << core);
    if(arg > 1)
    {
        cluster_idle_flag[cluster] |= (0x1 << core);
    }
}

void hisi_clear_cpuidle_flag(unsigned int cluster, unsigned int core)
{
    cores_idle_flag[cluster] &= ~(0x1 << core);
    cluster_idle_flag[cluster] &= ~(0x1 << core);
}

void hisi_set_cpu_boot_flag(unsigned int cluster, unsigned int core)
{
    cores_idle_flag[cluster] &= ~(0x1 << core);
    cluster_idle_flag[cluster] &= ~(0x1 << core);
}

void hisi_clear_cpu_boot_flag(unsigned int cluster, unsigned int core)
{
    cores_idle_flag[cluster] |= (0x1 << core);
    cluster_idle_flag[cluster] |= (0x1 << core);
}


/*
 *API
 */
void hisi_powerup_core(unsigned int cluster, unsigned int core)
{
    hisi_ipc_pm_on_off(core, cluster, PM_ON);
}

void hisi_powerdn_core(unsigned int cluster, unsigned int core)
{
    hisi_ipc_pm_on_off(core, cluster, PM_OFF);
}

void hisi_powerup_cluster(unsigned int cluster, unsigned int core)
{
    hisi_ipc_cls_on_off(core, cluster, PM_ON);
}

void hisi_powerdn_cluster(unsigned int cluster, unsigned int core)
{	hisi_ipc_cls_on_off(core, cluster, PM_OFF);
}

void hisi_enter_core_idle(unsigned int cluster, unsigned int core)
{
    hisi_ipc_pm_suspend(core, cluster, 0);
}

void hisi_enter_cluster_idle(unsigned int cluster, unsigned int core)
{
    hisi_ipc_cls_suspend(core, cluster, 0);
}

void hisi_enter_ap_suspend(unsigned int cluster, unsigned int core)
{
    hisi_ipc_psci_system_off(core, cluster);
}

int hisi_pm_ipc_init(void)
{
	cores_idle_flag[0] = 0x0E;
	cores_idle_flag[1] = 0x0F;

	cluster_idle_flag[0] = 0x0E;
	cluster_idle_flag[1] = 0x0F;

	return 0;
}


void set_core_power_on_addr(unsigned int cluster, unsigned int core, unsigned long entry_point)
{
	unsigned index = 0;
	struct core_entry_point_stru *core_entry = (struct core_entry_point_stru *)core_entry_point_base;

	if(0 == core_entry)
	{
		print_err("%s coreentrypoint_base is null\n", __FUNCTION__);
		return;
	}

	index = cluster * CORE_NUM_PER_CLUSTER + core;
	mmio_write_64((uintptr_t)&core_entry[index].core_entry, entry_point);
	//printf("core addr:0x%p entry:0x%p\n", (unsigned long)&core_entry[index].core_entry, entry_point);
}

void clear_core_power_on_addr(unsigned int cluster, unsigned int core)
{
	unsigned index = 0;
	struct core_entry_point_stru *core_entry = (struct core_entry_point_stru *)core_entry_point_base;

	if(0 == core_entry)
	{
		print_err("%s coreentrypoint_base is null\n", __FUNCTION__);
		return;
	}

	index = cluster * CORE_NUM_PER_CLUSTER + core;
	mmio_write_64((uintptr_t)&core_entry[index].core_entry, 0);
}

void hisi_set_cluster_wfi(unsigned int id)
{
    volatile unsigned int reg_val = 0;

    if(0 == id)
    {
        reg_val = mmio_read_32((uintptr_t)(((unsigned long)g_acpu_sc_base_map) + 0x0E4));
        /*reg_val |= BIT(SOC_ACPU_SCTRL_ACPU_SC_SNOOP_PWD_set_acinactm_high0_START);*/
        reg_val |= BIT(0);
        mmio_write_32((uintptr_t)((unsigned long)g_acpu_sc_base_map + 0x0E4), reg_val);
    }
    else if(1 == id)
    {
        reg_val = mmio_read_32((uintptr_t)(((unsigned long)g_acpu_sc_base_map) + 0x0E4));
        /*reg_val |= BIT(SOC_ACPU_SCTRL_ACPU_SC_SNOOP_PWD_set_acinactm_high1_START);*/
        reg_val |= BIT(16);
        mmio_write_32((uintptr_t)((unsigned long)g_acpu_sc_base_map + 0x0E4),reg_val);
    }

}

void coherent_init(void)
{
    /*CCI init*/
    writel(0x180003, (g_cci_base + 0x90004));
    writel(0x18, (g_cci_base + 0x90000));
}

void coherent_slave_port_config(void)
{

    writel(0x500050, (g_cci_base + 0x94130));
    writel(0x500050, (g_cci_base + 0x95130));

    writel(0x30003, (g_cci_base + 0x94134));
    writel(0x30003, (g_cci_base + 0x95134));

    writel(0x6010601, (g_cci_base + 0x94138));
    writel(0x6010601, (g_cci_base + 0x95138));

    writel(0x3, (g_cci_base + 0x9410c));
    writel(0x3, (g_cci_base + 0x9510c));

    return;
}

static int hi6xxx_pm_drvinit(void)
{
	volatile unsigned int reg_val = 0;

	print_err("%s enter\n",__FUNCTION__);

	g_cci_base = (unsigned long)(0xF6E00000);

	g_soc_acpu_sctrl_baseaddr = 0xF6504000;

	g_acpu_sc_base_map = (void  *)(SOC_ACPU_SCTRL_BASE_ADDR);
	print_err("%s g_acpu_sc_base_map:0x%p.\n", __FUNCTION__, g_acpu_sc_base_map);

	g_acpu_subsys_powerdown_flag_base = (void  *)(ACPU_SUBSYS_POWERDOWN_FLAGS_ADDR);
	print_err("%s acpu subsys power flag addr:0x%p.\n", __FUNCTION__, g_acpu_subsys_powerdown_flag_base);

	core_entry_point_base = (void  *)(PWRCTRL_ACPU_ASM_D_ARM_PARA_AD);
	printf("v core addr:0x%p, phy:0x%p\n", core_entry_point_base, (unsigned long)PWRCTRL_ACPU_ASM_D_ARM_PARA_AD);

	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x158);
	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x258);
	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x358);
	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x458);
	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x558);
	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x658);
	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x758);
	writel( PWRCTRL_ACPU_ASM_CODE_BASE >> 2, ((unsigned long)g_acpu_sc_base_map) + 0x858);

	memset((void *)(PWRCTRL_ACPU_ASM_SPACE_ADDR), 0, 0x400);

	writel(0xE1A00000, (((unsigned long)(PWRCTRL_ACPU_ASM_SPACE_ADDR))));
	writel(0xE3A02003, (((unsigned long)(PWRCTRL_ACPU_ASM_SPACE_ADDR)) + 4));
	writel(0xEE0C2F50, (((unsigned long)(PWRCTRL_ACPU_ASM_SPACE_ADDR)) + 8));
	writel(0xE320F003, (((unsigned long)(PWRCTRL_ACPU_ASM_SPACE_ADDR)) + 12));

	reg_val = readl((0xF7800000 + 0x004));
	reg_val |= BIT(0x1) | \
	                 BIT(17);
	writel(reg_val, (0xF7800000 + 0x004));

	memcpy((void *)(PWRCTRL_ACPU_ASM_CODE_BASE), (void *)(unsigned long)pm_asm_code_begin_flag, ((unsigned long)pm_asm_code_end_flag - (unsigned long)pm_asm_code_begin_flag));

	print_err("pm asm code begin addr:0x%p end addr:0x%p.\n", pm_asm_code_begin_flag, pm_asm_code_end_flag);
	print_err("pm asm code size:0x%x.\n", ((unsigned long)pm_asm_code_end_flag - (unsigned long)pm_asm_code_begin_flag));
	print_err("%s exit\n",__FUNCTION__);

	return 0;
}

/* Nothing else to do here apart from initializing the lock */
int hi6xxx_pwrc_setup(void)
{
	hisi_pm_ipc_init();
	hi6xxx_pm_drvinit();

	return 0;
}



