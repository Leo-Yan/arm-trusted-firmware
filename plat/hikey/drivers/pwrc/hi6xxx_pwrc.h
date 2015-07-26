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

#ifndef __HI6XXX_PWRC_H__
#define __HI6XXX_PWRC_H__

/* FVP Power controller register offset etc */
#define PPOFFR_OFF		0x0
#define PPONR_OFF		0x4
#define PCOFFR_OFF		0x8
#define PWKUPR_OFF		0xc
#define PSYSR_OFF		0x10

#define PWKUPR_WEN		(1ull << 31)

#define PSYSR_AFF_L2		(1 << 31)
#define PSYSR_AFF_L1		(1 << 30)
#define PSYSR_AFF_L0		(1 << 29)
#define PSYSR_WEN		(1 << 28)
#define PSYSR_PC		(1 << 27)
#define PSYSR_PP		(1 << 26)

#define PSYSR_WK_SHIFT		24
#define PSYSR_WK_MASK		0x3
#define PSYSR_WK(x)		(x >> PSYSR_WK_SHIFT) & PSYSR_WK_MASK

#define WKUP_COLD		0x0
#define WKUP_RESET		0x1
#define WKUP_PPONR		0x2
#define WKUP_GICREQ		0x3

#define PSYSR_INVALID		0xffffffff
#define CLUSTER_NUM 2
#define CORE_NUM_PER_CLUSTER 4

#ifndef __ASSEMBLY__

/*
 *return & modify Flag
 */
int hisi_cluster_is_idle(unsigned int cluster);
int hisi_cluster_is_powered_on(unsigned int cluster);
int hisi_cluster_allcores_powered_off(unsigned int cluster);
int hisi_core_is_powered_up(unsigned int cluster, unsigned int core);

//int hisi_cores_powered_off_besides_curr(unsigned int core);

void hisi_set_cpuidle_flag(unsigned int cluster, unsigned int core, unsigned long arg);
void hisi_clear_cpuidle_flag(unsigned int cluster, unsigned int core);
void hisi_set_cpu_boot_flag(unsigned int cluster, unsigned int core);
void hisi_clear_cpu_boot_flag(unsigned int cluster, unsigned int core);
int  hisi_pm_ipc_init(void);

/* api */
void hisi_powerup_core(unsigned int cluster, unsigned int core);
void hisi_powerdn_core(unsigned int cluster, unsigned int core);
void hisi_powerup_cluster(unsigned int cluster, unsigned int core);
void hisi_powerdn_cluster(unsigned int cluster, unsigned int core);
void hisi_enter_core_idle(unsigned int cluster, unsigned int core);
void hisi_enter_cluster_idle(unsigned int cluster, unsigned int core);
void hisi_enter_ap_suspend(unsigned int cluster, unsigned int core);
int hisi_cores_pd_in_cluster_besides_curr(unsigned int cluster, unsigned int core);
int hisi_cores_powered_off_besides_curr(unsigned int core);

void hisi_set_cluster_wfi(unsigned int id);
void coherent_init(void);
void coherent_slave_port_config(void);

void set_core_power_on_addr(unsigned int cluster, unsigned int core, unsigned long entry_point);
void clear_core_power_on_addr(unsigned int cluster, unsigned int core);
/*******************************************************************************
 * Function & variable prototypes
 ******************************************************************************/
int hi6xxx_pwrc_setup(void);

#endif /*__ASSEMBLY__*/

#endif /* __HI6XXX_PWRC_H__ */
