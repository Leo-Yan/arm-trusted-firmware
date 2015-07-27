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

#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <bakery_lock.h>
#include <cci400.h>
#include <errno.h>
#include <mmio.h>
#include <platform.h>
#include <plat_config.h>
#include <platform_def.h>
#include <psci.h>
#include "drivers/pwrc/hi6xxx_pwrc.h"
#include "hisi_def.h"
#include "hi6xxx_private.h"
#include <stdio.h>
#include "mdrv_ipc.h"
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
#include <hi6220.h>


extern void IPC_SpinLock (unsigned int u32SignalNum);
extern void IPC_SpinUnLock (unsigned int u32SignalNum);

//static unsigned int g_cpuidle_power_state[CLUSTER_NUM*CORE_NUM_PER_CLUSTER];

/*******************************************************************************
 * FVP handler called when an affinity instance is about to enter standby.
 ******************************************************************************/
int hi6xxx_affinst_standby(unsigned int power_state)
{
	unsigned int target_afflvl;

	/* Sanity check the requested state */
	target_afflvl = psci_get_pstate_afflvl(power_state);

	/*
	 * It's possible to enter standby only on affinity level 0 i.e. a cpu
	 * on the FVP. Ignore any other affinity level.
	 */
	if (target_afflvl != MPIDR_AFFLVL0)
		return PSCI_E_INVALID_PARAMS;

	/*
	 * Enter standby state
	 * dsb is good practice before using wfi to enter low power states
	 */
	dsb();
	wfi();

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * FVP handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
int hi6xxx_affinst_on(unsigned long mpidr,
		   unsigned long sec_entrypoint,
		   unsigned int afflvl,
		   unsigned int state)
{
	int rc = PSCI_E_SUCCESS;
	unsigned int core = mpidr & MPIDR_CPU_MASK;
	unsigned int cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFFINITY_BITS;

	printf("%s: %x %x %x %x %x %x %x %x\n", __func__,
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(0)),
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(1)),
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(2)),
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(3)),
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(4)),
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(5)),
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(6)),
		mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(7)));

	switch (afflvl) {
	case MPIDR_AFFLVL1:
		if (state == PSCI_STATE_OFF) {
			if(hisi_cluster_is_idle(cluster))
				hisi_powerup_cluster(cluster, core);
		}
		break;

	case MPIDR_AFFLVL0:
		hisi_set_cpu_boot_flag(cluster, core);
		set_core_power_on_addr(cluster, core, sec_entrypoint);
		hisi_powerup_core(cluster, core);
		break;

	default:
		assert(0);
	}

	return rc;
}

/*******************************************************************************
 * FVP handler called when an affinity instance is about to be turned off. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions.
 *
 * CAUTION: This function is called with coherent stacks so that caches can be
 * turned off, flushed and coherency disabled. There is no guarantee that caches
 * will remain turned on across calls to this function as each affinity level is
 * dealt with. So do not write & read global variables across calls. It will be
 * wise to do flush a write to the global to prevent unpredictable results.
 ******************************************************************************/
static void hi6xxx_affinst_off(unsigned int afflvl, unsigned int state)
{
	unsigned int mpidr = read_mpidr_el1();
	//int rc = PSCI_E_SUCCESS;
	//unsigned int ectlr;
	unsigned int core = mpidr & MPIDR_CPU_MASK;
	unsigned int cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFFINITY_BITS;

	switch (afflvl) {
	case MPIDR_AFFLVL1:
		if (state == PSCI_STATE_OFF) {
			IPC_SpinLock(IPC_SEM_CPUIDLE);
			if(hisi_cores_pd_in_cluster_besides_curr(cluster, core))
			{
				/*
				 * Disable coherency if this cluster is to be
				 * turned off
				 */
				cci_disable_cluster_coherency(mpidr);

				__asm__ volatile ("isb");
				__asm__ volatile ("dsb sy");
			}
			IPC_SpinUnLock(IPC_SEM_CPUIDLE);
			/*
			 * Program the power controller to turn the
			 * cluster off
			 */
			hisi_powerdn_core(cluster, core);
			hisi_powerdn_cluster(cluster, core);

		}
		else
		{
			hisi_powerdn_core(cluster, core);
		}
		break;

	case MPIDR_AFFLVL0:
		if (state == PSCI_STATE_OFF) {
			__asm__ volatile ("clrex");
			/*
			 * Take this cpu out of intra-cluster coherency if
			 * the FVP flavour supports the SMP bit.
			 */
			//if (get_plat_config()->flags & CONFIG_CPUECTLR_SMP_BIT) {
			//	ectlr = read_cpuectlr();
			//	ectlr &= ~CPUECTLR_SMP_BIT;
			//	write_cpuectlr(ectlr);
			//}

			//__asm__ volatile ("isb");
			//__asm__ volatile ("dsb sy");

			/*
			 * Prevent interrupts from spuriously waking up
			 * this cpu
			 */
			arm_gic_cpuif_deactivate();

			/*
			 * Program the power controller to power this
			 * cpu off
			 */
			hisi_clear_cpu_boot_flag(cluster, core);
		}
		break;

	default:
		assert(0);
	}

	return;
}

/*******************************************************************************
 * FVP handler called when an affinity instance is about to be suspended. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions.
 *
 * CAUTION: This function is called with coherent stacks so that caches can be
 * turned off, flushed and coherency disabled. There is no guarantee that caches
 * will remain turned on across calls to this function as each affinity level is
 * dealt with. So do not write & read global variables across calls. It will be
 * wise to do flush a write to the global to prevent unpredictable results.
 ******************************************************************************/
void hi6xxx_affinst_suspend(unsigned long sec_entrypoint,
			       unsigned int afflvl,
			       unsigned int state)
{
	//int rc = PSCI_E_SUCCESS;
	//unsigned int ectlr;
	unsigned int mpidr = read_mpidr_el1();
	unsigned int core = mpidr & MPIDR_CPU_MASK;
	unsigned int cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFFINITY_BITS;

	//printf("%s: enter\n", __func__);
	switch (afflvl) {
	case MPIDR_AFFLVL1:
		hisi_set_cpuidle_flag(cluster, core, 2);

		if (state == PSCI_STATE_OFF) {
			IPC_SpinLock(IPC_SEM_CPUIDLE);
			if(hisi_cores_pd_in_cluster_besides_curr(cluster, core))
			{
				/*
				 * Disable coherency if this cluster is to be
				 * turned off
				 */
				cci_disable_cluster_coherency(mpidr);

				__asm__ volatile ("isb");
				__asm__ volatile ("dsb sy");
			}
			IPC_SpinUnLock(IPC_SEM_CPUIDLE);
			/*
			 * Program the power controller to turn the
			 * cluster off
			 */
			hisi_enter_cluster_idle(cluster, core);


		}

		break;

	case MPIDR_AFFLVL0:
		if (state == PSCI_STATE_OFF) {
			//__asm__ volatile ("clrex");
			///*
			// * Take this cpu out of intra-cluster coherency if
			// * the FVP flavour supports the SMP bit.
			// */
			//if (get_plat_config()->flags & CONFIG_CPUECTLR_SMP_BIT) {
			//	ectlr = read_cpuectlr();
			//	ectlr &= ~CPUECTLR_SMP_BIT;
			//	write_cpuectlr(ectlr);
			//}

			//__asm__ volatile ("isb");
			//__asm__ volatile ("dsb sy");
			/*
			 * Prevent interrupts from spuriously waking up
			 * this cpu
			 */
			arm_gic_cpuif_deactivate();

			/* Program the jump address for the target cpu */
			set_core_power_on_addr(cluster, core, sec_entrypoint);

			hisi_set_cpuidle_flag(cluster, core, 0);
			/*
			 * Program the power controller to power this
			 * cpu off and enable wakeup interrupts.
			 */
			hisi_enter_core_idle(cluster, core);

			psci_program_mailbox(mpidr, sec_entrypoint);
		}
		break;

	default:
		assert(0);
	}

	return;
}

void hikey_affinst_on_finish(uint32_t afflvl, uint32_t state)
{
	unsigned long mpidr;
	unsigned long linear_id;

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();
	linear_id = platform_get_core_pos(mpidr);

	/*
	 * Perform the common cluster specific operations i.e enable coherency
	 * if this cluster was off.
	 */
	if (afflvl != MPIDR_AFFLVL0)
		cci_enable_cluster_coherency(mpidr);

	psci_program_mailbox(mpidr, 0x0);

	/* Cleanup cpu entry point */
	mmio_write_32(ACPU_SC_CPUx_RVBARADDR(linear_id), 0x0);

	/* Enable the gic cpu interface */
	arm_gic_cpuif_setup();

	/* TODO: if GIC in AON, then just need init for cold boot */
	arm_gic_pcpu_distif_setup();
}

/*******************************************************************************
 * FVP handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 ******************************************************************************/
void hi6xxx_affinst_on_finish(unsigned int afflvl, unsigned int state)
{
	unsigned int mpidr = read_mpidr_el1();
	//int rc = PSCI_E_SUCCESS;
	//unsigned int ectlr;
	unsigned int core = mpidr & MPIDR_CPU_MASK;
	unsigned int cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFFINITY_BITS;
	//unsigned long linear_id;

	//linear_id = platform_get_core_pos(mpidr);

	if (afflvl != MPIDR_AFFLVL0)
		cci_enable_cluster_coherency(mpidr);

	switch (afflvl) {

	case MPIDR_AFFLVL1:
		/* Enable coherency if this cluster was off */
		if (state == PSCI_STATE_OFF) {


#if 0 /*acpu sr,need the code as follows*/
			coherent_init();
			coherent_slave_port_config();
			__asm__ volatile ("isb");
			__asm__ volatile ("dsb sy");
#endif
			/*
			 * This CPU might have woken up whilst the
			 * cluster was attempting to power down. In
			 * this case the FVP power controller will
			 * have a pending cluster power off request
			 * which needs to be cleared by writing to the
			 * PPONR register. This prevents the power
			 * controller from interpreting a subsequent
			 * entry of this cpu into a simple wfi as a
			 * power down request.
			 */
			//hi6xxx_cci_setup();
			__asm__ volatile ("isb");
			__asm__ volatile ("dsb sy");
		}
		break;

	case MPIDR_AFFLVL0:
		/*
		 * Ignore the state passed for a cpu. It could only have
		 * been off if we are here.
		 */

		/*
		 * Turn on intra-cluster coherency if the FVP flavour supports
		 * it.
		 */
		//if (get_plat_config()->flags & CONFIG_CPUECTLR_SMP_BIT) {
		//	 ectlr = read_cpuectlr();
		//	 ectlr |= CPUECTLR_SMP_BIT;
		//	 write_cpuectlr(ectlr);
		//}

		//__asm__ volatile ("isb");
		//__asm__ volatile ("dsb sy");
		/* Zero the jump address in the mailbox for this cpu */
		clear_core_power_on_addr(cluster, core);

		/* Enable the gic cpu interface */
		arm_gic_cpuif_setup();

		/* TODO: This setup is needed only after a cold boot */
		arm_gic_pcpu_distif_setup();

		psci_program_mailbox(mpidr, 0x0);
		break;

	default:
		assert(0);
	}

	return;
}

/*******************************************************************************
 * FVP handler called when an affinity instance has just been powered on after
 * having been suspended earlier. The level and mpidr determine the affinity
 * instance.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
void hi6xxx_affinst_suspend_finish(unsigned int afflvl,
				      unsigned int state)
{
	unsigned int mpidr = read_mpidr_el1();
	unsigned int core = mpidr & MPIDR_CPU_MASK;
	unsigned int cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFFINITY_BITS;

	//printf("%s: enter %x %x\n", __func__,
	//	mpidr, mmio_read_32(ACPU_SC_CPUx_PW_ISO_STAT(7)));

	if(MPIDR_AFFLVL0 == afflvl)
		hisi_clear_cpuidle_flag(cluster, core);

	hi6xxx_affinst_on_finish(afflvl, state);

	return;
}

#if 0
static int32_t hikey_do_plat_actions(uint32_t afflvl, uint32_t state)
{
	uint32_t max_phys_off_afflvl;

	assert(afflvl <= MPIDR_AFFLVL1);

	if (state != PSCI_STATE_OFF)
		return -EAGAIN;

	/*
	 * Find the highest affinity level which will be suspended and postpone
	 * all the platform specific actions until that level is hit.
	 */
	max_phys_off_afflvl = psci_get_max_phys_off_afflvl();
	assert(max_phys_off_afflvl != PSCI_INVALID_DATA);
	assert(psci_get_suspend_afflvl() >= max_phys_off_afflvl);
	if (afflvl != max_phys_off_afflvl)
		return -EAGAIN;

	return 0;
}

static void hikey_affinst_suspend(uint64_t sec_entrypoint,
				  uint32_t afflvl,
				  uint32_t state)
{
	unsigned long mpidr = read_mpidr_el1();
	unsigned int core = mpidr & MPIDR_CPU_MASK;
	//unsigned long linear_id;
	unsigned int cluster;

	/* Get the mpidr for this cpu */
	//linear_id = platform_get_core_pos(mpidr);
	cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFFINITY_BITS;

	/* Determine if any platform actions need to be executed */
	if (hikey_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/*
	 * Prevent interrupts from spuriously waking up
	 * this cpu
	 */
	arm_gic_cpuif_deactivate();

	/* Program the jump address for the target cpu */
	set_core_power_on_addr(cluster, core, sec_entrypoint);

	hisi_set_cpuidle_flag(cluster, core, 0);
	/*
	 * Program the power controller to power this
	 * cpu off and enable wakeup interrupts.
	 */
	hisi_enter_core_idle(cluster, core);

	psci_program_mailbox(mpidr, sec_entrypoint);

	/* Cluster is to be turned off, so disable coherency */
	if (afflvl > MPIDR_AFFLVL0)
		cci_disable_cluster_coherency(mpidr);
}

static void hikey_affinst_suspend_finish(uint32_t afflvl,
					 uint32_t state)
{
	unsigned long mpidr = read_mpidr_el1();
	unsigned int core = mpidr & MPIDR_CPU_MASK;
	//unsigned long linear_id;
	unsigned int cluster;

	/* Get the mpidr for this cpu */
	//linear_id = platform_get_core_pos(mpidr);
	cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFFINITY_BITS;

	if (afflvl != MPIDR_AFFLVL0)
		cci_enable_cluster_coherency(mpidr);

	if (afflvl == MPIDR_AFFLVL0)
		hisi_clear_cpuidle_flag(cluster, core);

	clear_core_power_on_addr(cluster, core);

	/* Enable the gic cpu interface */
	arm_gic_cpuif_setup();

	/* TODO: This setup is needed only after a cold boot */
	arm_gic_pcpu_distif_setup();

	psci_program_mailbox(mpidr, 0x0);
}
#endif

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t hi6xxx_plat_pm_ops = {
	.affinst_on		= hi6xxx_affinst_on,
	.affinst_on_finish	= hi6xxx_affinst_on_finish,
	.affinst_off		= hi6xxx_affinst_off,
	.affinst_standby	= NULL,
	.affinst_suspend	= hi6xxx_affinst_suspend,
	.affinst_suspend_finish	= hi6xxx_affinst_suspend_finish,
	.system_off		= NULL,
	.system_reset		= NULL,
};

/*******************************************************************************
 * Export the platform specific power ops & initialize the fvp power controller
 ******************************************************************************/
int platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &hi6xxx_plat_pm_ops;
	return 0;
}
