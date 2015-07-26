#ifndef __HISI_IPC_H__
#define __HISI_IPC_H__
enum pm_mode {
	PM_ON = 0,
	PM_OFF,
};


unsigned int hisi_core_pd_flags_rd();
void hisi_ipc_pm_on_off(unsigned int core, unsigned int cluster, enum pm_mode mode);
void hisi_ipc_cls_on_off(unsigned int core, unsigned int cluster, enum pm_mode mode);
void hisi_ipc_pm_suspend(unsigned int core, unsigned int cluster, unsigned int affinity_level);
void hisi_ipc_cls_suspend(unsigned int core, unsigned int cluster, unsigned int affinity_level);
void hisi_ipc_psci_system_off(unsigned int core, unsigned int cluster);
int hisi_ipc_init(void);

#endif
