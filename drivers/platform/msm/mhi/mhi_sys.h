<<<<<<< HEAD
<<<<<<< HEAD
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
>>>>>>> 2617302... source
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
#ifndef _H_MHI_SYS_
#define _H_MHI_SYS_

#include <linux/mutex.h>
#include <linux/ipc_logging.h>
#include <linux/sysfs.h>
#include <linux/delay.h>

#include "mhi.h"

extern enum MHI_DEBUG_LEVEL mhi_msg_lvl;
extern enum MHI_DEBUG_LEVEL mhi_ipc_log_lvl;
extern enum MHI_DEBUG_CLASS mhi_msg_class;
extern u32 m3_timer_val_ms;

extern enum MHI_DEBUG_LEVEL mhi_xfer_db_interval;
extern enum MHI_DEBUG_LEVEL tx_mhi_intmodt;
extern enum MHI_DEBUG_LEVEL rx_mhi_intmodt;
extern void *mhi_ipc_log;

#define MHI_ASSERT(_x, _msg)\
	do {\
		if (!(_x)) {\
			pr_err("ASSERT- %s : Failure in %s:%d/%s()!\n",\
				_msg, __FILE__, __LINE__, __func__); \
			panic("ASSERT"); \
		} \
	} while (0)

#define mhi_log(_msg_lvl, _msg, ...) do { \
		if ((_msg_lvl) >= mhi_msg_lvl) \
			pr_alert("[%s] " _msg, __func__, ##__VA_ARGS__);\
<<<<<<< HEAD
=======
#ifndef _H_OSAL_
#define _H_OSAL_

#include <linux/mutex.h> /* mutex_t */
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/slab.h> /* kmalloc, kfree */
#include <linux/completion.h> /* completions */
#include <linux/io.h> /*readl_relaxed(), writel_relaxed() */
#include <linux/slab.h> /* GFP - general purpose allocator */
#include <linux/kthread.h>
#include <linux/wait.h> /* wait_event */
#include <linux/dma-mapping.h> /* dma_alloc_coherent */
#include <linux/delay.h>
#include <asm/barrier.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/device.h> /* struct device */
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/hrtimer.h>
#include <linux/ipc_logging.h>
#include <linux/spinlock_types.h>
#include <linux/pm.h>
#include <mach/msm_pcie.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/sched/rt.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include "mhi.h"
#include <linux/esoc_client.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/msm-bus.h>
#include <linux/cpu.h>
#include <linux/irq.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>

extern MHI_DEBUG_LEVEL mhi_msg_lvl;
extern MHI_DEBUG_LEVEL mhi_ipc_log_lvl;
extern MHI_DEBUG_CLASS mhi_msg_class;

extern void *mhi_ipc_log;
#define MHI_ASSERT(_x, _msg) if (!(_x)) {\
	pr_err("ASSERT- %s : Failure in %s:%d/%s()!\n",\
			_msg,__FILE__, __LINE__, __func__); \
	panic("ASSERT"); \
}

#define mhi_log(_msg_lvl, _msg, ...) do { \
		if ((_msg_lvl) >= mhi_msg_lvl) \
			pr_info("[%s] " _msg, __func__, ##__VA_ARGS__);\
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
		if (mhi_ipc_log && ((_msg_lvl) >= mhi_ipc_log_lvl))	\
			ipc_log_string(mhi_ipc_log,			\
			       "[%s] " _msg, __func__, ##__VA_ARGS__);	\
} while (0)

<<<<<<< HEAD
<<<<<<< HEAD
irqreturn_t mhi_msi_handlr(int msi_number, void *dev_id);
=======
#define pcie_read(base, offset, dest)	do {				\
	                        dest = readl_relaxed((volatile void *)		\
					      (uintptr_t)(base+offset)); \
				timestamp_log(base + offset);		\
	                        } while (0)

#define pcie_write(base, offset, val) do {				\
	                        timestamp_log (base + offset);		\
				writel_relaxed(val,			\
					(volatile void *)(uintptr_t)(base + offset)); \
				wmb();					\
				PULSE_L1_EXIT(0);			\
				} while (0)

irqreturn_t irq_cb(int msi_number, void *dev_id);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
irqreturn_t mhi_msi_handlr(int msi_number, void *dev_id);
>>>>>>> 2617302... source

struct mhi_meminfo {
	struct device *dev;
	uintptr_t pa_aligned;
	uintptr_t pa_unaligned;
	uintptr_t va_aligned;
	uintptr_t va_unaligned;
	uintptr_t size;
};

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
enum MHI_STATUS mhi_mallocmemregion(struct mhi_meminfo *meminfo, size_t size);

uintptr_t mhi_get_phy_addr(struct mhi_meminfo *meminfo);
void *mhi_get_virt_addr(struct mhi_meminfo *meminfo);
uintptr_t mhi_p2v_addr(struct mhi_meminfo *meminfo, phys_addr_t pa);
phys_addr_t mhi_v2p_addr(struct mhi_meminfo *meminfo, uintptr_t va);
u64 mhi_get_memregion_len(struct mhi_meminfo *meminfo);
void mhi_freememregion(struct mhi_meminfo *meminfo);

void print_ring(struct mhi_ring *local_chan_ctxt, u32 ring_id);
int mhi_init_debugfs(struct mhi_device_ctxt *mhi_dev_ctxt);
int mhi_probe(struct pci_dev *mhi_device,
		const struct pci_device_id *mhi_device_id);
ssize_t sysfs_init_m3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t sysfs_init_m0(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
<<<<<<< HEAD
=======
MHI_STATUS mhi_mallocmemregion(mhi_meminfo *meminfo, size_t size);

uintptr_t mhi_get_phy_addr(mhi_meminfo *meminfo);
void *mhi_get_virt_addr(mhi_meminfo *meminfo);
uintptr_t mhi_p2v_addr(mhi_meminfo *meminfo, uintptr_t pa);
uintptr_t mhi_v2p_addr(mhi_meminfo *meminfo, uintptr_t va);
u64 mhi_get_memregion_len(mhi_meminfo *meminfo);
void mhi_freememregion(mhi_meminfo *meminfo);

void print_ring(mhi_ring *local_chan_ctxt, u32 ring_id);
int mhi_init_debugfs(mhi_device_ctxt *mhi_dev_ctxt);
int mhi_probe(struct pci_dev *mhi_device,
		const struct pci_device_id *mhi_device_id);
ssize_t sysfs_init_M3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t sysfs_init_M0(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t sysfs_init_M1(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t sysfs_get_mhi_state(struct device *dev, struct device_attribute *attr,
			char *buf);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

#endif
