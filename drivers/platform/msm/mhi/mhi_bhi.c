<<<<<<< HEAD
<<<<<<< HEAD
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

<<<<<<< HEAD
=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
#include "mhi_sys.h"
#include "mhi.h"
#include "mhi_macros.h"
#include "mhi_hwio.h"
#include "mhi_bhi.h"

<<<<<<< HEAD
<<<<<<< HEAD
static int bhi_open(struct inode *mhi_inode, struct file *file_handle)
=======
static const struct file_operations bhi_fops = {
read: NULL,
write : bhi_write,
open : bhi_open,
release : bhi_release,
};

static struct miscdevice bhi_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bhi",
	.fops = &bhi_fops,
};

int bhi_open(struct inode *mhi_inode, struct file *file_handle)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
static int bhi_open(struct inode *mhi_inode, struct file *file_handle)
>>>>>>> 2617302... source
{
	file_handle->private_data = &mhi_devices.device_list[0];
	return 0;
}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source

static ssize_t bhi_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *offp)
{
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 pcie_word_val = 0;
	u32 i = 0;
	struct bhi_ctxt_t *bhi_ctxt =
		&(((struct mhi_pcie_dev_info *)file->private_data)->bhi_ctxt);
	struct mhi_device_ctxt *mhi_dev_ctxt =
		&((struct mhi_pcie_dev_info *)file->private_data)->mhi_ctxt;
<<<<<<< HEAD
=======
int bhi_release(struct inode *mhi_inode, struct file *file_handle)
{
	return 0;
}
ssize_t bhi_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *offp)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 pcie_word_val = 0;
	u32 i = 0;
	bhi_ctxt_t *bhi_ctxt =
		&(((mhi_pcie_dev_info *)file->private_data)->bhi_ctxt);
	mhi_device_ctxt *mhi_dev_ctxt =
		((mhi_pcie_dev_info *)file->private_data)->mhi_ctxt;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	size_t amount_copied = 0;
	uintptr_t align_len = 0x1000;
	u32 tx_db_val = 0;

<<<<<<< HEAD
<<<<<<< HEAD
	if (buf == NULL || 0 == count)
=======
	if (NULL == buf || 0 == count)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	if (buf == NULL || 0 == count)
>>>>>>> 2617302... source
		return -EIO;

	if (count > BHI_MAX_IMAGE_SIZE)
		return -ENOMEM;

<<<<<<< HEAD
<<<<<<< HEAD
	wait_event_interruptible(*mhi_dev_ctxt->bhi_event,
			mhi_dev_ctxt->mhi_state == MHI_STATE_BHI);

=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	wait_event_interruptible(*mhi_dev_ctxt->bhi_event,
			mhi_dev_ctxt->mhi_state == MHI_STATE_BHI);

>>>>>>> 2617302... source
	mhi_log(MHI_MSG_INFO, "Entered. User Image size 0x%x\n", count);

	bhi_ctxt->unaligned_image_loc = kmalloc(count + (align_len - 1),
						GFP_KERNEL);
<<<<<<< HEAD
<<<<<<< HEAD
	if (bhi_ctxt->unaligned_image_loc == NULL)
		return -ENOMEM;

=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	if (bhi_ctxt->unaligned_image_loc == NULL)
		return -ENOMEM;

>>>>>>> 2617302... source
	mhi_log(MHI_MSG_INFO, "Unaligned Img Loc: %p\n",
			bhi_ctxt->unaligned_image_loc);
	bhi_ctxt->image_loc =
			(void *)((uintptr_t)bhi_ctxt->unaligned_image_loc +
		 (align_len - (((uintptr_t)bhi_ctxt->unaligned_image_loc) %
			       align_len)));

	mhi_log(MHI_MSG_INFO, "Aligned Img Loc: %p\n", bhi_ctxt->image_loc);
<<<<<<< HEAD
<<<<<<< HEAD

=======
	if (NULL == bhi_ctxt->image_loc)
		ret_val = MHI_STATUS_ERROR;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======

>>>>>>> 2617302... source
	bhi_ctxt->image_size = count;

	if (0 != copy_from_user(bhi_ctxt->image_loc, buf, count)) {
		ret_val = -ENOMEM;
		goto bhi_copy_error;
	}
	amount_copied = count;
	mhi_log(MHI_MSG_INFO,
		"Copied image from user at addr: %p\n", bhi_ctxt->image_loc);
	bhi_ctxt->phy_image_loc = dma_map_single(NULL,
			bhi_ctxt->image_loc,
			bhi_ctxt->image_size,
			DMA_TO_DEVICE);

	if (dma_mapping_error(NULL, bhi_ctxt->phy_image_loc)) {
		ret_val = -EIO;
		goto bhi_copy_error;
	}
	mhi_log(MHI_MSG_INFO,
		"Mapped image to DMA addr 0x%lx:\n",
		(uintptr_t)bhi_ctxt->phy_image_loc);

	bhi_ctxt->image_size = count;

	/* Write the image size */
	pcie_word_val = HIGH_WORD(bhi_ctxt->phy_image_loc);
<<<<<<< HEAD
<<<<<<< HEAD
	mhi_reg_write_field(mhi_dev_ctxt, bhi_ctxt->bhi_base,
				BHI_IMGADDR_HIGH,
				0xFFFFFFFF,
=======
	MHI_REG_WRITE_FIELD(bhi_ctxt->bhi_base,
				BHI_IMGADDR_HIGH,
				0xFFFFffff,
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	mhi_reg_write_field(mhi_dev_ctxt, bhi_ctxt->bhi_base,
				BHI_IMGADDR_HIGH,
				0xFFFFFFFF,
>>>>>>> 2617302... source
				0,
				pcie_word_val);

	pcie_word_val = LOW_WORD(bhi_ctxt->phy_image_loc);

<<<<<<< HEAD
<<<<<<< HEAD
	mhi_reg_write_field(mhi_dev_ctxt, bhi_ctxt->bhi_base,
				BHI_IMGADDR_LOW,
				0xFFFFFFFF,
=======
	MHI_REG_WRITE_FIELD(bhi_ctxt->bhi_base,
				BHI_IMGADDR_LOW,
				0xFFFFffff,
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	mhi_reg_write_field(mhi_dev_ctxt, bhi_ctxt->bhi_base,
				BHI_IMGADDR_LOW,
				0xFFFFFFFF,
>>>>>>> 2617302... source
				0,
				pcie_word_val);

	pcie_word_val = bhi_ctxt->image_size;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	mhi_reg_write_field(mhi_dev_ctxt, bhi_ctxt->bhi_base, BHI_IMGSIZE,
			0xFFFFFFFF, 0, pcie_word_val);

	pcie_word_val = mhi_reg_read(bhi_ctxt->bhi_base, BHI_IMGTXDB);
	mhi_reg_write_field(mhi_dev_ctxt, bhi_ctxt->bhi_base,
			BHI_IMGTXDB, 0xFFFFFFFF, 0, ++pcie_word_val);

	for (i = 0; i < BHI_POLL_NR_RETRIES; ++i) {
		tx_db_val = mhi_reg_read(bhi_ctxt->bhi_base, BHI_STATUS);
<<<<<<< HEAD
=======
	MHI_REG_WRITE_FIELD(bhi_ctxt->bhi_base, BHI_IMGSIZE,
			0xFFFFffff, 0, pcie_word_val);

	MHI_REG_READ(bhi_ctxt->bhi_base, BHI_IMGTXDB, pcie_word_val);
	MHI_REG_WRITE_FIELD(bhi_ctxt->bhi_base,
			BHI_IMGTXDB, 0xFFFFffff, 0, ++pcie_word_val);

	for (i = 0; i < BHI_POLL_NR_RETRIES; ++i) {
		MHI_REG_READ(bhi_ctxt->bhi_base, BHI_STATUS, tx_db_val);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
		if ((0x80000000 | tx_db_val) == pcie_word_val)
			break;
		else
			mhi_log(MHI_MSG_CRITICAL,
				"BHI STATUS 0x%x\n", pcie_word_val);
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
		usleep_range(20000, 25000);
	}
	dma_unmap_single(NULL, bhi_ctxt->phy_image_loc,
			bhi_ctxt->image_size, DMA_TO_DEVICE);

	kfree(bhi_ctxt->unaligned_image_loc);

<<<<<<< HEAD
=======
		usleep(BHI_POLL_SLEEP_TIME * 1000);
	}
	dma_unmap_single(NULL, bhi_ctxt->phy_image_loc,
			bhi_ctxt->image_size, DMA_TO_DEVICE);
	kfree(bhi_ctxt->unaligned_image_loc);

	/* Fire off the state transition  thread */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	ret_val = mhi_init_state_transition(mhi_dev_ctxt,
					STATE_TRANSITION_RESET);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to start state change event\n");
	}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	return amount_copied;

bhi_copy_error:
	kfree(bhi_ctxt->unaligned_image_loc);
	return amount_copied;
}

static const struct file_operations bhi_fops = {
	.write = bhi_write,
	.open = bhi_open,
};

int bhi_probe(struct mhi_pcie_dev_info *mhi_pcie_device)
{
	struct bhi_ctxt_t *bhi_ctxt = &mhi_pcie_device->bhi_ctxt;
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 pcie_word_val = 0;
	int r;
<<<<<<< HEAD
=======
bhi_copy_error:
	return amount_copied;
}

int bhi_probe(mhi_pcie_dev_info *mhi_pcie_device)
{
	bhi_ctxt_t *bhi_ctxt = &mhi_pcie_device->bhi_ctxt;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 pcie_word_val = 0;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

	if (NULL == mhi_pcie_device || 0 == mhi_pcie_device->core.bar0_base
	    || 0 == mhi_pcie_device->core.bar0_end)
		return -EIO;

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	ret_val = alloc_chrdev_region(&bhi_ctxt->bhi_dev, 0, 1, "bhi");
	if (IS_ERR_VALUE(ret_val)) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to alloc char device %d\n",
			ret_val);
		return -EIO;
	}
	bhi_ctxt->bhi_class = class_create(THIS_MODULE, "bhi");
	if (IS_ERR(bhi_ctxt->bhi_class)) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to instantiate class %d\n",
			ret_val);
		r = (int)bhi_ctxt->bhi_class;
		goto err_class_create;
	}
	cdev_init(&bhi_ctxt->cdev, &bhi_fops);
	bhi_ctxt->cdev.owner = THIS_MODULE;
	ret_val = cdev_add(&bhi_ctxt->cdev, bhi_ctxt->bhi_dev, 1);
	bhi_ctxt->dev = device_create(bhi_ctxt->bhi_class, NULL,
					bhi_ctxt->bhi_dev, NULL,
					"bhi");
	if (IS_ERR(bhi_ctxt->dev)) {
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to add bhi cdev\n");
		r = (int)bhi_ctxt->dev;
		goto err_dev_create;
	}

	bhi_ctxt->bhi_base = mhi_pcie_device->core.bar0_base;
	pcie_word_val = mhi_reg_read(bhi_ctxt->bhi_base, BHIOFF);
	bhi_ctxt->bhi_base += pcie_word_val;

	mhi_log(MHI_MSG_INFO,
		"Successfully registered char dev. bhi base is: 0x%p.\n",
		bhi_ctxt->bhi_base);
	return 0;
err_dev_create:
	cdev_del(&bhi_ctxt->cdev);
	class_destroy(bhi_ctxt->bhi_class);
err_class_create:
	unregister_chrdev_region(MAJOR(bhi_ctxt->bhi_dev), 1);
	return r;
<<<<<<< HEAD
=======
	ret_val = misc_register(&bhi_misc_dev);
	if (0 != ret_val) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to register miscdevice with code %d\n",
			ret_val);
		return -EIO;
	}
	bhi_ctxt->bhi_base = mhi_pcie_device->core.bar0_base;
	MHI_REG_READ(bhi_ctxt->bhi_base, BHIOFF, pcie_word_val);
	bhi_ctxt->bhi_base += pcie_word_val;

	mhi_log(MHI_MSG_INFO,
		"Successfully registered misc dev. bhi base is: 0x%lx.\n",
		bhi_ctxt->bhi_base);
	return 0;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
}
