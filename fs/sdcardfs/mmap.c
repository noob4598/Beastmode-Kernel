/*
 * fs/sdcardfs/mmap.c
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun, 
 *               Sunghwan Yun, Sungjong Seo
 *                      
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by 
<<<<<<< HEAD
=======
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun,
 *               Sunghwan Yun, Sungjong Seo
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
 *
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009     Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

#include "sdcardfs.h"

static int sdcardfs_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int err;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	struct file *file, *lower_file;
	const struct vm_operations_struct *lower_vm_ops;
	struct vm_area_struct lower_vma;

	memcpy(&lower_vma, vma, sizeof(struct vm_area_struct));
	file = lower_vma.vm_file;
	lower_vm_ops = SDCARDFS_F(file)->lower_vm_ops;
	BUG_ON(!lower_vm_ops);

	lower_file = sdcardfs_lower_file(file);
	/*
	 * XXX: vm_ops->fault may be called in parallel.  Because we have to
	 * resort to temporarily changing the vma->vm_file to point to the
	 * lower file, a concurrent invocation of sdcardfs_fault could see a
	 * different value.  In this workaround, we keep a different copy of
	 * the vma structure in our stack, so we never expose a different
	 * value of the vma->vm_file called to us, even temporarily.  A
	 * better fix would be to change the calling semantics of ->fault to
	 * take an explicit file pointer.
	 */
	lower_vma.vm_file = lower_file;
	err = lower_vm_ops->fault(&lower_vma, vmf);
<<<<<<< HEAD
=======
	struct file *file;
	const struct vm_operations_struct *lower_vm_ops;

	file = (struct file *)vma->vm_private_data;
	lower_vm_ops = SDCARDFS_F(file)->lower_vm_ops;
	BUG_ON(!lower_vm_ops);

	err = lower_vm_ops->fault(vma, vmf);
	return err;
}

static void sdcardfs_vm_open(struct vm_area_struct *vma)
{
	struct file *file = (struct file *)vma->vm_private_data;

	get_file(file);
}

static void sdcardfs_vm_close(struct vm_area_struct *vma)
{
	struct file *file = (struct file *)vma->vm_private_data;

	fput(file);
}

static int sdcardfs_page_mkwrite(struct vm_area_struct *vma,
			       struct vm_fault *vmf)
{
	int err = 0;
	struct file *file;
	const struct vm_operations_struct *lower_vm_ops;

	file = (struct file *)vma->vm_private_data;
	lower_vm_ops = SDCARDFS_F(file)->lower_vm_ops;
	BUG_ON(!lower_vm_ops);
	if (!lower_vm_ops->page_mkwrite)
		goto out;

	err = lower_vm_ops->page_mkwrite(vma, vmf);
out:
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	return err;
}

static ssize_t sdcardfs_direct_IO(int rw, struct kiocb *iocb,
			      const struct iovec *iov, loff_t offset,
			      unsigned long nr_segs)
{
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	/* 
	 * This function returns zero on purpose in order to support direct IO.
	 * __dentry_open checks a_ops->direct_IO and returns EINVAL if it is null.
	 * 
	 * However, this function won't be called by certain file operations 
	 * including generic fs functions.  * reads and writes are delivered to 
	 * the lower file systems and the direct IOs will be handled by them. 
	 * 
	 * NOTE: exceptionally, on the recent kernels (since Linux 3.8.x), 
	 * swap_writepage invokes this function directly. 
	 */ 
	printk(KERN_INFO "%s, operation is not supported\n", __func__);
	return 0;
}

/*
 * XXX: the default address_space_ops for sdcardfs is empty.  We cannot set
 * our inode->i_mapping->a_ops to NULL because too many code paths expect
 * the a_ops vector to be non-NULL.
 */
const struct address_space_operations sdcardfs_aops = {
	/* empty on purpose */
<<<<<<< HEAD
=======
	/*
	 * This function should never be called directly.  We need it
	 * to exist, to get past a check in open_check_o_direct(),
	 * which is called from do_last().
	 */
	return -EINVAL;
}

const struct address_space_operations sdcardfs_aops = {
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	.direct_IO	= sdcardfs_direct_IO,
};

const struct vm_operations_struct sdcardfs_vm_ops = {
	.fault		= sdcardfs_fault,
<<<<<<< HEAD
<<<<<<< HEAD
=======
	.page_mkwrite	= sdcardfs_page_mkwrite,
	.open		= sdcardfs_vm_open,
	.close		= sdcardfs_vm_close,
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
};
