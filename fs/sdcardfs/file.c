/*
 * fs/sdcardfs/file.c
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
#ifdef CONFIG_SDCARD_FS_FADV_NOACTIVE
#include <linux/backing-dev.h>
#endif

static ssize_t sdcardfs_read(struct file *file, char __user *buf,
			   size_t count, loff_t *ppos)
{
	int err;
	struct file *lower_file;
	struct dentry *dentry = file->f_path.dentry;
#ifdef CONFIG_SDCARD_FS_FADV_NOACTIVE
	struct backing_dev_info *bdi;
#endif

	lower_file = sdcardfs_lower_file(file);

#ifdef CONFIG_SDCARD_FS_FADV_NOACTIVE
	if (file->f_mode & FMODE_NOACTIVE) {
		if (!(lower_file->f_mode & FMODE_NOACTIVE)) {
			bdi = lower_file->f_mapping->backing_dev_info;
			lower_file->f_ra.ra_pages = bdi->ra_pages * 2;
			spin_lock(&lower_file->f_lock);
			lower_file->f_mode |= FMODE_NOACTIVE;
			spin_unlock(&lower_file->f_lock);
		}
	}
#endif

	err = vfs_read(lower_file, buf, count, ppos);
	/* update our inode atime upon a successful lower read */
	if (err >= 0)
		fsstack_copy_attr_atime(dentry->d_inode,
					lower_file->f_path.dentry->d_inode);

	return err;
}

static ssize_t sdcardfs_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	int err = 0;
	struct file *lower_file;
	struct dentry *dentry = file->f_path.dentry;

	/* check disk space */
	if (!check_min_free_space(dentry, count, 0)) {
<<<<<<< HEAD
<<<<<<< HEAD
		printk(KERN_INFO "No minimum free space.\n");
=======
		pr_err("No minimum free space.\n");
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		printk(KERN_INFO "No minimum free space.\n");
>>>>>>> 2617302... source
		return -ENOSPC;
	}

	lower_file = sdcardfs_lower_file(file);
	err = vfs_write(lower_file, buf, count, ppos);
	/* update our inode times+sizes upon a successful lower write */
	if (err >= 0) {
		fsstack_copy_inode_size(dentry->d_inode,
					lower_file->f_path.dentry->d_inode);
		fsstack_copy_attr_times(dentry->d_inode,
					lower_file->f_path.dentry->d_inode);
	}

	return err;
}

<<<<<<< HEAD
<<<<<<< HEAD
static int sdcardfs_readdir(struct file *file, void *dirent, filldir_t filldir)
=======
static int sdcardfs_readdir(struct file *file, struct dir_context *ctx)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
static int sdcardfs_readdir(struct file *file, void *dirent, filldir_t filldir)
>>>>>>> 2617302... source
{
	int err = 0;
	struct file *lower_file = NULL;
	struct dentry *dentry = file->f_path.dentry;

	lower_file = sdcardfs_lower_file(file);

	lower_file->f_pos = file->f_pos;
<<<<<<< HEAD
<<<<<<< HEAD
	err = vfs_readdir(lower_file, filldir, dirent);
=======
	err = iterate_dir(lower_file, ctx);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	err = vfs_readdir(lower_file, filldir, dirent);
>>>>>>> 2617302... source
	file->f_pos = lower_file->f_pos;
	if (err >= 0)		/* copy the atime */
		fsstack_copy_attr_atime(dentry->d_inode,
					lower_file->f_path.dentry->d_inode);
	return err;
}

static long sdcardfs_unlocked_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	long err = -ENOTTY;
	struct file *lower_file;

	lower_file = sdcardfs_lower_file(file);

	/* XXX: use vfs_ioctl if/when VFS exports it */
	if (!lower_file || !lower_file->f_op)
		goto out;
	if (lower_file->f_op->unlocked_ioctl)
		err = lower_file->f_op->unlocked_ioctl(lower_file, cmd, arg);

<<<<<<< HEAD
<<<<<<< HEAD
=======
	/* some ioctls can change inode attributes (EXT2_IOC_SETFLAGS) */
	if (!err)
		sdcardfs_copy_and_fix_attrs(file_inode(file),
				      file_inode(lower_file));
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
out:
	return err;
}

#ifdef CONFIG_COMPAT
static long sdcardfs_compat_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	long err = -ENOTTY;
	struct file *lower_file;

	lower_file = sdcardfs_lower_file(file);

	/* XXX: use vfs_ioctl if/when VFS exports it */
	if (!lower_file || !lower_file->f_op)
		goto out;
	if (lower_file->f_op->compat_ioctl)
		err = lower_file->f_op->compat_ioctl(lower_file, cmd, arg);

out:
	return err;
}
#endif

static int sdcardfs_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err = 0;
	bool willwrite;
	struct file *lower_file;
	const struct vm_operations_struct *saved_vm_ops = NULL;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	/* this might be deferred to mmap's writepage */
	willwrite = ((vma->vm_flags | VM_SHARED | VM_WRITE) == vma->vm_flags);

	/*
	 * File systems which do not implement ->writepage may use
	 * generic_file_readonly_mmap as their ->mmap op.  If you call
	 * generic_file_readonly_mmap with VM_WRITE, you'd get an -EINVAL.
	 * But we cannot call the lower ->mmap op, so we can't tell that
	 * writeable mappings won't work.  Therefore, our only choice is to
	 * check if the lower file system supports the ->writepage, and if
	 * not, return EINVAL (the same error that
	 * generic_file_readonly_mmap returns in that case).
	 */
	lower_file = sdcardfs_lower_file(file);
	if (willwrite && !lower_file->f_mapping->a_ops->writepage) {
		err = -EINVAL;
<<<<<<< HEAD
<<<<<<< HEAD
		printk(KERN_ERR "sdcardfs: lower file system does not "
		       "support writeable mmap\n");
=======
		pr_err("sdcardfs: lower file system does not support writeable mmap\n");
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		printk(KERN_ERR "sdcardfs: lower file system does not "
		       "support writeable mmap\n");
>>>>>>> 2617302... source
		goto out;
	}

	/*
	 * find and save lower vm_ops.
	 *
	 * XXX: the VFS should have a cleaner way of finding the lower vm_ops
	 */
	if (!SDCARDFS_F(file)->lower_vm_ops) {
		err = lower_file->f_op->mmap(lower_file, vma);
		if (err) {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
			printk(KERN_ERR "sdcardfs: lower mmap failed %d\n", err);
			goto out;
		}
		saved_vm_ops = vma->vm_ops; /* save: came from lower ->mmap */
		err = do_munmap(current->mm, vma->vm_start,
				vma->vm_end - vma->vm_start);
		if (err) {
			printk(KERN_ERR "sdcardfs: do_munmap failed %d\n", err);
			goto out;
		}
<<<<<<< HEAD
=======
			pr_err("sdcardfs: lower mmap failed %d\n", err);
			goto out;
		}
		saved_vm_ops = vma->vm_ops; /* save: came from lower ->mmap */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	}

	/*
	 * Next 3 lines are all I need from generic_file_mmap.  I definitely
	 * don't want its test for ->readpage which returns -ENOEXEC.
	 */
	file_accessed(file);
	vma->vm_ops = &sdcardfs_vm_ops;

	file->f_mapping->a_ops = &sdcardfs_aops; /* set our aops */
	if (!SDCARDFS_F(file)->lower_vm_ops) /* save for our ->fault */
		SDCARDFS_F(file)->lower_vm_ops = saved_vm_ops;
<<<<<<< HEAD
<<<<<<< HEAD
=======
	vma->vm_private_data = file;
	get_file(lower_file);
	vma->vm_file = lower_file;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

out:
	return err;
}

static int sdcardfs_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct file *lower_file = NULL;
	struct path lower_path;
	struct dentry *dentry = file->f_path.dentry;
	struct dentry *parent = dget_parent(dentry);
<<<<<<< HEAD
<<<<<<< HEAD
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(dentry->d_sb); 
=======
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(dentry->d_sb);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(dentry->d_sb); 
>>>>>>> 2617302... source
	const struct cred *saved_cred = NULL;

	/* don't open unhashed/deleted files */
	if (d_unhashed(dentry)) {
		err = -ENOENT;
		goto out_err;
	}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	
	if(!check_caller_access_to_name(parent->d_inode, dentry->d_name.name)) {
		printk(KERN_INFO "%s: need to check the caller's gid in packages.list\n" 
                         "	dentry: %s, task:%s\n",
						 __func__, dentry->d_name.name, current->comm);
<<<<<<< HEAD
=======

	if (!check_caller_access_to_name(parent->d_inode, &dentry->d_name)) {
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
		err = -EACCES;
		goto out_err;
	}

	/* save current_cred and override it */
<<<<<<< HEAD
<<<<<<< HEAD
	OVERRIDE_CRED(sbi, saved_cred);
=======
	OVERRIDE_CRED(sbi, saved_cred, SDCARDFS_I(inode));
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	OVERRIDE_CRED(sbi, saved_cred);
>>>>>>> 2617302... source

	file->f_mode |= FMODE_NONMAPPABLE;
	file->private_data =
		kzalloc(sizeof(struct sdcardfs_file_info), GFP_KERNEL);
	if (!SDCARDFS_F(file)) {
		err = -ENOMEM;
		goto out_revert_cred;
	}

	/* open lower object and link sdcardfs's file struct to lower's */
<<<<<<< HEAD
<<<<<<< HEAD
	sdcardfs_copy_lower_path(file->f_path.dentry, &lower_path);
	lower_file = dentry_open(&lower_path, file->f_flags, current_cred());
=======
	sdcardfs_get_lower_path(file->f_path.dentry, &lower_path);
	lower_file = dentry_open(&lower_path, file->f_flags, current_cred());
	path_put(&lower_path);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	sdcardfs_copy_lower_path(file->f_path.dentry, &lower_path);
	lower_file = dentry_open(&lower_path, file->f_flags, current_cred());
>>>>>>> 2617302... source
	if (IS_ERR(lower_file)) {
		err = PTR_ERR(lower_file);
		lower_file = sdcardfs_lower_file(file);
		if (lower_file) {
			sdcardfs_set_lower_file(file, NULL);
			fput(lower_file); /* fput calls dput for lower_dentry */
		}
	} else {
		sdcardfs_set_lower_file(file, lower_file);
	}

	if (err)
		kfree(SDCARDFS_F(file));
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	else {
		mutex_lock(&inode->i_mutex);
		sdcardfs_copy_inode_attr(inode, sdcardfs_lower_inode(inode));
		fix_derived_permission(inode);
		mutex_unlock(&inode->i_mutex);
	}
<<<<<<< HEAD
=======
	else
		sdcardfs_copy_and_fix_attrs(inode, sdcardfs_lower_inode(inode));
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

out_revert_cred:
	REVERT_CRED(saved_cred);
out_err:
	dput(parent);
	return err;
}

static int sdcardfs_flush(struct file *file, fl_owner_t id)
{
	int err = 0;
	struct file *lower_file = NULL;

	lower_file = sdcardfs_lower_file(file);
	if (lower_file && lower_file->f_op && lower_file->f_op->flush)
		err = lower_file->f_op->flush(lower_file, id);

	return err;
}

/* release all lower object references & free the file info structure */
static int sdcardfs_file_release(struct inode *inode, struct file *file)
{
	struct file *lower_file;

	lower_file = sdcardfs_lower_file(file);
	if (lower_file) {
		sdcardfs_set_lower_file(file, NULL);
		fput(lower_file);
	}

	kfree(SDCARDFS_F(file));
	return 0;
}

<<<<<<< HEAD
<<<<<<< HEAD
static int
sdcardfs_fsync(struct file *file, loff_t start, loff_t end, int datasync)
=======
static int sdcardfs_fsync(struct file *file, loff_t start, loff_t end,
			int datasync)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
static int
sdcardfs_fsync(struct file *file, loff_t start, loff_t end, int datasync)
>>>>>>> 2617302... source
{
	int err;
	struct file *lower_file;
	struct path lower_path;
	struct dentry *dentry = file->f_path.dentry;

<<<<<<< HEAD
<<<<<<< HEAD
=======
	err = generic_file_fsync(file, start, end, datasync);
	if (err)
		goto out;

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	lower_file = sdcardfs_lower_file(file);
	sdcardfs_get_lower_path(dentry, &lower_path);
	err = vfs_fsync_range(lower_file, start, end, datasync);
	sdcardfs_put_lower_path(dentry, &lower_path);
<<<<<<< HEAD
<<<<<<< HEAD

=======
out:
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======

>>>>>>> 2617302... source
	return err;
}

static int sdcardfs_fasync(int fd, struct file *file, int flag)
{
	int err = 0;
	struct file *lower_file = NULL;

	lower_file = sdcardfs_lower_file(file);
	if (lower_file->f_op && lower_file->f_op->fasync)
		err = lower_file->f_op->fasync(fd, lower_file, flag);

	return err;
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
/*
 * Sdcardfs cannot use generic_file_llseek as ->llseek, because it would
 * only set the offset of the upper file.  So we have to implement our
 * own method to set both the upper and lower file offsets
 * consistently.
 */
static loff_t sdcardfs_file_llseek(struct file *file, loff_t offset, int whence)
{
	int err;
	struct file *lower_file;

	err = generic_file_llseek(file, offset, whence);
	if (err < 0)
		goto out;

	lower_file = sdcardfs_lower_file(file);
	err = generic_file_llseek(lower_file, offset, whence);

out:
	return err;
}


>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
static struct file *sdcardfs_get_lower_file(struct file *f)
{
	return sdcardfs_lower_file(f);
}

const struct file_operations sdcardfs_main_fops = {
	.llseek		= generic_file_llseek,
	.read		= sdcardfs_read,
	.write		= sdcardfs_write,
	.unlocked_ioctl	= sdcardfs_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= sdcardfs_compat_ioctl,
#endif
	.mmap		= sdcardfs_mmap,
	.open		= sdcardfs_open,
	.flush		= sdcardfs_flush,
	.release	= sdcardfs_file_release,
	.fsync		= sdcardfs_fsync,
	.fasync		= sdcardfs_fasync,
	.get_lower_file = sdcardfs_get_lower_file,
};

/* trimmed directory options */
const struct file_operations sdcardfs_dir_fops = {
<<<<<<< HEAD
<<<<<<< HEAD
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.readdir	= sdcardfs_readdir,
=======
	.llseek		= sdcardfs_file_llseek,
	.read		= generic_read_dir,
	.iterate	= sdcardfs_readdir,
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.readdir	= sdcardfs_readdir,
>>>>>>> 2617302... source
	.unlocked_ioctl	= sdcardfs_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= sdcardfs_compat_ioctl,
#endif
	.open		= sdcardfs_open,
	.release	= sdcardfs_file_release,
	.flush		= sdcardfs_flush,
	.fsync		= sdcardfs_fsync,
	.fasync		= sdcardfs_fasync,
	.get_lower_file = sdcardfs_get_lower_file,
};
