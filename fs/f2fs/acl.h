/*
 * fs/f2fs/acl.h
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * Portions of this code from linux/fs/ext2/acl.h
 *
 * Copyright (C) 2001-2003 Andreas Gruenbacher, <agruen@suse.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __F2FS_ACL_H__
#define __F2FS_ACL_H__

#include <linux/posix_acl_xattr.h>

#define F2FS_ACL_VERSION	0x0001

struct f2fs_acl_entry {
	__le16 e_tag;
	__le16 e_perm;
	__le32 e_id;
};

struct f2fs_acl_entry_short {
	__le16 e_tag;
	__le16 e_perm;
};

struct f2fs_acl_header {
	__le32 a_version;
};

#ifdef CONFIG_F2FS_FS_POSIX_ACL

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
extern struct posix_acl *f2fs_get_acl(struct inode *inode, int type);
extern int f2fs_acl_chmod(struct inode *inode);
extern int f2fs_init_acl(struct inode *inode, struct inode *dir);
#else
#define f2fs_check_acl	NULL
<<<<<<< HEAD
=======
extern struct posix_acl *f2fs_get_acl(struct inode *, int);
extern int f2fs_acl_chmod(struct inode *);
extern int f2fs_init_acl(struct inode *, struct inode *, struct page *,
							struct page *);
#else
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
#define f2fs_get_acl	NULL
#define f2fs_set_acl	NULL

static inline int f2fs_acl_chmod(struct inode *inode)
{
	return 0;
}

<<<<<<< HEAD
<<<<<<< HEAD
static inline int f2fs_init_acl(struct inode *inode, struct inode *dir)
=======
static inline int f2fs_init_acl(struct inode *inode, struct inode *dir,
				struct page *ipage, struct page *dpage)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
static inline int f2fs_init_acl(struct inode *inode, struct inode *dir)
>>>>>>> 2617302... source
{
	return 0;
}
#endif
#endif /* __F2FS_ACL_H__ */
