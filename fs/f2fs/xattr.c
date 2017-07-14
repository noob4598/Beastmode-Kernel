/*
 * fs/f2fs/xattr.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * Portions of this code from linux/fs/ext2/xattr.c
 *
 * Copyright (C) 2001-2003 Andreas Gruenbacher <agruen@suse.de>
 *
 * Fix by Harrison Xing <harrison@mountainviewdata.com>.
 * Extended attributes for symlinks and special files added per
 *  suggestion of Luka Renko <luka.renko@hermes.si>.
 * xattr consolidation Copyright (c) 2004 James Morris <jmorris@redhat.com>,
 *  Red Hat Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/rwsem.h>
#include <linux/f2fs_fs.h>
#include <linux/security.h>
#include "f2fs.h"
#include "xattr.h"

static size_t f2fs_xattr_generic_list(struct dentry *dentry, char *list,
<<<<<<< HEAD
		size_t list_size, const char *name, size_t name_len, int type)
=======
		size_t list_size, const char *name, size_t len, int type)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
{
	struct f2fs_sb_info *sbi = F2FS_SB(dentry->d_sb);
	int total_len, prefix_len = 0;
	const char *prefix = NULL;

	switch (type) {
	case F2FS_XATTR_INDEX_USER:
		if (!test_opt(sbi, XATTR_USER))
			return -EOPNOTSUPP;
		prefix = XATTR_USER_PREFIX;
		prefix_len = XATTR_USER_PREFIX_LEN;
		break;
	case F2FS_XATTR_INDEX_TRUSTED:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		prefix = XATTR_TRUSTED_PREFIX;
		prefix_len = XATTR_TRUSTED_PREFIX_LEN;
		break;
	case F2FS_XATTR_INDEX_SECURITY:
		prefix = XATTR_SECURITY_PREFIX;
		prefix_len = XATTR_SECURITY_PREFIX_LEN;
		break;
	default:
		return -EINVAL;
	}

<<<<<<< HEAD
	total_len = prefix_len + name_len + 1;
	if (list && total_len <= list_size) {
		memcpy(list, prefix, prefix_len);
		memcpy(list + prefix_len, name, name_len);
		list[prefix_len + name_len] = '\0';
=======
	total_len = prefix_len + len + 1;
	if (list && total_len <= list_size) {
		memcpy(list, prefix, prefix_len);
		memcpy(list + prefix_len, name, len);
		list[prefix_len + len] = '\0';
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	}
	return total_len;
}

static int f2fs_xattr_generic_get(struct dentry *dentry, const char *name,
		void *buffer, size_t size, int type)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dentry->d_sb);

	switch (type) {
	case F2FS_XATTR_INDEX_USER:
		if (!test_opt(sbi, XATTR_USER))
			return -EOPNOTSUPP;
		break;
	case F2FS_XATTR_INDEX_TRUSTED:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		break;
	case F2FS_XATTR_INDEX_SECURITY:
		break;
	default:
		return -EINVAL;
	}
	if (strcmp(name, "") == 0)
		return -EINVAL;
<<<<<<< HEAD
	return f2fs_getxattr(dentry->d_inode, type, name, buffer, size);
=======
	return f2fs_getxattr(d_inode(dentry), type, name, buffer, size, NULL);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
}

static int f2fs_xattr_generic_set(struct dentry *dentry, const char *name,
		const void *value, size_t size, int flags, int type)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dentry->d_sb);

	switch (type) {
	case F2FS_XATTR_INDEX_USER:
		if (!test_opt(sbi, XATTR_USER))
			return -EOPNOTSUPP;
		break;
	case F2FS_XATTR_INDEX_TRUSTED:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		break;
	case F2FS_XATTR_INDEX_SECURITY:
		break;
	default:
		return -EINVAL;
	}
	if (strcmp(name, "") == 0)
		return -EINVAL;

<<<<<<< HEAD
	return f2fs_setxattr(dentry->d_inode, type, name, value, size, NULL);
}

static size_t f2fs_xattr_advise_list(struct dentry *dentry, char *list,
		size_t list_size, const char *name, size_t name_len, int type)
=======
	return f2fs_setxattr(d_inode(dentry), type, name,
					value, size, NULL, flags);
}

static size_t f2fs_xattr_advise_list(struct dentry *dentry, char *list,
		size_t list_size, const char *name, size_t len, int type)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
{
	const char *xname = F2FS_SYSTEM_ADVISE_PREFIX;
	size_t size;

	if (type != F2FS_XATTR_INDEX_ADVISE)
		return 0;

	size = strlen(xname) + 1;
	if (list && size <= list_size)
		memcpy(list, xname, size);
	return size;
}

static int f2fs_xattr_advise_get(struct dentry *dentry, const char *name,
		void *buffer, size_t size, int type)
{
<<<<<<< HEAD
	struct inode *inode = dentry->d_inode;
=======
	struct inode *inode = d_inode(dentry);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

	if (strcmp(name, "") != 0)
		return -EINVAL;

<<<<<<< HEAD
	*((char *)buffer) = F2FS_I(inode)->i_advise;
=======
	if (buffer)
		*((char *)buffer) = F2FS_I(inode)->i_advise;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	return sizeof(char);
}

static int f2fs_xattr_advise_set(struct dentry *dentry, const char *name,
		const void *value, size_t size, int flags, int type)
{
<<<<<<< HEAD
	struct inode *inode = dentry->d_inode;
=======
	struct inode *inode = d_inode(dentry);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

	if (strcmp(name, "") != 0)
		return -EINVAL;
	if (!inode_owner_or_capable(inode))
		return -EPERM;
	if (value == NULL)
		return -EINVAL;

	F2FS_I(inode)->i_advise |= *(char *)value;
<<<<<<< HEAD
=======
	f2fs_mark_inode_dirty_sync(inode, true);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	return 0;
}

#ifdef CONFIG_F2FS_FS_SECURITY
static int f2fs_initxattrs(struct inode *inode, const struct xattr *xattr_array,
		void *page)
{
	const struct xattr *xattr;
	int err = 0;

	for (xattr = xattr_array; xattr->name != NULL; xattr++) {
		err = f2fs_setxattr(inode, F2FS_XATTR_INDEX_SECURITY,
				xattr->name, xattr->value,
<<<<<<< HEAD
				xattr->value_len, (struct page *)page);
=======
				xattr->value_len, (struct page *)page, 0);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		if (err < 0)
			break;
	}
	return err;
}

int f2fs_init_security(struct inode *inode, struct inode *dir,
				const struct qstr *qstr, struct page *ipage)
{
	return security_inode_init_security(inode, dir, qstr,
				&f2fs_initxattrs, ipage);
}
#endif

const struct xattr_handler f2fs_xattr_user_handler = {
	.prefix	= XATTR_USER_PREFIX,
	.flags	= F2FS_XATTR_INDEX_USER,
	.list	= f2fs_xattr_generic_list,
	.get	= f2fs_xattr_generic_get,
	.set	= f2fs_xattr_generic_set,
};

const struct xattr_handler f2fs_xattr_trusted_handler = {
	.prefix	= XATTR_TRUSTED_PREFIX,
	.flags	= F2FS_XATTR_INDEX_TRUSTED,
	.list	= f2fs_xattr_generic_list,
	.get	= f2fs_xattr_generic_get,
	.set	= f2fs_xattr_generic_set,
};

const struct xattr_handler f2fs_xattr_advise_handler = {
	.prefix = F2FS_SYSTEM_ADVISE_PREFIX,
	.flags	= F2FS_XATTR_INDEX_ADVISE,
	.list   = f2fs_xattr_advise_list,
	.get    = f2fs_xattr_advise_get,
	.set    = f2fs_xattr_advise_set,
};

const struct xattr_handler f2fs_xattr_security_handler = {
	.prefix	= XATTR_SECURITY_PREFIX,
	.flags	= F2FS_XATTR_INDEX_SECURITY,
	.list	= f2fs_xattr_generic_list,
	.get	= f2fs_xattr_generic_get,
	.set	= f2fs_xattr_generic_set,
};

static const struct xattr_handler *f2fs_xattr_handler_map[] = {
	[F2FS_XATTR_INDEX_USER] = &f2fs_xattr_user_handler,
#ifdef CONFIG_F2FS_FS_POSIX_ACL
	[F2FS_XATTR_INDEX_POSIX_ACL_ACCESS] = &f2fs_xattr_acl_access_handler,
	[F2FS_XATTR_INDEX_POSIX_ACL_DEFAULT] = &f2fs_xattr_acl_default_handler,
#endif
	[F2FS_XATTR_INDEX_TRUSTED] = &f2fs_xattr_trusted_handler,
#ifdef CONFIG_F2FS_FS_SECURITY
	[F2FS_XATTR_INDEX_SECURITY] = &f2fs_xattr_security_handler,
#endif
	[F2FS_XATTR_INDEX_ADVISE] = &f2fs_xattr_advise_handler,
};

const struct xattr_handler *f2fs_xattr_handlers[] = {
	&f2fs_xattr_user_handler,
#ifdef CONFIG_F2FS_FS_POSIX_ACL
	&f2fs_xattr_acl_access_handler,
	&f2fs_xattr_acl_default_handler,
#endif
	&f2fs_xattr_trusted_handler,
#ifdef CONFIG_F2FS_FS_SECURITY
	&f2fs_xattr_security_handler,
#endif
	&f2fs_xattr_advise_handler,
	NULL,
};

<<<<<<< HEAD
static inline const struct xattr_handler *f2fs_xattr_handler(int name_index)
{
	const struct xattr_handler *handler = NULL;

	if (name_index > 0 && name_index < ARRAY_SIZE(f2fs_xattr_handler_map))
		handler = f2fs_xattr_handler_map[name_index];
	return handler;
}

int f2fs_getxattr(struct inode *inode, int name_index, const char *name,
		void *buffer, size_t buffer_size)
{
	struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);
	struct f2fs_inode_info *fi = F2FS_I(inode);
	struct f2fs_xattr_entry *entry;
	struct page *page;
	void *base_addr;
	int error = 0, found = 0;
	size_t value_len, name_len;

	if (name == NULL)
		return -EINVAL;
	name_len = strlen(name);

	if (!fi->i_xattr_nid)
		return -ENODATA;

	page = get_node_page(sbi, fi->i_xattr_nid);
	if (IS_ERR(page))
		return PTR_ERR(page);
	base_addr = page_address(page);

	list_for_each_xattr(entry, base_addr) {
		if (entry->e_name_index != name_index)
			continue;
		if (entry->e_name_len != name_len)
			continue;
		if (!memcmp(entry->e_name, name, name_len)) {
			found = 1;
			break;
		}
	}
	if (!found) {
		error = -ENODATA;
		goto cleanup;
	}

	value_len = le16_to_cpu(entry->e_value_size);

	if (buffer && value_len > buffer_size) {
		error = -ERANGE;
		goto cleanup;
=======
static inline const struct xattr_handler *f2fs_xattr_handler(int index)
{
	const struct xattr_handler *handler = NULL;

	if (index > 0 && index < ARRAY_SIZE(f2fs_xattr_handler_map))
		handler = f2fs_xattr_handler_map[index];
	return handler;
}

static struct f2fs_xattr_entry *__find_xattr(void *base_addr, int index,
					size_t len, const char *name)
{
	struct f2fs_xattr_entry *entry;

	list_for_each_xattr(entry, base_addr) {
		if (entry->e_name_index != index)
			continue;
		if (entry->e_name_len != len)
			continue;
		if (!memcmp(entry->e_name, name, len))
			break;
	}
	return entry;
}

static struct f2fs_xattr_entry *__find_inline_xattr(void *base_addr,
					void **last_addr, int index,
					size_t len, const char *name)
{
	struct f2fs_xattr_entry *entry;
	unsigned int inline_size = F2FS_INLINE_XATTR_ADDRS << 2;

	list_for_each_xattr(entry, base_addr) {
		if ((void *)entry + sizeof(__u32) > base_addr + inline_size ||
			(void *)XATTR_NEXT_ENTRY(entry) + sizeof(__u32) >
			base_addr + inline_size) {
			*last_addr = entry;
			return NULL;
		}
		if (entry->e_name_index != index)
			continue;
		if (entry->e_name_len != len)
			continue;
		if (!memcmp(entry->e_name, name, len))
			break;
	}
	return entry;
}

static int lookup_all_xattrs(struct inode *inode, struct page *ipage,
				unsigned int index, unsigned int len,
				const char *name, struct f2fs_xattr_entry **xe,
				void **base_addr)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	void *cur_addr, *txattr_addr, *last_addr = NULL;
	nid_t xnid = F2FS_I(inode)->i_xattr_nid;
	unsigned int size = xnid ? VALID_XATTR_BLOCK_SIZE : 0;
	unsigned int inline_size = 0;
	int err = 0;

	inline_size = inline_xattr_size(inode);

	if (!size && !inline_size)
		return -ENODATA;

	txattr_addr = kzalloc(inline_size + size + sizeof(__u32),
							GFP_F2FS_ZERO);
	if (!txattr_addr)
		return -ENOMEM;

	/* read from inline xattr */
	if (inline_size) {
		struct page *page = NULL;
		void *inline_addr;

		if (ipage) {
			inline_addr = inline_xattr_addr(ipage);
		} else {
			page = get_node_page(sbi, inode->i_ino);
			if (IS_ERR(page)) {
				err = PTR_ERR(page);
				goto out;
			}
			inline_addr = inline_xattr_addr(page);
		}
		memcpy(txattr_addr, inline_addr, inline_size);
		f2fs_put_page(page, 1);

		*xe = __find_inline_xattr(txattr_addr, &last_addr,
						index, len, name);
		if (*xe)
			goto check;
	}

	/* read from xattr node block */
	if (xnid) {
		struct page *xpage;
		void *xattr_addr;

		/* The inode already has an extended attribute block. */
		xpage = get_node_page(sbi, xnid);
		if (IS_ERR(xpage)) {
			err = PTR_ERR(xpage);
			goto out;
		}

		xattr_addr = page_address(xpage);
		memcpy(txattr_addr + inline_size, xattr_addr, size);
		f2fs_put_page(xpage, 1);
	}

	if (last_addr)
		cur_addr = XATTR_HDR(last_addr) - 1;
	else
		cur_addr = txattr_addr;

	*xe = __find_xattr(cur_addr, index, len, name);
check:
	if (IS_XATTR_LAST_ENTRY(*xe)) {
		err = -ENODATA;
		goto out;
	}

	*base_addr = txattr_addr;
	return 0;
out:
	kzfree(txattr_addr);
	return err;
}

static int read_all_xattrs(struct inode *inode, struct page *ipage,
							void **base_addr)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct f2fs_xattr_header *header;
	size_t size = PAGE_SIZE, inline_size = 0;
	void *txattr_addr;
	int err;

	inline_size = inline_xattr_size(inode);

	txattr_addr = kzalloc(inline_size + size, GFP_F2FS_ZERO);
	if (!txattr_addr)
		return -ENOMEM;

	/* read from inline xattr */
	if (inline_size) {
		struct page *page = NULL;
		void *inline_addr;

		if (ipage) {
			inline_addr = inline_xattr_addr(ipage);
		} else {
			page = get_node_page(sbi, inode->i_ino);
			if (IS_ERR(page)) {
				err = PTR_ERR(page);
				goto fail;
			}
			inline_addr = inline_xattr_addr(page);
		}
		memcpy(txattr_addr, inline_addr, inline_size);
		f2fs_put_page(page, 1);
	}

	/* read from xattr node block */
	if (F2FS_I(inode)->i_xattr_nid) {
		struct page *xpage;
		void *xattr_addr;

		/* The inode already has an extended attribute block. */
		xpage = get_node_page(sbi, F2FS_I(inode)->i_xattr_nid);
		if (IS_ERR(xpage)) {
			err = PTR_ERR(xpage);
			goto fail;
		}

		xattr_addr = page_address(xpage);
		memcpy(txattr_addr + inline_size, xattr_addr, PAGE_SIZE);
		f2fs_put_page(xpage, 1);
	}

	header = XATTR_HDR(txattr_addr);

	/* never been allocated xattrs */
	if (le32_to_cpu(header->h_magic) != F2FS_XATTR_MAGIC) {
		header->h_magic = cpu_to_le32(F2FS_XATTR_MAGIC);
		header->h_refcount = cpu_to_le32(1);
	}
	*base_addr = txattr_addr;
	return 0;
fail:
	kzfree(txattr_addr);
	return err;
}

static inline int write_all_xattrs(struct inode *inode, __u32 hsize,
				void *txattr_addr, struct page *ipage)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	size_t inline_size = 0;
	void *xattr_addr;
	struct page *xpage;
	nid_t new_nid = 0;
	int err;

	inline_size = inline_xattr_size(inode);

	if (hsize > inline_size && !F2FS_I(inode)->i_xattr_nid)
		if (!alloc_nid(sbi, &new_nid))
			return -ENOSPC;

	/* write to inline xattr */
	if (inline_size) {
		struct page *page = NULL;
		void *inline_addr;

		if (ipage) {
			inline_addr = inline_xattr_addr(ipage);
			f2fs_wait_on_page_writeback(ipage, NODE, true);
			set_page_dirty(ipage);
		} else {
			page = get_node_page(sbi, inode->i_ino);
			if (IS_ERR(page)) {
				alloc_nid_failed(sbi, new_nid);
				return PTR_ERR(page);
			}
			inline_addr = inline_xattr_addr(page);
			f2fs_wait_on_page_writeback(page, NODE, true);
		}
		memcpy(inline_addr, txattr_addr, inline_size);
		f2fs_put_page(page, 1);

		/* no need to use xattr node block */
		if (hsize <= inline_size) {
			err = truncate_xattr_node(inode, ipage);
			alloc_nid_failed(sbi, new_nid);
			return err;
		}
	}

	/* write to xattr node block */
	if (F2FS_I(inode)->i_xattr_nid) {
		xpage = get_node_page(sbi, F2FS_I(inode)->i_xattr_nid);
		if (IS_ERR(xpage)) {
			alloc_nid_failed(sbi, new_nid);
			return PTR_ERR(xpage);
		}
		f2fs_bug_on(sbi, new_nid);
		f2fs_wait_on_page_writeback(xpage, NODE, true);
	} else {
		struct dnode_of_data dn;
		set_new_dnode(&dn, inode, NULL, NULL, new_nid);
		xpage = new_node_page(&dn, XATTR_NODE_OFFSET, ipage);
		if (IS_ERR(xpage)) {
			alloc_nid_failed(sbi, new_nid);
			return PTR_ERR(xpage);
		}
		alloc_nid_done(sbi, new_nid);
	}

	xattr_addr = page_address(xpage);
	memcpy(xattr_addr, txattr_addr + inline_size, MAX_XATTR_BLOCK_SIZE);
	set_page_dirty(xpage);
	f2fs_put_page(xpage, 1);

	return 0;
}

int f2fs_getxattr(struct inode *inode, int index, const char *name,
		void *buffer, size_t buffer_size, struct page *ipage)
{
	struct f2fs_xattr_entry *entry = NULL;
	int error = 0;
	unsigned int size, len;
	void *base_addr = NULL;

	if (name == NULL)
		return -EINVAL;

	len = strlen(name);
	if (len > F2FS_NAME_LEN)
		return -ERANGE;

	error = lookup_all_xattrs(inode, ipage, index, len, name,
				&entry, &base_addr);
	if (error)
		return error;

	size = le16_to_cpu(entry->e_value_size);

	if (buffer && size > buffer_size) {
		error = -ERANGE;
		goto out;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	}

	if (buffer) {
		char *pval = entry->e_name + entry->e_name_len;
<<<<<<< HEAD
		memcpy(buffer, pval, value_len);
	}
	error = value_len;

cleanup:
	f2fs_put_page(page, 1);
=======
		memcpy(buffer, pval, size);
	}
	error = size;
out:
	kzfree(base_addr);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	return error;
}

ssize_t f2fs_listxattr(struct dentry *dentry, char *buffer, size_t buffer_size)
{
<<<<<<< HEAD
	struct inode *inode = dentry->d_inode;
	struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);
	struct f2fs_inode_info *fi = F2FS_I(inode);
	struct f2fs_xattr_entry *entry;
	struct page *page;
=======
	struct inode *inode = d_inode(dentry);
	struct f2fs_xattr_entry *entry;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	void *base_addr;
	int error = 0;
	size_t rest = buffer_size;

<<<<<<< HEAD
	if (!fi->i_xattr_nid)
		return 0;

	page = get_node_page(sbi, fi->i_xattr_nid);
	if (IS_ERR(page))
		return PTR_ERR(page);
	base_addr = page_address(page);
=======
	error = read_all_xattrs(inode, NULL, &base_addr);
	if (error)
		return error;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

	list_for_each_xattr(entry, base_addr) {
		const struct xattr_handler *handler =
			f2fs_xattr_handler(entry->e_name_index);
		size_t size;

		if (!handler)
			continue;

		size = handler->list(dentry, buffer, rest, entry->e_name,
				entry->e_name_len, handler->flags);
		if (buffer && size > rest) {
			error = -ERANGE;
			goto cleanup;
		}

		if (buffer)
			buffer += size;
		rest -= size;
	}
	error = buffer_size - rest;
cleanup:
<<<<<<< HEAD
	f2fs_put_page(page, 1);
	return error;
}

int f2fs_setxattr(struct inode *inode, int name_index, const char *name,
			const void *value, size_t value_len, struct page *ipage)
{
	struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);
	struct f2fs_inode_info *fi = F2FS_I(inode);
	struct f2fs_xattr_header *header = NULL;
	struct f2fs_xattr_entry *here, *last;
	struct page *page;
	void *base_addr;
	int error, found, free, newsize;
	size_t name_len;
	char *pval;
	int ilock;
=======
	kzfree(base_addr);
	return error;
}

static bool f2fs_xattr_value_same(struct f2fs_xattr_entry *entry,
					const void *value, size_t size)
{
	void *pval = entry->e_name + entry->e_name_len;

	return (le16_to_cpu(entry->e_value_size) == size) &&
					!memcmp(pval, value, size);
}

static int __f2fs_setxattr(struct inode *inode, int index,
			const char *name, const void *value, size_t size,
			struct page *ipage, int flags)
{
	struct f2fs_xattr_entry *here, *last;
	void *base_addr;
	int found, newsize;
	size_t len;
	__u32 new_hsize;
	int error = 0;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

	if (name == NULL)
		return -EINVAL;

	if (value == NULL)
<<<<<<< HEAD
		value_len = 0;

	name_len = strlen(name);

	if (name_len > F2FS_NAME_LEN || value_len > MAX_VALUE_LEN)
		return -ERANGE;

	f2fs_balance_fs(sbi);

	ilock = mutex_lock_op(sbi);

	if (!fi->i_xattr_nid) {
		/* Allocate new attribute block */
		struct dnode_of_data dn;

		if (!alloc_nid(sbi, &fi->i_xattr_nid)) {
			error = -ENOSPC;
			goto exit;
		}
		set_new_dnode(&dn, inode, NULL, NULL, fi->i_xattr_nid);
		mark_inode_dirty(inode);

		page = new_node_page(&dn, XATTR_NODE_OFFSET, ipage);
		if (IS_ERR(page)) {
			alloc_nid_failed(sbi, fi->i_xattr_nid);
			fi->i_xattr_nid = 0;
			error = PTR_ERR(page);
			goto exit;
		}

		alloc_nid_done(sbi, fi->i_xattr_nid);
		base_addr = page_address(page);
		header = XATTR_HDR(base_addr);
		header->h_magic = cpu_to_le32(F2FS_XATTR_MAGIC);
		header->h_refcount = cpu_to_le32(1);
	} else {
		/* The inode already has an extended attribute block. */
		page = get_node_page(sbi, fi->i_xattr_nid);
		if (IS_ERR(page)) {
			error = PTR_ERR(page);
			goto exit;
		}

		base_addr = page_address(page);
		header = XATTR_HDR(base_addr);
	}

	if (le32_to_cpu(header->h_magic) != F2FS_XATTR_MAGIC) {
		error = -EIO;
		goto cleanup;
	}

	/* find entry with wanted name. */
	found = 0;
	list_for_each_xattr(here, base_addr) {
		if (here->e_name_index != name_index)
			continue;
		if (here->e_name_len != name_len)
			continue;
		if (!memcmp(here->e_name, name, name_len)) {
			found = 1;
			break;
		}
	}

	last = here;

	while (!IS_XATTR_LAST_ENTRY(last))
		last = XATTR_NEXT_ENTRY(last);

	newsize = XATTR_ALIGN(sizeof(struct f2fs_xattr_entry) +
			name_len + value_len);

	/* 1. Check space */
	if (value) {
		/* If value is NULL, it is remove operation.
		 * In case of update operation, we caculate free.
		 */
		free = MIN_OFFSET - ((char *)last - (char *)header);
		if (found)
			free = free - ENTRY_SIZE(here);

		if (free < newsize) {
			error = -ENOSPC;
			goto cleanup;
=======
		size = 0;

	len = strlen(name);

	if (len > F2FS_NAME_LEN)
		return -ERANGE;

	if (size > MAX_VALUE_LEN(inode))
		return -E2BIG;

	error = read_all_xattrs(inode, ipage, &base_addr);
	if (error)
		return error;

	/* find entry with wanted name. */
	here = __find_xattr(base_addr, index, len, name);

	found = IS_XATTR_LAST_ENTRY(here) ? 0 : 1;

	if (found) {
		if ((flags & XATTR_CREATE)) {
			error = -EEXIST;
			goto exit;
		}

		if (f2fs_xattr_value_same(here, value, size))
			goto exit;
	} else if ((flags & XATTR_REPLACE)) {
		error = -ENODATA;
		goto exit;
	}

	last = here;
	while (!IS_XATTR_LAST_ENTRY(last))
		last = XATTR_NEXT_ENTRY(last);

	newsize = XATTR_ALIGN(sizeof(struct f2fs_xattr_entry) + len + size);

	/* 1. Check space */
	if (value) {
		int free;
		/*
		 * If value is NULL, it is remove operation.
		 * In case of update operation, we calculate free.
		 */
		free = MIN_OFFSET(inode) - ((char *)last - (char *)base_addr);
		if (found)
			free = free + ENTRY_SIZE(here);

		if (unlikely(free < newsize)) {
			error = -E2BIG;
			goto exit;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		}
	}

	/* 2. Remove old entry */
	if (found) {
<<<<<<< HEAD
		/* If entry is found, remove old entry.
=======
		/*
		 * If entry is found, remove old entry.
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		 * If not found, remove operation is not needed.
		 */
		struct f2fs_xattr_entry *next = XATTR_NEXT_ENTRY(here);
		int oldsize = ENTRY_SIZE(here);

		memmove(here, next, (char *)last - (char *)next);
		last = (struct f2fs_xattr_entry *)((char *)last - oldsize);
		memset(last, 0, oldsize);
	}

<<<<<<< HEAD
	/* 3. Write new entry */
	if (value) {
		/* Before we come here, old entry is removed.
		 * We just write new entry. */
		memset(last, 0, newsize);
		last->e_name_index = name_index;
		last->e_name_len = name_len;
		memcpy(last->e_name, name, name_len);
		pval = last->e_name + name_len;
		memcpy(pval, value, value_len);
		last->e_value_size = cpu_to_le16(value_len);
	}

	set_page_dirty(page);
	f2fs_put_page(page, 1);

	if (is_inode_flag_set(fi, FI_ACL_MODE)) {
		inode->i_mode = fi->i_acl_mode;
		inode->i_ctime = CURRENT_TIME;
		clear_inode_flag(fi, FI_ACL_MODE);
	}
	if (ipage)
		update_inode(inode, ipage);
	else
		update_inode_page(inode);
	mutex_unlock_op(sbi, ilock);

	return 0;
cleanup:
	f2fs_put_page(page, 1);
exit:
	mutex_unlock_op(sbi, ilock);
	return error;
}
=======
	new_hsize = (char *)last - (char *)base_addr;

	/* 3. Write new entry */
	if (value) {
		char *pval;
		/*
		 * Before we come here, old entry is removed.
		 * We just write new entry.
		 */
		last->e_name_index = index;
		last->e_name_len = len;
		memcpy(last->e_name, name, len);
		pval = last->e_name + len;
		memcpy(pval, value, size);
		last->e_value_size = cpu_to_le16(size);
		new_hsize += newsize;
	}

	error = write_all_xattrs(inode, new_hsize, base_addr, ipage);
	if (error)
		goto exit;

	if (is_inode_flag_set(inode, FI_ACL_MODE)) {
		inode->i_mode = F2FS_I(inode)->i_acl_mode;
		inode->i_ctime = current_time(inode);
		clear_inode_flag(inode, FI_ACL_MODE);
	}
	if (index == F2FS_XATTR_INDEX_ENCRYPTION &&
			!strcmp(name, F2FS_XATTR_NAME_ENCRYPTION_CONTEXT))
		f2fs_set_encrypted_inode(inode);
	f2fs_mark_inode_dirty_sync(inode, true);
	if (!error && S_ISDIR(inode->i_mode))
		set_sbi_flag(F2FS_I_SB(inode), SBI_NEED_CP);
exit:
	kzfree(base_addr);
	return error;
}

int f2fs_setxattr(struct inode *inode, int index, const char *name,
				const void *value, size_t size,
				struct page *ipage, int flags)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	int err;

	/* this case is only from init_inode_metadata */
	if (ipage)
		return __f2fs_setxattr(inode, index, name, value,
						size, ipage, flags);
	f2fs_balance_fs(sbi, true);

	f2fs_lock_op(sbi);
	/* protect xattr_ver */
	down_write(&F2FS_I(inode)->i_sem);
	err = __f2fs_setxattr(inode, index, name, value, size, ipage, flags);
	up_write(&F2FS_I(inode)->i_sem);
	f2fs_unlock_op(sbi);

	f2fs_update_time(sbi, REQ_TIME);
	return err;
}
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
