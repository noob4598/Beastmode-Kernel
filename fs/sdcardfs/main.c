/*
 * fs/sdcardfs/main.c
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
#include "version.h"
#include <linux/module.h>
#include <linux/types.h>
#include <linux/parser.h>
#include "../internal.h"

enum {
	Opt_low_uid,
	Opt_low_gid,
	Opt_gid,
	Opt_userid,
	Opt_debug,
	Opt_lower_fs,
	Opt_reserved_mb,
	Opt_mask,
	Opt_multi_user,
	Opt_label,
	Opt_type,
<<<<<<< HEAD
=======
#include <linux/module.h>
#include <linux/types.h>
#include <linux/parser.h>

enum {
	Opt_fsuid,
	Opt_fsgid,
	Opt_gid,
	Opt_debug,
	Opt_mask,
	Opt_multiuser,
	Opt_userid,
	Opt_reserved_mb,
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	Opt_err,
};

static const match_table_t sdcardfs_tokens = {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	{Opt_low_uid, "low_uid=%u"},
	{Opt_low_gid, "low_gid=%u"},
	{Opt_gid, "gid=%u"},
	{Opt_userid, "userid=%u"},
	{Opt_debug, "debug"},
	{Opt_lower_fs, "lower_fs=%s"},
	{Opt_reserved_mb, "reserved_mb=%u"},
	{Opt_mask, "mask=%o"},
	{Opt_multi_user, "multi_user"},
	{Opt_label, "label=%s"},
	{Opt_type, "type=%s"},
	{Opt_err, NULL}
};

static int parse_options(struct super_block *sb, char *options, int silent, 
				int *debug, struct sdcardfs_mount_options *opts)
<<<<<<< HEAD
=======
	{Opt_fsuid, "fsuid=%u"},
	{Opt_fsgid, "fsgid=%u"},
	{Opt_gid, "gid=%u"},
	{Opt_debug, "debug"},
	{Opt_mask, "mask=%u"},
	{Opt_userid, "userid=%d"},
	{Opt_multiuser, "multiuser"},
	{Opt_reserved_mb, "reserved_mb=%u"},
	{Opt_err, NULL}
};

static int parse_options(struct super_block *sb, char *options, int silent,
				int *debug, struct sdcardfs_vfsmount_options *vfsopts,
				struct sdcardfs_mount_options *opts)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
{
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	char *string_option;
	char *label;

	/* by default, we use AID_MEDIA_RW as low_uid, low_gid */
	opts->fs_low_uid = AID_MEDIA_RW;
	opts->fs_low_gid = AID_MEDIA_RW;
	/* by default, userid is 0, gid is AID_EVERYBODY */
	opts->gid = AID_EVERYBODY;
	opts->userid = 0;
	/* by default, we use LOWER_FS_EXT4 as lower fs type */
	opts->lower_fs = LOWER_FS_EXT4;
	/* by default, 0MB is reserved */
	opts->reserved_mb = 0;
	/* by default, mask is 0 */
	opts->mask = 0;
	/* by default, multi_user is false */
	opts->multi_user = false;
	opts->label = NULL;
	opts->type = TYPE_NONE;
<<<<<<< HEAD
=======

	/* by default, we use AID_MEDIA_RW as uid, gid */
	opts->fs_low_uid = AID_MEDIA_RW;
	opts->fs_low_gid = AID_MEDIA_RW;
	vfsopts->mask = 0;
	opts->multiuser = false;
	opts->fs_user_id = 0;
	vfsopts->gid = 0;
	/* by default, 0MB is reserved */
	opts->reserved_mb = 0;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

	*debug = 0;

	if (!options)
		return 0;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
		if (!*p)
			continue;

		token = match_token(p, sdcardfs_tokens, args);
<<<<<<< HEAD
<<<<<<< HEAD
		
=======

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		
>>>>>>> 2617302... source
		switch (token) {
		case Opt_debug:
			*debug = 1;
			break;
<<<<<<< HEAD
<<<<<<< HEAD
		case Opt_low_uid:
=======
		case Opt_fsuid:
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		case Opt_low_uid:
>>>>>>> 2617302... source
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_uid = option;
			break;
<<<<<<< HEAD
<<<<<<< HEAD
		case Opt_low_gid:
=======
		case Opt_fsgid:
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		case Opt_low_gid:
>>>>>>> 2617302... source
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_gid = option;
			break;
		case Opt_gid:
			if (match_int(&args[0], &option))
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
				goto invalid_option;
			opts->gid = option;
			break;
		case Opt_userid:
			if (match_int(&args[0], &option))
				goto invalid_option;
			opts->userid = option;
			break;
		case Opt_lower_fs:
			string_option = match_strdup(&args[0]);
			if (!string_option)
				return -ENOMEM;
			if (!strcmp("ext4", string_option)) {
				opts->lower_fs = LOWER_FS_EXT4;
			} else if (!strcmp("fat", string_option)) {
				opts->lower_fs = LOWER_FS_FAT;
			} else {
				kfree(string_option);
				goto invalid_option;
			}
			kfree(string_option);
<<<<<<< HEAD
=======
				return 0;
			vfsopts->gid = option;
			break;
		case Opt_userid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_user_id = option;
			break;
		case Opt_mask:
			if (match_int(&args[0], &option))
				return 0;
			vfsopts->mask = option;
			break;
		case Opt_multiuser:
			opts->multiuser = true;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
			break;
		case Opt_reserved_mb:
			if (match_int(&args[0], &option))
				return 0;
			opts->reserved_mb = option;
			break;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
		case Opt_mask:
			if (match_octal(&args[0], &option))
				goto invalid_option;
			opts->mask = option;
			break;
		case Opt_multi_user:
			opts->multi_user = true;
			break;
		case Opt_label:
			label = match_strdup(&args[0]);
			if (!label)
				return -ENOMEM;
			opts->label = label;
			break;
		case Opt_type:
			string_option = match_strdup(&args[0]);
			if (!string_option)
				return -ENOMEM;
			if (!strcmp("default", string_option)) {
				opts->type = TYPE_DEFAULT;
			} else if (!strcmp("read", string_option)) {
				opts->type = TYPE_READ;
			} else if (!strcmp("write", string_option)) {
				opts->type = TYPE_WRITE;
			} else {
				kfree(string_option);
				goto invalid_option;
			}
			kfree(string_option);
			break;
		/* unknown option */
		default:
invalid_option:
			if (!silent) {
				printk( KERN_ERR "Unrecognized mount option \"%s\" "
						"or missing value", p);
			}
<<<<<<< HEAD
=======
		/* unknown option */
		default:
			if (!silent)
				pr_err("Unrecognized mount option \"%s\" or missing value", p);
			return -EINVAL;
		}
	}

	if (*debug) {
		pr_info("sdcardfs : options - debug:%d\n", *debug);
		pr_info("sdcardfs : options - uid:%d\n",
							opts->fs_low_uid);
		pr_info("sdcardfs : options - gid:%d\n",
							opts->fs_low_gid);
	}

	return 0;
}

int parse_options_remount(struct super_block *sb, char *options, int silent,
				struct sdcardfs_vfsmount_options *vfsopts)
{
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;
	int debug;

	if (!options)
		return 0;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;

		if (!*p)
			continue;

		token = match_token(p, sdcardfs_tokens, args);

		switch (token) {
		case Opt_debug:
			debug = 1;
			break;
		case Opt_gid:
			if (match_int(&args[0], &option))
				return 0;
			vfsopts->gid = option;

			break;
		case Opt_mask:
			if (match_int(&args[0], &option))
				return 0;
			vfsopts->mask = option;
			break;
		case Opt_multiuser:
		case Opt_userid:
		case Opt_fsuid:
		case Opt_fsgid:
		case Opt_reserved_mb:
			pr_warn("Option \"%s\" can't be changed during remount\n", p);
			break;
		/* unknown option */
		default:
			if (!silent)
				pr_err("Unrecognized mount option \"%s\" or missing value", p);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
			return -EINVAL;
		}
	}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	if (*debug) {
		printk( KERN_INFO "sdcardfs : options - debug:%d\n", *debug);
		printk( KERN_INFO "sdcardfs : options - uid:%d\n", 
							opts->fs_low_uid);
		printk( KERN_INFO "sdcardfs : options - gid:%d\n", 
							opts->fs_low_gid);
<<<<<<< HEAD
=======
	if (debug) {
		pr_info("sdcardfs : options - debug:%d\n", debug);
		pr_info("sdcardfs : options - gid:%d\n", vfsopts->gid);
		pr_info("sdcardfs : options - mask:%d\n", vfsopts->mask);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	}

	return 0;
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
#if 0
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
/*
 * our custom d_alloc_root work-alike
 *
 * we can't use d_alloc_root if we want to use our own interpose function
 * unchanged, so we simply call our own "fake" d_alloc_root
 */
static struct dentry *sdcardfs_d_alloc_root(struct super_block *sb)
{
	struct dentry *ret = NULL;

	if (sb) {
		static const struct qstr name = {
			.name = "/",
			.len = 1
		};

<<<<<<< HEAD
<<<<<<< HEAD
		ret = __d_alloc(sb, &name);
		if (ret) {
			d_set_d_op(ret, &sdcardfs_ci_dops);
=======
		ret = d_alloc(NULL, &name);
		if (ret) {
			d_set_d_op(ret, &sdcardfs_ci_dops);
			ret->d_sb = sb;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		ret = __d_alloc(sb, &name);
		if (ret) {
			d_set_d_op(ret, &sdcardfs_ci_dops);
>>>>>>> 2617302... source
			ret->d_parent = ret;
		}
	}
	return ret;
}
<<<<<<< HEAD
<<<<<<< HEAD
=======
#endif

DEFINE_MUTEX(sdcardfs_super_list_lock);
EXPORT_SYMBOL_GPL(sdcardfs_super_list_lock);
LIST_HEAD(sdcardfs_super_list);
EXPORT_SYMBOL_GPL(sdcardfs_super_list);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

/*
 * There is no need to lock the sdcardfs_super_info's rwsem as there is no
 * way anyone can have a reference to the superblock at this point in time.
 */
<<<<<<< HEAD
<<<<<<< HEAD
static int sdcardfs_read_super(struct super_block *sb, const char *dev_name, 
						void *raw_data, int silent)
=======
static int sdcardfs_read_super(struct vfsmount *mnt, struct super_block *sb,
		const char *dev_name, void *raw_data, int silent)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
static int sdcardfs_read_super(struct super_block *sb, const char *dev_name, 
						void *raw_data, int silent)
>>>>>>> 2617302... source
{
	int err = 0;
	int debug;
	struct super_block *lower_sb;
	struct path lower_path;
	struct sdcardfs_sb_info *sb_info;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	void *pkgl_id;

	printk(KERN_INFO "sdcardfs: version %s\n", SDCARDFS_VERSION);

	if (!dev_name) {
		printk(KERN_ERR
		       "sdcardfs: read_super: missing dev_name argument\n");
<<<<<<< HEAD
=======
	struct sdcardfs_vfsmount_options *mnt_opt = mnt->data;
	struct inode *inode;

	pr_info("sdcardfs version 2.0\n");

	if (!dev_name) {
		pr_err("sdcardfs: read_super: missing dev_name argument\n");
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
		err = -EINVAL;
		goto out;
	}

<<<<<<< HEAD
<<<<<<< HEAD
	printk(KERN_INFO "sdcardfs: dev_name -> %s\n", dev_name);
	printk(KERN_INFO "sdcardfs: options -> %s\n", (char *)raw_data);
=======
	pr_info("sdcardfs: dev_name -> %s\n", dev_name);
	pr_info("sdcardfs: options -> %s\n", (char *)raw_data);
	pr_info("sdcardfs: mnt -> %p\n", mnt);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	printk(KERN_INFO "sdcardfs: dev_name -> %s\n", dev_name);
	printk(KERN_INFO "sdcardfs: options -> %s\n", (char *)raw_data);
>>>>>>> 2617302... source

	/* parse lower path */
	err = kern_path(dev_name, LOOKUP_FOLLOW | LOOKUP_DIRECTORY,
			&lower_path);
	if (err) {
<<<<<<< HEAD
<<<<<<< HEAD
		printk(KERN_ERR	"sdcardfs: error accessing "
		       "lower directory '%s'\n", dev_name);
=======
		pr_err("sdcardfs: error accessing lower directory '%s'\n", dev_name);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		printk(KERN_ERR	"sdcardfs: error accessing "
		       "lower directory '%s'\n", dev_name);
>>>>>>> 2617302... source
		goto out;
	}

	/* allocate superblock private data */
	sb->s_fs_info = kzalloc(sizeof(struct sdcardfs_sb_info), GFP_KERNEL);
	if (!SDCARDFS_SB(sb)) {
<<<<<<< HEAD
<<<<<<< HEAD
		printk(KERN_CRIT "sdcardfs: read_super: out of memory\n");
=======
		pr_crit("sdcardfs: read_super: out of memory\n");
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		printk(KERN_CRIT "sdcardfs: read_super: out of memory\n");
>>>>>>> 2617302... source
		err = -ENOMEM;
		goto out_free;
	}

	sb_info = sb->s_fs_info;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source

	/* parse options */
	err = parse_options(sb, raw_data, silent, &debug, &sb_info->options);
	if (err) {
		printk(KERN_ERR	"sdcardfs: invalid options or out of memory\n");
		goto out_freesbi;
	}

	pkgl_id = packagelist_create();
	if(IS_ERR(pkgl_id))
		goto out_freesbi;
	else
		sb_info->pkgl_id = pkgl_id;

<<<<<<< HEAD
=======
	/* parse options */
	err = parse_options(sb, raw_data, silent, &debug, mnt_opt, &sb_info->options);
	if (err) {
		pr_err("sdcardfs: invalid options\n");
		goto out_freesbi;
	}

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	/* set the lower superblock field of upper superblock */
	lower_sb = lower_path.dentry->d_sb;
	atomic_inc(&lower_sb->s_active);
	sdcardfs_set_lower_super(sb, lower_sb);

<<<<<<< HEAD
<<<<<<< HEAD
=======
	sb->s_stack_depth = lower_sb->s_stack_depth + 1;
	if (sb->s_stack_depth > FILESYSTEM_MAX_STACK_DEPTH) {
		pr_err("sdcardfs: maximum fs stacking depth exceeded\n");
		err = -EINVAL;
		goto out_sput;
	}

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	/* inherit maxbytes from lower file system */
	sb->s_maxbytes = lower_sb->s_maxbytes;

	/*
	 * Our c/m/atime granularity is 1 ns because we may stack on file
	 * systems whose granularity is as good.
	 */
	sb->s_time_gran = 1;

	sb->s_magic = SDCARDFS_SUPER_MAGIC;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	if (sb_info->options.type != TYPE_NONE)
		sb->s_op = &sdcardfs_multimount_sops;
	else
		sb->s_op = &sdcardfs_sops;

	/* see comment next to the definition of sdcardfs_d_alloc_root */
	sb->s_root = sdcardfs_d_alloc_root(sb);
	if (!sb->s_root) {
		err = -ENOMEM;
		goto out_sput;
	}
<<<<<<< HEAD
=======
	sb->s_op = &sdcardfs_sops;

	/* get a new inode and allocate our root dentry */
	inode = sdcardfs_iget(sb, lower_path.dentry->d_inode, 0);
	if (IS_ERR(inode)) {
		err = PTR_ERR(inode);
		goto out_sput;
	}
	sb->s_root = d_make_root(inode);
	if (!sb->s_root) {
		err = -ENOMEM;
		goto out_iput;
	}
	d_set_d_op(sb->s_root, &sdcardfs_ci_dops);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

	/* link the upper and lower dentries */
	sb->s_root->d_fsdata = NULL;
	err = new_dentry_private_data(sb->s_root);
	if (err)
		goto out_freeroot;

	/* set the lower dentries for s_root */
	sdcardfs_set_lower_path(sb->s_root, &lower_path);

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	/* call interpose to create the upper level inode */
	err = sdcardfs_interpose(sb->s_root, sb, &lower_path);
	if (!err) {
		/* setup permission policy */
		if(sb_info->options.multi_user){
			setup_derived_state(sb->s_root->d_inode, 
				PERM_PRE_ROOT, sb_info->options.userid, AID_ROOT, sb_info->options.gid, false);
			sb_info->obbpath_s = kzalloc(PATH_MAX, GFP_KERNEL);
			snprintf(sb_info->obbpath_s, PATH_MAX, "%s/obb", dev_name);
			err =  prepare_dir(sb_info->obbpath_s, 
						sb_info->options.fs_low_uid,
						sb_info->options.fs_low_gid, 00775);
		} else {
			setup_derived_state(sb->s_root->d_inode,
				PERM_ROOT, sb_info->options.userid, AID_ROOT, sb_info->options.gid, false);
			sb_info->obbpath_s = kzalloc(PATH_MAX, GFP_KERNEL);
			snprintf(sb_info->obbpath_s, PATH_MAX, "%s/Android/obb", dev_name);
		}
		fix_derived_permission(sb->s_root->d_inode);

		sb_info->devpath = kzalloc(PATH_MAX, GFP_KERNEL);
		if(sb_info->devpath && dev_name)
			strncpy(sb_info->devpath, dev_name, strlen(dev_name));
		
		if (!silent && !err)
			printk(KERN_INFO "sdcardfs: mounted on top of %s type %s\n",
						dev_name, lower_sb->s_type->name);
		goto out;
	}
	/* else error: fall through */

	free_dentry_private_data(sb->s_root);
out_freeroot:
	dput(sb->s_root);
out_sput:
	/* drop refs we took earlier */
	atomic_dec(&lower_sb->s_active);
	packagelist_destroy(sb_info->pkgl_id);
<<<<<<< HEAD
=======
	/*
	 * No need to call interpose because we already have a positive
	 * dentry, which was instantiated by d_make_root.  Just need to
	 * d_rehash it.
	 */
	d_rehash(sb->s_root);

	/* setup permission policy */
	sb_info->obbpath_s = kzalloc(PATH_MAX, GFP_KERNEL);
	mutex_lock(&sdcardfs_super_list_lock);
	if (sb_info->options.multiuser) {
		setup_derived_state(sb->s_root->d_inode, PERM_PRE_ROOT,
					sb_info->options.fs_user_id, AID_ROOT,
					false, sb->s_root->d_inode);
		snprintf(sb_info->obbpath_s, PATH_MAX, "%s/obb", dev_name);
	} else {
		setup_derived_state(sb->s_root->d_inode, PERM_ROOT,
					sb_info->options.fs_user_id, AID_ROOT,
					false, sb->s_root->d_inode);
		snprintf(sb_info->obbpath_s, PATH_MAX, "%s/Android/obb", dev_name);
	}
	fixup_tmp_permissions(sb->s_root->d_inode);
	sb_info->sb = sb;
	list_add(&sb_info->list, &sdcardfs_super_list);
	mutex_unlock(&sdcardfs_super_list_lock);

	if (!silent)
		pr_info("sdcardfs: mounted on top of %s type %s\n",
				dev_name, lower_sb->s_type->name);
	goto out; /* all is well */

	/* no longer needed: free_dentry_private_data(sb->s_root); */
out_freeroot:
	dput(sb->s_root);
out_iput:
	iput(inode);
out_sput:
	/* drop refs we took earlier */
	atomic_dec(&lower_sb->s_active);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
out_freesbi:
	kfree(SDCARDFS_SB(sb));
	sb->s_fs_info = NULL;
out_free:
	path_put(&lower_path);

out:
	return err;
}

/* A feature which supports mount_nodev() with options */
<<<<<<< HEAD
<<<<<<< HEAD
static struct dentry *mount_nodev_with_options(struct file_system_type *fs_type,
        int flags, const char *dev_name, void *data,
        int (*fill_super)(struct super_block *, const char *, void *, int))
=======
static struct dentry *mount_nodev_with_options(struct vfsmount *mnt,
			struct file_system_type *fs_type, int flags,
			const char *dev_name, void *data,
			int (*fill_super)(struct vfsmount *, struct super_block *,
						const char *, void *, int))
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
static struct dentry *mount_nodev_with_options(struct file_system_type *fs_type,
        int flags, const char *dev_name, void *data,
        int (*fill_super)(struct super_block *, const char *, void *, int))
>>>>>>> 2617302... source

{
	int error;
	struct super_block *s = sget(fs_type, NULL, set_anon_super, flags, NULL);

	if (IS_ERR(s))
		return ERR_CAST(s);

	s->s_flags = flags;

<<<<<<< HEAD
<<<<<<< HEAD
	error = fill_super(s, dev_name, data, flags & MS_SILENT ? 1 : 0);
=======
	error = fill_super(mnt, s, dev_name, data, flags & MS_SILENT ? 1 : 0);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	error = fill_super(s, dev_name, data, flags & MS_SILENT ? 1 : 0);
>>>>>>> 2617302... source
	if (error) {
		deactivate_locked_super(s);
		return ERR_PTR(error);
	}
	s->s_flags |= MS_ACTIVE;
	return dget(s->s_root);
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
struct dentry *sdcardfs_mount(struct file_system_type *fs_type, int flags,
			    const char *dev_name, void *raw_data)
{
	/* 
	 * dev_name is a lower_path_name,
	 * raw_data is a option string.
	 */
	return mount_nodev_with_options(fs_type, flags, dev_name,
					raw_data, sdcardfs_read_super);
<<<<<<< HEAD
=======
static struct dentry *sdcardfs_mount(struct vfsmount *mnt,
		struct file_system_type *fs_type, int flags,
			    const char *dev_name, void *raw_data)
{
	/*
	 * dev_name is a lower_path_name,
	 * raw_data is a option string.
	 */
	return mount_nodev_with_options(mnt, fs_type, flags, dev_name,
						raw_data, sdcardfs_read_super);
}

static struct dentry *sdcardfs_mount_wrn(struct file_system_type *fs_type,
		    int flags, const char *dev_name, void *raw_data)
{
	WARN(1, "sdcardfs does not support mount. Use mount2.\n");
	return ERR_PTR(-EINVAL);
}

void *sdcardfs_alloc_mnt_data(void)
{
	return kmalloc(sizeof(struct sdcardfs_vfsmount_options), GFP_KERNEL);
}

void sdcardfs_kill_sb(struct super_block *sb)
{
	struct sdcardfs_sb_info *sbi;

	if (sb->s_magic == SDCARDFS_SUPER_MAGIC) {
		sbi = SDCARDFS_SB(sb);
		mutex_lock(&sdcardfs_super_list_lock);
		list_del(&sbi->list);
		mutex_unlock(&sdcardfs_super_list_lock);
	}
	generic_shutdown_super(sb);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
}

static struct file_system_type sdcardfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= SDCARDFS_NAME,
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	.mount		= sdcardfs_mount,
	.kill_sb	= generic_shutdown_super,
	.fs_flags	= 0,
};
<<<<<<< HEAD
=======
	.mount		= sdcardfs_mount_wrn,
	.mount2		= sdcardfs_mount,
	.alloc_mnt_data = sdcardfs_alloc_mnt_data,
	.kill_sb	= sdcardfs_kill_sb,
	.fs_flags	= 0,
};
MODULE_ALIAS_FS(SDCARDFS_NAME);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

static int __init init_sdcardfs_fs(void)
{
	int err;

	pr_info("Registering sdcardfs " SDCARDFS_VERSION "\n");

	err = sdcardfs_init_inode_cache();
	if (err)
		goto out;
	err = sdcardfs_init_dentry_cache();
	if (err)
		goto out;
	err = packagelist_init();
	if (err)
		goto out;
	err = register_filesystem(&sdcardfs_fs_type);
out:
	if (err) {
		sdcardfs_destroy_inode_cache();
		sdcardfs_destroy_dentry_cache();
		packagelist_exit();
	}
	return err;
}

static void __exit exit_sdcardfs_fs(void)
{
	sdcardfs_destroy_inode_cache();
	sdcardfs_destroy_dentry_cache();
	packagelist_exit();
	unregister_filesystem(&sdcardfs_fs_type);
	pr_info("Completed sdcardfs module unload\n");
}

<<<<<<< HEAD
<<<<<<< HEAD
MODULE_AUTHOR("Woojoong Lee, Daeho Jeong, Kitae Lee, Yeongjin Gil"
        " System Memory Lab., Samsung Electronics");
=======
/* Original wrapfs authors */
MODULE_AUTHOR("Erez Zadok, Filesystems and Storage Lab, Stony Brook University (http://www.fsl.cs.sunysb.edu/)");

/* Original sdcardfs authors */
MODULE_AUTHOR("Woojoong Lee, Daeho Jeong, Kitae Lee, Yeongjin Gil System Memory Lab., Samsung Electronics");

/* Current maintainer */
MODULE_AUTHOR("Daniel Rosenberg, Google");
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
MODULE_AUTHOR("Woojoong Lee, Daeho Jeong, Kitae Lee, Yeongjin Gil"
        " System Memory Lab., Samsung Electronics");
>>>>>>> 2617302... source
MODULE_DESCRIPTION("Sdcardfs " SDCARDFS_VERSION);
MODULE_LICENSE("GPL");

module_init(init_sdcardfs_fs);
module_exit(exit_sdcardfs_fs);
