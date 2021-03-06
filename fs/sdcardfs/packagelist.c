/*
 * fs/sdcardfs/packagelist.c
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
#include <linux/hashtable.h>
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/inotify.h>
#include <linux/delay.h>

#define STRING_BUF_SIZE		(512)

struct hashtable_entry {
        struct hlist_node hlist;
        void *key;
	int value;
};

struct packagelist_data {
	DECLARE_HASHTABLE(package_to_appid,8);
	struct mutex hashtable_lock;
	struct task_struct *thread_id;
	char read_buf[STRING_BUF_SIZE];
	char event_buf[STRING_BUF_SIZE];
	char app_name_buf[STRING_BUF_SIZE];
	char gids_buf[STRING_BUF_SIZE];
};

static struct kmem_cache *hashtable_entry_cachep;

/* Path to system-provided mapping of package name to appIds */
static const char* const kpackageslist_file = "/data/system/packages.list";
/* Supplementary groups to execute with */
static const gid_t kgroups[1] = { AID_PACKAGE_INFO };

static unsigned int str_hash(void *key) {
	int i;
	unsigned int h = strlen(key);
	char *data = (char *)key;

	for (i = 0; i < strlen(key); i++) {
		h = h * 31 + *data;
		data++;
	}
	return h;
}

appid_t get_appid(void *pkgl_id, const char *app_name)
{
	struct packagelist_data *pkgl_dat = (struct packagelist_data *)pkgl_id;
	struct hashtable_entry *hash_cur;
	unsigned int hash = str_hash((void *)app_name);
	appid_t ret_id;

	//printk(KERN_INFO "sdcardfs: %s: %s, %u\n", __func__, (char *)app_name, hash);
	mutex_lock(&pkgl_dat->hashtable_lock);
	hash_for_each_possible(pkgl_dat->package_to_appid, hash_cur, hlist, hash) {
		//printk(KERN_INFO "sdcardfs: %s: %s\n", __func__, (char *)hash_cur->key);
		if (!strcasecmp(app_name, hash_cur->key)) {
			ret_id = (appid_t)hash_cur->value;
			mutex_unlock(&pkgl_dat->hashtable_lock);
			//printk(KERN_INFO "=> app_id: %d\n", (int)ret_id);
			return ret_id;
		}
	}
	mutex_unlock(&pkgl_dat->hashtable_lock);
	//printk(KERN_INFO "=> app_id: %d\n", 0);
	return 0;
}

/* Kernel has already enforced everything we returned through
 * derive_permissions_locked(), so this is used to lock down access
 * even further, such as enforcing that apps hold sdcard_rw. */
int check_caller_access_to_name(struct inode *parent_node, const char* name) {
	/* Always block security-sensitive files at root */
	if (parent_node && SDCARDFS_I(parent_node)->perm == PERM_ROOT) {
		if (!strcasecmp(name, "autorun.inf")
			|| !strcasecmp(name, ".android_secure")
			|| !strcasecmp(name, "android_secure")) {
<<<<<<< HEAD
=======
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/radix-tree.h>
#include <linux/dcache.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/configfs.h>

struct hashtable_entry {
	struct hlist_node hlist;
	struct hlist_node dlist; /* for deletion cleanup */
	struct qstr key;
	atomic_t value;
};

static DEFINE_HASHTABLE(package_to_appid, 8);
static DEFINE_HASHTABLE(package_to_userid, 8);
static DEFINE_HASHTABLE(ext_to_groupid, 8);


static struct kmem_cache *hashtable_entry_cachep;

static unsigned int full_name_case_hash(const unsigned char *name, unsigned int len)
{
	unsigned long hash = init_name_hash();

	while (len--)
		hash = partial_name_hash(tolower(*name++), hash);
	return end_name_hash(hash);
}

static inline void qstr_init(struct qstr *q, const char *name)
{
	q->name = name;
	q->len = strlen(q->name);
	q->hash = full_name_case_hash(q->name, q->len);
}

static inline int qstr_copy(const struct qstr *src, struct qstr *dest)
{
	dest->name = kstrdup(src->name, GFP_KERNEL);
	dest->hash_len = src->hash_len;
	return !!dest->name;
}


static appid_t __get_appid(const struct qstr *key)
{
	struct hashtable_entry *hash_cur;
	unsigned int hash = key->hash;
	appid_t ret_id;

	rcu_read_lock();
	hash_for_each_possible_rcu(package_to_appid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key)) {
			ret_id = atomic_read(&hash_cur->value);
			rcu_read_unlock();
			return ret_id;
		}
	}
	rcu_read_unlock();
	return 0;
}

appid_t get_appid(const char *key)
{
	struct qstr q;

	qstr_init(&q, key);
	return __get_appid(&q);
}

static appid_t __get_ext_gid(const struct qstr *key)
{
	struct hashtable_entry *hash_cur;
	unsigned int hash = key->hash;
	appid_t ret_id;

	rcu_read_lock();
	hash_for_each_possible_rcu(ext_to_groupid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key)) {
			ret_id = atomic_read(&hash_cur->value);
			rcu_read_unlock();
			return ret_id;
		}
	}
	rcu_read_unlock();
	return 0;
}

appid_t get_ext_gid(const char *key)
{
	struct qstr q;

	qstr_init(&q, key);
	return __get_ext_gid(&q);
}

static appid_t __is_excluded(const struct qstr *app_name, userid_t user)
{
	struct hashtable_entry *hash_cur;
	unsigned int hash = app_name->hash;

	rcu_read_lock();
	hash_for_each_possible_rcu(package_to_userid, hash_cur, hlist, hash) {
		if (atomic_read(&hash_cur->value) == user &&
				qstr_case_eq(app_name, &hash_cur->key)) {
			rcu_read_unlock();
			return 1;
		}
	}
	rcu_read_unlock();
	return 0;
}

appid_t is_excluded(const char *key, userid_t user)
{
	struct qstr q;

	qstr_init(&q, key);
	return __is_excluded(&q, user);
}

/* Kernel has already enforced everything we returned through
 * derive_permissions_locked(), so this is used to lock down access
 * even further, such as enforcing that apps hold sdcard_rw.
 */
int check_caller_access_to_name(struct inode *parent_node, const struct qstr *name)
{
	struct qstr q_autorun = QSTR_LITERAL("autorun.inf");
	struct qstr q__android_secure = QSTR_LITERAL(".android_secure");
	struct qstr q_android_secure = QSTR_LITERAL("android_secure");

	/* Always block security-sensitive files at root */
	if (parent_node && SDCARDFS_I(parent_node)->perm == PERM_ROOT) {
		if (qstr_case_eq(name, &q_autorun)
			|| qstr_case_eq(name, &q__android_secure)
			|| qstr_case_eq(name, &q_android_secure)) {
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
			return 0;
		}
	}

	/* Root always has access; access for any other UIDs should always
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
	 * be controlled through packages.list. */
	if (current_fsuid() == 0) {
		return 1;
	}
<<<<<<< HEAD
=======
	 * be controlled through packages.list.
	 */
	if (current_fsuid() == 0)
		return 1;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

	/* No extra permissions to enforce */
	return 1;
}

/* This function is used when file opening. The open flags must be
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
 * checked before calling check_caller_access_to_name() */  
int open_flags_to_access_mode(int open_flags) {
	if((open_flags & O_ACCMODE) == O_RDONLY) {
		return 0; /* R_OK */
	} else if ((open_flags & O_ACCMODE) == O_WRONLY) {
		return 1; /* W_OK */
	} else {
		/* Probably O_RDRW, but treat as default to be safe */
		return 1; /* R_OK | W_OK */
	}
}

static int insert_str_to_int(struct packagelist_data *pkgl_dat, void *key, int value) {
	struct hashtable_entry *hash_cur;
	struct hashtable_entry *new_entry;
	unsigned int hash = str_hash(key);

	//printk(KERN_INFO "sdcardfs: %s: %s: %d, %u\n", __func__, (char *)key, value, hash);
	hash_for_each_possible(pkgl_dat->package_to_appid, hash_cur, hlist, hash) {
		if (!strcasecmp(key, hash_cur->key)) {
			hash_cur->value = value;
			return 0;
		}
	}
	new_entry = kmem_cache_alloc(hashtable_entry_cachep, GFP_KERNEL);
	if (!new_entry)
		return -ENOMEM;
	new_entry->key = kstrdup(key, GFP_KERNEL);
	new_entry->value = value;
	hash_add(pkgl_dat->package_to_appid, &new_entry->hlist, hash);
	return 0;
}

static void remove_str_to_int(struct hashtable_entry *h_entry) {
	//printk(KERN_INFO "sdcardfs: %s: %s: %d\n", __func__, (char *)h_entry->key, h_entry->value);
	kfree(h_entry->key);
	kmem_cache_free(hashtable_entry_cachep, h_entry);
}

/*static void remove_int_to_null(struct hashtable_entry *h_entry) {
	//printk(KERN_INFO "sdcardfs: %s: %d: %d\n", __func__, (int)h_entry->key, h_entry->value);
	kmem_cache_free(hashtable_entry_cachep, h_entry);
}*/

static void remove_all_hashentrys(struct packagelist_data *pkgl_dat)
{
	struct hashtable_entry *hash_cur;
	struct hlist_node *h_t;
	int i;

	hash_for_each_safe(pkgl_dat->package_to_appid, i, h_t, hash_cur, hlist)
		remove_str_to_int(hash_cur);

	hash_init(pkgl_dat->package_to_appid);
}

static int read_package_list(struct packagelist_data *pkgl_dat) {
	int ret;
	int fd;
	int read_amount;

	printk(KERN_INFO "sdcardfs: read_package_list\n");

	mutex_lock(&pkgl_dat->hashtable_lock);

	remove_all_hashentrys(pkgl_dat);

	fd = sys_open(kpackageslist_file, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_ERR "sdcardfs: failed to open package list\n");
		mutex_unlock(&pkgl_dat->hashtable_lock);
		return fd;
	}

	while ((read_amount = sys_read(fd, pkgl_dat->read_buf,
					sizeof(pkgl_dat->read_buf))) > 0) {
		int appid;
		int one_line_len = 0;
		int additional_read;
	
		while (one_line_len < read_amount) {
			if (pkgl_dat->read_buf[one_line_len] == '\n') {
				one_line_len++;
				break;
			}
			one_line_len++;
		}
		additional_read = read_amount - one_line_len;
		if (additional_read > 0)
			sys_lseek(fd, -additional_read, SEEK_CUR);	

		if (sscanf(pkgl_dat->read_buf, "%s %d %*d %*s %*s %s",
				pkgl_dat->app_name_buf, &appid,
				pkgl_dat->gids_buf) == 3) {
			ret = insert_str_to_int(pkgl_dat, pkgl_dat->app_name_buf, appid);
			if (ret) {
				sys_close(fd);
				mutex_unlock(&pkgl_dat->hashtable_lock);
				return ret;
			}
		}
	}

	sys_close(fd);
	mutex_unlock(&pkgl_dat->hashtable_lock);
	return 0;
}

static int packagelist_reader(void *thread_data)
{
	struct packagelist_data *pkgl_dat = (struct packagelist_data *)thread_data;
	struct inotify_event *event;
	bool active = false;
	int event_pos;
	int event_size;
	int res = 0;
	int nfd;

	allow_signal(SIGINT);

	nfd = sys_inotify_init();
	if (nfd < 0) {
		printk(KERN_ERR "sdcardfs: inotify_init failed: %d\n", nfd);
		return nfd;
	}

	while (!kthread_should_stop()) {
		if (signal_pending(current)) {
			msleep(100);
			continue;
		}

		if (!active) {
			res = sys_inotify_add_watch(nfd, kpackageslist_file, IN_DELETE_SELF);
			if (res < 0) {
				if (res == -ENOENT || res == -EACCES) {
				/* Framework may not have created yet, sleep and retry */
					printk(KERN_ERR "sdcardfs: missing packages.list; retrying\n");
					ssleep(2);
					printk(KERN_ERR "sdcardfs: missing packages.list_end; retrying\n");
					continue;
				} else {
					printk(KERN_ERR "sdcardfs: inotify_add_watch failed: %d\n", res);
					goto interruptable_sleep;
				}
			}
			/* Watch above will tell us about any future changes, so
			 * read the current state. */
			res = read_package_list(pkgl_dat);
			if (res) {
				printk(KERN_ERR "sdcardfs: read_package_list failed: %d\n", res);
				goto interruptable_sleep;
			}
			active = true;
		}

		event_pos = 0;
		res = sys_read(nfd, pkgl_dat->event_buf, sizeof(pkgl_dat->event_buf));
		if (res < (int) sizeof(*event)) {
			if (res == -EINTR)
				continue;
			printk(KERN_ERR "sdcardfs: failed to read inotify event: %d\n", res);
			goto interruptable_sleep;
		}

		while (res >= (int) sizeof(*event)) {
			event = (struct inotify_event *) (pkgl_dat->event_buf + event_pos);

			printk(KERN_INFO "sdcardfs: inotify event: %08x\n", event->mask);
			if ((event->mask & IN_IGNORED) == IN_IGNORED) {
				/* Previously watched file was deleted, probably due to move
				 * that swapped in new data; re-arm the watch and read. */
				active = false;
			}

			event_size = sizeof(*event) + event->len;
			res -= event_size;
			event_pos += event_size;
		}
		continue;

interruptable_sleep:
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	flush_signals(current);
	sys_close(nfd);
	return res;
}

void * packagelist_create(void)
{
	struct packagelist_data *pkgl_dat;
	struct task_struct *packagelist_thread;

	pkgl_dat = kmalloc(sizeof(*pkgl_dat), GFP_KERNEL | __GFP_ZERO);
	if (!pkgl_dat) {
		printk(KERN_ERR "sdcardfs: creating kthread failed\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_init(&pkgl_dat->hashtable_lock);
	hash_init(pkgl_dat->package_to_appid);

	packagelist_thread = kthread_run(packagelist_reader, (void *)pkgl_dat, "pkgld");
	if (IS_ERR(packagelist_thread)) {
		printk(KERN_ERR "sdcardfs: creating kthread failed\n");
		kfree(pkgl_dat);
		return packagelist_thread;
	}
	pkgl_dat->thread_id = packagelist_thread;	

	printk(KERN_INFO "sdcardfs: created packagelist pkgld/%d\n",
				(int)pkgl_dat->thread_id->pid);

	return (void *)pkgl_dat;
}

void packagelist_destroy(void *pkgl_id)
{
	struct packagelist_data *pkgl_dat = (struct packagelist_data *)pkgl_id;
	pid_t pkgl_pid = pkgl_dat->thread_id->pid;	

	force_sig_info(SIGINT, SEND_SIG_PRIV, pkgl_dat->thread_id);
	kthread_stop(pkgl_dat->thread_id);
	remove_all_hashentrys(pkgl_dat);
	printk(KERN_INFO "sdcardfs: destroyed packagelist pkgld/%d\n", (int)pkgl_pid);
	kfree(pkgl_dat);
<<<<<<< HEAD
=======
 * checked before calling check_caller_access_to_name()
 */
int open_flags_to_access_mode(int open_flags)
{
	if ((open_flags & O_ACCMODE) == O_RDONLY)
		return 0; /* R_OK */
	if ((open_flags & O_ACCMODE) == O_WRONLY)
		return 1; /* W_OK */
	/* Probably O_RDRW, but treat as default to be safe */
		return 1; /* R_OK | W_OK */
}

static struct hashtable_entry *alloc_hashtable_entry(const struct qstr *key,
		appid_t value)
{
	struct hashtable_entry *ret = kmem_cache_alloc(hashtable_entry_cachep,
			GFP_KERNEL);
	if (!ret)
		return NULL;
	INIT_HLIST_NODE(&ret->dlist);
	INIT_HLIST_NODE(&ret->hlist);

	if (!qstr_copy(key, &ret->key)) {
		kmem_cache_free(hashtable_entry_cachep, ret);
		return NULL;
	}

	atomic_set(&ret->value, value);
	return ret;
}

static int insert_packagelist_appid_entry_locked(const struct qstr *key, appid_t value)
{
	struct hashtable_entry *hash_cur;
	struct hashtable_entry *new_entry;
	unsigned int hash = key->hash;

	hash_for_each_possible_rcu(package_to_appid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key)) {
			atomic_set(&hash_cur->value, value);
			return 0;
		}
	}
	new_entry = alloc_hashtable_entry(key, value);
	if (!new_entry)
		return -ENOMEM;
	hash_add_rcu(package_to_appid, &new_entry->hlist, hash);
	return 0;
}

static int insert_ext_gid_entry_locked(const struct qstr *key, appid_t value)
{
	struct hashtable_entry *hash_cur;
	struct hashtable_entry *new_entry;
	unsigned int hash = key->hash;

	/* An extension can only belong to one gid */
	hash_for_each_possible_rcu(ext_to_groupid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key))
			return -EINVAL;
	}
	new_entry = alloc_hashtable_entry(key, value);
	if (!new_entry)
		return -ENOMEM;
	hash_add_rcu(ext_to_groupid, &new_entry->hlist, hash);
	return 0;
}

static int insert_userid_exclude_entry_locked(const struct qstr *key, userid_t value)
{
	struct hashtable_entry *hash_cur;
	struct hashtable_entry *new_entry;
	unsigned int hash = key->hash;

	/* Only insert if not already present */
	hash_for_each_possible_rcu(package_to_userid, hash_cur, hlist, hash) {
		if (atomic_read(&hash_cur->value) == value &&
				qstr_case_eq(key, &hash_cur->key))
			return 0;
	}
	new_entry = alloc_hashtable_entry(key, value);
	if (!new_entry)
		return -ENOMEM;
	hash_add_rcu(package_to_userid, &new_entry->hlist, hash);
	return 0;
}

static void fixup_all_perms_name(const struct qstr *key)
{
	struct sdcardfs_sb_info *sbinfo;
	struct limit_search limit = {
		.flags = BY_NAME,
		.name = QSTR_INIT(key->name, key->len),
	};
	list_for_each_entry(sbinfo, &sdcardfs_super_list, list) {
		if (sbinfo_has_sdcard_magic(sbinfo))
			fixup_perms_recursive(sbinfo->sb->s_root, &limit);
	}
}

static void fixup_all_perms_name_userid(const struct qstr *key, userid_t userid)
{
	struct sdcardfs_sb_info *sbinfo;
	struct limit_search limit = {
		.flags = BY_NAME | BY_USERID,
		.name = QSTR_INIT(key->name, key->len),
		.userid = userid,
	};
	list_for_each_entry(sbinfo, &sdcardfs_super_list, list) {
		if (sbinfo_has_sdcard_magic(sbinfo))
			fixup_perms_recursive(sbinfo->sb->s_root, &limit);
	}
}

static void fixup_all_perms_userid(userid_t userid)
{
	struct sdcardfs_sb_info *sbinfo;
	struct limit_search limit = {
		.flags = BY_USERID,
		.userid = userid,
	};
	list_for_each_entry(sbinfo, &sdcardfs_super_list, list) {
		if (sbinfo_has_sdcard_magic(sbinfo))
			fixup_perms_recursive(sbinfo->sb->s_root, &limit);
	}
}

static int insert_packagelist_entry(const struct qstr *key, appid_t value)
{
	int err;

	mutex_lock(&sdcardfs_super_list_lock);
	err = insert_packagelist_appid_entry_locked(key, value);
	if (!err)
		fixup_all_perms_name(key);
	mutex_unlock(&sdcardfs_super_list_lock);

	return err;
}

static int insert_ext_gid_entry(const struct qstr *key, appid_t value)
{
	int err;

	mutex_lock(&sdcardfs_super_list_lock);
	err = insert_ext_gid_entry_locked(key, value);
	mutex_unlock(&sdcardfs_super_list_lock);

	return err;
}

static int insert_userid_exclude_entry(const struct qstr *key, userid_t value)
{
	int err;

	mutex_lock(&sdcardfs_super_list_lock);
	err = insert_userid_exclude_entry_locked(key, value);
	if (!err)
		fixup_all_perms_name_userid(key, value);
	mutex_unlock(&sdcardfs_super_list_lock);

	return err;
}

static void free_hashtable_entry(struct hashtable_entry *entry)
{
	kfree(entry->key.name);
	kmem_cache_free(hashtable_entry_cachep, entry);
}

static void remove_packagelist_entry_locked(const struct qstr *key)
{
	struct hashtable_entry *hash_cur;
	unsigned int hash = key->hash;
	struct hlist_node *h_t;
	HLIST_HEAD(free_list);

	hash_for_each_possible_rcu(package_to_userid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key)) {
			hash_del_rcu(&hash_cur->hlist);
			hlist_add_head(&hash_cur->dlist, &free_list);
		}
	}
	hash_for_each_possible_rcu(package_to_appid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key)) {
			hash_del_rcu(&hash_cur->hlist);
			hlist_add_head(&hash_cur->dlist, &free_list);
			break;
		}
	}
	synchronize_rcu();
	hlist_for_each_entry_safe(hash_cur, h_t, &free_list, dlist)
		free_hashtable_entry(hash_cur);
}

static void remove_packagelist_entry(const struct qstr *key)
{
	mutex_lock(&sdcardfs_super_list_lock);
	remove_packagelist_entry_locked(key);
	fixup_all_perms_name(key);
	mutex_unlock(&sdcardfs_super_list_lock);
}

static void remove_ext_gid_entry_locked(const struct qstr *key, gid_t group)
{
	struct hashtable_entry *hash_cur;
	unsigned int hash = key->hash;

	hash_for_each_possible_rcu(ext_to_groupid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key) && atomic_read(&hash_cur->value) == group) {
			hash_del_rcu(&hash_cur->hlist);
			synchronize_rcu();
			free_hashtable_entry(hash_cur);
			break;
		}
	}
}

static void remove_ext_gid_entry(const struct qstr *key, gid_t group)
{
	mutex_lock(&sdcardfs_super_list_lock);
	remove_ext_gid_entry_locked(key, group);
	mutex_unlock(&sdcardfs_super_list_lock);
}

static void remove_userid_all_entry_locked(userid_t userid)
{
	struct hashtable_entry *hash_cur;
	struct hlist_node *h_t;
	HLIST_HEAD(free_list);
	int i;

	hash_for_each_rcu(package_to_userid, i, hash_cur, hlist) {
		if (atomic_read(&hash_cur->value) == userid) {
			hash_del_rcu(&hash_cur->hlist);
			hlist_add_head(&hash_cur->dlist, &free_list);
		}
	}
	synchronize_rcu();
	hlist_for_each_entry_safe(hash_cur, h_t, &free_list, dlist) {
		free_hashtable_entry(hash_cur);
	}
}

static void remove_userid_all_entry(userid_t userid)
{
	mutex_lock(&sdcardfs_super_list_lock);
	remove_userid_all_entry_locked(userid);
	fixup_all_perms_userid(userid);
	mutex_unlock(&sdcardfs_super_list_lock);
}

static void remove_userid_exclude_entry_locked(const struct qstr *key, userid_t userid)
{
	struct hashtable_entry *hash_cur;
	unsigned int hash = key->hash;

	hash_for_each_possible_rcu(package_to_userid, hash_cur, hlist, hash) {
		if (qstr_case_eq(key, &hash_cur->key) &&
				atomic_read(&hash_cur->value) == userid) {
			hash_del_rcu(&hash_cur->hlist);
			synchronize_rcu();
			free_hashtable_entry(hash_cur);
			break;
		}
	}
}

static void remove_userid_exclude_entry(const struct qstr *key, userid_t userid)
{
	mutex_lock(&sdcardfs_super_list_lock);
	remove_userid_exclude_entry_locked(key, userid);
	fixup_all_perms_name_userid(key, userid);
	mutex_unlock(&sdcardfs_super_list_lock);
}

static void packagelist_destroy(void)
{
	struct hashtable_entry *hash_cur;
	struct hlist_node *h_t;
	HLIST_HEAD(free_list);
	int i;

	mutex_lock(&sdcardfs_super_list_lock);
	hash_for_each_rcu(package_to_appid, i, hash_cur, hlist) {
		hash_del_rcu(&hash_cur->hlist);
		hlist_add_head(&hash_cur->dlist, &free_list);
	}
	hash_for_each_rcu(package_to_userid, i, hash_cur, hlist) {
		hash_del_rcu(&hash_cur->hlist);
		hlist_add_head(&hash_cur->dlist, &free_list);
	}
	synchronize_rcu();
	hlist_for_each_entry_safe(hash_cur, h_t, &free_list, dlist)
		free_hashtable_entry(hash_cur);
	mutex_unlock(&sdcardfs_super_list_lock);
	pr_info("sdcardfs: destroyed packagelist pkgld\n");
}

struct package_details {
	struct config_item item;
	struct qstr name;
};

static inline struct package_details *to_package_details(struct config_item *item)
{
	return item ? container_of(item, struct package_details, item) : NULL;
}

CONFIGFS_ATTR_STRUCT(package_details);
#define PACKAGE_DETAILS_ATTR(_name, _mode, _show, _store)	\
struct package_details_attribute package_details_attr_##_name = __CONFIGFS_ATTR(_name, _mode, _show, _store)
#define PACKAGE_DETAILS_ATTRIBUTE(name) (&package_details_attr_##name.attr)

static ssize_t package_details_appid_show(struct package_details *package_details,
				      char *page)
{
	return scnprintf(page, PAGE_SIZE, "%u\n", __get_appid(&package_details->name));
}

static ssize_t package_details_appid_store(struct package_details *package_details,
				       const char *page, size_t count)
{
	unsigned int tmp;
	int ret;

	ret = kstrtouint(page, 10, &tmp);
	if (ret)
		return ret;

	ret = insert_packagelist_entry(&package_details->name, tmp);

	if (ret)
		return ret;

	return count;
}

static ssize_t package_details_excluded_userids_show(struct package_details *package_details,
				      char *page)
{
	struct hashtable_entry *hash_cur;
	unsigned int hash = package_details->name.hash;
	int count = 0;

	rcu_read_lock();
	hash_for_each_possible_rcu(package_to_userid, hash_cur, hlist, hash) {
		if (qstr_case_eq(&package_details->name, &hash_cur->key))
			count += scnprintf(page + count, PAGE_SIZE - count,
					"%d ", atomic_read(&hash_cur->value));
	}
	rcu_read_unlock();
	if (count)
		count--;
	count += scnprintf(page + count, PAGE_SIZE - count, "\n");
	return count;
}

static ssize_t package_details_excluded_userids_store(struct package_details *package_details,
				       const char *page, size_t count)
{
	unsigned int tmp;
	int ret;

	ret = kstrtouint(page, 10, &tmp);
	if (ret)
		return ret;

	ret = insert_userid_exclude_entry(&package_details->name, tmp);

	if (ret)
		return ret;

	return count;
}

static ssize_t package_details_clear_userid_store(struct package_details *package_details,
				       const char *page, size_t count)
{
	unsigned int tmp;
	int ret;

	ret = kstrtouint(page, 10, &tmp);
	if (ret)
		return ret;
	remove_userid_exclude_entry(&package_details->name, tmp);
	return count;
}

static void package_details_release(struct config_item *item)
{
	struct package_details *package_details = to_package_details(item);

	pr_info("sdcardfs: removing %s\n", package_details->name.name);
	remove_packagelist_entry(&package_details->name);
	kfree(package_details->name.name);
	kfree(package_details);
}

PACKAGE_DETAILS_ATTR(appid, S_IRUGO | S_IWUGO, package_details_appid_show, package_details_appid_store);
PACKAGE_DETAILS_ATTR(excluded_userids, S_IRUGO | S_IWUGO,
		package_details_excluded_userids_show, package_details_excluded_userids_store);
PACKAGE_DETAILS_ATTR(clear_userid, S_IWUGO, NULL, package_details_clear_userid_store);

static struct configfs_attribute *package_details_attrs[] = {
	PACKAGE_DETAILS_ATTRIBUTE(appid),
	PACKAGE_DETAILS_ATTRIBUTE(excluded_userids),
	PACKAGE_DETAILS_ATTRIBUTE(clear_userid),
	NULL,
};

CONFIGFS_ATTR_OPS(package_details);

static struct configfs_item_operations package_details_item_ops = {
	.release = package_details_release,
	.show_attribute = package_details_attr_show,
	.store_attribute = package_details_attr_store,
};

static struct config_item_type package_appid_type = {
	.ct_item_ops	= &package_details_item_ops,
	.ct_attrs	= package_details_attrs,
	.ct_owner	= THIS_MODULE,
};

struct extensions_value {
	struct config_group group;
	unsigned int num;
};

struct extension_details {
	struct config_item item;
	struct qstr name;
	unsigned int num;
};

static inline struct extensions_value *to_extensions_value(struct config_item *item)
{
	return item ? container_of(to_config_group(item), struct extensions_value, group) : NULL;
}

static inline struct extension_details *to_extension_details(struct config_item *item)
{
	return item ? container_of(item, struct extension_details, item) : NULL;
}

static void extension_details_release(struct config_item *item)
{
	struct extension_details *extension_details = to_extension_details(item);

	pr_info("sdcardfs: No longer mapping %s files to gid %d\n",
			extension_details->name.name, extension_details->num);
	remove_ext_gid_entry(&extension_details->name, extension_details->num);
	kfree(extension_details->name.name);
	kfree(extension_details);
}

static struct configfs_item_operations extension_details_item_ops = {
	.release = extension_details_release,
};

static struct config_item_type extension_details_type = {
	.ct_item_ops = &extension_details_item_ops,
	.ct_owner = THIS_MODULE,
};

static struct config_item *extension_details_make_item(struct config_group *group, const char *name)
{
	struct extensions_value *extensions_value = to_extensions_value(&group->cg_item);
	struct extension_details *extension_details = kzalloc(sizeof(struct extension_details), GFP_KERNEL);
	const char *tmp;
	int ret;

	if (!extension_details)
		return ERR_PTR(-ENOMEM);

	tmp = kstrdup(name, GFP_KERNEL);
	if (!tmp) {
		kfree(extension_details);
		return ERR_PTR(-ENOMEM);
	}
	qstr_init(&extension_details->name, tmp);
	ret = insert_ext_gid_entry(&extension_details->name, extensions_value->num);

	if (ret) {
		kfree(extension_details->name.name);
		kfree(extension_details);
		return ERR_PTR(ret);
	}
	config_item_init_type_name(&extension_details->item, name, &extension_details_type);

	return &extension_details->item;
}

static struct configfs_group_operations extensions_value_group_ops = {
	.make_item = extension_details_make_item,
};

static struct config_item_type extensions_name_type = {
	.ct_group_ops	= &extensions_value_group_ops,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *extensions_make_group(struct config_group *group, const char *name)
{
	struct extensions_value *extensions_value;
	unsigned int tmp;
	int ret;

	extensions_value = kzalloc(sizeof(struct extensions_value), GFP_KERNEL);
	if (!extensions_value)
		return ERR_PTR(-ENOMEM);
	ret = kstrtouint(name, 10, &tmp);
	if (ret) {
		kfree(extensions_value);
		return ERR_PTR(ret);
	}

	extensions_value->num = tmp;
	config_group_init_type_name(&extensions_value->group, name,
						&extensions_name_type);
	return &extensions_value->group;
}

static void extensions_drop_group(struct config_group *group, struct config_item *item)
{
	struct extensions_value *value = to_extensions_value(item);

	pr_info("sdcardfs: No longer mapping any files to gid %d\n", value->num);
	kfree(value);
}

static struct configfs_group_operations extensions_group_ops = {
	.make_group	= extensions_make_group,
	.drop_item	= extensions_drop_group,
};

static struct config_item_type extensions_type = {
	.ct_group_ops	= &extensions_group_ops,
	.ct_owner	= THIS_MODULE,
};

struct config_group extension_group = {
	.cg_item = {
		.ci_namebuf = "extensions",
		.ci_type = &extensions_type,
	},
};

struct packages {
	struct configfs_subsystem subsystem;
};

static inline struct packages *to_packages(struct config_item *item)
{
	return item ? container_of(to_configfs_subsystem(to_config_group(item)), struct packages, subsystem) : NULL;
}

CONFIGFS_ATTR_STRUCT(packages);
#define PACKAGES_ATTR(_name, _mode, _show, _store)	\
struct packages_attribute packages_attr_##_name = __CONFIGFS_ATTR(_name, _mode, _show, _store)
#define PACKAGES_ATTR_RO(_name, _show)	\
struct packages_attribute packages_attr_##_name = __CONFIGFS_ATTR_RO(_name, _show)

static struct config_item *packages_make_item(struct config_group *group, const char *name)
{
	struct package_details *package_details;
	const char *tmp;

	package_details = kzalloc(sizeof(struct package_details), GFP_KERNEL);
	if (!package_details)
		return ERR_PTR(-ENOMEM);
	tmp = kstrdup(name, GFP_KERNEL);
	if (!tmp) {
		kfree(package_details);
		return ERR_PTR(-ENOMEM);
	}
	qstr_init(&package_details->name, tmp);
	config_item_init_type_name(&package_details->item, name,
						&package_appid_type);

	return &package_details->item;
}

static ssize_t packages_list_show(struct packages *packages,
					 char *page)
{
	struct hashtable_entry *hash_cur_app;
	struct hashtable_entry *hash_cur_user;
	int i;
	int count = 0, written = 0;
	const char errormsg[] = "<truncated>\n";
	unsigned int hash;

	rcu_read_lock();
	hash_for_each_rcu(package_to_appid, i, hash_cur_app, hlist) {
		written = scnprintf(page + count, PAGE_SIZE - sizeof(errormsg) - count, "%s %d\n",
					hash_cur_app->key.name, atomic_read(&hash_cur_app->value));
		hash = hash_cur_app->key.hash;
		hash_for_each_possible_rcu(package_to_userid, hash_cur_user, hlist, hash) {
			if (qstr_case_eq(&hash_cur_app->key, &hash_cur_user->key)) {
				written += scnprintf(page + count + written - 1,
					PAGE_SIZE - sizeof(errormsg) - count - written + 1,
					" %d\n", atomic_read(&hash_cur_user->value)) - 1;
			}
		}
		if (count + written == PAGE_SIZE - sizeof(errormsg) - 1) {
			count += scnprintf(page + count, PAGE_SIZE - count, errormsg);
			break;
		}
		count += written;
	}
	rcu_read_unlock();

	return count;
}

static ssize_t packages_remove_userid_store(struct packages *packages,
				       const char *page, size_t count)
{
	unsigned int tmp;
	int ret;

	ret = kstrtouint(page, 10, &tmp);
	if (ret)
		return ret;
	remove_userid_all_entry(tmp);
	return count;
}

struct packages_attribute packages_attr_packages_gid_list = __CONFIGFS_ATTR_RO(packages_gid.list, packages_list_show);
PACKAGES_ATTR(remove_userid, S_IWUGO, NULL, packages_remove_userid_store);

static struct configfs_attribute *packages_attrs[] = {
	&packages_attr_packages_gid_list.attr,
	&packages_attr_remove_userid.attr,
	NULL,
};

CONFIGFS_ATTR_OPS(packages)
static struct configfs_item_operations packages_item_ops = {
	.show_attribute = packages_attr_show,
	.store_attribute = packages_attr_store,
};

/*
 * Note that, since no extra work is required on ->drop_item(),
 * no ->drop_item() is provided.
 */
static struct configfs_group_operations packages_group_ops = {
	.make_item	= packages_make_item,
};

static struct config_item_type packages_type = {
	.ct_item_ops	= &packages_item_ops,
	.ct_group_ops	= &packages_group_ops,
	.ct_attrs	= packages_attrs,
	.ct_owner	= THIS_MODULE,
};

struct config_group *sd_default_groups[] = {
	&extension_group,
	NULL,
};

static struct packages sdcardfs_packages = {
	.subsystem = {
		.su_group = {
			.cg_item = {
				.ci_namebuf = "sdcardfs",
				.ci_type = &packages_type,
			},
			.default_groups = sd_default_groups,
		},
	},
};

static int configfs_sdcardfs_init(void)
{
	int ret, i;
	struct configfs_subsystem *subsys = &sdcardfs_packages.subsystem;

	for (i = 0; sd_default_groups[i]; i++)
		config_group_init(sd_default_groups[i]);
	config_group_init(&subsys->su_group);
	mutex_init(&subsys->su_mutex);
	ret = configfs_register_subsystem(subsys);
	if (ret) {
		pr_err("Error %d while registering subsystem %s\n",
		       ret,
		       subsys->su_group.cg_item.ci_namebuf);
	}
	return ret;
}

static void configfs_sdcardfs_exit(void)
{
	configfs_unregister_subsystem(&sdcardfs_packages.subsystem);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
}

int packagelist_init(void)
{
	hashtable_entry_cachep =
		kmem_cache_create("packagelist_hashtable_entry",
					sizeof(struct hashtable_entry), 0, 0, NULL);
	if (!hashtable_entry_cachep) {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
		printk(KERN_ERR "sdcardfs: failed creating pkgl_hashtable entry slab cache\n");
		return -ENOMEM;
	}

        return 0;
<<<<<<< HEAD
=======
		pr_err("sdcardfs: failed creating pkgl_hashtable entry slab cache\n");
		return -ENOMEM;
	}

	configfs_sdcardfs_init();
	return 0;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
}

void packagelist_exit(void)
{
<<<<<<< HEAD
<<<<<<< HEAD
	if (hashtable_entry_cachep)
		kmem_cache_destroy(hashtable_entry_cachep);
=======
	configfs_sdcardfs_exit();
	packagelist_destroy();
	kmem_cache_destroy(hashtable_entry_cachep);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	if (hashtable_entry_cachep)
		kmem_cache_destroy(hashtable_entry_cachep);
>>>>>>> 2617302... source
}
