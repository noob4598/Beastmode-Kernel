/*
 * fs/f2fs/gc.h
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define GC_THREAD_MIN_WB_PAGES		1	/*
						 * a threshold to determine
						 * whether IO subsystem is idle
						 * or not
						 */
<<<<<<< HEAD
<<<<<<< HEAD
#define GC_THREAD_MIN_SLEEP_TIME	30000	/* milliseconds */
#define GC_THREAD_MAX_SLEEP_TIME	60000
#define GC_THREAD_NOGC_SLEEP_TIME	300000	/* wait 5 min */
=======
#define DEF_GC_THREAD_MIN_SLEEP_TIME	30000	/* milliseconds */
#define DEF_GC_THREAD_MAX_SLEEP_TIME	60000
#define DEF_GC_THREAD_NOGC_SLEEP_TIME	300000	/* wait 5 min */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
#define GC_THREAD_MIN_SLEEP_TIME	30000	/* milliseconds */
#define GC_THREAD_MAX_SLEEP_TIME	60000
#define GC_THREAD_NOGC_SLEEP_TIME	300000	/* wait 5 min */
>>>>>>> 2617302... source
#define LIMIT_INVALID_BLOCK	40 /* percentage over total user space */
#define LIMIT_FREE_BLOCK	40 /* percentage over invalid + free space */

/* Search max. number of dirty segments to select a victim segment */
<<<<<<< HEAD
<<<<<<< HEAD
#define MAX_VICTIM_SEARCH	20
=======
#define DEF_MAX_VICTIM_SEARCH 4096 /* covers 8GB */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
#define MAX_VICTIM_SEARCH	20
>>>>>>> 2617302... source

struct f2fs_gc_kthread {
	struct task_struct *f2fs_gc_task;
	wait_queue_head_t gc_wait_queue_head;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
};

struct inode_entry {
	struct list_head list;
	struct inode *inode;
<<<<<<< HEAD
=======

	/* for gc sleep time */
	unsigned int min_sleep_time;
	unsigned int max_sleep_time;
	unsigned int no_gc_sleep_time;

	/* for changing gc mode */
	unsigned int gc_idle;
};

struct gc_inode_list {
	struct list_head ilist;
	struct radix_tree_root iroot;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
};

/*
 * inline functions
 */
static inline block_t free_user_blocks(struct f2fs_sb_info *sbi)
{
	if (free_segments(sbi) < overprovision_segments(sbi))
		return 0;
	else
		return (free_segments(sbi) - overprovision_segments(sbi))
			<< sbi->log_blocks_per_seg;
}

static inline block_t limit_invalid_user_blocks(struct f2fs_sb_info *sbi)
{
	return (long)(sbi->user_block_count * LIMIT_INVALID_BLOCK) / 100;
}

static inline block_t limit_free_user_blocks(struct f2fs_sb_info *sbi)
{
	block_t reclaimable_user_blocks = sbi->user_block_count -
		written_block_count(sbi);
	return (long)(reclaimable_user_blocks * LIMIT_FREE_BLOCK) / 100;
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
static inline long increase_sleep_time(long wait)
{
	if (wait == GC_THREAD_NOGC_SLEEP_TIME)
		return wait;

	wait += GC_THREAD_MIN_SLEEP_TIME;
	if (wait > GC_THREAD_MAX_SLEEP_TIME)
		wait = GC_THREAD_MAX_SLEEP_TIME;
	return wait;
}

static inline long decrease_sleep_time(long wait)
{
	if (wait == GC_THREAD_NOGC_SLEEP_TIME)
		wait = GC_THREAD_MAX_SLEEP_TIME;

	wait -= GC_THREAD_MIN_SLEEP_TIME;
	if (wait <= GC_THREAD_MIN_SLEEP_TIME)
		wait = GC_THREAD_MIN_SLEEP_TIME;
	return wait;
<<<<<<< HEAD
=======
static inline void increase_sleep_time(struct f2fs_gc_kthread *gc_th,
								long *wait)
{
	if (*wait == gc_th->no_gc_sleep_time)
		return;

	*wait += gc_th->min_sleep_time;
	if (*wait > gc_th->max_sleep_time)
		*wait = gc_th->max_sleep_time;
}

static inline void decrease_sleep_time(struct f2fs_gc_kthread *gc_th,
								long *wait)
{
	if (*wait == gc_th->no_gc_sleep_time)
		*wait = gc_th->max_sleep_time;

	*wait -= gc_th->min_sleep_time;
	if (*wait <= gc_th->min_sleep_time)
		*wait = gc_th->min_sleep_time;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
}

static inline bool has_enough_invalid_blocks(struct f2fs_sb_info *sbi)
{
	block_t invalid_user_blocks = sbi->user_block_count -
					written_block_count(sbi);
	/*
<<<<<<< HEAD
<<<<<<< HEAD
	 * Background GC is triggered with the following condition.
=======
	 * Background GC is triggered with the following conditions.
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
	 * Background GC is triggered with the following condition.
>>>>>>> 2617302... source
	 * 1. There are a number of invalid blocks.
	 * 2. There is not enough free space.
	 */
	if (invalid_user_blocks > limit_invalid_user_blocks(sbi) &&
			free_user_blocks(sbi) < limit_free_user_blocks(sbi))
		return true;
	return false;
}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source

static inline int is_idle(struct f2fs_sb_info *sbi)
{
	struct block_device *bdev = sbi->sb->s_bdev;
	struct request_queue *q = bdev_get_queue(bdev);
	struct request_list *rl = &q->root_rl;
	return !(rl->count[BLK_RW_SYNC]) && !(rl->count[BLK_RW_ASYNC]);
}
<<<<<<< HEAD
=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
