/*
 * fs/f2fs/gc.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/backing-dev.h>
#include <linux/init.h>
#include <linux/f2fs_fs.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/freezer.h>
<<<<<<< HEAD
#include <linux/blkdev.h>
=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

#include "f2fs.h"
#include "node.h"
#include "segment.h"
#include "gc.h"
#include <trace/events/f2fs.h>

<<<<<<< HEAD
static struct kmem_cache *winode_slab;

static int gc_thread_func(void *data)
{
	struct f2fs_sb_info *sbi = data;
	wait_queue_head_t *wq = &sbi->gc_thread->gc_wait_queue_head;
	long wait_ms;

	wait_ms = GC_THREAD_MIN_SLEEP_TIME;
=======
static int gc_thread_func(void *data)
{
	struct f2fs_sb_info *sbi = data;
	struct f2fs_gc_kthread *gc_th = sbi->gc_thread;
	wait_queue_head_t *wq = &sbi->gc_thread->gc_wait_queue_head;
	long wait_ms;

	wait_ms = gc_th->min_sleep_time;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

	do {
		if (try_to_freeze())
			continue;
		else
			wait_event_interruptible_timeout(*wq,
						kthread_should_stop(),
						msecs_to_jiffies(wait_ms));
		if (kthread_should_stop())
			break;

		if (sbi->sb->s_writers.frozen >= SB_FREEZE_WRITE) {
<<<<<<< HEAD
			wait_ms = GC_THREAD_MAX_SLEEP_TIME;
			continue;
		}

=======
			increase_sleep_time(gc_th, &wait_ms);
			continue;
		}

#ifdef CONFIG_F2FS_FAULT_INJECTION
		if (time_to_inject(sbi, FAULT_CHECKPOINT)) {
			f2fs_show_injection_info(FAULT_CHECKPOINT);
			f2fs_stop_checkpoint(sbi, false);
		}
#endif

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		/*
		 * [GC triggering condition]
		 * 0. GC is not conducted currently.
		 * 1. There are enough dirty segments.
		 * 2. IO subsystem is idle by checking the # of writeback pages.
		 * 3. IO subsystem is idle by checking the # of requests in
		 *    bdev's request list.
		 *
<<<<<<< HEAD
		 * Note) We have to avoid triggering GCs too much frequently.
=======
		 * Note) We have to avoid triggering GCs frequently.
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		 * Because it is possible that some segments can be
		 * invalidated soon after by user update or deletion.
		 * So, I'd like to wait some time to collect dirty segments.
		 */
		if (!mutex_trylock(&sbi->gc_mutex))
			continue;

		if (!is_idle(sbi)) {
<<<<<<< HEAD
			wait_ms = increase_sleep_time(wait_ms);
=======
			increase_sleep_time(gc_th, &wait_ms);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
			mutex_unlock(&sbi->gc_mutex);
			continue;
		}

		if (has_enough_invalid_blocks(sbi))
<<<<<<< HEAD
			wait_ms = decrease_sleep_time(wait_ms);
		else
			wait_ms = increase_sleep_time(wait_ms);

#ifdef CONFIG_F2FS_STAT_FS
		sbi->bg_gc++;
#endif

		/* if return value is not zero, no victim was selected */
		if (f2fs_gc(sbi))
			wait_ms = GC_THREAD_NOGC_SLEEP_TIME;
=======
			decrease_sleep_time(gc_th, &wait_ms);
		else
			increase_sleep_time(gc_th, &wait_ms);

		stat_inc_bggc_count(sbi);

		/* if return value is not zero, no victim was selected */
		if (f2fs_gc(sbi, test_opt(sbi, FORCE_FG_GC), true))
			wait_ms = gc_th->no_gc_sleep_time;

		trace_f2fs_background_gc(sbi->sb, wait_ms,
				prefree_segments(sbi), free_segments(sbi));

		/* balancing f2fs's metadata periodically */
		f2fs_balance_fs_bg(sbi);

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	} while (!kthread_should_stop());
	return 0;
}

int start_gc_thread(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_kthread *gc_th;
	dev_t dev = sbi->sb->s_bdev->bd_dev;
	int err = 0;

<<<<<<< HEAD
	if (!test_opt(sbi, BG_GC))
		goto out;
	gc_th = kmalloc(sizeof(struct f2fs_gc_kthread), GFP_KERNEL);
=======
	gc_th = f2fs_kmalloc(sbi, sizeof(struct f2fs_gc_kthread), GFP_KERNEL);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	if (!gc_th) {
		err = -ENOMEM;
		goto out;
	}

<<<<<<< HEAD
=======
	gc_th->min_sleep_time = DEF_GC_THREAD_MIN_SLEEP_TIME;
	gc_th->max_sleep_time = DEF_GC_THREAD_MAX_SLEEP_TIME;
	gc_th->no_gc_sleep_time = DEF_GC_THREAD_NOGC_SLEEP_TIME;

	gc_th->gc_idle = 0;

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	sbi->gc_thread = gc_th;
	init_waitqueue_head(&sbi->gc_thread->gc_wait_queue_head);
	sbi->gc_thread->f2fs_gc_task = kthread_run(gc_thread_func, sbi,
			"f2fs_gc-%u:%u", MAJOR(dev), MINOR(dev));
	if (IS_ERR(gc_th->f2fs_gc_task)) {
		err = PTR_ERR(gc_th->f2fs_gc_task);
		kfree(gc_th);
		sbi->gc_thread = NULL;
	}
<<<<<<< HEAD

=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
out:
	return err;
}

void stop_gc_thread(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_kthread *gc_th = sbi->gc_thread;
	if (!gc_th)
		return;
	kthread_stop(gc_th->f2fs_gc_task);
	kfree(gc_th);
	sbi->gc_thread = NULL;
}

<<<<<<< HEAD
static int select_gc_type(int gc_type)
{
	return (gc_type == BG_GC) ? GC_CB : GC_GREEDY;
=======
static int select_gc_type(struct f2fs_gc_kthread *gc_th, int gc_type)
{
	int gc_mode = (gc_type == BG_GC) ? GC_CB : GC_GREEDY;

	if (gc_th && gc_th->gc_idle) {
		if (gc_th->gc_idle == 1)
			gc_mode = GC_CB;
		else if (gc_th->gc_idle == 2)
			gc_mode = GC_GREEDY;
	}
	return gc_mode;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
}

static void select_policy(struct f2fs_sb_info *sbi, int gc_type,
			int type, struct victim_sel_policy *p)
{
	struct dirty_seglist_info *dirty_i = DIRTY_I(sbi);

	if (p->alloc_mode == SSR) {
		p->gc_mode = GC_GREEDY;
		p->dirty_segmap = dirty_i->dirty_segmap[type];
<<<<<<< HEAD
		p->ofs_unit = 1;
	} else {
		p->gc_mode = select_gc_type(gc_type);
		p->dirty_segmap = dirty_i->dirty_segmap[DIRTY];
		p->ofs_unit = sbi->segs_per_sec;
	}
=======
		p->max_search = dirty_i->nr_dirty[type];
		p->ofs_unit = 1;
	} else {
		p->gc_mode = select_gc_type(sbi->gc_thread, gc_type);
		p->dirty_segmap = dirty_i->dirty_segmap[DIRTY];
		p->max_search = dirty_i->nr_dirty[DIRTY];
		p->ofs_unit = sbi->segs_per_sec;
	}

	/* we need to check every dirty segments in the FG_GC case */
	if (gc_type != FG_GC && p->max_search > sbi->max_victim_search)
		p->max_search = sbi->max_victim_search;

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	p->offset = sbi->last_victim[p->gc_mode];
}

static unsigned int get_max_cost(struct f2fs_sb_info *sbi,
				struct victim_sel_policy *p)
{
	/* SSR allocates in a segment unit */
	if (p->alloc_mode == SSR)
<<<<<<< HEAD
		return 1 << sbi->log_blocks_per_seg;
	if (p->gc_mode == GC_GREEDY)
		return (1 << sbi->log_blocks_per_seg) * p->ofs_unit;
=======
		return sbi->blocks_per_seg;
	if (p->gc_mode == GC_GREEDY)
		return sbi->blocks_per_seg * p->ofs_unit;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	else if (p->gc_mode == GC_CB)
		return UINT_MAX;
	else /* No other gc_mode */
		return 0;
}

static unsigned int check_bg_victims(struct f2fs_sb_info *sbi)
{
	struct dirty_seglist_info *dirty_i = DIRTY_I(sbi);
<<<<<<< HEAD
	unsigned int hint = 0;
=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	unsigned int secno;

	/*
	 * If the gc_type is FG_GC, we can select victim segments
	 * selected by background GC before.
	 * Those segments guarantee they have small valid blocks.
	 */
<<<<<<< HEAD
next:
	secno = find_next_bit(dirty_i->victim_secmap, TOTAL_SECS(sbi), hint++);
	if (secno < TOTAL_SECS(sbi)) {
		if (sec_usage_check(sbi, secno))
			goto next;
=======
	for_each_set_bit(secno, dirty_i->victim_secmap, MAIN_SECS(sbi)) {
		if (sec_usage_check(sbi, secno))
			continue;

		if (no_fggc_candidate(sbi, secno))
			continue;

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		clear_bit(secno, dirty_i->victim_secmap);
		return secno * sbi->segs_per_sec;
	}
	return NULL_SEGNO;
}

static unsigned int get_cb_cost(struct f2fs_sb_info *sbi, unsigned int segno)
{
	struct sit_info *sit_i = SIT_I(sbi);
	unsigned int secno = GET_SECNO(sbi, segno);
	unsigned int start = secno * sbi->segs_per_sec;
	unsigned long long mtime = 0;
	unsigned int vblocks;
	unsigned char age = 0;
	unsigned char u;
	unsigned int i;

	for (i = 0; i < sbi->segs_per_sec; i++)
		mtime += get_seg_entry(sbi, start + i)->mtime;
	vblocks = get_valid_blocks(sbi, segno, sbi->segs_per_sec);

	mtime = div_u64(mtime, sbi->segs_per_sec);
	vblocks = div_u64(vblocks, sbi->segs_per_sec);

	u = (vblocks * 100) >> sbi->log_blocks_per_seg;

<<<<<<< HEAD
	/* Handle if the system time is changed by user */
=======
	/* Handle if the system time has changed by the user */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	if (mtime < sit_i->min_mtime)
		sit_i->min_mtime = mtime;
	if (mtime > sit_i->max_mtime)
		sit_i->max_mtime = mtime;
	if (sit_i->max_mtime != sit_i->min_mtime)
		age = 100 - div64_u64(100 * (mtime - sit_i->min_mtime),
				sit_i->max_mtime - sit_i->min_mtime);

	return UINT_MAX - ((100 * (100 - u) * age) / (100 + u));
}

<<<<<<< HEAD
static unsigned int get_gc_cost(struct f2fs_sb_info *sbi, unsigned int segno,
					struct victim_sel_policy *p)
=======
static unsigned int get_greedy_cost(struct f2fs_sb_info *sbi,
						unsigned int segno)
{
	unsigned int valid_blocks =
			get_valid_blocks(sbi, segno, sbi->segs_per_sec);

	return IS_DATASEG(get_seg_entry(sbi, segno)->type) ?
				valid_blocks * 2 : valid_blocks;
}

static inline unsigned int get_gc_cost(struct f2fs_sb_info *sbi,
			unsigned int segno, struct victim_sel_policy *p)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
{
	if (p->alloc_mode == SSR)
		return get_seg_entry(sbi, segno)->ckpt_valid_blocks;

	/* alloc_mode == LFS */
	if (p->gc_mode == GC_GREEDY)
<<<<<<< HEAD
		return get_valid_blocks(sbi, segno, sbi->segs_per_sec);
=======
		return get_greedy_cost(sbi, segno);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	else
		return get_cb_cost(sbi, segno);
}

<<<<<<< HEAD
=======
static unsigned int count_bits(const unsigned long *addr,
				unsigned int offset, unsigned int len)
{
	unsigned int end = offset + len, sum = 0;

	while (offset < end) {
		if (test_bit(offset++, addr))
			++sum;
	}
	return sum;
}

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
/*
 * This function is called from two paths.
 * One is garbage collection and the other is SSR segment selection.
 * When it is called during GC, it just gets a victim segment
 * and it does not remove it from dirty seglist.
 * When it is called from SSR segment selection, it finds a segment
 * which has minimum valid blocks and removes it from dirty seglist.
 */
static int get_victim_by_default(struct f2fs_sb_info *sbi,
		unsigned int *result, int gc_type, int type, char alloc_mode)
{
	struct dirty_seglist_info *dirty_i = DIRTY_I(sbi);
	struct victim_sel_policy p;
<<<<<<< HEAD
	unsigned int secno, max_cost;
	int nsearched = 0;
=======
	unsigned int secno, last_victim;
	unsigned int last_segment = MAIN_SEGS(sbi);
	unsigned int nsearched = 0;

	mutex_lock(&dirty_i->seglist_lock);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

	p.alloc_mode = alloc_mode;
	select_policy(sbi, gc_type, type, &p);

	p.min_segno = NULL_SEGNO;
<<<<<<< HEAD
	p.min_cost = max_cost = get_max_cost(sbi, &p);

	mutex_lock(&dirty_i->seglist_lock);

=======
	p.min_cost = get_max_cost(sbi, &p);

	if (p.max_search == 0)
		goto out;

	last_victim = sbi->last_victim[p.gc_mode];
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	if (p.alloc_mode == LFS && gc_type == FG_GC) {
		p.min_segno = check_bg_victims(sbi);
		if (p.min_segno != NULL_SEGNO)
			goto got_it;
	}

	while (1) {
		unsigned long cost;
		unsigned int segno;

<<<<<<< HEAD
		segno = find_next_bit(p.dirty_segmap,
						TOTAL_SEGS(sbi), p.offset);
		if (segno >= TOTAL_SEGS(sbi)) {
			if (sbi->last_victim[p.gc_mode]) {
=======
		segno = find_next_bit(p.dirty_segmap, last_segment, p.offset);
		if (segno >= last_segment) {
			if (sbi->last_victim[p.gc_mode]) {
				last_segment = sbi->last_victim[p.gc_mode];
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
				sbi->last_victim[p.gc_mode] = 0;
				p.offset = 0;
				continue;
			}
			break;
		}
<<<<<<< HEAD
		p.offset = ((segno / p.ofs_unit) * p.ofs_unit) + p.ofs_unit;
		secno = GET_SECNO(sbi, segno);

		if (sec_usage_check(sbi, secno))
			continue;
		if (gc_type == BG_GC && test_bit(secno, dirty_i->victim_secmap))
			continue;
=======

		p.offset = segno + p.ofs_unit;
		if (p.ofs_unit > 1) {
			p.offset -= segno % p.ofs_unit;
			nsearched += count_bits(p.dirty_segmap,
						p.offset - p.ofs_unit,
						p.ofs_unit);
		} else {
			nsearched++;
		}

		secno = GET_SECNO(sbi, segno);

		if (sec_usage_check(sbi, secno))
			goto next;
		if (gc_type == BG_GC && test_bit(secno, dirty_i->victim_secmap))
			goto next;
		if (gc_type == FG_GC && p.alloc_mode == LFS &&
					no_fggc_candidate(sbi, secno))
			goto next;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

		cost = get_gc_cost(sbi, segno, &p);

		if (p.min_cost > cost) {
			p.min_segno = segno;
			p.min_cost = cost;
		}
<<<<<<< HEAD

		if (cost == max_cost)
			continue;

		if (nsearched++ >= MAX_VICTIM_SEARCH) {
			sbi->last_victim[p.gc_mode] = segno;
=======
next:
		if (nsearched >= p.max_search) {
			if (!sbi->last_victim[p.gc_mode] && segno <= last_victim)
				sbi->last_victim[p.gc_mode] = last_victim + 1;
			else
				sbi->last_victim[p.gc_mode] = segno + 1;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
			break;
		}
	}
	if (p.min_segno != NULL_SEGNO) {
got_it:
		if (p.alloc_mode == LFS) {
			secno = GET_SECNO(sbi, p.min_segno);
			if (gc_type == FG_GC)
				sbi->cur_victim_sec = secno;
			else
				set_bit(secno, dirty_i->victim_secmap);
		}
		*result = (p.min_segno / p.ofs_unit) * p.ofs_unit;

		trace_f2fs_get_victim(sbi->sb, type, gc_type, &p,
				sbi->cur_victim_sec,
				prefree_segments(sbi), free_segments(sbi));
	}
<<<<<<< HEAD
=======
out:
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	mutex_unlock(&dirty_i->seglist_lock);

	return (p.min_segno == NULL_SEGNO) ? 0 : 1;
}

static const struct victim_selection default_v_ops = {
	.get_victim = get_victim_by_default,
};

<<<<<<< HEAD
static struct inode *find_gc_inode(nid_t ino, struct list_head *ilist)
{
	struct inode_entry *ie;

	list_for_each_entry(ie, ilist, list)
		if (ie->inode->i_ino == ino)
			return ie->inode;
	return NULL;
}

static void add_gc_inode(struct inode *inode, struct list_head *ilist)
{
	struct inode_entry *new_ie;

	if (inode == find_gc_inode(inode->i_ino, ilist)) {
		iput(inode);
		return;
	}
repeat:
	new_ie = kmem_cache_alloc(winode_slab, GFP_NOFS);
	if (!new_ie) {
		cond_resched();
		goto repeat;
	}
	new_ie->inode = inode;
	list_add_tail(&new_ie->list, ilist);
}

static void put_gc_inode(struct list_head *ilist)
{
	struct inode_entry *ie, *next_ie;
	list_for_each_entry_safe(ie, next_ie, ilist, list) {
		iput(ie->inode);
		list_del(&ie->list);
		kmem_cache_free(winode_slab, ie);
=======
static struct inode *find_gc_inode(struct gc_inode_list *gc_list, nid_t ino)
{
	struct inode_entry *ie;

	ie = radix_tree_lookup(&gc_list->iroot, ino);
	if (ie)
		return ie->inode;
	return NULL;
}

static void add_gc_inode(struct gc_inode_list *gc_list, struct inode *inode)
{
	struct inode_entry *new_ie;

	if (inode == find_gc_inode(gc_list, inode->i_ino)) {
		iput(inode);
		return;
	}
	new_ie = f2fs_kmem_cache_alloc(inode_entry_slab, GFP_NOFS);
	new_ie->inode = inode;

	f2fs_radix_tree_insert(&gc_list->iroot, inode->i_ino, new_ie);
	list_add_tail(&new_ie->list, &gc_list->ilist);
}

static void put_gc_inode(struct gc_inode_list *gc_list)
{
	struct inode_entry *ie, *next_ie;
	list_for_each_entry_safe(ie, next_ie, &gc_list->ilist, list) {
		radix_tree_delete(&gc_list->iroot, ie->inode->i_ino);
		iput(ie->inode);
		list_del(&ie->list);
		kmem_cache_free(inode_entry_slab, ie);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	}
}

static int check_valid_map(struct f2fs_sb_info *sbi,
				unsigned int segno, int offset)
{
	struct sit_info *sit_i = SIT_I(sbi);
	struct seg_entry *sentry;
	int ret;

	mutex_lock(&sit_i->sentry_lock);
	sentry = get_seg_entry(sbi, segno);
	ret = f2fs_test_bit(offset, sentry->cur_valid_map);
	mutex_unlock(&sit_i->sentry_lock);
	return ret;
}

/*
 * This function compares node address got in summary with that in NAT.
 * On validity, copy that node with cold status, otherwise (invalid node)
 * ignore that.
 */
static void gc_node_segment(struct f2fs_sb_info *sbi,
		struct f2fs_summary *sum, unsigned int segno, int gc_type)
{
<<<<<<< HEAD
	bool initial = true;
	struct f2fs_summary *entry;
	int off;
=======
	struct f2fs_summary *entry;
	block_t start_addr;
	int off;
	int phase = 0;

	start_addr = START_BLOCK(sbi, segno);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

next_step:
	entry = sum;

	for (off = 0; off < sbi->blocks_per_seg; off++, entry++) {
		nid_t nid = le32_to_cpu(entry->nid);
		struct page *node_page;
<<<<<<< HEAD

		/* stop BG_GC if there is not enough free sections. */
		if (gc_type == BG_GC && has_not_enough_free_secs(sbi, 0))
=======
		struct node_info ni;

		/* stop BG_GC if there is not enough free sections. */
		if (gc_type == BG_GC && has_not_enough_free_secs(sbi, 0, 0))
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
			return;

		if (check_valid_map(sbi, segno, off) == 0)
			continue;

<<<<<<< HEAD
		if (initial) {
			ra_node_page(sbi, nid);
			continue;
		}
=======
		if (phase == 0) {
			ra_meta_pages(sbi, NAT_BLOCK_OFFSET(nid), 1,
							META_NAT, true);
			continue;
		}

		if (phase == 1) {
			ra_node_page(sbi, nid);
			continue;
		}

		/* phase == 2 */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		node_page = get_node_page(sbi, nid);
		if (IS_ERR(node_page))
			continue;

<<<<<<< HEAD
		/* set page dirty and write it */
		if (gc_type == FG_GC) {
			f2fs_submit_bio(sbi, NODE, true);
			wait_on_page_writeback(node_page);
			set_page_dirty(node_page);
		} else {
			if (!PageWriteback(node_page))
				set_page_dirty(node_page);
		}
		f2fs_put_page(node_page, 1);
		stat_inc_node_blk_count(sbi, 1);
	}

	if (initial) {
		initial = false;
		goto next_step;
	}

	if (gc_type == FG_GC) {
		struct writeback_control wbc = {
			.sync_mode = WB_SYNC_ALL,
			.nr_to_write = LONG_MAX,
			.for_reclaim = 0,
		};
		sync_node_pages(sbi, 0, &wbc);

		/*
		 * In the case of FG_GC, it'd be better to reclaim this victim
		 * completely.
		 */
		if (get_valid_blocks(sbi, segno, 1) != 0)
			goto next_step;
	}
=======
		/* block may become invalid during get_node_page */
		if (check_valid_map(sbi, segno, off) == 0) {
			f2fs_put_page(node_page, 1);
			continue;
		}

		get_node_info(sbi, nid, &ni);
		if (ni.blk_addr != start_addr + off) {
			f2fs_put_page(node_page, 1);
			continue;
		}

		move_node_page(node_page, gc_type);
		stat_inc_node_blk_count(sbi, 1, gc_type);
	}

	if (++phase < 3)
		goto next_step;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
}

/*
 * Calculate start block index indicating the given node offset.
 * Be careful, caller should give this node offset only indicating direct node
 * blocks. If any node offsets, which point the other types of node blocks such
 * as indirect or double indirect node blocks, are given, it must be a caller's
 * bug.
 */
<<<<<<< HEAD
block_t start_bidx_of_node(unsigned int node_ofs)
=======
block_t start_bidx_of_node(unsigned int node_ofs, struct inode *inode)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
{
	unsigned int indirect_blks = 2 * NIDS_PER_BLOCK + 4;
	unsigned int bidx;

	if (node_ofs == 0)
		return 0;

	if (node_ofs <= 2) {
		bidx = node_ofs - 1;
	} else if (node_ofs <= indirect_blks) {
		int dec = (node_ofs - 4) / (NIDS_PER_BLOCK + 1);
		bidx = node_ofs - 2 - dec;
	} else {
		int dec = (node_ofs - indirect_blks - 3) / (NIDS_PER_BLOCK + 1);
		bidx = node_ofs - 5 - dec;
	}
<<<<<<< HEAD
	return bidx * ADDRS_PER_BLOCK + ADDRS_PER_INODE;
}

static int check_dnode(struct f2fs_sb_info *sbi, struct f2fs_summary *sum,
=======
	return bidx * ADDRS_PER_BLOCK + ADDRS_PER_INODE(inode);
}

static bool is_alive(struct f2fs_sb_info *sbi, struct f2fs_summary *sum,
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
		struct node_info *dni, block_t blkaddr, unsigned int *nofs)
{
	struct page *node_page;
	nid_t nid;
	unsigned int ofs_in_node;
	block_t source_blkaddr;

	nid = le32_to_cpu(sum->nid);
	ofs_in_node = le16_to_cpu(sum->ofs_in_node);

	node_page = get_node_page(sbi, nid);
	if (IS_ERR(node_page))
<<<<<<< HEAD
		return 0;
=======
		return false;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03

	get_node_info(sbi, nid, dni);

	if (sum->version != dni->version) {
		f2fs_put_page(node_page, 1);
<<<<<<< HEAD
		return 0;
=======
		return false;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	}

	*nofs = ofs_of_node(node_page);
	source_blkaddr = datablock_addr(node_page, ofs_in_node);
	f2fs_put_page(node_page, 1);

	if (source_blkaddr != blkaddr)
<<<<<<< HEAD
		return 0;
	return 1;
}

static void move_data_page(struct inode *inode, struct page *page, int gc_type)
{
=======
		return false;
	return true;
}

static void move_encrypted_block(struct inode *inode, block_t bidx,
							unsigned int segno, int off)
{
	struct f2fs_io_info fio = {
		.sbi = F2FS_I_SB(inode),
		.type = DATA,
		.op = REQ_OP_READ,
		.op_flags = REQ_SYNC,
		.encrypted_page = NULL,
	};
	struct dnode_of_data dn;
	struct f2fs_summary sum;
	struct node_info ni;
	struct page *page;
	block_t newaddr;
	int err;

	/* do not read out */
	page = f2fs_grab_cache_page(inode->i_mapping, bidx, false);
	if (!page)
		return;

	if (!check_valid_map(F2FS_I_SB(inode), segno, off))
		goto out;

	if (f2fs_is_atomic_file(inode))
		goto out;

	set_new_dnode(&dn, inode, NULL, NULL, 0);
	err = get_dnode_of_data(&dn, bidx, LOOKUP_NODE);
	if (err)
		goto out;

	if (unlikely(dn.data_blkaddr == NULL_ADDR)) {
		ClearPageUptodate(page);
		goto put_out;
	}

	/*
	 * don't cache encrypted data into meta inode until previous dirty
	 * data were writebacked to avoid racing between GC and flush.
	 */
	f2fs_wait_on_page_writeback(page, DATA, true);

	get_node_info(fio.sbi, dn.nid, &ni);
	set_summary(&sum, dn.nid, dn.ofs_in_node, ni.version);

	/* read page */
	fio.page = page;
	fio.new_blkaddr = fio.old_blkaddr = dn.data_blkaddr;

	allocate_data_block(fio.sbi, NULL, fio.old_blkaddr, &newaddr,
							&sum, CURSEG_COLD_DATA);

	fio.encrypted_page = f2fs_grab_cache_page(META_MAPPING(fio.sbi),
							newaddr, true);
	if (!fio.encrypted_page) {
		err = -ENOMEM;
		goto recover_block;
	}

	err = f2fs_submit_page_bio(&fio);
	if (err)
		goto put_page_out;

	/* write page */
	lock_page(fio.encrypted_page);

	if (unlikely(fio.encrypted_page->mapping != META_MAPPING(fio.sbi))) {
		err = -EIO;
		goto put_page_out;
	}
	if (unlikely(!PageUptodate(fio.encrypted_page))) {
		err = -EIO;
		goto put_page_out;
	}

	set_page_dirty(fio.encrypted_page);
	f2fs_wait_on_page_writeback(fio.encrypted_page, DATA, true);
	if (clear_page_dirty_for_io(fio.encrypted_page))
		dec_page_count(fio.sbi, F2FS_DIRTY_META);

	set_page_writeback(fio.encrypted_page);

	/* allocate block address */
	f2fs_wait_on_page_writeback(dn.node_page, NODE, true);

	fio.op = REQ_OP_WRITE;
	fio.op_flags = REQ_SYNC | REQ_NOIDLE;
	fio.new_blkaddr = newaddr;
	f2fs_submit_page_mbio(&fio);

	f2fs_update_data_blkaddr(&dn, newaddr);
	set_inode_flag(inode, FI_APPEND_WRITE);
	if (page->index == 0)
		set_inode_flag(inode, FI_FIRST_BLOCK_WRITTEN);
put_page_out:
	f2fs_put_page(fio.encrypted_page, 1);
recover_block:
	if (err)
		__f2fs_replace_block(fio.sbi, &sum, newaddr, fio.old_blkaddr,
								true, true);
put_out:
	f2fs_put_dnode(&dn);
out:
	f2fs_put_page(page, 1);
}

static void move_data_page(struct inode *inode, block_t bidx, int gc_type,
							unsigned int segno, int off)
{
	struct page *page;

	page = get_lock_data_page(inode, bidx, true);
	if (IS_ERR(page))
		return;

	if (!check_valid_map(F2FS_I_SB(inode), segno, off))
		goto out;

	if (f2fs_is_atomic_file(inode))
		goto out;

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	if (gc_type == BG_GC) {
		if (PageWriteback(page))
			goto out;
		set_page_dirty(page);
		set_cold_data(page);
	} else {
<<<<<<< HEAD
		struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);

		if (PageWriteback(page)) {
			f2fs_submit_bio(sbi, DATA, true);
			wait_on_page_writeback(page);
		}

		if (clear_page_dirty_for_io(page) &&
			S_ISDIR(inode->i_mode)) {
			dec_page_count(sbi, F2FS_DIRTY_DENTS);
			inode_dec_dirty_dents(inode);
		}
		set_cold_data(page);
		do_write_data_page(page);
		clear_cold_data(page);
=======
		struct f2fs_io_info fio = {
			.sbi = F2FS_I_SB(inode),
			.type = DATA,
			.op = REQ_OP_WRITE,
			.op_flags = REQ_SYNC | REQ_NOIDLE,
			.page = page,
			.encrypted_page = NULL,
		};
		bool is_dirty = PageDirty(page);
		int err;

retry:
		set_page_dirty(page);
		f2fs_wait_on_page_writeback(page, DATA, true);
		if (clear_page_dirty_for_io(page)) {
			inode_dec_dirty_pages(inode);
			remove_dirty_inode(inode);
		}

		set_cold_data(page);

		err = do_write_data_page(&fio);
		if (err == -ENOMEM && is_dirty) {
			congestion_wait(BLK_RW_ASYNC, HZ/50);
			goto retry;
		}
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	}
out:
	f2fs_put_page(page, 1);
}

/*
 * This function tries to get parent node of victim data block, and identifies
 * data block validity. If the block is valid, copy that with cold status and
 * modify parent node.
 * If the parent node is not valid or the data block address is different,
 * the victim data block is ignored.
 */
static void gc_data_segment(struct f2fs_sb_info *sbi, struct f2fs_summary *sum,
<<<<<<< HEAD
		struct list_head *ilist, unsigned int segno, int gc_type)
=======
		struct gc_inode_list *gc_list, unsigned int segno, int gc_type)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
{
	struct super_block *sb = sbi->sb;
	struct f2fs_summary *entry;
	block_t start_addr;
	int off;
	int phase = 0;

	start_addr = START_BLOCK(sbi, segno);

next_step:
	entry = sum;

	for (off = 0; off < sbi->blocks_per_seg; off++, entry++) {
		struct page *data_page;
		struct inode *inode;
		struct node_info dni; /* dnode info for the data */
		unsigned int ofs_in_node, nofs;
		block_t start_bidx;
<<<<<<< HEAD

		/* stop BG_GC if there is not enough free sections. */
		if (gc_type == BG_GC && has_not_enough_free_secs(sbi, 0))
=======
		nid_t nid = le32_to_cpu(entry->nid);

		/* stop BG_GC if there is not enough free sections. */
		if (gc_type == BG_GC && has_not_enough_free_secs(sbi, 0, 0))
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
			return;

		if (check_valid_map(sbi, segno, off) == 0)
			continue;

		if (phase == 0) {
<<<<<<< HEAD
			ra_node_page(sbi, le32_to_cpu(entry->nid));
=======
			ra_meta_pages(sbi, NAT_BLOCK_OFFSET(nid), 1,
							META_NAT, true);
			continue;
		}

		if (phase == 1) {
			ra_node_page(sbi, nid);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
			continue;
		}

		/* Get an inode by ino with checking validity */
<<<<<<< HEAD
		if (check_dnode(sbi, entry, &dni, start_addr + off, &nofs) == 0)
			continue;

		if (phase == 1) {
=======
		if (!is_alive(sbi, entry, &dni, start_addr + off, &nofs))
			continue;

		if (phase == 2) {
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
			ra_node_page(sbi, dni.ino);
			continue;
		}

<<<<<<< HEAD
		start_bidx = start_bidx_of_node(nofs);
		ofs_in_node = le16_to_cpu(entry->ofs_in_node);

		if (phase == 2) {
			inode = f2fs_iget(sb, dni.ino);
			if (IS_ERR(inode))
				continue;

			data_page = find_data_page(inode,
					start_bidx + ofs_in_node, false);
			if (IS_ERR(data_page))
				goto next_iput;

			f2fs_put_page(data_page, 0);
			add_gc_inode(inode, ilist);
		} else {
			inode = find_gc_inode(dni.ino, ilist);
			if (inode) {
				data_page = get_lock_data_page(inode,
						start_bidx + ofs_in_node);
				if (IS_ERR(data_page))
					continue;
				move_data_page(inode, data_page, gc_type);
				stat_inc_data_blk_count(sbi, 1);
			}
		}
		continue;
next_iput:
		iput(inode);
	}

	if (++phase < 4)
		goto next_step;

	if (gc_type == FG_GC) {
		f2fs_submit_bio(sbi, DATA, true);

		/*
		 * In the case of FG_GC, it'd be better to reclaim this victim
		 * completely.
		 */
		if (get_valid_blocks(sbi, segno, 1) != 0) {
			phase = 2;
			goto next_step;
		}
	}
}

static int __get_victim(struct f2fs_sb_info *sbi, unsigned int *victim,
						int gc_type, int type)
{
	struct sit_info *sit_i = SIT_I(sbi);
	int ret;
	mutex_lock(&sit_i->sentry_lock);
	ret = DIRTY_I(sbi)->v_ops->get_victim(sbi, victim, gc_type, type, LFS);
=======
		ofs_in_node = le16_to_cpu(entry->ofs_in_node);

		if (phase == 3) {
			inode = f2fs_iget(sb, dni.ino);
			if (IS_ERR(inode) || is_bad_inode(inode))
				continue;

			/* if encrypted inode, let's go phase 3 */
			if (f2fs_encrypted_inode(inode) &&
						S_ISREG(inode->i_mode)) {
				add_gc_inode(gc_list, inode);
				continue;
			}

			start_bidx = start_bidx_of_node(nofs, inode);
			data_page = get_read_data_page(inode,
					start_bidx + ofs_in_node, REQ_RAHEAD,
					true);
			if (IS_ERR(data_page)) {
				iput(inode);
				continue;
			}

			f2fs_put_page(data_page, 0);
			add_gc_inode(gc_list, inode);
			continue;
		}

		/* phase 4 */
		inode = find_gc_inode(gc_list, dni.ino);
		if (inode) {
			struct f2fs_inode_info *fi = F2FS_I(inode);
			bool locked = false;

			if (S_ISREG(inode->i_mode)) {
				if (!down_write_trylock(&fi->dio_rwsem[READ]))
					continue;
				if (!down_write_trylock(
						&fi->dio_rwsem[WRITE])) {
					up_write(&fi->dio_rwsem[READ]);
					continue;
				}
				locked = true;
			}

			start_bidx = start_bidx_of_node(nofs, inode)
								+ ofs_in_node;
			if (f2fs_encrypted_inode(inode) && S_ISREG(inode->i_mode))
				move_encrypted_block(inode, start_bidx, segno, off);
			else
				move_data_page(inode, start_bidx, gc_type, segno, off);

			if (locked) {
				up_write(&fi->dio_rwsem[WRITE]);
				up_write(&fi->dio_rwsem[READ]);
			}

			stat_inc_data_blk_count(sbi, 1, gc_type);
		}
	}

	if (++phase < 5)
		goto next_step;
}

static int __get_victim(struct f2fs_sb_info *sbi, unsigned int *victim,
			int gc_type)
{
	struct sit_info *sit_i = SIT_I(sbi);
	int ret;

	mutex_lock(&sit_i->sentry_lock);
	ret = DIRTY_I(sbi)->v_ops->get_victim(sbi, victim, gc_type,
					      NO_CHECK_TYPE, LFS);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	mutex_unlock(&sit_i->sentry_lock);
	return ret;
}

<<<<<<< HEAD
static void do_garbage_collect(struct f2fs_sb_info *sbi, unsigned int segno,
				struct list_head *ilist, int gc_type)
=======
static int do_garbage_collect(struct f2fs_sb_info *sbi,
				unsigned int start_segno,
				struct gc_inode_list *gc_list, int gc_type)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
{
	struct page *sum_page;
	struct f2fs_summary_block *sum;
	struct blk_plug plug;
<<<<<<< HEAD

	/* read segment summary of victim */
	sum_page = get_sum_page(sbi, segno);
	if (IS_ERR(sum_page))
		return;

	blk_start_plug(&plug);

	sum = page_address(sum_page);

	switch (GET_SUM_TYPE((&sum->footer))) {
	case SUM_TYPE_NODE:
		gc_node_segment(sbi, sum->entries, segno, gc_type);
		break;
	case SUM_TYPE_DATA:
		gc_data_segment(sbi, sum->entries, ilist, segno, gc_type);
		break;
	}
	blk_finish_plug(&plug);

	stat_inc_seg_count(sbi, GET_SUM_TYPE((&sum->footer)));
	stat_inc_call_count(sbi->stat_info);

	f2fs_put_page(sum_page, 1);
}

int f2fs_gc(struct f2fs_sb_info *sbi)
{
	struct list_head ilist;
	unsigned int segno, i;
	int gc_type = BG_GC;
	int nfree = 0;
	int ret = -1;

	INIT_LIST_HEAD(&ilist);
gc_more:
	if (!(sbi->sb->s_flags & MS_ACTIVE))
		goto stop;

	if (gc_type == BG_GC && has_not_enough_free_secs(sbi, nfree)) {
		gc_type = FG_GC;
		write_checkpoint(sbi, false);
	}

	if (!__get_victim(sbi, &segno, gc_type, NO_CHECK_TYPE))
		goto stop;
	ret = 0;

	for (i = 0; i < sbi->segs_per_sec; i++)
		do_garbage_collect(sbi, segno + i, &ilist, gc_type);

	if (gc_type == FG_GC) {
		sbi->cur_victim_sec = NULL_SEGNO;
		nfree++;
		WARN_ON(get_valid_blocks(sbi, segno, sbi->segs_per_sec));
	}

	if (has_not_enough_free_secs(sbi, nfree))
		goto gc_more;

	if (gc_type == FG_GC)
		write_checkpoint(sbi, false);
stop:
	mutex_unlock(&sbi->gc_mutex);

	put_gc_inode(&ilist);
=======
	unsigned int segno = start_segno;
	unsigned int end_segno = start_segno + sbi->segs_per_sec;
	int sec_freed = 0;
	unsigned char type = IS_DATASEG(get_seg_entry(sbi, segno)->type) ?
						SUM_TYPE_DATA : SUM_TYPE_NODE;

	/* readahead multi ssa blocks those have contiguous address */
	if (sbi->segs_per_sec > 1)
		ra_meta_pages(sbi, GET_SUM_BLOCK(sbi, segno),
					sbi->segs_per_sec, META_SSA, true);

	/* reference all summary page */
	while (segno < end_segno) {
		sum_page = get_sum_page(sbi, segno++);
		unlock_page(sum_page);
	}

	blk_start_plug(&plug);

	for (segno = start_segno; segno < end_segno; segno++) {

		/* find segment summary of victim */
		sum_page = find_get_page(META_MAPPING(sbi),
					GET_SUM_BLOCK(sbi, segno));
		f2fs_put_page(sum_page, 0);

		if (get_valid_blocks(sbi, segno, 1) == 0 ||
				!PageUptodate(sum_page) ||
				unlikely(f2fs_cp_error(sbi)))
			goto next;

		sum = page_address(sum_page);
		f2fs_bug_on(sbi, type != GET_SUM_TYPE((&sum->footer)));

		/*
		 * this is to avoid deadlock:
		 * - lock_page(sum_page)         - f2fs_replace_block
		 *  - check_valid_map()            - mutex_lock(sentry_lock)
		 *   - mutex_lock(sentry_lock)     - change_curseg()
		 *                                  - lock_page(sum_page)
		 */

		if (type == SUM_TYPE_NODE)
			gc_node_segment(sbi, sum->entries, segno, gc_type);
		else
			gc_data_segment(sbi, sum->entries, gc_list, segno,
								gc_type);

		stat_inc_seg_count(sbi, type, gc_type);
next:
		f2fs_put_page(sum_page, 0);
	}

	if (gc_type == FG_GC)
		f2fs_submit_merged_bio(sbi,
				(type == SUM_TYPE_NODE) ? NODE : DATA, WRITE);

	blk_finish_plug(&plug);

	if (gc_type == FG_GC &&
		get_valid_blocks(sbi, start_segno, sbi->segs_per_sec) == 0)
		sec_freed = 1;

	stat_inc_call_count(sbi->stat_info);

	return sec_freed;
}

int f2fs_gc(struct f2fs_sb_info *sbi, bool sync, bool background)
{
	unsigned int segno;
	int gc_type = sync ? FG_GC : BG_GC;
	int sec_freed = 0;
	int ret = -EINVAL;
	struct cp_control cpc;
	struct gc_inode_list gc_list = {
		.ilist = LIST_HEAD_INIT(gc_list.ilist),
		.iroot = RADIX_TREE_INIT(GFP_NOFS),
	};

	cpc.reason = __get_cp_reason(sbi);
gc_more:
	if (unlikely(!(sbi->sb->s_flags & MS_ACTIVE)))
		goto stop;
	if (unlikely(f2fs_cp_error(sbi))) {
		ret = -EIO;
		goto stop;
	}

	if (gc_type == BG_GC && has_not_enough_free_secs(sbi, 0, 0)) {
		/*
		 * For example, if there are many prefree_segments below given
		 * threshold, we can make them free by checkpoint. Then, we
		 * secure free segments which doesn't need fggc any more.
		 */
		ret = write_checkpoint(sbi, &cpc);
		if (ret)
			goto stop;
		if (has_not_enough_free_secs(sbi, 0, 0))
			gc_type = FG_GC;
	}

	/* f2fs_balance_fs doesn't need to do BG_GC in critical path. */
	if (gc_type == BG_GC && !background)
		goto stop;
	if (!__get_victim(sbi, &segno, gc_type))
		goto stop;
	ret = 0;

	if (do_garbage_collect(sbi, segno, &gc_list, gc_type) &&
			gc_type == FG_GC)
		sec_freed++;

	if (gc_type == FG_GC)
		sbi->cur_victim_sec = NULL_SEGNO;

	if (!sync) {
		if (has_not_enough_free_secs(sbi, sec_freed, 0))
			goto gc_more;

		if (gc_type == FG_GC)
			ret = write_checkpoint(sbi, &cpc);
	}
stop:
	mutex_unlock(&sbi->gc_mutex);

	put_gc_inode(&gc_list);

	if (sync)
		ret = sec_freed ? 0 : -EAGAIN;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
	return ret;
}

void build_gc_manager(struct f2fs_sb_info *sbi)
{
<<<<<<< HEAD
	DIRTY_I(sbi)->v_ops = &default_v_ops;
}

int __init create_gc_caches(void)
{
	winode_slab = f2fs_kmem_cache_create("f2fs_gc_inodes",
			sizeof(struct inode_entry), NULL);
	if (!winode_slab)
		return -ENOMEM;
	return 0;
}

void destroy_gc_caches(void)
{
	kmem_cache_destroy(winode_slab);
=======
	u64 main_count, resv_count, ovp_count, blocks_per_sec;

	DIRTY_I(sbi)->v_ops = &default_v_ops;

	/* threshold of # of valid blocks in a section for victims of FG_GC */
	main_count = SM_I(sbi)->main_segments << sbi->log_blocks_per_seg;
	resv_count = SM_I(sbi)->reserved_segments << sbi->log_blocks_per_seg;
	ovp_count = SM_I(sbi)->ovp_segments << sbi->log_blocks_per_seg;
	blocks_per_sec = sbi->blocks_per_seg * sbi->segs_per_sec;

	sbi->fggc_threshold = div64_u64((main_count - ovp_count) * blocks_per_sec,
					(main_count - resv_count));
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
}
