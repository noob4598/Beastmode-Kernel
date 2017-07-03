#ifndef _LINEAR_H
#define _LINEAR_H

struct dev_info {
	struct md_rdev	*rdev;
	sector_t	end_sector;
};

struct linear_conf
{
	struct rcu_head		rcu;
	sector_t		array_sectors;
<<<<<<< HEAD
<<<<<<< HEAD
=======
	int			raid_disks; /* a copy of mddev->raid_disks */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	struct dev_info		disks[0];
};
#endif
