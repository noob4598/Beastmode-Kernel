#ifndef __LOCKD_NETNS_H__
#define __LOCKD_NETNS_H__

#include <linux/fs.h>
#include <net/netns/generic.h>

struct lockd_net {
	unsigned int nlmsvc_users;
	unsigned long next_gc;
	unsigned long nrhosts;

	struct delayed_work grace_period_end;
	struct lock_manager lockd_manager;
	struct list_head grace_list;

	spinlock_t nsm_clnt_lock;
	unsigned int nsm_users;
	struct rpc_clnt *nsm_clnt;
<<<<<<< HEAD
<<<<<<< HEAD
=======
	struct list_head nsm_handles;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
};

extern int lockd_net_id;

#endif
