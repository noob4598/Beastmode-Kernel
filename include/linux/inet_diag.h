#ifndef _INET_DIAG_H_
#define _INET_DIAG_H_ 1

#include <uapi/linux/inet_diag.h>

<<<<<<< HEAD
<<<<<<< HEAD
=======
struct net;
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
struct sock;
struct inet_hashinfo;
struct nlattr;
struct nlmsghdr;
struct sk_buff;
struct netlink_callback;

struct inet_diag_handler {
	void			(*dump)(struct sk_buff *skb,
					struct netlink_callback *cb,
					struct inet_diag_req_v2 *r,
					struct nlattr *bc);

	int			(*dump_one)(struct sk_buff *in_skb,
					const struct nlmsghdr *nlh,
					struct inet_diag_req_v2 *req);

	void			(*idiag_get_info)(struct sock *sk,
						  struct inet_diag_msg *r,
						  void *info);
<<<<<<< HEAD
<<<<<<< HEAD
=======

	int			(*destroy)(struct sk_buff *in_skb,
					   struct inet_diag_req_v2 *req);

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
	__u16                   idiag_type;
};

struct inet_connection_sock;
int inet_sk_diag_fill(struct sock *sk, struct inet_connection_sock *icsk,
			      struct sk_buff *skb, struct inet_diag_req_v2 *req,
			      struct user_namespace *user_ns,
			      u32 pid, u32 seq, u16 nlmsg_flags,
<<<<<<< HEAD
<<<<<<< HEAD
			      const struct nlmsghdr *unlh);
=======
			      const struct nlmsghdr *unlh, bool net_admin);
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
			      const struct nlmsghdr *unlh);
>>>>>>> 2617302... source
void inet_diag_dump_icsk(struct inet_hashinfo *h, struct sk_buff *skb,
		struct netlink_callback *cb, struct inet_diag_req_v2 *r,
		struct nlattr *bc);
int inet_diag_dump_one_icsk(struct inet_hashinfo *hashinfo,
		struct sk_buff *in_skb, const struct nlmsghdr *nlh,
		struct inet_diag_req_v2 *req);

<<<<<<< HEAD
<<<<<<< HEAD
=======
struct sock *inet_diag_find_one_icsk(struct net *net,
				     struct inet_hashinfo *hashinfo,
				     struct inet_diag_req_v2 *req);

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
int inet_diag_bc_sk(const struct nlattr *_bc, struct sock *sk);

extern int  inet_diag_register(const struct inet_diag_handler *handler);
extern void inet_diag_unregister(const struct inet_diag_handler *handler);
#endif /* _INET_DIAG_H_ */
