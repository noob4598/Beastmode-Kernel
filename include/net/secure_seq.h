#ifndef _NET_SECURE_SEQ
#define _NET_SECURE_SEQ

#include <linux/types.h>

<<<<<<< HEAD
<<<<<<< HEAD
extern __u32 secure_ip_id(__be32 daddr);
extern __u32 secure_ipv6_id(const __be32 daddr[4]);
=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
extern __u32 secure_ip_id(__be32 daddr);
extern __u32 secure_ipv6_id(const __be32 daddr[4]);
>>>>>>> 2617302... source
extern u32 secure_ipv4_port_ephemeral(__be32 saddr, __be32 daddr, __be16 dport);
extern u32 secure_ipv6_port_ephemeral(const __be32 *saddr, const __be32 *daddr,
				      __be16 dport);
extern __u32 secure_tcp_sequence_number(__be32 saddr, __be32 daddr,
					__be16 sport, __be16 dport);
extern __u32 secure_tcpv6_sequence_number(const __be32 *saddr, const __be32 *daddr,
					  __be16 sport, __be16 dport);
extern u64 secure_dccp_sequence_number(__be32 saddr, __be32 daddr,
				       __be16 sport, __be16 dport);
extern u64 secure_dccpv6_sequence_number(__be32 *saddr, __be32 *daddr,
					 __be16 sport, __be16 dport);

#endif /* _NET_SECURE_SEQ */
