/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx.h"
/* The existing UniXNU shim uses a single serialized dispatcher. */
struct hlist_node { struct hlist_node *next,**pprev; };
struct hlist_head { struct hlist_node *first; };
#define HLIST_HEAD_INIT { .first=0 }
static inline void INIT_HLIST_HEAD(struct hlist_head *h) { h->first=0; }
static inline void INIT_HLIST_NODE(struct hlist_node *n) { n->next=0; n->pprev=0; }
static inline bool hlist_empty(const struct hlist_head *h) { return !h->first; }
static inline bool hlist_unhashed(const struct hlist_node *n) { return !n->pprev; }
static inline void hlist_add_head(struct hlist_node *n,struct hlist_head *h) {
  n->next=h->first; if(n->next) n->next->pprev=&n->next; h->first=n; n->pprev=&h->first;
}
static inline void hlist_del_init(struct hlist_node *n) {
  if(n->pprev) { *n->pprev=n->next; if(n->next) n->next->pprev=n->pprev; }
  INIT_HLIST_NODE(n);
}
#define hlist_add_head_rcu(n,h) hlist_add_head(n,h)
#define hlist_del_init_rcu(n) hlist_del_init(n)
#define hlist_del(n) hlist_del_init(n)
#define hlist_entry_safe(p,t,m) ({ __typeof__(p) _p=(p); _p?container_of(_p,t,m):0; })
#define hlist_for_each_entry(p,h,m) for((p)=hlist_entry_safe((h)->first,__typeof__(*(p)),m);(p);(p)=hlist_entry_safe((p)->m.next,__typeof__(*(p)),m))
#define hlist_for_each_entry_rcu(p,h,m,...) hlist_for_each_entry(p,h,m)
#define hlist_for_each_entry_safe(p,n,h,m) for((p)=hlist_entry_safe((h)->first,__typeof__(*(p)),m);(p)&&((n)=(p)->m.next,1);(p)=hlist_entry_safe((n),__typeof__(*(p)),m))
