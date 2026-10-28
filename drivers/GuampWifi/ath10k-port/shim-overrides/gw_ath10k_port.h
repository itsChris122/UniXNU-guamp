/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx_pci.h"
#undef IS_ENABLED
#define GW_ARG_1 0,
#define GW_SECOND(ignored,value,...) value
#define GW_DEFINED(x) GW_DEFINED_I(x)
#define GW_DEFINED_I(value) GW_DEFINED_II(GW_ARG_##value)
#define GW_DEFINED_II(arg) GW_SECOND(arg 1,0)
#define IS_ENABLED(option) (GW_DEFINED(option)||GW_DEFINED(option##_MODULE))
#define IS_REACHABLE(option) IS_ENABLED(option)
#define __nonstring
struct timespec64 { s64 tv_sec; long tv_nsec; };
typedef uint32_t u_int32_t;
#define ENOSR 63
#define ELNRNG 48
#define ENOMEDIUM 123
#define DMA_MAPPING_ERROR (~(dma_addr_t)0)
#define round_up(x,y) ALIGN(x,y)
static inline void bitmap_clear(unsigned long *bits,unsigned start,unsigned count) {
  for(unsigned i=0;i<count;i++) clear_bit(start+i,bits);
}
/* The UniXNU dispatcher already runs NAPI from a kernel thread. */
#define netif_threaded_enable(device) ((void)(device))
#define lockdep_assert_not_held(l) ((void)(l))
#define KERN_DEBUG ""
#define KERN_ERR ""
#define KERN_WARNING ""
#define KERN_INFO ""
#define printk gw_printk
int gw_printk(const char *,...);
int of_property_read_string_index(const struct device_node *,const char *,int,const char **);
int of_property_read_u8_array(const struct device_node *,const char *,u8 *,size_t);
int of_property_read_u8(const struct device_node *,const char *,u8 *);
int device_get_mac_address(struct device *,u8 *);
void eth_random_addr(u8 *);
size_t strlcat(char *,const char *,size_t);
void *devm_kzalloc(struct device *,size_t,gfp_t);
#define BITS_PER_TYPE(type) (sizeof(type)*8)
static inline bool bitmap_empty(const unsigned long *a,unsigned int n) {
  for(unsigned int i=0;i<n/64;i++) if(a[i]) return false;
  return !(n%64&&(a[n/64]&((1UL<<(n%64))-1)));
}
struct sk_buff *skb_get(struct sk_buff *);
static inline struct sk_buff *skb_dequeue_tail(struct sk_buff_head *q) {
  if(skb_queue_empty(q)) return 0;
  struct sk_buff *s=q->prev; __skb_unlink(s,q); return s;
}
static inline void skb_queue_splice_init(struct sk_buff_head *a,struct sk_buff_head *b) {
  struct sk_buff *s; while((s=skb_dequeue_tail(a))) __skb_queue_head(b,s);
}
/* guamp uses Device Tree; the ACPI provider is absent, so the WRDD path exits
 * with EOPNOTSUPP before attempting to evaluate anything. */
typedef void *acpi_handle;
typedef char *acpi_string;
typedef unsigned int acpi_status;
union acpi_object { unsigned int type; struct { unsigned int type; unsigned int count; union acpi_object *elements; } package; struct { unsigned int type; u64 value; } integer; };
struct acpi_buffer { unsigned long length; void *pointer; };
#define ACPI_TYPE_INTEGER 1
#define ACPI_TYPE_PACKAGE 4
#define ACPI_ALLOCATE_BUFFER (~0UL)
#define ACPI_HANDLE(d) ((acpi_handle)0)
#define ACPI_FAILURE(s) ((s)!=0)
acpi_status acpi_get_handle(acpi_handle,acpi_string,acpi_handle *);
acpi_status acpi_evaluate_object(acpi_handle,acpi_string,void *,struct acpi_buffer *);
