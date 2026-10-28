/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx.h"
struct gw_kfifo { void *data; unsigned int head,tail,mask,size; };
#define DECLARE_KFIFO_PTR(name,type) struct { struct gw_kfifo fifo; type *typecheck; } name
int gw_kfifo_alloc(struct gw_kfifo *,unsigned int,unsigned int);
void gw_kfifo_free(struct gw_kfifo *);
unsigned int gw_kfifo_in(struct gw_kfifo *,const void *,unsigned int);
unsigned int gw_kfifo_out(struct gw_kfifo *,void *,unsigned int);
#define kfifo_alloc(f,n,gfp) gw_kfifo_alloc(&(f)->fifo,(n),sizeof(*(f)->typecheck))
#define kfifo_free(f) gw_kfifo_free(&(f)->fifo)
#define kfifo_in(f,p,n) gw_kfifo_in(&(f)->fifo,(p),(n))
#define kfifo_out(f,p,n) gw_kfifo_out(&(f)->fifo,(p),(n))
#define kfifo_put(f,v) ({ __typeof__(*(f)->typecheck) _v=(v); gw_kfifo_in(&(f)->fifo,&_v,1); })
#define kfifo_get(f,p) gw_kfifo_out(&(f)->fifo,(p),1)
#define kfifo_len(f) ((f)->fifo.head-(f)->fifo.tail)
#define kfifo_is_empty(f) (kfifo_len(f)==0)
#define kfifo_is_full(f) (kfifo_len(f)>(f)->fifo.mask)
#define kfifo_reset(f) ((f)->fifo.head=(f)->fifo.tail=0)
