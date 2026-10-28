/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx_os.h"
struct dma_pool;
struct dma_pool *dma_pool_create(const char *,struct device *,size_t,size_t,size_t);
void dma_pool_destroy(struct dma_pool *);
void *dma_pool_alloc(struct dma_pool *,gfp_t,dma_addr_t *);
void dma_pool_free(struct dma_pool *,void *,dma_addr_t);
