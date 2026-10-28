/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx_os.h"
struct nvmem_cell;
struct nvmem_cell *nvmem_cell_get(struct device *,const char *);
struct nvmem_cell *devm_nvmem_cell_get(struct device *,const char *);
void *nvmem_cell_read(struct nvmem_cell *,size_t *);
void nvmem_cell_put(struct nvmem_cell *);
