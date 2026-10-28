/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx_ath.h"
/* All DT lookups remain failures until provided by the SNOC board adapter. */
const void *device_get_match_data(const struct device *);
