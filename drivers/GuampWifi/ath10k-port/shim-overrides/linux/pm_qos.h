/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx_os.h"
#define PM_QOS_DEFAULT_VALUE (-1)
struct pm_qos_request { int value; bool active; };
void cpu_latency_qos_add_request(struct pm_qos_request *,s32);
void cpu_latency_qos_update_request(struct pm_qos_request *,s32);
void cpu_latency_qos_remove_request(struct pm_qos_request *);
static inline bool cpu_latency_qos_request_active(struct pm_qos_request *r) { return r->active; }
