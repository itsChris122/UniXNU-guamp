/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
#include "lx.h"
#ifndef roundup_pow_of_two
#define roundup_pow_of_two(n) ((n)<=1?1UL:(1UL<<(64-__builtin_clzll((unsigned long long)(n)-1))))
#endif
