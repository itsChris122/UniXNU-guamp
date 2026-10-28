/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#pragma once
/* CONFIG_ATH10K_LEDS is disabled: guamp has no host WLAN activity LED. */
struct led_classdev { const char *name; };
struct gpio_led { const char *name; unsigned int gpio; unsigned int active_low; };
