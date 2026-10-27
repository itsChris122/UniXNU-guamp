// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp (Moto G9 Play, SM6115 / bengal) PMIC facts for the buttons, the charger and the fuel gauge. Every constant states
// its source; nothing here touches hardware. Not linked into any kernel yet: the drivers that use it are still to come.
//
// All of it sits behind one bus: the SPMI PMIC arbiter. Sources: guamp boot_b DTB (decompiled ~/moto/guamp.dts) and the
// dtbo_b overlay (~/moto/dtbo/2.dts, board-id 0x400 0xb200).
#ifndef GUAMP_PMIC_BOARD_H
#define GUAMP_PMIC_BOARD_H
#include <stdint.h>

// qcom,spmi@1c40000, compatible "qcom,spmi-pmic-arb", qcom,ee = <0>, qcom,channel = <0>, interrupts <0 0xb7 4> (SPI 183):
//   reg-names "core" 0x1c40000/0x1100, "chnls" 0x1e00000/0x2000000, "obsrvr" 0x3e00000/0x100000,
//             "intr" 0x3f00000/0xa0000, "cnfg" 0x1c0a000/0x26000
// The 0x1100-byte core and the 32 MiB channel window are the PMIC arbiter v5 layout (spmi-pmic-arb.c: the APID table in
// "core", one 0x10000-byte page per APID in "chnls" for writes and in "obsrvr" for reads). Check the version register
// (core + 0x0) on the device before the first command; the per-APID command layout depends on it.
#define GP_SPMI_CORE 0x01c40000ULL
#define GP_SPMI_CORE_SIZE 0x1100ULL
#define GP_SPMI_CHNLS 0x01e00000ULL
#define GP_SPMI_CHNLS_SIZE 0x2000000ULL
#define GP_SPMI_OBSRVR 0x03e00000ULL
#define GP_SPMI_OBSRVR_SIZE 0x100000ULL
#define GP_SPMI_INTR 0x03f00000ULL
#define GP_SPMI_CNFG 0x01c0a000ULL
#define GP_SPMI_EE 0
#define GP_SPMI_IRQ_SPI 183

// Slave IDs (child nodes qcom,pm6125@0/@1, qcom,pmi632@2/@3, reg <sid 0>).
#define GP_SID_PM6125 0
#define GP_SID_PM6125_B 1
#define GP_SID_PMI632 2
#define GP_SID_PMI632_B 3

// Power and Volume Down: PM6125 power-on module qcom,power-on@800 ("qcom,qpnp-power-on"), interrupts kpdpwr (bit 0) and
// resin (bit 1); qcom,pon_1 pon-type 0 = KPDPWR, linux,code 0x74 (KEY_POWER); qcom,pon_2 pon-type 1 = RESIN, linux,code
// 0x72 (KEY_VOLUMEDOWN). The real-time interrupt status of a qpnp peripheral is at +0x10 (INT_RT_STS), so both keys are
// one read of PM6125 0x810: bit set = pressed (the PON inputs are reported active).
#define GP_PON_BASE 0x0800
#define GP_PON_RT_STS (GP_PON_BASE + 0x10)
#define GP_PON_KPDPWR (1u << 0)
#define GP_PON_RESIN (1u << 1)
// PMIC_PON qcom,kpdpwr-sw-debounce / qcom,pon-dbc-delay = <0x3d09> (15625 us): debounce in software on our side too.
#define GP_PON_DEBOUNCE_US 15625

// Volume Up: gpio-keys vol_up, gpios = <&pm6125_gpios 5 GPIO_ACTIVE_LOW>, linux,code 0x73 (KEY_VOLUMEUP); pinctrl
// key_vol_up_default: pins "gpio5", function "normal", input-enable, bias-pull-up. pm6125_gpios = pinctrl@c000
// ("qcom,spmi-gpio", reg 0xc000 0x900): one 0x100 page per GPIO starting at GPIO 1, so GPIO 5 is PM6125 0xc400;
// pinctrl-spmi-gpio.c: STATUS1 at +0x08, bit 0 = input level (active low: 0 = pressed).
#define GP_VOLUP_GPIO_BASE 0xc400
#define GP_VOLUP_STATUS1 (GP_VOLUP_GPIO_BASE + 0x08)
#define GP_VOLUP_LEVEL (1u << 0)
// Google Assistant key: gpio-keys assist_key, gpios = <&tlmm 0x21 1> (TLMM GPIO 33, active low), linux,code 0xd9,
// debounce-interval 15 ms; pinctrl gpio_key_active (dtbo fragment@42): function "gpio", drive-strength 2, bias-pull-up.
// A TLMM pin, not a PMIC one, and in the EAST tile: pinctrl-bengal.c PINGROUP(33, EAST, ...), EAST = 0x900000 inside
// the TLMM window pinctrl@400000 (reg <0x400000 0xc00000>), so the pin's 0x1000 block is 0xd00000 + 33 * 0x1000:
// cfg at +0 (pull 1:0, 3 = pull-up; function 5:2; drive 8:6; OE bit 9), IN_OUT at +4 (bit 0 = input level).
// No gpio-reserved-ranges in the guamp DT. Used as the ring/silent switch (PmicService.inc).
#define GP_ASSIST_TLMM_GPIO 33
#define GP_ASSIST_TLMM_BLOCK 0x00d21000ULL
#define GP_ASSIST_CFG 0x0
#define GP_ASSIST_IN_OUT 0x4
#define GP_ASSIST_CFG_MASK 0x23fu      // OE, function, pull: what must read "GPIO input with pull-up"
#define GP_ASSIST_CFG_INPUT_PU 0x003u

// Charger: PMI632 qcom,qpnp-smb5 (SID 2) with peripherals chgr@1000, dcdc@1100, batif@1200, usb@1300, typec@1500,
// misc@1600 (misc has wdog-snarl/wdog-bark: the charger watchdog falls back to safe currents when nobody pets it, so the
// phone still charges slowly without a driver).
#define GP_SMB5_CHGR 0x1000
#define GP_SMB5_DCDC 0x1100
#define GP_SMB5_BATIF 0x1200
#define GP_SMB5_USB 0x1300
#define GP_SMB5_TYPEC 0x1500
#define GP_SMB5_MISC 0x1600
// Fuel gauge: PMI632 qcom,qpnp-qg with qgauge@4800 and its SDAM at 0xb100. QG measures voltage and current; the state of
// charge is computed by the Linux driver from the battery profile, and only its last result is kept in the SDAM. Our
// first battery level will be an estimate from the measured voltage.
#define GP_QG_BASE 0x4800
#define GP_QG_SDAM 0xb100
// Battery node in the base DTB (qcom,max-voltage-uv 0x426030 = 4350000, qcom,fastchg-current-ma 0x1518 = 5400,
// battery-type "Alium_860_89032_0000_3600mAh_Jun15th2018"): the bengal reference battery, not necessarily the Moto 5000
// mAh pack; the overlay may override it. Use 4.35 V as the voltage ceiling until the Moto profile is found.
#define GP_BATT_MAX_UV 4350000u

#endif
