// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp 03/11 (auto-lock): one kernel log line per period with what each HID input source produced in it, to see which one
// keeps backboardd's idle timer (AttentionAwareness, com.apple.springboard.GlobalBacklightIdleTimer) from running out.
// guamp-hid-stats=<seconds> (default 60, 0 off). Producers (NativeEmmc): panel touch frames published to NativeHidInput
// (TouchService.inc gEvents, of them the lost-finger releases, plus the contacts the grid filter kept back), USB mirror touches
// (UsbMirror.inc), PMIC button reports (on change / the same state again once a second, which IOHIDEventDriver does not turn
// into an event), sensor samples pushed / delivered to their HID device (GuampSensors.inc). Consumer side (NativeHidInput,
// weak: absent symbols read 0): touch reports to IOHIDEventDriver, button reports (report 2, refreshes included), sensor reports
// per kind and ambient light events. Included after GuampSensors.inc, UsbMirror.inc, TouchService.inc and PmicService.inc.
extern "C" {
extern volatile uint32_t native_touch_hid_reports __attribute__((weak_import));
extern volatile uint32_t native_hid_input_events[8] __attribute__((weak_import));
extern volatile uint32_t native_hid_sensor_reports[6] __attribute__((weak_import));
extern volatile uint32_t native_hid_als_stats[3] __attribute__((weak_import));
}

namespace hidstats {
unsigned gPeriod = 60;

struct Snapshot {
  uint32_t panel, releases, grid, buttons, refreshes, hidTouch, hidButtons, als;
  uint64_t mirror;
  uint32_t pushed[guamp_sensors::kKinds], delivered[guamp_sensors::kKinds], reports[guamp_sensors::kKinds];
};

void take(Snapshot &s) {
  s.panel = touch::gEvents;
  s.releases = touch::gReleases;
  s.grid = touch::gGridDrops;
  s.mirror = native_usb_touches;
  s.buttons = pmic::gSinkChanges;
  s.refreshes = pmic::gSinkRefreshes;
  s.hidTouch = &native_touch_hid_reports ? native_touch_hid_reports : 0;
  s.hidButtons = &native_hid_input_events ? native_hid_input_events[4] : 0;
  s.als = &native_hid_als_stats ? native_hid_als_stats[1] : 0;
  for (unsigned k = 0; k < guamp_sensors::kKinds; ++k) {
    s.pushed[k] = guamp_sensor_pushes[k];
    s.delivered[k] = guamp_sensor_delivered[k];
    s.reports[k] = &native_hid_sensor_reports ? native_hid_sensor_reports[k] : 0;
  }
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-hid-stats");
  Snapshot last = {}, now = {};
  take(last);
  for (;;) {
    IOSleep(gPeriod * 1000);
    take(now);
    using namespace guamp_sensors;
    uint32_t d[kKinds], r[kKinds], p[kKinds];
    for (unsigned k = 0; k < kKinds; ++k) {
      p[k] = now.pushed[k] - last.pushed[k];
      d[k] = now.delivered[k] - last.delivered[k];
      r[k] = now.reports[k] - last.reports[k];
    }
    IOLog("GuampHidStats: %us: panel touch %u (releases %u, grid kept back %u), mirror touches %llu, buttons %u changes "
          "%u refreshes | HID touch %u buttons %u | sensors pushed/delivered/reports accel %u/%u/%u gyro %u/%u/%u orient "
          "%u/%u/%u prox %u/%u/%u light %u/%u/%u (light events %u) | display %s\n",
          gPeriod, now.panel - last.panel, now.releases - last.releases, now.grid - last.grid,
          static_cast<unsigned long long>(now.mirror - last.mirror), now.buttons - last.buttons,
          now.refreshes - last.refreshes, now.hidTouch - last.hidTouch, now.hidButtons - last.hidButtons, p[kAccel],
          d[kAccel], r[kAccel], p[kGyro], d[kGyro], r[kGyro], p[kOrient], d[kOrient], r[kOrient], p[kProx], d[kProx],
          r[kProx], p[kAls], d[kAls], r[kAls], now.als - last.als, touch::gDisplayOn ? "on" : "off");
    last = now;
  }
}

void start() {
  PE_parse_boot_argn("guamp-hid-stats", &gPeriod, sizeof(gPeriod));
  if (!gPeriod) return;
  if (gPeriod > 3600) gPeriod = 3600;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace hidstats
