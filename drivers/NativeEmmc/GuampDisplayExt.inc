// SPDX-License-Identifier: BSD-2-Clause-Patent
// IOMobileFramebuffer's iomfb_match_callback types a framebuffer as external (IOMFB display type 1) when its
// "external" registry property is true; QuartzCore then opens it as AppleExternalDisplay, FrontBoard's display
// identity says isExternal and UIKit's -[UIScreen _isEmbeddedScreen] is false, so the status bar falls back to
// _UIStatusBarVisualProvider_Fallback (red, rdar://45025538). The panel is the built-in one: on first publish,
// before backboardd enumerates, record what the framebuffer published and clear "external".
// guamp-display-internal=0 only records.
namespace dispext {
static volatile uint32_t gSeen = 0, gExternalBefore = 0, gBuiltIn = 0, gForced = 0;
static bool gForce = true;

static bool published(void *, void *, IOService *service, IONotifier *)
{
  OSObject *external = service->getProperty("external");
  // 0 absent, 1 true, 2 false, 3 another type
  uint32_t before = !external ? 0 : external == kOSBooleanTrue ? 1 : external == kOSBooleanFalse ? 2 : 3;
  gExternalBefore = before;
  gBuiltIn = service->getProperty("BuiltIn") == kOSBooleanTrue;
  if (gForce && (before == 1 || before == 3)) {
    service->setProperty("external", kOSBooleanFalse);
    gForced = gForced + 1;
  }
  gSeen = gSeen + 1;
  return true;
}

static void start()
{
  unsigned force = 1;
  PE_parse_boot_argn("guamp-display-internal", &force, sizeof(force));
  gForce = force != 0;
  OSDictionary *matching = IOService::serviceMatching("IOMobileFramebuffer");
  if (!matching) return;
  IONotifier *notifier = IOService::addMatchingNotification(gIOFirstPublishNotification, matching, published, nullptr);
  matching->release();
  IOLog("GuampDisplay: framebuffer publish hook %s, force internal %u\n", notifier ? "armed" : "failed", gForce);
}

// One line per report for the first few minutes: the USB mirror connects after the framebuffer is published.
static void log()
{
  static unsigned reports = 0;
  if (reports++ >= 30) return;
  IOLog("GuampDisplay: IOMFB seen %u external-before %u (0 absent 1 true 2 false 3 other) builtin %u forced %u\n",
        gSeen, gExternalBefore, gBuiltIn, gForced);
}
}
