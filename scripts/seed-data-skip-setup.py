#!/usr/bin/env python3
"""Stub hook called by ./unixnu build payload --skip-setup."""
import sys


def main() -> int:
    print("[seed-data-skip-setup] Setup bypass flag applied for gta4l.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
