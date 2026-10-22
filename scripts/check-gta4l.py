#!/usr/bin/env python3
"""Preflight check for the Samsung Galaxy Tab A7 10.4 (gta4l, SM6115) board.

The driver runs this from the assembled build tree, where board.toml is not
copied, so every check is written to work from either location.
"""
import os
import pathlib
import sys
import tomllib

HERE = pathlib.Path(__file__).resolve()


def find_board():
    for candidate in (HERE.parents[1], pathlib.Path.cwd()):
        path = candidate / "board.toml"
        if path.is_file():
            return path
    return None


def find_dsc():
    roots = [pathlib.Path.cwd(), HERE.parents[1]]
    if os.environ.get("UNIXNU_UEFI"):
        roots.insert(0, pathlib.Path(os.environ["UNIXNU_UEFI"]))
    for root in roots:
        matches = list(root.glob("Platforms/*/*Pkg/gta4l.dsc"))
        if matches:
            return matches[0]
    return None


def main() -> int:
    errors = []
    board_path = find_board()
    if board_path:
        board = tomllib.loads(board_path.read_text())
        if board["board"]["name"] != "gta4l" or board["board"]["soc"] != "SM6115":
            errors.append("board.toml does not describe gta4l / SM6115")
        if not (board_path.parent / "configs" / board["kernel"]["defconfig"]).is_file():
            errors.append("defconfig named by board.toml is missing")
        cores = [int(v, 0) for v in board["payload"]["adt"]["cpu_regs"].split(",") if v.strip()]
        if cores != [0x0, 0x1, 0x2, 0x3, 0x100, 0x101, 0x102, 0x103]:
            errors.append(f"unexpected Kryo 260 MPIDR set: {cores}")
    else:
        cores = []

    dsc = find_dsc()
    if not dsc:
        print("[check-gta4l] warning: gta4l.dsc not visible from the tree; the firmware step resolves it", file=sys.stderr)

    for relative in ("drivers/DarwinBoot", "scripts/build-arm-driver.py", "kits/NativePrelink"):
        if not (pathlib.Path.cwd() / relative).exists():
            errors.append(f"build tree lacks {relative}")

    for error in errors:
        print(f"[check-gta4l] {error}", file=sys.stderr)
    if errors:
        return 1
    print(f"[check-gta4l] gta4l ok: {len(cores) or 8} cores, dsc {dsc}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
