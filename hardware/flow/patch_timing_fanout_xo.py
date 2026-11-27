#!/usr/bin/env python3
"""Constrain the high-fanout HLS controller in the packaged TAPA XO.

The one-hot controller generated for SequentialSparseLU drives several
512-bit register enables.  Without an explicit limit Vivado keeps a single
state flop with more than 500 loads, producing route-only critical paths.
MAX_FANOUT lets synthesis make equivalent local copies; it does not add a
cycle or change the controller's logical behaviour.
"""

from __future__ import annotations

import argparse
import os
import re
import tempfile
import zipfile
from pathlib import Path


MODULE_PATH_SUFFIX = "/src/SequentialSparseLU.v"
MARKER = "SALUT_TIMING_MAX_FANOUT"
MAX_FANOUT = 32


def patch_controller(text: str) -> tuple[str, bool]:
    if MARKER in text:
        return text, False

    pattern = re.compile(
        r'\(\*\s*fsm_encoding\s*=\s*"none"\s*\*\)'
        r'(\s+reg\s+\[\d+:\d+\]\s+ap_CS_fsm\s*;)'
    )
    replacement = lambda match: (  # noqa: E731 - keeps the backreference explicit
        f'(* fsm_encoding = "none", max_fanout = {MAX_FANOUT} *)'
        f' /* {MARKER} */{match.group(1)}'
    )
    text, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError("cannot locate the SequentialSparseLU controller register")
    return text, True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xo", required=True, type=Path)
    args = parser.parse_args()

    temp_fd, temp_name = tempfile.mkstemp(
        prefix=args.xo.name + ".", suffix=".tmp", dir=args.xo.parent
    )
    os.close(temp_fd)
    handled = False
    changed = False
    try:
        with zipfile.ZipFile(args.xo, "r") as source, zipfile.ZipFile(
            temp_name, "w"
        ) as target:
            target.comment = source.comment
            for info in source.infolist():
                data = source.read(info.filename)
                if info.filename.endswith(MODULE_PATH_SUFFIX):
                    text, changed = patch_controller(data.decode("utf-8"))
                    data = text.encode("utf-8")
                    handled = True
                target.writestr(info, data)
        if handled:
            os.replace(temp_name, args.xo)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)

    if not handled:
        print(f"skipped SequentialSparseLU-only MAX_FANOUT patch in {args.xo}")
        return 0
    action = "inserted" if changed else "kept"
    print(f"{action} SequentialSparseLU MAX_FANOUT={MAX_FANOUT} in {args.xo}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
