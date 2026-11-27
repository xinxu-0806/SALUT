#!/usr/bin/env python3
"""Add compatibility handshake ports to TAPA autorun RTL modules.

TAPA 0.1.20250709 correctly synthesizes detached tasks with ap_ctrl_none, but
its upper-level RTL generator still connects ap_start/ap_done/ap_idle/ap_ready
to those modules.  Vivado therefore rejects the generated XO during fast
cosim elaboration.  The ports added here are intentionally inert: ap_ctrl_none
modules continue to run whenever reset is deasserted, which is the intended
autorun behavior.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import tempfile
import zipfile
from pathlib import Path


HANDSHAKE_PORTS = """\
  ap_start,
  ap_done,
  ap_idle,
  ap_ready,
"""

HANDSHAKE_DECLARATIONS = """\
input ap_start;
output ap_done;
output ap_idle;
output ap_ready;
assign ap_done = 1'b0;
assign ap_idle = 1'b0;
assign ap_ready = 1'b1;

"""


def patch_module(text: str, module_name: str) -> tuple[str, bool]:
    header = re.search(
        rf"(module\s+{re.escape(module_name)}\s*\()(.*?)(\);)",
        text,
        flags=re.DOTALL,
    )
    if header is None:
        raise RuntimeError(f"cannot find module declaration for {module_name}")
    if re.search(r"\bap_start\b", header.group(2)):
        return text, False

    replacement = header.group(1) + HANDSHAKE_PORTS + header.group(2) + header.group(3)
    text = text[: header.start()] + replacement + text[header.end() :]

    # Insert declarations immediately after the primary module header.
    declaration_pos = header.start() + len(replacement)
    text = text[:declaration_pos] + "\n" + HANDSHAKE_DECLARATIONS + text[declaration_pos:]
    return text, True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xo", required=True, type=Path)
    parser.add_argument("--graph", required=True, type=Path)
    args = parser.parse_args()

    graph = json.loads(args.graph.read_text(encoding="utf-8"))
    top = graph["top"]
    instances = graph["tasks"][top]["tasks"]
    autorun_tasks = {
        name
        for name, task_instances in instances.items()
        if task_instances and all(instance["step"] < 0 for instance in task_instances)
    }

    temp_fd, temp_name = tempfile.mkstemp(
        prefix=args.xo.name + ".", suffix=".tmp", dir=args.xo.parent
    )
    os.close(temp_fd)
    handled: set[str] = set()
    patched: set[str] = set()
    try:
        with zipfile.ZipFile(args.xo, "r") as source, zipfile.ZipFile(
            temp_name, "w"
        ) as target:
            target.comment = source.comment
            for info in source.infolist():
                data = source.read(info.filename)
                module_name = Path(info.filename).stem
                if (
                    module_name in autorun_tasks
                    and info.filename.endswith(f"/src/{module_name}.v")
                ):
                    text, changed = patch_module(data.decode("utf-8"), module_name)
                    data = text.encode("utf-8")
                    handled.add(module_name)
                    if changed:
                        patched.add(module_name)
                target.writestr(info, data)

        missing = autorun_tasks - handled
        if missing:
            raise RuntimeError(
                "autorun RTL modules were not patched: " + ", ".join(sorted(missing))
            )
        os.replace(temp_name, args.xo)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)

    print(
        f"checked {len(handled)} TAPA autorun modules; "
        f"patched {len(patched)} missing handshake interface(s) in {args.xo}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
