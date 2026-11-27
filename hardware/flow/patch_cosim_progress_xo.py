#!/usr/bin/env python3
"""Inject synthesis-ignored column progress messages into the HLS RTL XO."""

from __future__ import annotations

import argparse
import os
import re
import tempfile
import zipfile
from pathlib import Path


MODULE_PATH_SUFFIX = "/src/SequentialSparseLU.v"
PARALLEL_MODULE_PATH_SUFFIX = "/src/SparseLUKernel.v"
MARKER = "SALUT_COSIM_COLUMN_PROGRESS"
PARALLEL_DETAIL_MARKER = "SALUT_COSIM_BANKED_DETAIL_V1"


def patch_progress(text: str) -> tuple[str, bool, str]:
    if MARKER in text:
        return text, False, "already present"

    match = re.search(r"reg\s+\[16:0\]\s+(j_fu_\d+)\s*;", text)
    if match is None:
        raise RuntimeError("cannot locate the synthesized column register")
    j_reg = match.group(1)

    end_marker = "endmodule //SequentialSparseLU"
    pos = text.rfind(end_marker)
    if pos < 0:
        raise RuntimeError("cannot locate SequentialSparseLU endmodule")

    monitor = f"""
// {MARKER}
// synthesis translate_off
reg [16:0] salut_cosim_prev_j;
always @(posedge ap_clk) begin
  if (!ap_rst_n) begin
    salut_cosim_prev_j <= 17'h1ffff;
  end else if ({j_reg} != salut_cosim_prev_j) begin
    salut_cosim_prev_j <= {j_reg};
    if ({j_reg}[9:0] == 10'd0)
      $display("[COSIM_PROGRESS] column=%0d time=%0t", {j_reg}, $time);
  end
end
// synthesis translate_on

"""
    return text[:pos] + monitor + text[pos:], True, j_reg


def patch_parallel_progress(text: str, progress_interval: int = 1048576) -> tuple[str, bool, str]:
    if type(progress_interval) is not int or progress_interval <= 0:
        raise RuntimeError("progress interval must be a positive integer")
    if PARALLEL_DETAIL_MARKER in text:
        # Change only a passive monitor predicate, never a synthesized signal.
        monitor = re.search(
            rf"// {MARKER}\n// synthesis translate_off\n.*?"
            rf"// synthesis translate_on\n// {PARALLEL_DETAIL_MARKER}",
            text, flags=re.DOTALL)
        if monitor is None:
            raise RuntimeError("cannot locate the existing passive progress monitor")
        predicate = re.compile(r"(salut_cycles\s*%\s*)(\d+)(\s*==\s*0)")
        matches = list(predicate.finditer(monitor.group()))
        if len(matches) != 1:
            raise RuntimeError("expected one passive progress interval predicate")
        if int(matches[0].group(2)) != progress_interval:
            updated = predicate.sub(lambda m: m.group(1) + str(progress_interval) + m.group(3),
                                    monitor.group(), count=1)
            return (text[:monitor.start()] + updated + text[monitor.end():], True,
                    "parallel detail monitor interval updated")
        return text, False, "parallel detail monitor already present"
    if "SalutSmBank_" not in text:
        return text, False, "not a banked PLU2 top"
    if MARKER in text:
        text, count = re.subn(
            rf"\n// {MARKER}\n// synthesis translate_off\n.*?"
            rf"// synthesis translate_on\n(?:// SALUT_COSIM_(?:PLU2|BANKED)_DETAIL_V\d+\n)?",
            "\n", text, count=1, flags=re.DOTALL)
        if count != 1:
            raise RuntimeError("cannot replace an existing top progress monitor")
    signals = (
        "front_task__write", "front_done__write", "seg_done__write",
        "wr_ack__write", "rd_omega_0_0__write", "wr_omega_0_0__write",
        "rsp_omega_3_0__write", "bank_drained_0__write",
        "SalutLMLane_0__ap_start", "SalutSmBank_0__ap_start",
        "SalutMemoryDrainJoin_0__ap_done",
    )
    missing = [signal for signal in signals if signal not in text]
    if missing:
        raise RuntimeError("missing banked progress signals: " + ", ".join(missing))
    pos = text.rfind("endmodule")
    if pos < 0:
        raise RuntimeError("cannot locate SparseLUKernel endmodule")
    monitor = f"""
// {MARKER}
// synthesis translate_off
integer salut_cycles, salut_front_tasks, salut_front_done, salut_segment_done;
integer salut_read_requests, salut_write_beats, salut_responses, salut_publications;
always @(posedge ap_clk) begin
  if (!ap_rst_n) begin
    salut_cycles <= 0; salut_front_tasks <= 0; salut_front_done <= 0;
    salut_segment_done <= 0; salut_read_requests <= 0;
    salut_write_beats <= 0; salut_responses <= 0; salut_publications <= 0;
  end else begin
    salut_cycles <= salut_cycles + 1;
    if (front_task__write) salut_front_tasks <= salut_front_tasks + 1;
    if (front_done__write) salut_front_done <= salut_front_done + 1;
    if (seg_done__write) salut_segment_done <= salut_segment_done + 1;
    if (rd_omega_0_0__write) salut_read_requests <= salut_read_requests + 1;
    if (wr_omega_0_0__write) salut_write_beats <= salut_write_beats + 1;
    if (rsp_omega_3_0__write) salut_responses <= salut_responses + 1;
    if (wr_ack__write) salut_publications <= salut_publications + 1;
    if (salut_cycles % {progress_interval} == 0)
      $display("[COSIM_BANKED_PROGRESS] cycles=%0d fronts=%0d/%0d segments=%0d lane0_read=%0d lane0_write=%0d lane0_response=%0d publish=%0d lm0_start=%b sm0_start=%b bank0_drain=%b final_done=%b",
               salut_cycles, salut_front_done, salut_front_tasks, salut_segment_done,
               salut_read_requests, salut_write_beats, salut_responses, salut_publications,
               SalutLMLane_0__ap_start, SalutSmBank_0__ap_start,
               bank_drained_0__write, SalutMemoryDrainJoin_0__ap_done);
  end
end
// synthesis translate_on
// {PARALLEL_DETAIL_MARKER}

"""
    return text[:pos] + monitor + text[pos:], True, "banked transport/physical-drain counters"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xo", required=True, type=Path)
    parser.add_argument("--progress-interval", type=int, default=1048576)
    args = parser.parse_args()

    temp_fd, temp_name = tempfile.mkstemp(
        prefix=args.xo.name + ".", suffix=".tmp", dir=args.xo.parent
    )
    os.close(temp_fd)
    handled = False
    changed = False
    detail = ""
    try:
        with zipfile.ZipFile(args.xo, "r") as source, zipfile.ZipFile(
            temp_name, "w"
        ) as target:
            target.comment = source.comment
            for info in source.infolist():
                data = source.read(info.filename)
                if info.filename.endswith(MODULE_PATH_SUFFIX):
                    text, changed, detail = patch_progress(data.decode("utf-8"))
                    data = text.encode("utf-8")
                    handled = True
                elif info.filename.endswith(PARALLEL_MODULE_PATH_SUFFIX):
                    text, parallel_changed, parallel_detail = patch_parallel_progress(
                        data.decode("utf-8"), progress_interval=args.progress_interval
                    )
                    if (parallel_changed or
                            parallel_detail == "parallel detail monitor already present"):
                        changed = changed or parallel_changed
                        detail = parallel_detail
                        data = text.encode("utf-8")
                        handled = True
                target.writestr(info, data)
        if handled:
            os.replace(temp_name, args.xo)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)

    if not handled:
        print(f"skipped SequentialSparseLU-only cosim monitor in {args.xo}")
        return 0
    action = "injected" if changed else "kept"
    print(f"{action} cosim column progress monitor ({detail}) in {args.xo}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
