#!/usr/bin/env python3
"""Apply resource- and timing-aware TAPA FIFO implementations in a packaged XO.

TAPA's default FIFO policy shifts the complete payload on every enqueue for
every depth below 128.  In this design that leaves about 118k SRL LUTs after
the single-beat response-packet conversion.  Replace that implementation with
a circular distributed-RAM FIFO.  It keeps the same registered-empty/FWFT
interface and capacity (including TAPA's minimum physical depth of four), but
only writes one memory word per enqueue.  Vivado can consequently pack several
shallow memory bits into each LUT instead of allocating one SRL per data bit.

Very wide, genuinely deep queues still use block RAM.  The current response
packet is 533 bits, so include it explicitly in that policy; doing this for all
shallow wide queues would waste over a thousand BRAM tiles.

The AXI wrappers have a separate relay_station fifo_almost_full policy. Its
512-bit payload FIFOs have real depth 38 and otherwise infer two SRLs per bit.
Select its existing BRAM implementation without changing protocol or capacity;
graph fifo.v policy alone does not affect these relay buffers.

TAPA's depth-1 ``fifo_fwd`` also has a combinational input-to-output bypass.
That bypass forms very long combinational handshake/data paths across the
multi-stage PE/HBM network and can prevent RTL simulation from advancing once
several paths become active.  Keep the one-entry capacity, but make the entry
fully registered.  This uses FFs for the very-wide transient streams without
turning every data bit into an SRL LUT.
"""

from __future__ import annotations

import argparse
import os
import tempfile
import zipfile
from pathlib import Path
from patch_axi_relay_bram import patch_relay_station


FIFO_SUFFIX = "/src/fifo.v"
FIFO_FWD_SUFFIX = "/src/fifo_fwd.v"
FIFO_SRL_SUFFIX = "/src/fifo_srl.v"
RELAY_SUFFIX = "/src/relay_station.v"
BRAM_MARKER = "SALUT_WIDE_FIFO_BRAM_POLICY"
REGISTERED_FWD_MARKER = "SALUT_REGISTERED_DEPTH1_FIFO"
CIRCULAR_LUTRAM_MARKER = "SALUT_CIRCULAR_LUTRAM_FIFO"
OLD_POLICY = "else if (DEPTH >=128) begin : bram"
NEW_POLICY = """// SALUT_WIDE_FIFO_BRAM_POLICY
  else if (DEPTH >= 128 || (DATA_WIDTH >= 512 && DEPTH >= 32)) begin : bram"""
LEGACY_PATCHED_POLICY = """// SALUT_WIDE_FIFO_BRAM_POLICY
  else if (DEPTH >= 128 || (DATA_WIDTH >= 1024 && DEPTH >= 32)) begin : bram"""

OLD_FWD_COMMENT = "// first-word fall-through (FWFT) FIFO with latency=0 and depth=1"
NEW_FWD_COMMENT = "// SALUT_REGISTERED_DEPTH1_FIFO: one-entry FIFO with registered output"
OLD_FWD_STATUS = """  // If mem is valid, the FIFO is not empty and is full; if read, mem becomes
  // invalid. If mem is not valid, the FIFO is not empty if and only if written
  // and is not full; if written but not read, mem becomes valid.

  assign if_empty_n = is_mem_valid || (if_write && if_write_ce);
  assign if_full_n  = !is_mem_valid;
  assign if_dout    = is_mem_valid ? mem : if_din;"""
NEW_FWD_STATUS = """  // Break the combinational ready/valid and data path between adjacent tasks.
  assign if_empty_n = is_mem_valid;
  assign if_full_n  = !is_mem_valid;
  assign if_dout    = mem;"""
OLD_FWD_WRITE = """      end else begin
        if (if_write && if_write_ce && !(if_read && if_read_ce)) begin
          is_mem_valid <= 1'b1;
        end
      end"""
NEW_FWD_WRITE = """      end else begin
        if (if_write && if_write_ce) begin
          is_mem_valid <= 1'b1;
        end
      end"""

OLD_SRL_BODY_START = "  parameter REAL_DEPTH = DEPTH < 4 ? 4 : DEPTH;"
OLD_SRL_BODY_END = "\nendmodule  // fifo_srl"
NEW_SRL_BODY = """  // SALUT_CIRCULAR_LUTRAM_FIFO
  // Preserve TAPA's minimum physical capacity of four entries.  All FIFO
  // depths emitted for this kernel are powers of two, but explicit wrapping
  // also keeps the implementation correct for an arbitrary REAL_DEPTH.
  localparam integer REAL_DEPTH = DEPTH < 4 ? 4 : DEPTH;
  localparam integer PTR_WIDTH = $clog2(REAL_DEPTH);

  (* ram_style = "distributed" *)
  reg [DATA_WIDTH-1:0] mem [0:REAL_DEPTH-1];
  reg [PTR_WIDTH-1:0] read_ptr;
  reg [PTR_WIDTH-1:0] write_ptr;
  reg [PTR_WIDTH:0] count;

  wire do_read = if_read && if_read_ce && if_empty_n;
  wire do_write = if_write && if_write_ce && if_full_n;

  assign if_empty_n = (count != 0);
  assign if_full_n = (count != REAL_DEPTH);
  assign if_dout = mem[read_ptr];

  always @(posedge clk) begin
    if (reset) begin
      read_ptr <= {PTR_WIDTH{1'b0}};
      write_ptr <= {PTR_WIDTH{1'b0}};
      count <= {(PTR_WIDTH+1){1'b0}};
    end else begin
      if (do_read)
        read_ptr <= read_ptr == REAL_DEPTH - 1 ?
                    {PTR_WIDTH{1'b0}} : read_ptr + 1'b1;
      if (do_write)
        write_ptr <= write_ptr == REAL_DEPTH - 1 ?
                     {PTR_WIDTH{1'b0}} : write_ptr + 1'b1;

      case ({do_write, do_read})
        2'b10: count <= count + 1'b1;
        2'b01: count <= count - 1'b1;
        default: count <= count;
      endcase
    end
  end

  always @(posedge clk) begin
    if (do_write)
      mem[write_ptr] <= if_din;
  end
"""


def patch_fifo(text: str) -> tuple[str, bool]:
    if BRAM_MARKER in text:
        if NEW_POLICY in text:
            return text, False
        if text.count(LEGACY_PATCHED_POLICY) == 1:
            return text.replace(LEGACY_PATCHED_POLICY, NEW_POLICY, 1), True
        raise RuntimeError("cannot identify the patched TAPA BRAM FIFO policy")
    if text.count(OLD_POLICY) != 1:
        raise RuntimeError("cannot locate the TAPA BRAM FIFO policy")
    return text.replace(OLD_POLICY, NEW_POLICY, 1), True


def patch_fifo_fwd(text: str) -> tuple[str, bool]:
    if REGISTERED_FWD_MARKER in text:
        return text, False
    if text.count(OLD_FWD_COMMENT) != 1:
        raise RuntimeError("cannot locate the TAPA depth-1 FIFO header")
    if text.count(OLD_FWD_STATUS) != 1 or text.count(OLD_FWD_WRITE) != 1:
        raise RuntimeError("cannot locate the TAPA depth-1 FIFO implementation")
    text = text.replace(OLD_FWD_COMMENT, NEW_FWD_COMMENT, 1)
    text = text.replace(OLD_FWD_STATUS, NEW_FWD_STATUS, 1)
    text = text.replace(OLD_FWD_WRITE, NEW_FWD_WRITE, 1)
    return text, True


def patch_fifo_srl(text: str) -> tuple[str, bool]:
    if CIRCULAR_LUTRAM_MARKER in text:
        return text, False
    if text.count(OLD_SRL_BODY_START) != 1 or text.count(OLD_SRL_BODY_END) != 1:
        raise RuntimeError("cannot locate the TAPA SRL FIFO implementation")
    prefix, remainder = text.split(OLD_SRL_BODY_START, 1)
    _, suffix = remainder.split(OLD_SRL_BODY_END, 1)
    return prefix + NEW_SRL_BODY + OLD_SRL_BODY_END + suffix, True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xo", required=True, type=Path)
    args = parser.parse_args()

    temp_fd, temp_name = tempfile.mkstemp(
        prefix=args.xo.name + ".", suffix=".tmp", dir=args.xo.parent
    )
    os.close(temp_fd)
    handled_fifo = False
    handled_fwd = False
    handled_srl = False
    handled_relay = 0
    changed_fifo = False
    changed_fwd = False
    changed_srl = False
    changed_relay = False
    try:
        with zipfile.ZipFile(args.xo, "r") as source, zipfile.ZipFile(
            temp_name, "w"
        ) as target:
            target.comment = source.comment
            for info in source.infolist():
                data = source.read(info.filename)
                if info.filename.endswith(FIFO_SUFFIX):
                    text, changed_fifo = patch_fifo(data.decode("utf-8"))
                    data = text.encode("utf-8")
                    handled_fifo = True
                elif info.filename.endswith(FIFO_FWD_SUFFIX):
                    text, changed_fwd = patch_fifo_fwd(data.decode("utf-8"))
                    data = text.encode("utf-8")
                    handled_fwd = True
                elif info.filename.endswith(FIFO_SRL_SUFFIX):
                    text, changed_srl = patch_fifo_srl(data.decode("utf-8"))
                    data = text.encode("utf-8")
                    handled_srl = True
                elif info.filename.endswith(RELAY_SUFFIX):
                    handled_relay += 1
                    if handled_relay != 1:
                        raise RuntimeError("AXI_RELAY_RTL_MISSING_OR_AMBIGUOUS")
                    text, changed_relay = patch_relay_station(data.decode("utf-8"))
                    data = text.encode("utf-8")
                target.writestr(info, data)
        if not handled_fifo or not handled_fwd or not handled_srl:
            raise RuntimeError(
                "cannot find src/fifo.v, src/fifo_fwd.v, and src/fifo_srl.v in XO"
            )
        if handled_relay != 1:
            raise RuntimeError("AXI_RELAY_RTL_MISSING_OR_AMBIGUOUS")
        os.replace(temp_name, args.xo)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)

    bram_action = "patched" if changed_fifo else "kept"
    fwd_action = "patched" if changed_fwd else "kept"
    srl_action = "patched" if changed_srl else "kept"
    relay_action = "patched" if changed_relay else "kept"
    print(f"{bram_action} wide/deep FIFO BRAM policy in {args.xo}")
    print(f"{fwd_action} registered depth-1 FIFO policy in {args.xo}")
    print(f"{srl_action} circular distributed-RAM FIFO policy in {args.xo}")
    print(f"{relay_action} AXI relay wide/deep FIFO BRAM policy in {args.xo}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
