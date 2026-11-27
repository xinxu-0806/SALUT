# Hardware build flow of the released bitstream

`../bitstream/` was produced with the flow below. The scripts in `flow/` are kept as they were run;
they contain absolute paths of the build machine and are provided as a record, not as a turnkey flow.

1. **HLS / XO** (`flow/build_pe16gb_base_xo.sh`): `tapa compile` of `src/lu_kernel.cpp` at 5.000 ns, then the
   XO patches `patch_tapa_autorun_xo.py`, `patch_wide_fifo_bram_xo.py`, `patch_cosim_progress_xo.py`,
   `patch_timing_fanout_xo.py`.
2. **SLR partition**: `flow/solve_pe16_perslr_ilp.py` (per-SLR LUT caps 0.57/0.58/0.59, `ILP_pe16_selected.json`);
   the 4:1 cross-SLR gearboxes are already in `src/` (`PE16_GEARBOX4_INSERTION_RECORD.json`).
3. **XO transforms**: `flow/t4c_build_pe16_candidate.py` replaces cross-SLR FIFOs with
   `rtl/salut_relay_station.v` and adds `rtl/salut_dispatch_fifo.v` / `rtl/salut_credit_fifo.v`;
   `flow/t4g_build_pe16_candidate.py` adds the reset tree, scalar / ap_start pipelines and META AXI skids
   (`rtl/salut_ctrl_pipe.v`) and `keep_hierarchy` on `SparseLUKernel`. Compute RTL is unchanged.
4. **SDK capture**: normal `v++ -l` (`flow/run_pnr_screen.sh --capture-only`) at `--kernel_frequency 150`.
5. **Place and route** (`flow/place_route_candidate_NOT_product.tcl`, one Vivado session):
   floorplan `flow/t7g_pe16_apply_floorplan.tcl` (+ owner data), child pblocks
   `flow/v694_path27_hop_pblocks.tcl` that keep the platform HBM path_27 SLR0/SLR1 crossing registers
   next to the only usable SLL column, `place_design -directive SSI_SpreadSLLs`,
   `route_design -directive AlternateCLBRouting`.
6. **Sign-off**: `flow/final_gates.tcl` (fresh reload, timing, DRC, CDC, check_timing, bus skew,
   `pr_verify -full_check` against the original shell).
7. **Package**: `flow/package_reuse_impl.py` (`v++ -l --reuse_impl`, via `run_postopt_link.package_preserving_project`).

Notes:
- At 200 MHz the best fully routed result was kernel −0.076 ns / hbm_aclk −0.182 ns; the platform
  `hbm_aclk` (450 MHz) is the limiter and does not depend on the kernel clock.
- The 15 platform partition-boundary nets only route when placement and routing run in the same session.
