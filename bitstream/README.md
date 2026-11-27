# U280 bitstream: build record and sign-off

| Item | Value |
|---|---|
| File | `SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.xclbin` (distributed separately, not stored in this repository) |
| SHA-256 | `64b848577d60e07d09d67f8c0a6689a5334be1dcfaf96dbdc684905f5d304f13` |
| xclbin UUID | `c6366cb4-d215-ec26-7ff8-b709f4fa7580` |
| Platform | `xilinx_u280_gen3x16_xdma_1_202211_1` |
| Kernel clock | **150 MHz** (`DATA_CLK`); platform clocks unchanged (`hbm_aclk` 450 MHz, `KERNEL_CLK` 500 MHz) |
| Toolchain | TAPA + Vitis / Vivado 2022.2 |
| Kernel XO | `SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.hw.xo` (HLS at 200 MHz, not stored here), SHA-256 `9d7f9a5c247bb4c32a02caa3340b5426fd7c07c3cd5e6a7dd3927b75a933ce3b` |

## Configuration

- 8 PE groups x 2 PEs (16 segment PEs), 8 column fronts, 8 LM lanes, 8 SM banks.
- Complete segment/column triggering and full L/U numerical computation.
- 27 HBM channels: `A_0..7 -> HBM[0..7]`, `L_0..7 -> HBM[8..15]`, `U_0..7 -> HBM[16..23]`,
  `META_SCHED/FRONT/SEGMENT -> HBM[24..26]` (see `../hbm_config.ini`; verified in `xclbin_info.txt`).

## Sign-off

This directory keeps the sign-off reports of that build. The xclbin is packaged by a normal `v++ -l --reuse_impl` run from the routed checkpoint that passed:

- Full route: 0 routing errors, 0 unrouted nets (`route_status.rpt`).
- All clocks met after a fresh reload: WNS +0.003 ns, WHS +0.004 ns, 0 failing endpoints
  (`timing_summary_clocks.rpt`). No timing exceptions were added; the HDPR-5 DRC stays an Error.
- Default DRC: 0 errors (`drc_default.rpt`). CDC criticals are all inside platform IP; none in the kernel.
- `pr_verify -full_check` against the original locked platform shell: compatible (`pr_verify_full_F5.rpt`).

Functional prerequisites for this XO: the software numerical regression (rajat15) and the RTL
regressions (15 task-level leaf tests, bounded 5- and 200-column top-level co-simulations) passed.
