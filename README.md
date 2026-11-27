# SALUT

> **DAC 2026** — 63rd ACM/IEEE Design Automation Conference

SALUT is an HBM-based FPGA accelerator for sparse LU factorization. This
repository contains the TAPA kernel, host program, preprocessing code, matrix
inputs, and reproducible flows for simulation and hardware builds.

## Paper

[SALUT: Efficient Sparse LU Factorization via Asynchronous Task Triggering on HBM FPGAs](paper/SALUT.pdf)

If you find this work useful, please cite:

```bibtex
@inproceedings{salutdac26,
  author = {Xu, Xin and Wu, Zhenhua and Niu, Dan and Jin, Zhou and Zhuo, Cheng},
  booktitle = {63rd ACM/IEEE Design Automation Conference (DAC 2026)},
  title = {SALUT: Efficient Sparse LU Factorization via Asynchronous Task Triggering on HBM FPGAs},
  year = {2026}
}
```

## Requirements

- TAPA
- Xilinx Vitis/Vitis HLS 2022.2
- Xilinx Runtime (XRT)
- U280 platform: `xilinx_u280_gen3x16_xdma_1_202211_1`

The scripts use the local default installation paths and accept overrides
through `TAPA_ROOT`, `XILINX_VITIS`, `XILINX_VITIS_HLS`,
`XILINX_XRT`, and `PLATFORM`.

## Software emulation

```bash
./scripts/run_sw_emu.sh /path/to/matrix.mtx
```

Without an argument, the script uses `data2/bcsstm37/bcsstm37.mtx`.

## Hardware synthesis

```bash
./scripts/run_hls_synth.sh
```

## Hardware emulation

```bash
./scripts/run_hw_emu.sh /path/to/matrix.mtx
```

The script builds the hardware object if required, links an `hw_emu`
xclbin, creates `emconfig.json`, and invokes the host program.

## Hardware build

```bash
./run_generate.sh
# equivalent: ./scripts/generate_bitstream.sh
```

This performs local hardware synthesis and linking. Generated hardware
artifacts are excluded from version control.

## Released bitstream

A U280 xclbin of this source has been built: 16 PEs (8 PE groups x 2), 8 column fronts,
8 LM lanes, 8 SM banks and 27 HBM channels, with the kernel clock at **150 MHz**. It is fully
routed, meets all timing, and passes `pr_verify` against the original platform shell. The binary
is distributed separately; [bitstream/README.md](bitstream/README.md) lists its hash and keeps the
sign-off reports, and [hardware/README.md](hardware/README.md) describes the implementation flow.
