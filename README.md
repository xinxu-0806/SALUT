# SALUT

SALUT is an HBM-based FPGA accelerator for sparse LU factorization. This
repository contains the TAPA kernel, host program, preprocessing code, matrix
inputs, and reproducible flows for simulation and hardware builds.

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

The TAPA hardware object is generated under `build/fpga/`. Override
`CLOCK_PERIOD` (default `8.0`, ns) and `JOBS` (default `12`) as needed.

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
