#!/usr/bin/env bash
# v661 PE16 (SLR0-relief partition) + 4:1 gearbox base XO: same steps as run_fpga.sh, cosim progress interval 64 (as the v538 lineage).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
export TAPA_ROOT=$HOME/.rapidstream-tapa XILINX_VITIS_HLS=/tools/Xilinx/Vitis_HLS/2022.2 XILINX_XRT=/opt/xilinx/xrt XILINX_VITIS=/tools/Xilinx/Vitis/2022.2
export PATH="$TAPA_ROOT/usr/bin:$XILINX_VITIS_HLS/bin:$XILINX_VITIS/bin:$XILINX_XRT/bin:$PATH"
F=build/fpga/banked-lu-pe16gb-v661.c0ITWmv2
W=$F/SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.tapa_work
XO=$F/SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.hw.xo
python3 scripts/salut_banked_provenance.py capture --kind full --manifest $W/salut_provenance.json --clock-mhz 200
tapa -w $W compile -f src/lu_kernel.cpp -t SparseLUKernel -p xilinx_u280_gen3x16_xdma_1_202211_1 \
  --clock-period 5.000000 -j 12 -o $XO --keep-hls-work-dir --no-skip-hls-based-on-mtime > $F/tapa_compile_output.log 2>&1
python3 scripts/patch_tapa_autorun_xo.py --xo $XO --graph $W/graph.json
python3 scripts/patch_wide_fifo_bram_xo.py --xo $XO
python3 scripts/patch_cosim_progress_xo.py --xo $XO --progress-interval 64
python3 scripts/patch_timing_fanout_xo.py --xo $XO
python3 scripts/salut_banked_provenance.py seal --kind full --manifest $W/salut_provenance.json --hls-root $W/hls --xo $XO --graph $W/graph.json
echo BASE_XO_BUILD_DONE
