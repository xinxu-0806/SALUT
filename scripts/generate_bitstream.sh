#!/usr/bin/env bash
# Link the synthesized .xo into a deployable hardware xclbin.
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

require_file "$HBM_CONFIG"
if [[ ! -f "$XO_FILE" ]]; then
  "$SCRIPT_DIR/run_hls_synth.sh"
fi

xclbin="$FPGA_DIR/${TOP_FUNCTION}_${PLATFORM}.hw.xclbin"
mkdir -p "$FPGA_DIR"

v++ -l --target hw \
  --platform "$PLATFORM" \
  --config "$HBM_CONFIG" \
  --temp_dir "$FPGA_DIR/link_tmp" \
  "$XO_FILE" \
  -o "$xclbin"
