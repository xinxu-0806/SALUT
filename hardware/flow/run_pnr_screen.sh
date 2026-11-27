#!/usr/bin/env bash
# Full-platform post-opt cache + retained project -> Quick legality -> formal P&R.
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
SALUT_PATH_ASSIGNMENTS=$(python3 "${PROJECT_DIR}/scripts/salut_build_paths.py" --shell)
eval "$SALUT_PATH_ASSIGNMENTS"
SCREEN_ONLY=0
CAPTURE_ONLY=0
case "${1:-}" in
  --screen-only) SCREEN_ONLY=1; shift ;;
  --capture-only) CAPTURE_ONLY=1; shift ;;
esac
[[ $# == 0 ]] || { echo 'Usage: run_pnr_screen.sh [--screen-only|--capture-only]' >&2; exit 1; }
KERNEL_FREQUENCY_MHZ="${KERNEL_FREQUENCY_MHZ:-200}"
export SALUT_ENABLE_STREAM_STRIPES="${SALUT_ENABLE_STREAM_STRIPES:-0}"
export SALUT_ENABLE_LANE_COLUMNS="${SALUT_ENABLE_LANE_COLUMNS:-1}"
export SALUT_BANKED_OWNER_TOOL="${PROJECT_DIR}/scripts/banked_floorplan_owners.py"
PLATFORM=xilinx_u280_gen3x16_xdma_1_202211_1
PLATFORM_DIR="${SALUT_PLATFORM_ROOT:-/opt/xilinx/platforms}/$PLATFORM"
XO_FILE="$SALUT_FPGA_BUILD_DIR/SparseLUKernel_${PLATFORM}.hw.xo"
python3 "${PROJECT_DIR}/scripts/verify_banked_functional.py" \
  --manifest "${SALUT_FUNCTIONAL_MANIFEST:-${SALUT_FPGA_BUILD_DIR}/functional_acceptance.json}" \
  --expect-xo "$XO_FILE" --expect-hls-root "$SALUT_HLS_DIR" \
  --expect-clock-mhz "${SALUT_HLS_CLOCK_MHZ:-$KERNEL_FREQUENCY_MHZ}"
VITIS_SETTINGS="${SALUT_VITIS_SETTINGS:-/tools/Xilinx/Vitis/2022.2/settings64.sh}"
XRT_SETTINGS="${SALUT_XRT_SETTINGS:-/opt/xilinx/xrt/setup.sh}"
source "$VITIS_SETTINGS" >/dev/null 2>&1
source "$XRT_SETTINGS" >/dev/null 2>&1
for tool in v++ vivado make sha256sum flock; do command -v "$tool" >/dev/null; done
SCREEN_ROOT="$SALUT_FPGA_BUILD_DIR/quick_pnr"
mkdir -p "$SCREEN_ROOT/cache" "$SCREEN_ROOT/runs"
exec 9>"$SCREEN_ROOT/driver.lock"
flock -n 9 || { echo 'ERROR: another P&R screen is running' >&2; exit 1; }
# Floorplan is intentionally excluded: all floorplans reuse the same pristine
# post-opt platform. XO/config/platform/tool/clock changes invalidate the cache.
fingerprint() {
  sha256sum "$XO_FILE" hbm_config.ini Makefile "$VITIS_SETTINGS" \
    "$PLATFORM_DIR/$PLATFORM.xpfm" "$PLATFORM_DIR/hw/hw.xsa" \
    scripts/capture_postopt_only.tcl scripts/assert_full_platform.tcl \
    scripts/postopt_capture_context.py || return
  printf 'clock_mhz=%s\n' "$KERNEL_FREQUENCY_MHZ"
  v++ --version || return
  vivado -version || return
}
INPUTS="$(fingerprint)"
KEY="$(printf '%s\n' "$INPUTS" | sha256sum | cut -d ' ' -f1)"
CACHE="$SCREEN_ROOT/cache/$KEY"
mkdir -p "$CACHE"
export SALUT_POSTOPT_DCP="$CACHE/postopt.dcp"
export SALUT_POSTOPT_CONTEXT="${SALUT_POSTOPT_CONTEXT:-$CACHE/capture_context.json}"
verify_capture_record() {
  if [[ "$SALUT_POSTOPT_CONTEXT" == "$CACHE/capture_context.json" ]]; then
    python3 scripts/postopt_capture_context.py verify \
      --cache "$CACHE" --xo "$XO_FILE" --key "$KEY" --clock-mhz "$KERNEL_FREQUENCY_MHZ"
  else
    # An explicitly recovered record still passes the unchanged strict core
    # verifier. No file is omitted from identity checks; the original is kept.
    python3 scripts/verify_postopt_context_record.py verify \
      --record "$SALUT_POSTOPT_CONTEXT" --cache "$CACHE" --xo "$XO_FILE" \
      --key "$KEY" --clock-mhz "$KERNEL_FREQUENCY_MHZ"
  fi
}
if [[ -e "$CACHE/postopt.sha256" ]]; then
  (cd "$CACHE"; sha256sum --check postopt.sha256) || { echo 'ERROR: corrupt post-opt cache' >&2; exit 1; }
  echo "Reusing full-platform post-opt baseline: $SALUT_POSTOPT_DCP"
else
  # An incomplete capture is never adopted as a valid baseline or overwritten.
  [[ ! -e "$SALUT_POSTOPT_DCP" && ! -e "$SALUT_POSTOPT_DCP.ready" ]] || {
    echo "ERROR: incomplete capture retained for inspection: $CACHE" >&2; exit 1;
  }
  printf '%s\n' "$INPUTS" > "$CACHE/inputs.txt"
  CAPTURE_DIR="$(mktemp -d "$SCREEN_ROOT/runs/capture.XXXXXX")"
  ln -s "$XO_FILE" "$CAPTURE_DIR/$(basename "$XO_FILE")"
  echo "Capturing full post-opt platform at ${KERNEL_FREQUENCY_MHZ} MHz: $CAPTURE_DIR"
  capture_rc=0
  make --no-print-directory hw FPGA_BUILD_DIR="$CAPTURE_DIR" \
    PLATFORM_FILE="$PLATFORM_DIR/$PLATFORM.xpfm" \
    KERNEL_FREQUENCY_MHZ="$KERNEL_FREQUENCY_MHZ" \
    SALUT_PRE_PLACE_SCRIPT=scripts/capture_postopt_only.tcl \
    > "$CAPTURE_DIR/driver.log" 2>&1 || capture_rc=$?
  if [[ $capture_rc == 0 || ! -s "$SALUT_POSTOPT_DCP" || \
        ! -f "$SALUT_POSTOPT_DCP.ready" ]] || \
      [[ "$(< "$SALUT_POSTOPT_DCP.ready")" != SALUT_CAPTURE_ONLY_COMPLETE ]]; then
    echo "ERROR: capture failed (exit $capture_rc); see $CAPTURE_DIR/driver.log" >&2
    exit 1
  fi
  [[ "$(fingerprint)" == "$INPUTS" ]] || { echo 'ERROR: inputs changed during capture' >&2; exit 1; }
  # Keep and pin the original SDK project, not only the standalone DCP.
  # A cache lacking these retained inputs cannot be used for formal recovery.
  python3 scripts/postopt_capture_context.py capture \
    --cache "$CACHE" --capture-dir "$CAPTURE_DIR" --xo "$XO_FILE" \
    --key "$KEY" --clock-mhz "$KERNEL_FREQUENCY_MHZ"
  (cd "$CACHE"; sha256sum postopt.dcp > postopt.sha256)
  echo "Full post-opt capture saved; deliberate capture-only stop verified."
fi
verify_capture_record
# Staged physical work may inspect/derive this NEW complete platform before
# choosing its Quick screen. Still require the unchanged numerical, checksum
# and retained-project gates; this branch never emits a placement PASS.
if [[ $CAPTURE_ONLY == 1 ]]; then
  echo 'Full-platform post-opt capture verified; Quick and formal P&R were NOT started.'
  exit 0
fi
RUN_DIR="$(mktemp -d "$SCREEN_ROOT/runs/quick.XXXXXX")"
export SALUT_SCREEN_STATUS="$RUN_DIR/status.txt"
cp -p scripts/constrain_banked_task_slrs.tcl "$RUN_DIR/constrain_task_slrs.tcl"
cp -p scripts/{quick_place_screen,assert_full_platform,report_place_failure}.tcl "$RUN_DIR/"
sha256sum "$RUN_DIR/"*.tcl > "$RUN_DIR/inputs.sha256"
sha256sum scripts/banked_floorplan_owners.py scripts/salut_banked_provenance.py > "$RUN_DIR/owner_inputs.sha256"
printf 'baseline=%s\nclock_mhz=%s\nstripes=%s\nlanes=%s\n' \
  "$KEY" "$KERNEL_FREQUENCY_MHZ" "$SALUT_ENABLE_STREAM_STRIPES" \
  "$SALUT_ENABLE_LANE_COLUMNS" > "$RUN_DIR/run.txt"
echo "Quick full-platform legality screen: $RUN_DIR"
if ! (cd "$RUN_DIR"; vivado -mode batch -nojournal -log vivado.log \
    -source quick_place_screen.tcl -tclargs "$SALUT_POSTOPT_DCP" > driver.log 2>&1); then
  echo "ERROR: Quick screen failed; formal P&R was NOT started. See $RUN_DIR/driver.log" >&2
  exit 1
fi
[[ -f "$SALUT_SCREEN_STATUS" && "$(< "$SALUT_SCREEN_STATUS")" == SALUT_QUICK_PLACE_PASS ]] || {
  echo 'ERROR: Quick screen produced no verified PASS marker' >&2; exit 1;
}
echo 'Quick legality PASS; this is not timing closure or a deployable bitstream.'
if [[ $SCREEN_ONLY == 1 ]]; then exit 0; fi
# Do not certify a different floorplan than the one screened.
cmp -s scripts/constrain_banked_task_slrs.tcl "$RUN_DIR/constrain_task_slrs.tcl" || {
  echo 'ERROR: banked floorplan changed during screen' >&2; exit 1;
}
sha256sum --check --status "$RUN_DIR/owner_inputs.sha256" || {
  echo 'ERROR: banked owner inputs changed during screen' >&2; exit 1;
}
for name in quick_place_screen assert_full_platform report_place_failure; do
  cmp -s "scripts/$name.tcl" "$RUN_DIR/$name.tcl" || { echo "ERROR: $name changed during screen" >&2; exit 1; }
done
[[ "$(fingerprint)" == "$INPUTS" ]] || { echo 'ERROR: build inputs changed during screen' >&2; exit 1; }
verify_capture_record
echo 'Starting formal full-platform P&R with the normal routed timing/artifact gates.'
KERNEL_FREQUENCY_MHZ="$KERNEL_FREQUENCY_MHZ" \
  SALUT_PLACE_DIRECTIVE="${SALUT_PLACE_DIRECTIVE:-SSI_BalanceSLLs}" \
  SALUT_ROUTE_DIRECTIVE="${SALUT_ROUTE_DIRECTIVE:-Default}" bash ./run_link.sh
