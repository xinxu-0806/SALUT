#!/usr/bin/env python3
"""Formal full-platform post-opt recovery and guarded same-project SDK reuse.

Neither a Quick marker nor a tool exit code is deployment acceptance. The
existing functional, all-clock routed, build-input and SDK package gates are
mandatory. Every attempt and every changed SDK project is retained.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time

from postopt_capture_context import verify_context, digest
from salut_banked_provenance import ROOT
from verify_routed_checkpoint import verify_checkpoint
from verify_controlset_quick import verify_controlset_quick

PLATFORM = 'xilinx_u280_gen3x16_xdma_1_202211_1'
XCLBIN = 'SparseLUKernel_' + PLATFORM + '.xclbin'


def verify_capture_inputs(context, platform, root=ROOT):
    """Do not resume an old platform/config merely because its DCP hash matches."""
    path = Path(context['postopt']['path']).parent / 'inputs.txt'
    pins = {}
    clocks = []
    for line in path.read_text().splitlines():
        match = re.fullmatch(r'([0-9a-f]{64}) [ *](.+)', line)
        if match:
            item = Path(match[2])
            item = (item if item.is_absolute() else root / item).resolve()
            if item in pins:
                raise RuntimeError('POSTOPT_CAPTURE_INPUT_DUPLICATE')
            pins[item] = match[1]
        if line.startswith('clock_mhz='):
            clocks.append(line.partition('=')[2])
    platform = Path(platform).resolve()
    required = [Path(context['xo']['path']).resolve(), platform, platform.parent / 'hw/hw.xsa',
                root / 'Makefile', root / 'hbm_config.ini',
                root / 'scripts/capture_postopt_only.tcl', root / 'scripts/assert_full_platform.tcl',
                root / 'scripts/postopt_capture_context.py']
    if any(item not in pins for item in required) or len(clocks) != 1 or \
            float(clocks[0]) != context['clock_mhz']:
        raise RuntimeError('POSTOPT_CAPTURE_INPUTS_INCOMPLETE_OR_WRONG_PLATFORM')
    for item, sha in pins.items():
        if not item.is_file() or digest(item) != sha:
            raise RuntimeError('POSTOPT_CAPTURE_INPUT_CHANGED ' + str(item))


def checked_sha_file(path, expected, base=ROOT):
    """Resolve SHA manifest names against their producer's fixed base, not cwd."""
    pins = {}
    for line in path.read_text().splitlines():
        match = re.fullmatch(r'([0-9a-f]{64}) [ *](.+)', line)
        if not match:
            raise RuntimeError('POSTOPT_QUICK_INPUT_PINS_INVALID')
        item = Path(match[2])
        item = (item if item.is_absolute() else Path(base) / item).resolve()
        if item in pins:
            raise RuntimeError('POSTOPT_QUICK_INPUT_PINS_INVALID')
        pins[item] = match[1]
    if set(pins) != {Path(p).resolve() for p in expected}:
        raise RuntimeError('POSTOPT_QUICK_INPUT_PINS_INCOMPLETE')
    for name, sha in pins.items():
        if not name.is_file() or digest(name) != sha:
            raise RuntimeError('POSTOPT_QUICK_INPUT_CHANGED ' + str(name))


def verify_quick(quick, context, clock, root=ROOT):
    quick = Path(quick).resolve()
    runs = Path(context['postopt']['path']).parent.parent.parent / 'runs'
    if quick.parent != runs or not quick.name.startswith('quick.'):
        raise RuntimeError('POSTOPT_SAME_BASELINE_QUICK_REQUIRED')
    if not (quick / 'status.txt').is_file() or \
            (quick / 'status.txt').read_text().strip() != 'SALUT_QUICK_PLACE_PASS':
        raise RuntimeError('POSTOPT_QUICK_NOT_PASS')
    fields = {}
    for line in (quick / 'run.txt').read_text().splitlines():
        name, separator, value = line.partition('=')
        if not separator or name in fields:
            raise RuntimeError('POSTOPT_QUICK_SETTINGS_INVALID')
        fields[name] = value
    if set(fields) != {'baseline', 'clock_mhz', 'stripes', 'lanes'} or \
            fields['baseline'] != context['cache_key'] or float(fields['clock_mhz']) != clock:
        raise RuntimeError('POSTOPT_QUICK_BASELINE_MISMATCH')
    for name, env, default in (('stripes', 'SALUT_ENABLE_STREAM_STRIPES', '0'),
                               ('lanes', 'SALUT_ENABLE_LANE_COLUMNS', '1')):
        if fields[name] not in ('0', '1') or fields[name] != os.environ.get(env, default):
            raise RuntimeError('POSTOPT_QUICK_FLOORPLAN_SETTINGS_CHANGED')
    hooks = {'constrain_task_slrs.tcl': 'constrain_banked_task_slrs.tcl',
             'quick_place_screen.tcl': 'quick_place_screen.tcl',
             'assert_full_platform.tcl': 'assert_full_platform.tcl',
             'report_place_failure.tcl': 'report_place_failure.tcl'}
    checked_sha_file(quick / 'inputs.sha256', [quick / name for name in hooks])
    for frozen, current in hooks.items():
        if digest(quick / frozen) != digest(root / 'scripts' / current):
            raise RuntimeError('POSTOPT_QUICK_FLOORPLAN_CHANGED ' + current)
    checked_sha_file(quick / 'owner_inputs.sha256',
                     [root / 'scripts' / name for name in
                      ('banked_floorplan_owners.py', 'salut_banked_provenance.py')], base=root)
    for name in ('quick_utilization_slr.rpt', 'quick_congestion.rpt', 'driver.log'):
        if not (quick / name).is_file() or not (quick / name).stat().st_size:
            raise RuntimeError('POSTOPT_QUICK_REPORT_MISSING ' + name)
    log = (quick / 'driver.log').read_text()
    if not re.search(r'SALUT Quick legality PASS: [1-9][0-9]* kernel primitives placed;', log) or \
            re.search(r'^\s*(ERROR:|Fatal:|FATAL:)', log, re.M):
        raise RuntimeError('POSTOPT_QUICK_TOOL_LOG_INVALID')


def run_logged(command, cwd, log):
    print('POSTOPT_STAGE ' + str(log) + ' ' + json.dumps(command), flush=True)
    with Path(log).open('x') as output:
        result = subprocess.run(command, cwd=cwd, stdout=output, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError('POSTOPT_TOOL_FAILED exit=' + str(result.returncode) + ' log=' + str(log))


def sdk_command(context, routed, platform, config, clock, run):
    capture = Path(context['capture_dir'])
    return ['v++', '-l', '--platform', str(platform), '--config', str(config),
            '--kernel_frequency', str(clock), '--save-temps', '--target', 'hw',
            '--temp_dir', context['temp_dir'], '--log_dir', str(run / 'sdk_logs'),
            '--report_dir', str(run / 'sdk_reports'), '--reuse_impl', str(routed),
            context['xo']['path'], '-o', str(capture / XCLBIN)]


def package_preserving_project(context, run, routed, platform, config, clock, fpga,
                               root=ROOT):
    """Back up before SDK writes; retain mutated project and restore baseline.

    Exact directories are validated by verify_context before this function.
    No recursive removal or overwrite of historical runs is performed.
    """
    capture = Path(context['capture_dir'])
    backup, changed = run / 'capture_before_sdk', run / 'capture_after_sdk'
    if backup.exists() or changed.exists() or (capture / XCLBIN).exists():
        raise RuntimeError('POSTOPT_NEW_SDK_ATTEMPT_REQUIRED')
    run_logged(['cp', '-a', '--reflink=auto', str(capture), str(backup)],
               root, run / 'backup.stdout')
    # Ensure the preserved bytes match the original captured project BEFORE
    # allowing SDK writes. Resolve targets only in the original namespace.
    for name, pin in context['project_files'].items():
        if digest(backup / '_x' / name) != pin['sha256']:
            raise RuntimeError('POSTOPT_PROJECT_BACKUP_MISMATCH ' + name)
    if digest(backup / 'driver.log') != context['driver_log_sha256']:
        raise RuntimeError('POSTOPT_CAPTURE_LOG_BACKUP_MISMATCH')
    try:
        run_logged(sdk_command(context, routed, platform, config, clock, run),
                   root, run / 'sdk.stdout')
        generated = capture / XCLBIN
        if not generated.is_file() or not generated.stat().st_size:
            raise RuntimeError('POSTOPT_SDK_XCLBIN_MISSING')
        # Keep actual vendor-generated .bit files, not a manufactured dump.
        bits = sorted((capture / '_x').rglob('*.bit'))
        if not bits:
            raise RuntimeError('POSTOPT_ACTUAL_BUILD_BIT_MISSING')
        bit_dir = run / 'generated_bits'
        bit_dir.mkdir()
        for index, bit in enumerate(bits):
            shutil.copy2(bit, bit_dir / (str(index) + '-' + bit.name))
        destination = fpga / XCLBIN
        if destination.exists():
            raise RuntimeError('POSTOPT_XCLBIN_OVERWRITE_FORBIDDEN')
        shutil.copy2(generated, destination)
        if digest(generated) != digest(destination):
            raise RuntimeError('POSTOPT_XCLBIN_COPY_MISMATCH')
    finally:
        # Preserve both complete trees. Restoring the original absolute path
        # retains the SDK context for future attempts without recapturing.
        capture.rename(changed)
        backup.rename(capture)


def implementation_input(quick, transformation, reuse_placed_quick):
    """A legal placed seed is explicit and already covered by the strict gate."""
    if not reuse_placed_quick:
        return transformation['dcp_path'] if transformation else None
    if not transformation or transformation.get('status') != 'derived-full-platform-quick-verified-not-routed':
        raise RuntimeError('POSTOPT_PLACED_SEED_REQUIRES_STRICT_DERIVED_QUICK')
    placed = Path(quick).resolve() / 'salut_level0_placed.dcp'
    expected = transformation.get('placed_sha256')
    if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-f]{64}', expected) or \
            not placed.is_file() or digest(placed) != expected or \
            transformation.get('pinned_files', {}).get(str(placed)) != expected:
        raise RuntimeError('POSTOPT_PLACED_SEED_IDENTITY_MISMATCH')
    return str(placed)


def main():
    parser = argparse.ArgumentParser()
    for name in ('context', 'quick-dir', 'fpga-root', 'hls-root', 'functional', 'platform'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--clock-mhz', type=float, default=200)
    parser.add_argument('--place-directive', default='SSI_BalanceSLLs')
    parser.add_argument('--route-directive', default='Default')
    parser.add_argument('--controlset-probe', type=Path,
                        help='explicit verified property-only derivative; never replace the pristine cache')
    parser.add_argument('--reuse-placed-quick', action='store_true',
                        help='explicit strict Quick placed seed; timing-driven post-place opt, then mandatory physopt/route/all-clock/package gates')
    args = parser.parse_args()
    if not math.isfinite(args.clock_mhz) or args.clock_mhz < 200 or \
            'Quick' in (args.place_directive, args.route_directive) or \
            (args.reuse_placed_quick and (args.controlset_probe is None or args.place_directive != 'Default')):
        raise RuntimeError('POSTOPT_FORMAL_CLOCK_OR_DIRECTIVE_INVALID')
    fpga, context_file = args.fpga_root.resolve(), args.context.resolve()
    # Protect the original SDK namespace and final outputs across manual runs.
    lock = (fpga / 'formal-postopt.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock.close()
        raise RuntimeError('POSTOPT_FORMAL_ATTEMPT_ALREADY_RUNNING') from error
    try:
        run_attempt(args, fpga, context_file)
    finally:
        lock.close()


def run_attempt(args, fpga, context_file):
    context = json.loads(context_file.read_text())
    xo = fpga / ('SparseLUKernel_' + PLATFORM + '.hw.xo')
    cache = context_file.parent
    verify_context(context, cache, xo, cache.name, args.clock_mhz)
    verify_capture_inputs(context, args.platform)
    derivative_probe = getattr(args, 'controlset_probe', None)
    def verify_selected_quick():
        if derivative_probe is None:
            verify_quick(args.quick_dir, context, args.clock_mhz)
            return None
        return verify_controlset_quick(args.quick_dir, derivative_probe, context,
            args.clock_mhz, context_file, args.functional, args.hls_root)
    transformation = verify_selected_quick()
    reuse_placed = getattr(args, 'reuse_placed_quick', False)
    selected_dcp = implementation_input(args.quick_dir, transformation, reuse_placed) or context['postopt']['path']
    functional_command = ['python3', str(ROOT / 'scripts/verify_banked_functional.py'),
        '--manifest', str(args.functional.resolve()), '--expect-xo', str(xo),
        '--expect-hls-root', str(args.hls_root.resolve()), '--expect-clock-mhz', str(args.clock_mhz)]
    # Verify prerequisites before a new attempt directory or vendor job exists.
    subprocess.run(functional_command, cwd=ROOT, check=True)
    if (fpga / XCLBIN).exists() or (fpga / 'acceptance.json').exists():
        raise RuntimeError('POSTOPT_PRIOR_OUTPUT_PRESERVE_REQUIRED')
    attempts = fpga / 'physical_attempts'
    attempts.mkdir(exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='postopt.', dir=attempts))
    os.environ.update(SALUT_FUNCTIONAL_MANIFEST=str(args.functional.resolve()),
        SALUT_FPGA_BUILD_DIR=str(fpga), SALUT_HLS_DIR=str(args.hls_root.resolve()),
        SALUT_BANKED_OWNER_TOOL=str(ROOT / 'scripts/banked_floorplan_owners.py'))
    workflow = {'schema_version': 1, 'kind': 'formal-postopt-reuse', 'status': 'running',
                'start_epoch': time.time(), 'context_sha256': digest(context_file),
                'postopt_sha256': digest(selected_dcp), 'xo_sha256': digest(xo),
                'pristine_postopt_sha256': context['postopt']['sha256']}
    if transformation:
        workflow['verified_controlset_transformation'] = transformation
    if reuse_placed:
        workflow['input_stage'] = 'strict_quick_placed_seed_not_routed'
        workflow['placement_mode'] = 'timing_driven_post_place_opt_then_physopt_and_route'
        workflow['placed_seed_sha256'] = transformation['placed_sha256']
    with (run / 'workflow-start.json').open('x') as stream:
        stream.write(json.dumps(workflow, indent=2, sort_keys=True) + '\n')
    record = run / 'build.json'
    build_tool = ['python3', str(ROOT / 'scripts/banked_build_record.py')]
    extra_inputs = [context_file, Path(context['postopt']['path']),
                    args.quick_dir.resolve() / 'status.txt']
    if transformation:
        extra_inputs.extend(Path(p) for p in transformation['pinned_files'])
    extra_arguments = [arg for path in sorted(set(extra_inputs))
                       for arg in ('--extra-input', str(path))]
    run_logged([*build_tool, 'start', '--record', str(record), '--xo', str(xo),
                '--platform', str(args.platform.resolve()), '--clock-mhz', str(args.clock_mhz),
                *extra_arguments],
               ROOT, run / 'build-start.stdout')
    rc = 1
    try:
        if reuse_placed:
            feedback = args.quick_dir.resolve() / 'salut_fifo_feedback_constraints.rpt'
            expected = transformation['pinned_files'].get(str(feedback))
            if expected != digest(feedback):
                raise RuntimeError('POSTOPT_PLACED_SEED_FEEDBACK_IDENTITY_MISMATCH')
            shutil.copy2(feedback, run / feedback.name)
            axi_constraints = args.quick_dir.resolve() / 'salut_axi_control_constraints.rpt'
            axi_expected = transformation['pinned_files'].get(str(axi_constraints))
            if axi_expected != digest(axi_constraints):
                raise RuntimeError('POSTOPT_PLACED_SEED_AXI_IDENTITY_MISMATCH')
            shutil.copy2(axi_constraints, run / axi_constraints.name)
            os.environ.update(SALUT_FORMAL_PLACED_SEED_SHA256=transformation['placed_sha256'],
                              SALUT_FORMAL_SEED_FEEDBACK_SHA256=expected,
                              SALUT_FORMAL_SEED_AXI_SHA256=axi_expected)
        command = ['vivado', '-mode', 'batch', '-nojournal', '-log', 'vivado.log',
            '-source', str(ROOT / 'scripts/implement_banked_postopt.tcl'), '-tclargs',
            selected_dcp, args.place_directive, args.route_directive,
            str(args.clock_mhz)]
        if reuse_placed:
            command.append('quick_placed_seed')
        run_logged(command, run, run / 'implementation.stdout')
        routed = run / 'salut_routed.dcp'
        workflow['routed_sha256'] = verify_checkpoint(
            json.loads((run / 'salut_routed_audit.json').read_text()),
            json.loads((run / 'salut_architecture.json').read_text()), routed, args.clock_mhz,
            args.hls_root.resolve().parent / 'graph.json')
        verify_context(context, cache, xo, cache.name, args.clock_mhz)
        verify_capture_inputs(context, args.platform)
        checked_transformation = verify_selected_quick()
        if checked_transformation != transformation:
            raise RuntimeError('POSTOPT_TRANSFORMATION_CHAIN_CHANGED_DURING_IMPLEMENTATION')
        subprocess.run(functional_command, cwd=ROOT, check=True)
        package_preserving_project(context, run, routed, args.platform.resolve(),
                                   ROOT / 'hbm_config.ini', args.clock_mhz, fpga)
        verify_context(context, cache, xo, cache.name, args.clock_mhz)
        rc = 0
    except BaseException as error:
        workflow['error'] = str(error)
        raise
    finally:
        workflow.update(status='tools-pass' if rc == 0 else 'failed', exit_code=rc,
                        end_epoch=time.time())
        with (run / 'workflow-finished.json').open('x') as stream:
            stream.write(json.dumps(workflow, indent=2, sort_keys=True) + '\n')
        run_logged([*build_tool, 'finish', '--record', str(record), '--xo', str(xo),
                    '--exit-code', str(rc)], ROOT, run / 'build-finish.stdout')
    # Actual generated bit must equal independent SDK extraction; final clock,
    # all-clock timing, mappings and complete input identities remain mandatory.
    run_logged(['python3', str(ROOT / 'scripts/build_banked_acceptance.py'),
        '--functional', str(args.functional.resolve()), '--build-record', str(run / 'build-finished.json'),
        '--fpga-root', str(fpga), '--routed-dir', str(run),
        '--output', str(fpga / 'acceptance.json')],
        ROOT, run / 'acceptance.stdout')
    print('BANKED_POSTOPT_DEPLOYMENT_ACCEPTANCE_PASS ' + str(fpga / XCLBIN), flush=True)


if __name__ == '__main__':
    main()
