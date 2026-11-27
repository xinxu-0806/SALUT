#!/usr/bin/env python3
"""Gate one full-platform derived Quick, never routed/package acceptance.

The pristine capture remains immutable. This gate validates its explicit
property-only transformation and a NEW native Quick on that exact derivative.
"""
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import zipfile

from banked_floorplan_owners import build_owners
from postopt_capture_context import digest
from salut_banked_provenance import ROOT
from verify_banked_task_resources import verify_resource_census
from verify_controlset_probe import verify as verify_probe, MEMORY_REFS
from verify_feedback_placement import verify_placed_feedback_placement, EXPECTED_FIFOS
from verify_axi_control_placement import verify_axi_control_placement, verify_axi_control_constraints
from verify_close_fifo_placement import verify_close_fifo_placement

HOOKS = {
    'quick_place_screen.tcl': 'screen_controlset_postopt.tcl',
    'assert_full_platform.tcl': 'assert_full_platform.tcl',
    'constrain_task_slrs.tcl': 'constrain_banked_task_slrs.tcl',
    'report_place_failure.tcl': 'report_place_failure.tcl',
    'report_task_placement.tcl': 'report_task_placement.tcl',
    'collect_feedback_placement.tcl': 'collect_feedback_placement.tcl',
    'collect_banked_task_resources.tcl': 'collect_banked_task_resources.tcl',
    'collect_axi_control_placement.tcl': 'collect_axi_control_placement.tcl',
    'collect_close_fifo_placement.tcl': 'collect_close_fifo_placement.tcl',
}
DEPENDENCIES = ('banked_floorplan_owners.py', 'salut_banked_provenance.py',
                'verify_banked_functional.py', 'verify_controlset_probe.py',
                'verify_axi_control_placement.py', 'verify_controlset_quick.py',
                'verify_banked_task_resources.py', 'verify_feedback_placement.py',
                'postopt_capture_context.py', 'run_controlset_quick.py',
                'verify_close_fifo_placement.py')
OUTPUTS = ('driver.log', 'status.txt', 'quick_utilization_slr.rpt',
           'quick_congestion.rpt', 'salut_level0_placed.dcp',
           'salut_task_clock_regions.rpt', 'salut_feedback_placed.rpt',
           'salut_fifo_feedback_constraints.rpt', 'quick_resources_not_accepted.json',
           'salut_axi_control_constraints.rpt', 'salut_axi_control_placed.rpt',
           'salut_close_fifo_placed.rpt')


def checked_pins(pins, expected):
    expected = {str(Path(p).resolve()) for p in expected}
    if not isinstance(pins, dict) or set(pins) != expected:
        raise RuntimeError('CONTROLSET_QUICK_PIN_SCOPE_MISMATCH')
    for name, sha in pins.items():
        if not isinstance(sha, str) or not re.fullmatch(r'[0-9a-f]{64}', sha) or \
                not Path(name).is_file() or digest(name) != sha:
            raise RuntimeError('CONTROLSET_QUICK_PIN_CHANGED ' + name)


def checked_archive(path):
    with zipfile.ZipFile(path) as archive:
        files = archive.infolist()
        if not 1 <= len(files) <= 256 or sum(f.file_size for f in files) > 8 * 1024**3 or \
                archive.testzip() is not None:
            raise RuntimeError('CONTROLSET_QUICK_PLACED_ARCHIVE_INVALID')


def worker_live(worker):
    pid, ticks = worker.get('pid'), worker.get('start_ticks')
    if type(pid) is not int or type(ticks) is not int or pid <= 0 or ticks <= 0:
        raise RuntimeError('CONTROLSET_QUICK_WORKER_IDENTITY_INVALID')
    path = Path('/proc') / str(pid)
    try:
        stat = (path / 'stat').read_text()
        fields = stat[stat.rfind(')') + 2:].split()
        args = [b.decode() for b in (path / 'cmdline').read_bytes().split(b'\0') if b]
    except FileNotFoundError:
        return False
    return int(fields[19]) == ticks and args == worker.get('argv')


def verify_assigned_placement(text, owners):
    """Require every current task and every constrained FIFO on its sole SLR."""
    expected = dict(owners['tasks'])
    if len(expected) != 211:
        raise RuntimeError('CONTROLSET_QUICK_FULL_TASK_COUNT_MISSING')
    for name, stream in owners['streams'].items():
        slr = stream['from_slr'] if stream['hard_local'] else stream['source_feedback_slr']
        if stream.get('consumer_feedback_slr') is not None:
            if slr is not None:
                raise RuntimeError('CONTROLSET_QUICK_FEEDBACK_POLICY_CONFLICT')
            slr = stream['consumer_feedback_slr']
        if slr is not None:
            expected[name] = slr
    reader = csv.DictReader(io.StringIO(text), delimiter='\t')
    columns = ['task', 'requested_slr', 'placed_sites', 'actual_slrs', 'clock_regions']
    if reader.fieldnames != columns:
        raise RuntimeError('CONTROLSET_QUICK_PLACEMENT_HEADER_INVALID')
    seen = set()
    for row in reader:
        name = row.get('task')
        if set(row) != set(columns) or name not in expected or name in seen:
            raise RuntimeError('CONTROLSET_QUICK_PLACEMENT_OWNER_SCOPE_INVALID')
        seen.add(name)
        slr = expected[name]
        count = row['placed_sites']
        if slr not in ('SLR0', 'SLR1', 'SLR2') or row['requested_slr'] != slr or \
                not re.fullmatch(r'[1-9][0-9]*', count or '') or \
                row['actual_slrs'] != slr + ':' + count:
            raise RuntimeError('CONTROLSET_QUICK_PLACEMENT_SLR_MISMATCH ' + name)
        regions = (row['clock_regions'] or '').split(',')
        if any(not re.fullmatch(r'X[0-9]+Y[0-9]+:[1-9][0-9]*', r) for r in regions) or \
                len({r.split(':')[0] for r in regions}) != len(regions) or \
                sum(int(r.split(':')[1]) for r in regions) != int(count):
            raise RuntimeError('CONTROLSET_QUICK_PLACEMENT_REGIONS_INVALID ' + name)
    if seen != set(expected):
        raise RuntimeError('CONTROLSET_QUICK_PLACEMENT_INCOMPLETE')
    return len(seen)


def verify_feedback_owners(constraints, owners):
    expected = {name: stream['source_feedback_slr'] for name, stream in owners['streams'].items()
                if stream['source_feedback_slr'] is not None}
    if set(expected) != EXPECTED_FIFOS:
        raise RuntimeError('CONTROLSET_QUICK_FEEDBACK_GRAPH_SCOPE_MISSING')
    seen = set()
    for row in csv.DictReader(io.StringIO(constraints), delimiter='\t'):
        name = row.get('fifo')
        if name not in expected or row.get('producer_slr') != expected[name]:
            raise RuntimeError('CONTROLSET_QUICK_FEEDBACK_GRAPH_OWNER_MISMATCH')
        seen.add(name)
    if seen != set(expected):
        raise RuntimeError('CONTROLSET_QUICK_FEEDBACK_GRAPH_SCOPE_MISSING')


def verify_controlset_transformation(probe, context, root=ROOT):
    """Before launching native placement, validate the immutable full derivative."""
    probe = Path(probe).resolve()
    pristine = Path(context['postopt']['path']).resolve()
    if probe.parent != pristine.parents[3] / 'physical_audits':
        raise RuntimeError('CONTROLSET_QUICK_CONTEXT_OR_LOCATION_MISMATCH')
    proof = verify_probe(probe)
    probe_start = json.loads((probe / 'start-checked.json').read_text())
    original = probe_start['pristine_input']
    probe_hooks = ('assert_full_platform.tcl', 'collect_banked_task_resources.tcl',
                   'lowfanout_controlset_probe.tcl', 'probe_full_controlset.tcl')
    checked_pins(probe_start['native_start']['snapshots'],
                 [probe / 'snapshot' / name for name in probe_hooks])
    for name in probe_hooks:
        if digest(probe / 'snapshot' / name) != digest(root / 'scripts' / name):
            raise RuntimeError('CONTROLSET_QUICK_TRANSFORMATION_SCRIPT_CHANGED ' + name)
    if Path(original['path']).resolve() != pristine or \
            proof['original_sha256'] != context['postopt']['sha256']:
        raise RuntimeError('CONTROLSET_QUICK_TRANSFORMATION_IDENTITY_MISMATCH')
    audit_path = probe / 'native-completed-and-diagnostic-checked.json'
    audit = json.loads(audit_path.read_text())
    if audit.get('status') != 'native_controlset_optimization_and_independent_diagnostic_verification_completed_not_routed' or \
            type(audit.get('actual_native_exit_code')) is not int or audit['actual_native_exit_code'] != 0 or \
            type(audit.get('actual_verifier_exit_code')) is not int or audit['actual_verifier_exit_code'] != 0 or \
            audit.get('diagnostic_checks') != proof:
        raise RuntimeError('CONTROLSET_QUICK_OPTIMIZATION_EXIT_NOT_VERIFIED')
    evidence = audit['evidence']['completed_files']
    expected_evidence = ('start-checked.json', 'controlset-full-net.log',
                         'resources_before.json', 'resources_after.json',
                         'controlsets_before.rpt', 'controlsets_after.rpt',
                         'ce_candidates.tsv', 'modified_registers.tsv', 'complete.txt')
    if set(evidence) != set(expected_evidence):
        raise RuntimeError('CONTROLSET_QUICK_OPTIMIZATION_EVIDENCE_INCOMPLETE')
    for name, item in evidence.items():
        p = Path(item['path'])
        if p != probe / name or not p.is_file() or p.stat().st_size != item['bytes'] or \
                digest(p) != item['sha256']:
            raise RuntimeError('CONTROLSET_QUICK_OPTIMIZATION_EVIDENCE_CHANGED')
    return proof


def verify_controlset_quick(quick, probe, context, clock, context_file,
                           functional, hls_root, root=ROOT):
    quick, probe = Path(quick).resolve(), Path(probe).resolve()
    pristine = Path(context['postopt']['path']).resolve()
    fpga = pristine.parents[3]
    if not math.isfinite(clock) or clock != 200 or context['clock_mhz'] != clock or \
            quick.parent != fpga / 'quick_pnr/runs' or \
            not quick.name.startswith('controlset-quick.') or \
            probe.parent != fpga / 'physical_audits':
        raise RuntimeError('CONTROLSET_QUICK_CONTEXT_OR_LOCATION_MISMATCH')
    final_path = quick / 'native-finished.json'
    if not final_path.is_file():
        raise RuntimeError('CONTROLSET_QUICK_NATIVE_EXIT_NOT_OBSERVED')
    final = json.loads(final_path.read_text())
    start_path = quick / 'start-checked.json'
    if final.get('schema_version') != 1 or \
            final.get('status') != 'native_derivative_quick_complete_not_routed' or \
            type(final.get('actual_exit_code')) is not int or final['actual_exit_code'] != 0 or \
            final.get('start_sha256') != digest(start_path):
        raise RuntimeError('CONTROLSET_QUICK_TERMINAL_RECORD_INVALID')
    start = json.loads(start_path.read_text())
    if start.get('schema_version') != 1 or \
            start.get('status') != 'native_derivative_full_platform_quick_started_not_accepted':
        raise RuntimeError('CONTROLSET_QUICK_START_RECORD_INVALID')
    if worker_live(start['native_start']['worker']):
        raise RuntimeError('CONTROLSET_QUICK_WORKER_STILL_LIVE')
    inputs = start['inputs']
    checked_pins(inputs['snapshots'], [quick / name for name in HOOKS])
    for frozen, current in HOOKS.items():
        if digest(quick / frozen) != digest(root / 'scripts' / current):
            raise RuntimeError('CONTROLSET_QUICK_CURRENT_HOOK_CHANGED ' + current)
    checked_pins(inputs['canonical_dependencies'], [root / 'scripts' / n for n in DEPENDENCIES])
    checked_pins(final['output_sha256'], [quick / name for name in OUTPUTS])
    derivative = probe / 'controlset_postopt_not_accepted.dcp'
    proof = verify_controlset_transformation(probe, context, root)
    if Path(inputs['source_checkpoint']).resolve() != derivative or \
            inputs['optimization_checks'] != proof:
        raise RuntimeError('CONTROLSET_QUICK_TRANSFORMATION_IDENTITY_MISMATCH')
    audit_path = probe / 'native-completed-and-diagnostic-checked.json'
    audit = json.loads(audit_path.read_text())
    hls_root = Path(hls_root).resolve()
    identities = [audit_path, probe / 'start-checked.json', Path(functional).resolve(),
                  Path(context['xo']['path']).resolve(), hls_root.parent / 'salut_provenance.json',
                  hls_root.parent / 'graph.json', Path(context_file).resolve()]
    pins = inputs['identity_pins']
    checked_pins({k: v.get('sha256') for k, v in pins.items()}, identities)
    if any(Path(k).stat().st_size != v.get('bytes') for k, v in pins.items()):
        raise RuntimeError('CONTROLSET_QUICK_IDENTITY_SIZE_CHANGED')
    env = inputs['environment']
    expected_env = {
        'SALUT_SCREEN_DCP_SHA256': proof['derivative_sha256'],
        'SALUT_WRITE_PLACED_DCP': '1', 'SALUT_HLS_DIR': str(hls_root),
        'SALUT_BANKED_OWNER_TOOL': str(root / 'scripts/banked_floorplan_owners.py'),
        'SALUT_SCREEN_STATUS': str(quick / 'status.txt'),
        'SALUT_ENABLE_STREAM_STRIPES': os.environ.get('SALUT_ENABLE_STREAM_STRIPES', '0'),
        'SALUT_ENABLE_LANE_COLUMNS': os.environ.get('SALUT_ENABLE_LANE_COLUMNS', '1'),
    }
    if env != expected_env or env['SALUT_ENABLE_STREAM_STRIPES'] not in ('0', '1') or \
            env['SALUT_ENABLE_LANE_COLUMNS'] not in ('0', '1'):
        raise RuntimeError('CONTROLSET_QUICK_SETTINGS_CHANGED')
    worker = start['native_start']['worker']
    if worker.get('cwd') != str(quick) or worker.get('argv', [])[1:] != [
            '-mode', 'batch', '-nojournal', '-log', 'vivado.log', '-source',
            str(quick / 'quick_place_screen.tcl'), '-tclargs', str(derivative)]:
        raise RuntimeError('CONTROLSET_QUICK_NATIVE_COMMAND_MISMATCH')
    log = (quick / 'driver.log').read_text()
    prefix = start['native_start']['log_prefix']
    if hashlib.sha256((quick / 'driver.log').read_bytes()[:prefix['bytes']]).hexdigest() != prefix['sha256'] or \
            not re.search(r'^SALUT Quick legality PASS: [1-9][0-9]* kernel primitives placed; timing not certified$', log, re.M) or \
            re.search(r'^\s*(ERROR:|Fatal:|FATAL:)', log, re.M) or \
            'INFO: [Common 17-206] Exiting Vivado at ' not in log or \
            (quick / 'status.txt').read_text().strip() != 'SALUT_QUICK_PLACE_PASS':
        raise RuntimeError('CONTROLSET_QUICK_NATIVE_LOG_OR_STATUS_INVALID')
    placed = quick / 'salut_level0_placed.dcp'
    checked_archive(placed)
    placed_sha = digest(placed)
    census = json.loads((quick / 'quick_resources_not_accepted.json').read_text())
    verify_resource_census(census, placed_sha)
    baseline = json.loads((probe / 'resources_after.json').read_text())['task_resources']
    for owner, refs in baseline.items():
        if any(refs.get(ref, 0) != census['task_resources'][owner].get(ref, 0) for ref in MEMORY_REFS):
            raise RuntimeError('CONTROLSET_QUICK_ACTOR_MEMORY_OR_DSP_CHANGED ' + owner)
    owners = build_owners(json.loads((hls_root.parent / 'graph.json').read_text()))
    verify_axi_control_constraints((quick / 'salut_axi_control_constraints.rpt').read_text(), owners)
    axi_controls = verify_axi_control_placement(
        (quick / 'salut_axi_control_placed.rpt').read_text(), owners, 'placed', placed_sha)
    assigned = verify_assigned_placement((quick / 'salut_task_clock_regions.rpt').read_text(), owners)
    close_feedback = verify_close_fifo_placement(
        (quick / 'salut_close_fifo_placed.rpt').read_text(), owners, 'placed', placed_sha)
    verify_feedback_owners((quick / 'salut_fifo_feedback_constraints.rpt').read_text(), owners)
    feedback = verify_placed_feedback_placement(
        (quick / 'salut_fifo_feedback_constraints.rpt').read_text(),
        (quick / 'salut_feedback_placed.rpt').read_text())
    # Pin the entire input/proof chain in the later formal build record.
    paths = {pristine, derivative, start_path, final_path, audit_path, probe / 'start-checked.json'}
    paths.update(Path(p) for p in inputs['snapshots'])
    paths.update(Path(p) for p in inputs['canonical_dependencies'])
    paths.update(Path(p) for p in inputs['identity_pins'])
    paths.update(Path(p) for p in json.loads((probe / 'start-checked.json').read_text())['native_start']['snapshots'])
    paths.update(Path(v['path']) for v in audit['evidence']['completed_files'].values())
    paths.update(quick / name for name in OUTPUTS)
    return {'status': 'derived-full-platform-quick-verified-not-routed',
            'dcp_path': str(derivative), 'dcp_sha256': proof['derivative_sha256'],
            'pristine_sha256': proof['original_sha256'], 'placed_sha256': placed_sha,
            'assigned_objects': assigned, 'feedback': feedback, 'axi_controls': axi_controls,
            'consumer_close_feedback': close_feedback,
            'pinned_files': {str(p): digest(p) for p in sorted(paths)}}
