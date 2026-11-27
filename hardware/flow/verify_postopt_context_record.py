#!/usr/bin/env python3
"""Strict selected-record verification and one narrow terminated-capture repair.

The original capture record is never overwritten. Recovery permits ONLY the
late empty Vivado wrapper error marker after the intentional capture-only stop.
Existing files, logs, XO, DCP and fingerprint must remain byte-identical.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import time
import xml.etree.ElementTree as ET

from postopt_capture_context import capture_context, digest, verify_context

ERROR_MARKER = 'link/vivado/vpl/prj/prj.runs/impl_1/.vivado.error.rst'
EMPTY_SHA = hashlib.sha256(b'').hexdigest()


def process_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def recover_terminated_context(old, cache, xo, key, clock, *,
                               pid_alive=process_alive, pause=lambda: time.sleep(1)):
    if not isinstance(old, dict) or old.get('schema_version') != 1 or \
            old.get('kind') != 'full-postopt-capture':
        raise RuntimeError('CAPTURE_QUIESCENCE_ORIGINAL_RECORD_INVALID')
    first = capture_context(cache, old['capture_dir'], xo, key, clock)
    if set(first) != set(old) or any(first[name] != old[name] for name in first
                                   if name != 'project_files'):
        raise RuntimeError('CAPTURE_QUIESCENCE_BASELINE_OR_LOG_CHANGED')
    before, after = old['project_files'], first['project_files']
    if not isinstance(before, dict) or set(after) - set(before) != {ERROR_MARKER} or \
            set(before) - set(after) or any(after[name] != pin for name, pin in before.items()):
        raise RuntimeError('CAPTURE_QUIESCENCE_UNEXPECTED_PROJECT_CHANGE')
    marker = Path(first['temp_dir']) / ERROR_MARKER
    if marker.is_symlink() or marker.stat().st_size != 0 or \
            after[ERROR_MARKER]['sha256'] != EMPTY_SHA:
        raise RuntimeError('CAPTURE_QUIESCENCE_NONEMPTY_OR_LINKED_ERROR_MARKER')
    if 'SALUT_CAPTURE_ONLY_COMPLETE' not in (Path(first['capture_dir']) / 'driver.log').read_text():
        raise RuntimeError('CAPTURE_QUIESCENCE_NOT_INTENTIONAL_CAPTURE_STOP')
    begin = marker.with_name('.vivado.begin.rst')
    begin_name = str(begin.relative_to(first['temp_dir']))
    if begin_name not in before:
        raise RuntimeError('CAPTURE_QUIESCENCE_WORKER_IDENTITY_MISSING')
    processes = ET.parse(begin).getroot().findall('Process')
    if len(processes) != 1 or processes[0].get('Command') != 'vivado':
        raise RuntimeError('CAPTURE_QUIESCENCE_WORKER_IDENTITY_INVALID')
    if processes[0].get('Host', socket.gethostname()) != socket.gethostname():
        raise RuntimeError('CAPTURE_QUIESCENCE_REMOTE_WORKER_UNVERIFIED')
    pid_text = processes[0].get('Pid', '')
    if not pid_text.isdigit() or int(pid_text) <= 0:
        raise RuntimeError('CAPTURE_QUIESCENCE_WORKER_PID_INVALID')
    pid = int(pid_text)
    if pid_alive(pid):
        raise RuntimeError('CAPTURE_QUIESCENCE_WORKER_STILL_LIVE')
    pause()
    second = capture_context(cache, old['capture_dir'], xo, key, clock)
    if first != second or pid_alive(pid):
        raise RuntimeError('CAPTURE_QUIESCENCE_NOT_QUIESCENT')
    # Reuse the original strict verifier. No file is excluded or ignored.
    verify_context(second, cache, xo, key, clock)
    return second, {'schema_version': 1, 'kind': 'late-empty-wrapper-marker-recovery',
                    'late_marker': ERROR_MARKER, 'late_marker_sha256': EMPTY_SHA,
                    'worker_pid': pid, 'worker_alive': False,
                    'postopt_sha256': second['postopt']['sha256'],
                    'xo_sha256': second['xo']['sha256'],
                    'original_file_count': len(before), 'settled_file_count': len(after),
                    'all_original_file_hashes_unchanged': True,
                    'two_identical_inventories': True, 'checked_epoch': time.time()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('verify', 'recover-terminated-capture'))
    for name in ('cache', 'xo', 'record'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--key', required=True)
    parser.add_argument('--clock-mhz', type=float, required=True)
    parser.add_argument('--output-record', type=Path)
    parser.add_argument('--recovery-audit', type=Path)
    args = parser.parse_args()
    cache, record = args.cache.resolve(), args.record.resolve()
    if record.parent != cache or not record.is_file():
        raise RuntimeError('CAPTURE_SELECTED_RECORD_MUST_BELONG_TO_CACHE')
    original_sha = digest(record)
    old = json.loads(record.read_text())
    if args.action == 'verify':
        verify_context(old, cache, args.xo, args.key, args.clock_mhz)
        print('POSTOPT_SELECTED_CONTEXT_PASS ' + str(record))
        return
    output, audit = args.output_record, args.recovery_audit
    if output is None or audit is None or output.resolve().parent != cache or \
            audit.resolve().parent != cache or output.resolve() == audit.resolve() or \
            output.exists() or audit.exists():
        raise RuntimeError('CAPTURE_RECOVERY_NEW_OUTPUTS_REQUIRED')
    settled, proof = recover_terminated_context(old, cache, args.xo, args.key, args.clock_mhz)
    if digest(record) != original_sha:
        raise RuntimeError('CAPTURE_RECOVERY_ORIGINAL_RECORD_CHANGED')
    with output.open('x') as stream:
        stream.write(json.dumps(settled, indent=2, sort_keys=True) + '\n')
    proof.update(original_record=str(record), original_record_sha256=original_sha,
                 settled_record=str(output.resolve()), settled_record_sha256=digest(output),
                 verifier_sha256=digest(Path(__file__)))
    with audit.open('x') as stream:
        stream.write(json.dumps(proof, indent=2, sort_keys=True) + '\n')
    print('POSTOPT_QUIESCENT_RECORD_CREATED ' + str(output.resolve()) +
          '; original record preserved, Quick and timing not yet accepted')


if __name__ == '__main__':
    main()
