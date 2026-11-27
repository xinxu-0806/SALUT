#!/usr/bin/env python3
"""Pin the original capture project for later implementation/SDK recovery.

This validates file identity, not physical legality or routed timing. The
functional gate and Vivado full-platform assertion remain separate mandatory
checks. Original project paths are retained rather than rewritten on copying.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re


REQUIRED_PROJECT_FILES = (
    'link/vivado/vpl/prj/prj.xpr',
    'link/int/kernel_info.dat',
    'link/int/xclbin_orig.xml',
    'link/int/vplConfig.ini',
)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def inventory(temp):
    files = {}
    for path in sorted(temp.rglob('*')):
        if path.is_file():
            files[str(path.relative_to(temp))] = {
                'path': str(path.resolve()), 'sha256': digest(path)}
        elif path.is_symlink():
            raise RuntimeError('CAPTURE_CONTEXT_BROKEN_SYMLINK ' + str(path))
    for name in REQUIRED_PROJECT_FILES:
        if name not in files or not (temp / name).stat().st_size:
            raise RuntimeError('CAPTURE_CONTEXT_PROJECT_MISSING ' + name)
    return files


def baseline(cache, xo, key, clock):
    cache, xo = Path(cache).resolve(), Path(xo).resolve()
    if not isinstance(key, str) or not re.fullmatch('[0-9a-f]{64}', key) or cache.name != key:
        raise RuntimeError('CAPTURE_CONTEXT_CACHE_KEY_INVALID')
    if isinstance(clock, bool) or not isinstance(clock, (int, float)) or \
            not math.isfinite(clock) or clock < 100:  # user-authorised low-frequency fallback (2026-11-24); was 200
        raise RuntimeError('CAPTURE_CONTEXT_CLOCK_INVALID')
    inputs, dcp = cache / 'inputs.txt', cache / 'postopt.dcp'
    ready = cache / 'postopt.dcp.ready'
    if not inputs.is_file() or digest(inputs) != key:
        raise RuntimeError('CAPTURE_CONTEXT_FINGERPRINT_MISMATCH')
    if not dcp.is_file() or not dcp.stat().st_size or not ready.is_file() or \
            ready.read_text().strip() != 'SALUT_CAPTURE_ONLY_COMPLETE':
        raise RuntimeError('CAPTURE_CONTEXT_POSTOPT_NOT_COMPLETE')
    if not xo.is_file() or not xo.stat().st_size:
        raise RuntimeError('CAPTURE_CONTEXT_XO_MISSING')
    return {'cache_key': key, 'clock_mhz': clock,
            'xo': {'path': str(xo), 'sha256': digest(xo)},
            'postopt': {'path': str(dcp), 'sha256': digest(dcp)},
            'inputs_sha256': digest(inputs), 'ready_sha256': digest(ready)}


def capture_context(cache, capture_dir, xo, key, clock):
    cache, capture = Path(cache).resolve(), Path(capture_dir).resolve()
    expected = baseline(cache, xo, key, clock)
    # The original capture must be retained in this cache's own run tree.
    runs = cache.parent.parent / 'runs'
    if capture.parent != runs or not capture.name.startswith('capture.'):
        raise RuntimeError('CAPTURE_CONTEXT_ORIGINAL_RUN_REQUIRED')
    temp = capture / '_x'
    if not temp.is_dir() or temp.is_symlink():
        raise RuntimeError('CAPTURE_CONTEXT_TEMP_PROJECT_MISSING')
    log = capture / 'driver.log'
    if not log.is_file() or not log.stat().st_size:
        raise RuntimeError('CAPTURE_CONTEXT_DRIVER_LOG_MISSING')
    return {'schema_version': 1, 'kind': 'full-postopt-capture', **expected,
            'capture_dir': str(capture), 'temp_dir': str(temp),
            'driver_log_sha256': digest(log), 'project_files': inventory(temp)}


def verify_context(record, cache, xo, key, clock):
    expected = baseline(cache, xo, key, clock)
    if not isinstance(record, dict) or record.get('schema_version') != 1 or \
            record.get('kind') != 'full-postopt-capture':
        raise RuntimeError('CAPTURE_CONTEXT_SCHEMA_INVALID')
    for name, value in expected.items():
        if record.get(name) != value:
            raise RuntimeError('CAPTURE_CONTEXT_IDENTITY_MISMATCH ' + name)
    original = record.get('capture_dir')
    if not isinstance(original, str) or not Path(original).is_absolute():
        raise RuntimeError('CAPTURE_CONTEXT_ORIGINAL_RUN_REQUIRED')
    actual = capture_context(cache, original, xo, key, clock)
    if actual != record:
        raise RuntimeError('CAPTURE_CONTEXT_PROJECT_CHANGED')
    return Path(original)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('capture', 'verify'))
    for name in ('cache', 'xo'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--key', required=True)
    parser.add_argument('--clock-mhz', type=float, required=True)
    parser.add_argument('--capture-dir', type=Path)
    args = parser.parse_args()
    output = args.cache / 'capture_context.json'
    if args.action == 'capture':
        if args.capture_dir is None or output.exists():
            raise RuntimeError('CAPTURE_CONTEXT_NEW_RECORD_REQUIRED')
        record = capture_context(args.cache, args.capture_dir, args.xo, args.key, args.clock_mhz)
        with output.open('x') as stream:
            stream.write(json.dumps(record, indent=2, sort_keys=True) + '\n')
    else:
        if not output.is_file():
            raise RuntimeError('CAPTURE_CONTEXT_RECORD_MISSING')
        record = json.loads(output.read_text())
        verify_context(record, args.cache, args.xo, args.key, args.clock_mhz)
    print('POSTOPT_CAPTURE_CONTEXT_PASS ' + str(output.resolve()) +
          '; physical implementation and SDK packaging are not yet accepted')


if __name__ == '__main__':
    main()
