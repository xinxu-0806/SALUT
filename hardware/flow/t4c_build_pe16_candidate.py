#!/usr/bin/env python3
"""v659 PE16 copy of T4c. Original: T4c: deployable-candidate XO from the v642 gearbox base XO.

Per FIFO instance in SparseLUKernel.v (all 843 are plain `fifo` in the base):
  cross-SLR stream (v642 plan)        -> salut_relay_station (.LEVEL 3; 4 for SLR0<->SLR2 streams)
  non-cross seg_task_*                -> salut_dispatch_fifo   (exact capacity, as the v538 lineage)
  non-cross lm_installed_/sm_published_ -> salut_credit_fifo   (as v538)
  everything else                     -> unchanged fifo
Four RTL files are added byte-identically (three from the v538 XO, salut_relay_station.v from v636)
and their synthesis/simulation file-set entries are cloned from src/relay_station.v.
HLS/compute RTL is unchanged. The base XO, HLS tree and manifest are never overwritten.
"""
from pathlib import Path
import copy, hashlib, json, re, shutil, sys, zipfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OUT = HERE/'candidate_pe16_c'
BASE = next(ROOT.glob('build/fpga/banked-lu-pe16gb-v661.*'))
WORK = 'SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.tapa_work'
XO_NAME = 'SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.hw.xo'
PLAN = HERE/'PE16_owner_plan_NOT_acceptance.json'
V538_XO = HERE.parent/'v538-publishCredit-fullCandidate.6UK8H2Oo'/XO_NAME
V538_XO_SHA = 'b41678a58998a85661cd5567dc7475a7a5388c74ea28bc77746741478c11a5bc'
RELAY_RTL = HERE.parent/'v636-CrossSLRRelayCandidate.g5fJxb4i/rtl/salut_relay_station.v'
RELAY_RTL_SHA = 'ea43920a81a1409343e870dab3c35017df230fd810c5bfd9b2e4d38ca59e8db3'
PREFIX = 'ip_repo/tapa_xrtl_SparseLUKernel_1_0/src/'
PATTERN = re.compile(r'\b(?P<module>fifo|salut_relay_station|salut_dispatch_fifo|salut_credit_fifo)'
                     r'\s*#\s*\((?P<parameters>[^;]*?)\)\s*(?P<name>[A-Za-z_][A-Za-z_0-9]*)\s*\(', re.S)
sys.path.insert(0, str(ROOT/'scripts'))
from salut_banked_provenance import verify_provenance, sha256, verify_graph
from audit_banked_rtl_cuts import integer_expression


def require(value, message):
    if not value:
        raise RuntimeError(message)


def inventory(text):
    found = {}
    for m in PATTERN.finditer(text):
        require(m['name'] not in found, 'DUPLICATE_FIFO ' + m['name'])
        found[m['name']] = m
    return found


def params_of(m):
    p = m['parameters']
    w = re.findall(r'\.DATA_WIDTH\s*\(([^()]*)\)', p); d = re.findall(r'\.DEPTH\s*\((\d+)\)', p)
    a = re.findall(r'\.ADDR_WIDTH\s*\((\d+)\)', p)
    require(len(w) == 1 and len(d) == 1 and len(a) == 1 and 'LEVEL' not in p, 'EXACT_PARAMS ' + m['name'])
    return integer_expression(w[0]), int(d[0]), int(a[0])


def decide(plan):
    cross = set(plan['cross_slr_streams']); double = set(plan['double_skip_streams'])
    out = {}
    for n in plan['streams']:
        fam = re.sub(r'_\d+$', '', n)
        if n in cross: out[n] = ('salut_relay_station', 4 if n in double else 3)
        elif fam == 'seg_task': out[n] = ('salut_dispatch_fifo', None)
        elif fam in ('lm_installed', 'sm_published'): out[n] = ('salut_credit_fifo', None)
        else: out[n] = ('fifo', None)
    return out


def transform_top(text, decision):
    found = inventory(text)
    require(set(found) == set(decision), 'ALL_FIFOS_IN_PLAN')
    edits = []
    for name, m in found.items():
        module, level = decision[name]
        require(m['module'] == 'fifo', 'BASE_IS_PLAIN_FIFO ' + name)
        w, d, a = params_of(m)
        if module == 'salut_dispatch_fifo': require((w, d, a) == (385, 8, 3), 'DISPATCH_PROTOCOL ' + name)
        if module == 'salut_credit_fifo': require((w, d, a) == (41, 4, 2), 'CREDIT_PROTOCOL ' + name)
        if module == 'fifo': continue
        body = text[m.start('parameters'):m.end('parameters')].rstrip()
        insert_at = m.start('parameters') + len(body)
        addition = ',\n  .LEVEL(%d)' % level if level else ''
        edits.append((m.start('module'), m.end('module'), module, insert_at, addition))
    result = text
    for first, last, module, insert_at, addition in sorted(edits, key=lambda e: e[0], reverse=True):
        result = result[:first] + module + result[last:insert_at] + addition + result[insert_at:]
    return result


def inverse(new_text):
    result = new_text
    for name, m in sorted(inventory(new_text).items(), key=lambda kv: kv[1].start(), reverse=True):
        if m['module'] == 'fifo': continue
        params = m['parameters']; stripped = params.rstrip()
        mm = re.search(r',\n  \.LEVEL\((\d+)\)$', stripped)
        new_params = (stripped[:mm.start()] if mm else stripped) + params[len(stripped):]
        require((mm is not None) == (m['module'] == 'salut_relay_station'), 'LEVEL_ONLY_ON_RELAYS ' + name)
        result = result[:m.start('module')] + 'fifo' + result[m.end('module'):m.start('parameters')] + new_params + result[m.end('parameters'):]
    return result


def transform_component(text, new_files):
    for f in new_files: require(f'src/{f}' not in text, 'NEW_FILE_ALREADY_PRESENT ' + f)
    pattern = r'(?m)^[ \t]*<spirit:file>\s*<spirit:name>src/relay_station\.v</spirit:name>.*?</spirit:file>'
    matches = list(re.finditer(pattern, text, re.S))
    require(len(matches) == 2, 'EXACT_SYNTHESIS_AND_SIMULATION_FILESETS_REQUIRED')
    result = text
    for m in reversed(matches):
        clones = ''.join('\n' + m[0].replace('src/relay_station.v', f'src/{f}') for f in new_files)
        result = result[:m.end()] + clones + result[m.end():]
    old = ET.fromstring(text); new = ET.fromstring(result); removed = 0
    for parent in new.iter():
        children = list(parent)
        for index in range(len(children) - 1, -1, -1):
            child = children[index]
            if child.tag.endswith('}file') and any(n.tag.endswith('}name') and n.text in {f'src/{f}' for f in new_files} for n in child):
                children[index - 1].tail = child.tail
                parent.remove(child); removed += 1
    require(removed == 2 * len(new_files) and ET.tostring(old) == ET.tostring(new), 'ALL_OLD_COMPONENT_METADATA_UNCHANGED')
    return result


def main():
    plan = json.loads(PLAN.read_text())
    decision = decide(plan)
    manifest_path = BASE/WORK/'salut_provenance.json'
    manifest = json.loads(manifest_path.read_text())
    hls = BASE/WORK/'hls'; base_xo = BASE/XO_NAME
    verify_provenance(manifest, hls, base_xo, expected_kind='full')
    require(sha256(V538_XO) == V538_XO_SHA and sha256(RELAY_RTL) == RELAY_RTL_SHA, 'PINNED_RTL_SOURCES')
    OUT.mkdir(exist_ok=False)
    inputs = {str(p): sha256(p) for p in (manifest_path, base_xo, Path(__file__), PLAN, V538_XO, RELAY_RTL)}
    with zipfile.ZipFile(V538_XO) as v538:
        donor = {f: v538.read(PREFIX + f) for f in ('salut_dispatch_fifo.v', 'fifo_bram_exact_capacity.v', 'salut_credit_fifo.v')}
    donor['salut_relay_station.v'] = RELAY_RTL.read_bytes()
    with zipfile.ZipFile(base_xo) as archive:
        names = archive.namelist()
        require(len(names) == len(set(names)) and archive.testzip() is None, 'BASE_ARCHIVE_CRC')
        originals = {n: archive.read(n) for n in names}
        top_name = PREFIX + 'SparseLUKernel.v'; component = 'ip_repo/tapa_xrtl_SparseLUKernel_1_0/component.xml'
        require(top_name in originals and component in originals, 'TOP_AND_COMPONENT')
        require(originals[PREFIX + 'relay_station.v'] and hashlib.sha256(originals[PREFIX + 'relay_station.v']).hexdigest() ==
                'c65141af17759766738a8f5f51e67adc91cc79ffefc9cd7116776189b65953cb', 'SAME_PATCHED_RELAY_STATION_AS_TESTED')
        old_top = originals[top_name].decode()
        new_top = transform_top(old_top, decision)
        require(inverse(new_top) == old_top, 'INVERSE_TRANSFORM_REPRODUCES_BASE_TOP')
        graph = json.loads(Path(manifest['graph']['path']).read_text()); verify_graph(graph, 'full')
        new_files = sorted(donor)
        changed = {top_name: new_top.encode(), component: transform_component(originals[component].decode(), new_files).encode()}
        added = {PREFIX + f: donor[f] for f in new_files}
        require(not set(added) & set(originals), 'NO_OVERWRITE')
        with zipfile.ZipFile(OUT/XO_NAME, 'x') as output:
            output.comment = archive.comment
            for info in archive.infolist(): output.writestr(info, changed.get(info.filename, originals[info.filename]))
            for n in sorted(added): output.writestr(n, added[n], compress_type=zipfile.ZIP_DEFLATED)
    with zipfile.ZipFile(OUT/XO_NAME) as archive:
        require(archive.testzip() is None and set(archive.namelist()) == set(originals) | set(added), 'CANDIDATE_ARCHIVE_CRC')
        hashes = {}
        for n in archive.namelist():
            data = archive.read(n); require(data == changed.get(n, added.get(n, originals.get(n))), 'ENTRY_EXACT ' + n)
            hashes[n] = hashlib.sha256(data).hexdigest()
    work = OUT/WORK
    for rel, digest in manifest['rtl_files'].items():
        t = work/'hls'/rel; t.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(hls/rel, t)
        require(sha256(t) == digest, 'COPIED_HLS ' + rel)
    shutil.copy2(Path(manifest['graph']['path']), work/'graph.json')
    counts = {}
    for m in inventory(new_top).values(): counts[m['module']] = counts.get(m['module'], 0) + 1
    value = copy.deepcopy(manifest)
    value['xo'] = {'path': str(OUT/XO_NAME), 'sha256': sha256(OUT/XO_NAME)}
    value['rtl_transform'] = {'kind': 'v659-pe16-gearbox4nb-base + relay/dispatch/credit FIFO variant',
        'base_xo': manifest['xo'], 'parent_manifest': {'path': str(manifest_path), 'sha256': inputs[str(manifest_path)]},
        'changed_archive_entries': sorted(changed), 'added_archive_entries': sorted(added),
        'final_module_counts': counts, 'relay_levels': {str(l): sum(1 for v in decision.values() if v[1] == l) for l in (3, 4)},
        'owner_plan': {'path': str(PLAN), 'sha256': inputs[str(PLAN)]}, 'complete_archive_source_hashes': hashes,
        'compute_HLS_RTL_unchanged_by_this_transform': True, 'full_top_numerical_regression_passed': False, 'deployment_qualified': False}
    for p in (OUT/'salut_provenance.json', work/'salut_provenance.json'):
        with p.open('x') as s: json.dump(value, s, sort_keys=True, indent=2); s.write('\n')
    verify_provenance(value, work/'hls', OUT/XO_NAME, expected_kind='full')
    for p, d in inputs.items(): require(sha256(Path(p)) == d, 'INPUT_CHANGED ' + p)
    record = {'accepted': False, 'kind': 'v659-pe16-candidate-XO-awaiting-functional-regression', 'new_xo': value['xo'],
              'final_module_counts': counts, 'relay_levels': value['rtl_transform']['relay_levels'],
              'changed_entries': sorted(changed), 'added_entries': sorted(added), 'HLS_files': len(value['rtl_files'])}
    with (OUT/'T4c_candidate_build_NOT_product.json').open('x') as s: json.dump(record, s, indent=2); s.write('\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
