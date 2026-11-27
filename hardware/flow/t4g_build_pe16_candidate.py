#!/usr/bin/env python3
"""v659 PE16 copy of T4g (base = PE16 candidate_pe16_c).
T4g: T4f of v652 with ap_start stages as keep flops (salut_start_stage).

v654 showed synthesis merged the per-task ap_start pipeline flops (their FSM sources are
logically identical), recreating one high-fanout net. salut_start_stage carries keep.


v651 failed the SDK capture gate because synthesis (flatten_hierarchy rebuilt) renamed the
kernel-boundary pins of the skid-driven META buses (inst/data_r_reg[..] instead of
inst/m_axi_META_*_mmap_ARADDR[..]). keep_hierarchy on module SparseLUKernel keeps the original
port names at that one boundary (thin IP wrapper outside); everything inside is unchanged.


1. Reset tree (as v648): each reset consumer gets a dedicated leaf flop at depth 4
   (ap_rst_n -> a_slrS (SLR0) -> b_slrS (SLR0 for S=0, else SLR1) -> c_slrS_gJ (SLR S) -> leaf).
   Consumers are the 1194 original reset ports plus the reset of every new AXI skid stage.
2. Scalars (as v648): each mmap offset consumed outside SLR0 gets two free-running stages.
3. ap_start: each detached task outside SLR0 (sticky ap_start in the TAPA FSM) gets two
   free-running stages (SLR1, then the task SLR). The one handshaked task
   (SalutMemoryDrainJoin_0) is left untouched.
4. META AXI: the three META m_axi interfaces (tasks outside SLR0) get two
   salut_axi_skid stages per channel (AR/AW/W towards memory, R/B back), one in SLR1 and
   one in the task SLR.
HLS RTL, stream FIFOs/relays and all data paths are byte-identical to candidate_c.
"""
from pathlib import Path
import copy, hashlib, json, re, shutil, sys, zipfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OUT = HERE/'candidate_pe16_g'
BASE = HERE/'candidate_pe16_c'   # v659 PE16 relay/dispatch/credit candidate
BASE_XO_SHA = '547b523433b3c518444f4abb735dd6f864f30520307b3b78f88c99d39c81652e'
WORK = 'SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.tapa_work'
XO_NAME = 'SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.hw.xo'
PLAN = HERE/'PE16_owner_plan_NOT_acceptance.json'
CTRL_RTL = HERE/'rtl'/'salut_ctrl_pipe.v'
PREFIX = 'ip_repo/tapa_xrtl_SparseLUKernel_1_0/src/'
TOP = PREFIX + 'SparseLUKernel.v'
COMPONENT = 'ip_repo/tapa_xrtl_SparseLUKernel_1_0/component.xml'
GROUP = 32
HANDSHAKED_TASKS = {'SalutMemoryDrainJoin_0'}
META_TASKS = {'ParallelFrontEmitter_0': 'META_FRONT', 'ParallelScheduler_0': 'META_SCHED',
              'ParallelSegmentEmitter_0': 'META_SEGMENT'}
BEGIN, END = '// SALUT_CTRL_PIPE_BEGIN', '// SALUT_CTRL_PIPE_END'
MODULE_HEADER = 'module SparseLUKernel (\n'
KEEP_HIER = '(* keep_hierarchy = "yes" *) // SALUT_CTRL_PIPE_KEEP_HIERARCHY\n'
DECL_BEGIN, DECL_END = '// SALUT_CTRL_PIPE_DECL_BEGIN', '// SALUT_CTRL_PIPE_DECL_END'
RESET_FORMS = {  # original connection -> (port, active_high)
    '.reset(~ap_rst_n)': ('reset', True), '.ap_rst_n(ap_rst_n)': ('ap_rst_n', False),
    '.rst(~ap_rst_n)': ('rst', True), '.ARESET(ap_rst_n_inv)': ('ARESET', True)}
sys.path.insert(0, str(ROOT/'scripts'))
from salut_banked_provenance import verify_provenance, sha256, verify_graph


def require(value, message):
    if not value:
        raise RuntimeError(message)


def instance_of(text, pos):
    """Name of the module instance whose port list contains position pos."""
    depth, i = 0, pos - 1
    while i >= 0:
        ch = text[i]
        if ch == ')': depth += 1
        elif ch == '(':
            if depth == 0: break
            depth -= 1
        i -= 1
    require(i >= 0, 'NO_PORT_LIST_OPENER')
    m = re.search(r'([A-Za-z_][A-Za-z_0-9]*)\s*$', text[:i])
    require(m, 'NO_INSTANCE_NAME')
    return m.group(1)


def owners(plan):
    own = dict(plan['tasks'])
    for n, s in plan['streams'].items():
        require(n not in own, 'NAME_CLASH ' + n); own[n] = s['to_slr']
    for i in range(8):
        own[f'A_{i}__m_axi'] = plan['tasks'][f'SalutHBMReadWordPort_{i}']
        own[f'L_{i}__m_axi'] = plan['tasks'][f'SalutHBMWordPort_{2*i}']
        own[f'U_{i}__m_axi'] = plan['tasks'][f'SalutHBMWordPort_{2*i+1}']
    own['__tapa_fsm_unit'] = 'SLR0'; own['control_s_axi_U'] = 'SLR0'
    return own


def channel_of(sig):
    for ch in ('AR', 'AW', 'W', 'R', 'B'):
        if sig.startswith(ch): return ch
    raise RuntimeError('AXI_SIGNAL_CHANNEL ' + sig)


def port_widths(text):
    widths = {}
    for m in re.finditer(r'^\s*(input|output)\s+(?:wire\s+)?(?:\[(\d+):0\]\s*)?(m_axi_META_\w+);', text, re.M):
        widths[m.group(3)] = (m.group(1), int(m.group(2)) + 1 if m.group(2) else 1)
    return widths


def transform_top(text, plan):
    own = owners(plan)
    edits, placement = [], {}
    # 1. reset consumers
    sites = []
    for form, (port, high) in RESET_FORMS.items():
        for m in re.finditer(re.escape(form), text):
            sites.append((m.start(), m.end(), instance_of(text, m.start()), port, high))
    names = [s[2] for s in sites]
    n_expect = len(plan['streams']) + len(plan['tasks']) + 1 + 24 + 1   # FIFOs, tasks, FSM, async_mmap, control
    require(len(sites) == n_expect and len(set(names)) == n_expect, f'RESET_SITES {len(sites)} {len(set(names))} {n_expect}')
    require(set(names) == set(own), 'RESET_CONSUMERS_EQUAL_OWNERS ' + str(sorted(set(names) ^ set(own))[:8]))
    for a, b, inst, port, high in sites:
        edits.append((a, b, f'.{port}(salut_rst_{inst})'))
    consumers = {inst: (own[inst], high) for a, b, inst, port, high in sites}
    # 2. scalar offsets consumed outside SLR0
    scalars = []
    for m in re.finditer(r'\.(\w+)\((?P<task>\w+)___(?P<arg>\w+_offset)__q0\)', text):
        task = m['task']
        if instance_of(text, m.start()) != task or own[task] == 'SLR0': continue
        scalars.append((m.start(), m.end(), m.group(1), task, m['arg'], own[task]))
    require(sorted((s[3], s[4]) for s in scalars) == [('ParallelFrontEmitter_0', 'META_FRONT_mmap_offset'),
            ('ParallelScheduler_0', 'META_SCHED_mmap_offset'), ('ParallelSegmentEmitter_0', 'META_SEGMENT_mmap_offset')],
            'SCALAR_SET')
    for a, b, port, task, arg, slr in scalars:
        edits.append((a, b, f'.{port}(salut_scalar_{task}_{arg})'))
    # 3. sticky ap_start of detached tasks outside SLR0
    starts = []
    for m in re.finditer(r'\.ap_start\((?P<task>\w+)__ap_start\)', text):
        task = m['task']
        if instance_of(text, m.start()) != task: continue
        require(task in plan['tasks'], 'AP_START_OF_UNKNOWN_TASK ' + task)
        if own[task] == 'SLR0' or task in HANDSHAKED_TASKS: continue
        starts.append((m.start(), m.end(), task, own[task]))
    started = {s[2] for s in starts}
    require(len(started) == len(starts) and not started & HANDSHAKED_TASKS, 'AP_START_SET')
    for task in started:  # sticky: reset clears it, the global start sets it, nothing else assigns it
        require(len(re.findall(r'\b' + task + r'__ap_start <=', FSM_TEXT)) == 2 and
                FSM_TEXT.count(f"end else if({task}__ap_start_global__q0) begin\n    {task}__ap_start <= 1'b1;") == 1 and
                FSM_TEXT.count(f"if(~ap_rst_n) begin\n    {task}__ap_start <= 1'b0;") == 1, 'STICKY_AP_START ' + task)
    for a, b, task, slr in starts:
        edits.append((a, b, f'.ap_start(salut_start_{task})'))
    # 4. META AXI channels
    widths = port_widths(text)
    axi = []
    for task, port in META_TASKS.items():
        require(own[task] != 'SLR0', 'META_TASK_OUTSIDE_SLR0 ' + task)
        found = {}
        for m in re.finditer(r'\.m_axi_meta_(\w+)\(m_axi_' + port + r'_mmap_(\w+)\)', text):
            require(m.group(1) == m.group(2) and instance_of(text, m.start()) == task, 'META_CONN ' + m.group(0))
            found[m.group(1)] = (m.start(), m.end())
        require(len(found) == 37, f'META_SIGNALS {task} {len(found)}')
        for sig, (a, b) in found.items():
            edits.append((a, b, f'.m_axi_meta_{sig}(salut_axi_{task}_{sig})'))
        axi.append((task, port, found))
    result = text
    for a, b, new in sorted(edits, key=lambda e: e[0], reverse=True):
        result = result[:a] + new + result[b:]
    decl, body = [DECL_BEGIN], [BEGIN]
    # scalar pipelines
    for a, b, port, task, arg, slr in scalars:
        w = f'salut_scalar_{task}_{arg}'
        decl.append(f'  wire [63:0] {w}_s1, {w};')
        body.append(f'salut_scalar_stage #(.WIDTH(64)) salut_scalar_s1_{task} (.clk(ap_clk), .d({task}___{arg}__q0), .q({w}_s1));')
        body.append(f'salut_scalar_stage #(.WIDTH(64)) salut_scalar_s2_{task} (.clk(ap_clk), .d({w}_s1), .q({w}));')
        placement[f'salut_scalar_s1_{task}'] = 'SLR1'; placement[f'salut_scalar_s2_{task}'] = slr
    # ap_start pipelines
    for a, b, task, slr in sorted(starts, key=lambda s: s[2]):
        decl.append(f'  wire salut_start_{task}_s1, salut_start_{task};')
        body.append(f'salut_start_stage salut_start_s1_{task} (.clk(ap_clk), .d({task}__ap_start), .q(salut_start_{task}_s1));')
        body.append(f'salut_start_stage salut_start_s2_{task} (.clk(ap_clk), .d(salut_start_{task}_s1), .q(salut_start_{task}));')
        placement[f'salut_start_s1_{task}'] = 'SLR1'; placement[f'salut_start_s2_{task}'] = slr
    # META AXI skid stages: stage t sits in the task SLR, stage p in SLR1
    axi_record = {}
    for task, port, found in axi:
        slr = own[task]
        for sig in found:
            d, w = widths['m_axi_' + port + '_mmap_' + sig]
            decl.append(f'  wire [{w - 1}:0] salut_axi_{task}_{sig};')
        for ch in ('AR', 'AW', 'W', 'R', 'B'):
            sigs = sorted(s for s in found if channel_of(s) == ch and s not in (ch + 'VALID', ch + 'READY'))
            require(ch + 'VALID' in found and ch + 'READY' in found and sigs, f'CHANNEL {task} {ch}')
            width = sum(widths['m_axi_' + port + '_mmap_' + s][1] for s in sigs)
            towards_memory = ch in ('AR', 'AW', 'W')
            require(all(widths['m_axi_' + port + '_mmap_' + s][0] == ('output' if towards_memory else 'input') for s in sigs + [ch + 'VALID']),
                    f'CHANNEL_DIRECTION {task} {ch}')
            cat_task = '{' + ', '.join(f'salut_axi_{task}_{s}' for s in sigs) + '}'
            cat_port = '{' + ', '.join(f'm_axi_{port}_mmap_{s}' for s in sigs) + '}'
            mid = f'salut_axi_{task}_{ch}_mid'
            decl.append(f'  wire {mid}_valid, {mid}_ready; wire [{width - 1}:0] {mid}_data;')
            t, p = f'salut_axi_{task}_{ch}_t', f'salut_axi_{task}_{ch}_p'
            if towards_memory:
                first = (t, f'salut_axi_{task}_{ch}VALID', f'salut_axi_{task}_{ch}READY', cat_task)
                last = (p, f'm_axi_{port}_mmap_{ch}VALID', f'm_axi_{port}_mmap_{ch}READY', cat_port)
            else:
                first = (p, f'm_axi_{port}_mmap_{ch}VALID', f'm_axi_{port}_mmap_{ch}READY', cat_port)
                last = (t, f'salut_axi_{task}_{ch}VALID', f'salut_axi_{task}_{ch}READY', cat_task)
            body.append(f'salut_axi_skid #(.WIDTH({width})) {first[0]} (.clk(ap_clk), .rst(salut_rst_{first[0]}), '
                        f'.s_valid({first[1]}), .s_ready({first[2]}), .s_data({first[3]}), '
                        f'.m_valid({mid}_valid), .m_ready({mid}_ready), .m_data({mid}_data));')
            body.append(f'salut_axi_skid #(.WIDTH({width})) {last[0]} (.clk(ap_clk), .rst(salut_rst_{last[0]}), '
                        f'.s_valid({mid}_valid), .s_ready({mid}_ready), .s_data({mid}_data), '
                        f'.m_valid({last[1]}), .m_ready({last[2]}), .m_data({last[3]}));')
            placement[t] = slr; placement[p] = 'SLR1'
            consumers[t] = (slr, True); consumers[p] = ('SLR1', True)
            axi_record[f'{task}.{ch}'] = {'signals': sigs, 'width': width}
        require(sorted(s for c in ('AR', 'AW', 'W', 'R', 'B') for s in axi_record[f'{task}.{c}']['signals'] + [c + 'VALID', c + 'READY'])
                == sorted(found), 'ALL_META_SIGNALS_PIPELINED ' + task)
    # reset tree over all consumers
    by_slr = {s: sorted(n for n, (slr, h) in consumers.items() if slr == s) for s in ('SLR0', 'SLR1', 'SLR2')}
    for s, members in by_slr.items():
        k = s[-1]
        decl.append(f'  wire salut_rst_a_slr{k}, salut_rst_b_slr{k};')
        body.append(f'salut_rst_stage salut_rst_stage_a_slr{k} (.clk(ap_clk), .d(ap_rst_n), .q(salut_rst_a_slr{k}));')
        body.append(f'salut_rst_stage salut_rst_stage_b_slr{k} (.clk(ap_clk), .d(salut_rst_a_slr{k}), .q(salut_rst_b_slr{k}));')
        placement[f'salut_rst_stage_a_slr{k}'] = 'SLR0'
        placement[f'salut_rst_stage_b_slr{k}'] = 'SLR0' if s == 'SLR0' else 'SLR1'
        for g in range(0, len(members), GROUP):
            j = g // GROUP; c = f'salut_rst_c_slr{k}_g{j}'
            decl.append(f'  wire {c};')
            body.append(f'salut_rst_stage salut_rst_stage_c_slr{k}_g{j} (.clk(ap_clk), .d(salut_rst_b_slr{k}), .q({c}));')
            placement[f'salut_rst_stage_c_slr{k}_g{j}'] = s
            for inst in members[g:g + GROUP]:
                decl.append(f'  wire salut_rst_{inst};')
                body.append(f'salut_rst_stage #(.INVERT({1 if consumers[inst][1] else 0})) salut_rst_leaf_{inst} '
                            f'(.clk(ap_clk), .d({c}), .q(salut_rst_{inst}));')
                placement[f'salut_rst_leaf_{inst}'] = s
    decl.append(DECL_END); body.append(END)
    anchor = '  wire ap_rst_n_inv;\n'
    require(result.count(anchor) == 1, 'DECL_ANCHOR')
    result = result.replace(anchor, anchor + '\n'.join(decl) + '\n', 1)
    require(result.count('endmodule') == 1, 'SINGLE_MODULE_TOP')
    last = result.rindex('endmodule')
    result = result[:last] + '\n'.join(body) + '\n' + result[last:]
    require(result.count(MODULE_HEADER) == 1 and KEEP_HIER not in result, 'TOP_MODULE_HEADER')
    result = result.replace(MODULE_HEADER, KEEP_HIER + MODULE_HEADER, 1)
    summary = {'reset_leaves_per_slr': {s: len(v) for s, v in by_slr.items()},
               'scalar_pipelines': [(s[3], s[4], s[5]) for s in scalars],
               'ap_start_pipelines': len(starts), 'ap_start_tasks_per_slr': {s: sum(1 for x in starts if x[3] == s) for s in ('SLR1', 'SLR2')},
               'meta_axi_channels': axi_record}
    return result, placement, summary


def inverse(new_text):
    require(new_text.count(KEEP_HIER) == 1, 'KEEP_HIER_ONCE')
    new_text = new_text.replace(KEEP_HIER, '', 1)
    r = re.sub(re.escape(DECL_BEGIN) + r'.*?' + re.escape(DECL_END) + r'\n', '', new_text, flags=re.S)
    r = re.sub(re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n', '', r, flags=re.S)
    r = re.sub(r'\.(\w+)\(salut_scalar_(\w+?)_(META_\w+_mmap_offset)\)', r'.\1(\2___\3__q0)', r)
    r = re.sub(r'\.ap_start\(salut_start_(\w+)\)', r'.ap_start(\1__ap_start)', r)
    for task, port in META_TASKS.items():
        r = re.sub(r'\.m_axi_meta_(\w+)\(salut_axi_' + task + r'_\1\)', lambda m, p=port: f'.m_axi_meta_{m.group(1)}(m_axi_{p}_mmap_{m.group(1)})', r)
    back = {'reset': '.reset(~ap_rst_n)', 'ap_rst_n': '.ap_rst_n(ap_rst_n)', 'rst': '.rst(~ap_rst_n)', 'ARESET': '.ARESET(ap_rst_n_inv)'}
    r = re.sub(r'\.(reset|ap_rst_n|rst|ARESET)\(salut_rst_\w+\)', lambda m: back[m.group(1)], r)
    return r


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


FSM_TEXT = ''


def main():
    global FSM_TEXT
    plan = json.loads(PLAN.read_text())
    manifest_path = BASE/'salut_provenance.json'
    manifest = json.loads(manifest_path.read_text())
    hls = BASE/WORK/'hls'; base_xo = BASE/XO_NAME
    require(sha256(base_xo) == BASE_XO_SHA, 'BASE_XO_IS_V661_PE16_CANDIDATE_C')
    verify_provenance(manifest, hls, base_xo, expected_kind='full')
    OUT.mkdir(exist_ok=False)
    inputs = {str(p): sha256(p) for p in (manifest_path, base_xo, Path(__file__), PLAN, CTRL_RTL)}
    with zipfile.ZipFile(base_xo) as archive:
        names = archive.namelist()
        require(len(names) == len(set(names)) and archive.testzip() is None, 'BASE_ARCHIVE_CRC')
        originals = {n: archive.read(n) for n in names}
        FSM_TEXT = originals[PREFIX + 'SparseLUKernel_fsm.v'].decode()
        old_top = originals[TOP].decode()
        new_top, placement, summary = transform_top(old_top, plan)
        require(inverse(new_top) == old_top, 'INVERSE_TRANSFORM_REPRODUCES_BASE_TOP')
        graph = json.loads(Path(manifest['graph']['path']).read_text()); verify_graph(graph, 'full')
        changed = {TOP: new_top.encode(), COMPONENT: transform_component(originals[COMPONENT].decode(), ['salut_ctrl_pipe.v']).encode()}
        added = {PREFIX + 'salut_ctrl_pipe.v': CTRL_RTL.read_bytes()}
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
    value = copy.deepcopy(manifest)
    value['xo'] = {'path': str(OUT/XO_NAME), 'sha256': sha256(OUT/XO_NAME)}
    value['rtl_transform'] = {'kind': 'v645-candidate_c + reset tree + scalar/ap_start/META-AXI pipelines + top keep_hierarchy + keep ap_start stages (control infrastructure only)',
        'base_xo': manifest['xo'], 'parent_manifest': {'path': str(manifest_path), 'sha256': inputs[str(manifest_path)]},
        'parent_rtl_transform': manifest.get('rtl_transform'),
        'changed_archive_entries': sorted(changed), 'added_archive_entries': sorted(added),
        'reset_depth': 4, 'scalar_depth': 2, 'ap_start_depth': 2, 'meta_axi_skid_stages': 2, **summary,
        'owner_plan': {'path': str(PLAN), 'sha256': inputs[str(PLAN)]}, 'complete_archive_source_hashes': hashes,
        'compute_HLS_RTL_unchanged_by_this_transform': True, 'full_top_numerical_regression_passed': False, 'deployment_qualified': False}
    for p in (OUT/'salut_provenance.json', work/'salut_provenance.json'):
        with p.open('x') as s: json.dump(value, s, sort_keys=True, indent=2); s.write('\n')
    verify_provenance(value, work/'hls', OUT/XO_NAME, expected_kind='full')
    with (HERE/'ctrl_pipe_placement.json').open('x') as s: json.dump(placement, s, indent=1, sort_keys=True); s.write('\n')
    for p, d in inputs.items(): require(sha256(Path(p)) == d, 'INPUT_CHANGED ' + p)
    record = {'accepted': False, 'kind': 'v661-pe16-ctrl-candidate-XO-awaiting-functional-regression', 'new_xo': value['xo'],
              'ctrl_cells': len(placement), 'changed_entries': sorted(changed), 'added_entries': sorted(added),
              'HLS_files': len(value['rtl_files']), **summary}
    with (OUT/'T4g_pe16_candidate_build_NOT_product.json').open('x') as s: json.dump(record, s, indent=2); s.write('\n')
    print(json.dumps(record, indent=1))


if __name__ == '__main__':
    main()
