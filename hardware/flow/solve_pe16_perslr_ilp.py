#!/usr/bin/env python3
"""v661 per-SLR LUT caps (relieve SLR0). Base: v659 (PE16 copy of the v641 gearbox4 ILP): measured 32-PE per-instance resources/widths are
reused by instance/stream name; the PE16 graph (no 16..31 PEs) and its base top RTL replace the
32-PE T1 plan and /tmp top. Missing rows fall back to the family median (counted and reported).

Original: v641: global task->SLR ILP. Minimises the larger SLR boundary SLL demand (incl. platform fixed use).

Units: 211 tasks + control_s_axi_U. Each X_i__m_axi wrapper and every FIFO's resources ride with
their owning task / consumer task. Per-boundary crossing cost:
  streams     : min(din_driven, dout_used) + 2 per crossed boundary (relay data + valid + full_n)
  HBM / META  : the owning unit's kernel-port nets cross towards the platform in SLR0
  control args: control_s_axi_U -> task argument bits (RTL widths)
Resource caps per SLR: LUT+LUTRAM+SRL, BRAM tiles, URAM, DSP incl. measured platform use (v634).
Output is a plan only; nothing native is changed.
"""
import argparse, collections, csv, json, re, sys
from pathlib import Path
import numpy as np
from scipy.optimize import milp, LinearConstraint, Bounds
from scipy.sparse import lil_matrix

HERE = Path(__file__).resolve().parent
AUD = HERE.parent
V659 = next(AUD.glob('v659-PE16Implementation.*'))  # v661: inputs live in v659
V641 = next(AUD.glob('v641-GlobalILPRepartition.*'))
T1 = V659/'PE16_graph_plan_NOT_acceptance.json'
TOP_V = V659/'pe16_base_top.v'   # extracted from the PE16 base XO
SLRS = ('SLR0', 'SLR1', 'SLR2')
# measured v634 (whole - kernel) platform use and device SLR totals
PLAT = {'LUT': (89955, 58571, 26283), 'BRAM': (67.5, 82, 50), 'URAM': (0, 0, 0), 'DSP': (0, 0, 4)}
TOTAL = {'LUT': (439672, 432006, 432026), 'BRAM': (672, 672, 672), 'URAM': (320, 320, 320), 'DSP': (2928, 3072, 3072)}
FIXED = {'01': 7022, '12': 5182}   # static + ULP-platform SLL use not modelled per unit
STAGE0 = AUD/'v638-RelayCandidateThreeDirectivePlaceConstOptOK.pp5ksKJ2/probe_sll/relay_stage0_ff.tsv'
BROADCAST = ('seg_dep', 'seg_task', 'front_dep_spu')
GEARBOX_FAMILIES = {'rsp_omega','wr_omega','front_rd_rsp_spu','pe_rd_rsp','peg_lower_rsp','a_rsp','factor_rsp','rsp_omega_0','rsp_omega_1','rsp_omega_2','rsp_omega_3',
  'a_word_rsp','factor_word_rsp','wr_omega_0','wr_omega_1','wr_omega_2','wr_omega_3','factor_writeback','front_global_data','front_wr_elem_spu',
  'lane_data','lm_snoop_data','pe_wr_elem','peg_global_data'}


def load():
    plan = json.loads(T1.read_text())
    tasks = list(plan['tasks'])
    res = {}
    for r in csv.DictReader(open(V641/'instance_resources.tsv'), delimiter='\t'):
        res[r['inst']] = r
    # Directional boundary bits (pin names may be optimized away). Inputs include clk/reset/read/write,
    # outputs include empty_n/full_n; relay data width ~= min(in-4, out-2).
    eff = {}
    for r in csv.DictReader(open(V641/'fifo_directional_bits.tsv'), delimiter='\t'):
        eff[r['inst']] = max(min(int(r['in_driven']) - 4, int(r['out_used']) - 2), 0)
    # Relays: exact unique bits = stage0 FFs - 2. Synthesis merged identical broadcast stage
    # registers in three families; their shared bits cross once per boundary per group.
    stage0 = {r['relay']: int(r['stage0_FF']) for r in csv.DictReader(open(STAGE0), delimiter='\t')}
    fam_unique = {}; fam_shared = {}
    for fam in BROADCAST:
        rows = [(k, v) for k, v in stage0.items() if k.rsplit('_', 1)[0] == fam]
        uniq = sorted(v - 2 for _, v in rows); fam_unique[fam] = uniq[len(uniq)//2]
        sh = sorted(eff[k] - (v - 2) for k, v in rows); fam_shared[fam] = sh[len(sh)//2]
    for k, v in stage0.items():
        eff[k] = v - 2
    return plan, tasks, res, eff, fam_unique, fam_shared


def control_args(tasks):
    """argument bits per task from __tapa_fsm_unit/control outputs (RTL)."""
    t = TOP_V.read_text()
    width = {}
    for m in re.finditer(r'\b(?:wire|reg|input|output)\s+(?:wire\s+|reg\s+)?(?:\[(\d+):(\d+)\])?\s*([A-Za-z_]\w*)\s*;', t):
        width[m.group(3)] = (int(m.group(1)) - int(m.group(2)) + 1) if m.group(1) else 1
    inst_re = re.compile(r'^\s*(\w+)\s*(?:#\s*\(.*?\)\s*)?(\w+)\s*\((.*)\)\s*$', re.S)
    conn = {}
    for s in re.split(r';\s*\n', t):
        m = inst_re.match(s)
        if m:
            conn[m.group(2)] = re.findall(r'\.(\w+)\s*\(\s*([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*\)', m.group(3))
    fsm_sigs = {sig for _, sig in conn['__tapa_fsm_unit']}
    bits = collections.Counter()
    for name, cs in conn.items():
        if name in tasks:
            for _, sig in cs:
                if sig in fsm_sigs:
                    bits[name] += width.get(sig, 1)
    return bits


def build(args):
    plan, tasks, res, eff, fam_unique, fam_shared = load()
    units = tasks + ['control_s_axi_U']
    idx = {u: i for i, u in enumerate(units)}
    n = len(units)
    owner_of_axi = {}
    for k, v in {'A': 'SalutHBMReadWordPort_{}', }.items():
        pass
    for i in range(8):
        owner_of_axi[f'A_{i}__m_axi'] = f'SalutHBMReadWordPort_{i}'
        owner_of_axi[f'L_{i}__m_axi'] = f'SalutHBMWordPort_{2*i}'
        owner_of_axi[f'U_{i}__m_axi'] = f'SalutHBMWordPort_{2*i+1}'
    R = collections.defaultdict(lambda: collections.Counter())
    port = collections.Counter()
    def add(unit, r):
        R[unit]['LUT'] += int(r['LUT']) + int(r['LUTRAM']) + int(r['SRL'])
        R[unit]['BRAM'] += int(r['RAMB36']) + 0.5*int(r['RAMB18'])
        R[unit]['URAM'] += int(r['URAM'])
        R[unit]['DSP'] += int(r['DSP'])
        port[unit] += int(r['port_nets'])
    streams = plan['streams']
    fam = lambda n: re.sub(r'(_\d+)+$', '', n)
    by_fam = collections.defaultdict(list)
    for inst, r in res.items(): by_fam[fam(inst)].append(r)
    dropped, filled = [], []
    for inst in list(idx) + list(streams) + list(owner_of_axi) + ['__tapa_fsm_unit']:
        if inst == 'control_s_axi_U': continue
        if inst not in res:
            rows = sorted(by_fam.get(fam(inst), []), key=lambda r: int(r['LUT']))
            if not rows: raise SystemExit('NO_RESOURCE_FAMILY ' + inst)
            res[inst] = rows[len(rows)//2]; filled.append(inst)
    for inst, r in res.items():
        if inst in idx: add(inst, r)
        elif inst in owner_of_axi: add(owner_of_axi[inst], r)
        elif inst in streams: add(streams[inst]['consumer'], r)
        elif inst == '__tapa_fsm_unit': add('control_s_axi_U', r)
        else: dropped.append(inst)
    print('PE16 resources: dropped 32-PE-only rows', len(dropped), 'family-filled', len(filled), filled[:12], flush=True)
    # control_s_axi port nets are AXI-lite from the platform control interconnect (SLR0 side)
    edges = []; groups = collections.defaultdict(list)
    for name, s in streams.items():
        if name not in eff:   # not measured in the 32-PE design: RTL width
            eff[name] = s['width']
        fam = name.rsplit('_', 1)[0]
        w = fam_unique[fam] if fam in BROADCAST else eff[name]
        base = re.sub(r'(_\d+)+$', '', name)
        if base in GEARBOX_FAMILIES and w > 64:
            w = (w + 3) // 4 + 8   # 4-beat gearbox on crossing links (+ beat-phase/valid overhead)
        edges.append((idx[s['producer']], idx[s['consumer']], w + 2, name))
        if fam in BROADCAST:
            groups[(s['producer'], fam)].append(idx[s['consumer']])
    group_list = [(idx[p], members, fam_shared[f], f) for (p, f), members in groups.items()]
    print('broadcast groups', [(f, len(m), sh) for _, m, sh, f in group_list], 'unique', fam_unique, flush=True)
    for task, b in control_args(tasks).items():
        edges.append((idx['control_s_axi_U'], idx[task], b, 'ctrl->' + task))
    return units, idx, R, port, edges, plan, group_list


def solve(units, idx, R, port, edges, groups, args):
    n = len(units); m = len(edges); G = len(groups)
    # variables: a[n] (SLR0), c[n] (SLR2), y01[m], y12[m], g01[G], g12[G], Z
    na, nc = 0, n
    ny01, ny12 = 2*n, 2*n + m
    ng01, ng12 = 2*n + 2*m, 2*n + 2*m + G
    nz = 2*n + 2*m + 2*G
    nv = nz + 1
    lo = []; hi = []
    nrows = n + 4*m + 4*sum(len(g[1]) for g in groups) + 2 + 12
    A = lil_matrix((nrows, nv))
    r = 0
    def row(coeffs, l, h):
        nonlocal r
        for j, v in coeffs: A[r, j] = v
        lo.append(l); hi.append(h); r += 1
    for i in range(n):
        row([(na+i, 1), (nc+i, 1)], 0, 1)
    def absdiff(y, base, u, v):
        row([(y, 1), (base+u, -1), (base+v, 1)], 0, np.inf)
        row([(y, 1), (base+u, 1), (base+v, -1)], 0, np.inf)
    for k, (u, v, w, _) in enumerate(edges):
        absdiff(ny01+k, na, u, v); absdiff(ny12+k, nc, u, v)
    for gi, (pidx, members, sh, _) in enumerate(groups):
        for v in members:
            absdiff(ng01+gi, na, pidx, v); absdiff(ng12+gi, nc, pidx, v)
    total_port = sum(port.values())
    c01 = [(ny01+k, e[2]) for k, e in enumerate(edges)] + [(ng01+gi, g[2]) for gi, g in enumerate(groups)] + [(na+idx[u], -port[u]) for u in units if port[u]]
    row(c01 + [(nz, -1)], -np.inf, -FIXED['01'] - total_port)
    c12 = [(ny12+k, e[2]) for k, e in enumerate(edges)] + [(ng12+gi, g[2]) for gi, g in enumerate(groups)] + [(nc+idx[u], port[u]) for u in units if port[u]]
    row(c12 + [(nz, -1)], -np.inf, -FIXED['12'])
    for kind in ('LUT', 'BRAM', 'URAM', 'DSP'):
        u = args.lut_per_slr if kind == 'LUT' else [args.util[kind]] * 3   # v661: per-SLR LUT caps
        caps = [u[s] * TOTAL[kind][s] - PLAT[kind][s] for s in range(3)]
        tot = sum(R[u][kind] for u in units)
        row([(na+idx[u], R[u][kind]) for u in units if R[u][kind]], 0, caps[0])
        row([(nc+idx[u], R[u][kind]) for u in units if R[u][kind]], 0, caps[2])
        row([(na+idx[u], -R[u][kind]) for u in units if R[u][kind]] + [(nc+idx[u], -R[u][kind]) for u in units if R[u][kind]], -np.inf, caps[1] - tot)
    A = A[:r].tocsr()
    cost = np.zeros(nv); cost[nz] = 1.0
    w = np.array([e[2] for e in edges]); gw = np.array([g[2] for g in groups])
    cost[ny01:ny01+m] = 1e-3*w; cost[ny12:ny12+m] = 1e-3*w
    cost[ng01:ng01+G] = 1e-3*gw; cost[ng12:ng12+G] = 1e-3*gw
    integrality = np.zeros(nv); integrality[:2*n] = 1
    lb = np.zeros(nv); ub = np.ones(nv); ub[nz] = np.inf
    res = milp(cost, constraints=[LinearConstraint(A, lo, hi)], integrality=integrality, bounds=Bounds(lb, ub),
               options={'time_limit': args.time_limit, 'disp': False, 'mip_rel_gap': 1e-4})
    return res, (na, nc)


def evaluate(assign, units, idx, R, port, edges, groups):
    cut = {'01': FIXED['01'], '12': FIXED['12']}
    for u in units:
        s = assign[u]
        if s >= 1: cut['01'] += port[u]
        if s == 2: cut['12'] += port[u]
    for (u, v, w, _) in edges:
        su, sv = assign[units[u]], assign[units[v]]
        if (su == 0) != (sv == 0): cut['01'] += w
        if (su == 2) != (sv == 2): cut['12'] += w
    for (pidx, members, sh, _) in groups:
        sp = assign[units[pidx]]
        if any((assign[units[v]] == 0) != (sp == 0) for v in members): cut['01'] += sh
        if any((assign[units[v]] == 2) != (sp == 2) for v in members): cut['12'] += sh
    use = {k: [PLAT[k][s] for s in range(3)] for k in PLAT}
    for u in units:
        for k in PLAT: use[k][assign[u]] += R[u][k]
    util = {k: [round(use[k][s]/TOTAL[k][s]*100, 1) for s in range(3)] for k in PLAT}
    return cut, util


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--util-lut', type=float, default=0.70)
    ap.add_argument('--lut-per-slr', required=True, help='SLR0,SLR1,SLR2 model LUT caps, e.g. 0.50,0.64,0.60')
    ap.add_argument('--util-bram', type=float, default=0.85)
    ap.add_argument('--time-limit', type=float, default=1800)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    args.lut_per_slr = [float(x) for x in args.lut_per_slr.split(',')]; assert len(args.lut_per_slr) == 3
    args.util = {'LUT': args.util_lut, 'BRAM': args.util_bram, 'URAM': 0.90, 'DSP': 0.90}
    units, idx, R, port, edges, plan, groups = build(args)
    ref = json.loads((V641/'ILP_all4_lut0.64.json').read_text())['assignment']  # 32-PE choice, reference only
    cur = {u: int(ref[u][-1]) if u in ref else 1 for u in units}
    cur['control_s_axi_U'] = 0
    cut0, util0 = evaluate(cur, units, idx, R, port, edges, groups)
    print('CURRENT_T1 model cut', cut0, 'util%', util0, flush=True)
    res, (na, nc) = solve(units, idx, R, port, edges, groups, args)
    print('MILP status', res.status, res.message, flush=True)
    if res.x is None: raise SystemExit('NO_SOLUTION')
    x = res.x
    assign = {u: (0 if x[na+idx[u]] > 0.5 else (2 if x[nc+idx[u]] > 0.5 else 1)) for u in units}
    cut, util = evaluate(assign, units, idx, R, port, edges, groups)
    moved = sorted(u for u in units if u in cur and cur[u] != assign[u])
    out = {'kind': 'ILP_task_SLR_plan_with_gearbox4_NOT_native', 'gearbox_families': sorted(GEARBOX_FAMILIES), 'model_fixed_platform_SLL': FIXED, 'caps_util': args.util,
           'current_T1_model_cut': cut0, 'current_T1_util_pct': util0,
           'ilp_cut': cut, 'ilp_util_pct': util, 'mip_gap': getattr(res, 'mip_gap', None),
           'objective_bound': getattr(res, 'mip_dual_bound', None), 'status': res.message,
           'assignment': {u: SLRS[s] for u, s in assign.items()}, 'moved_vs_T1': {u: [SLRS[cur[u]], SLRS[assign[u]]] for u in moved},
           'accepted': False}
    with open(args.out, 'x') as f: json.dump(out, f, indent=1, sort_keys=True)
    print('ILP cut', cut, 'util%', util, 'moved', len(moved), 'gap', out['mip_gap'], 'bound', out['objective_bound'], flush=True)


if __name__ == '__main__':
    main()
