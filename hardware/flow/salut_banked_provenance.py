#!/usr/bin/env python3
"""Bind new source inputs, task graph, HLS RTL and final patched XO together.

This is a build/RTL freshness gate, NOT routed timing or bitstream proof.
"""
from pathlib import Path
import argparse
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
FULL_INSTANCES = {
    'ParallelSegmentPE':16, 'ParallelColumnFront':8,  # 8 PEG x 2 PE (2026-10-22 user-approved 16-PE fallback)
    'SalutLMLane':8, 'SalutSmBank':8, 'SalutHBMWordPort':16,
    'SalutHBMReadWordPort':8, 'SalutReadSwitch2':12,
    'SalutWriteSwitch2':12, 'SalutResponseSwitch2':12,
    'SalutPublishJoin':1, 'SalutMemoryDrainJoin':1,
    'ParallelScheduler':1, 'ParallelDegreeStore':1, 'ParallelPegAckDemux':1,
    'ParallelPivotReadGate':1,
}
MEMORY_MODULES = ('SalutLMLane','SalutSmBank','SalutHBMWordPort',
                  'SalutHBMReadWordPort','SalutReadSwitch2','SalutWriteSwitch2',
                  'SalutResponseSwitch2','SalutPublishJoin','SalutMemoryDrainJoin')
RETAINED_MODULES=('ParallelScheduler','ParallelDegreeStore','ParallelPegAckDemux','ParallelPivotReadGate')
WRITE_ADAPTER_MODULES=('SalutPegWriteFork','SalutWriteLaneMerge')
TASK_MODULES=MEMORY_MODULES+RETAINED_MODULES+WRITE_ADAPTER_MODULES
DIAGNOSTIC_TOPS=('SalutMemoryValidationTop','SalutRetainedValidationTop',*TASK_MODULES)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024*1024),b''): digest.update(block)
    return digest.hexdigest()


def source_fingerprint(root=ROOT,kind='full'):
    root = Path(root)
    names = ['Makefile','src/lu_kernel.cpp','src/lu_kernel.h','src/black_hole_utils.h',
             'host/main.cpp','host/salut_layout_validation.h']
    names += sorted(str(p.relative_to(root)) for pattern in ('salut_*.h','salut_*.inc')
                    for p in (root/'src').glob(pattern))
    if kind == 'diagnostic': names.append('tests/memory/hls_top.cpp')
    for name in names:
        if not (root/name).is_file(): raise RuntimeError(f'SOURCE_INPUT_MISSING {name}')
    return {name:sha256(root/name) for name in names}


def capture_provenance(root=ROOT,kind='full',clock_mhz=200,diagnostic_top=None):
    if kind not in ('full','diagnostic'): raise RuntimeError('invalid artifact kind')
    if diagnostic_top is not None and (kind!='diagnostic' or
            diagnostic_top not in DIAGNOSTIC_TOPS):
        raise RuntimeError('WRONG_ARTIFACT_TOP '+str(diagnostic_top))
    return {'schema_version':1, 'kind':kind, 'phase':'captured',
            'top':'SparseLUKernel' if kind == 'full' else diagnostic_top or 'SalutMemoryValidationTop',
            'clock_mhz':clock_mhz, 'sources':source_fingerprint(root,kind)}


def verify_sources(manifest,root=ROOT,expected_kind=None):
    if expected_kind == 'full' and manifest.get('kind') != 'full':
        raise RuntimeError('DIAGNOSTIC_ARTIFACT_NOT_DEPLOYABLE')
    if manifest.get('schema_version') != 1 or manifest.get('kind') not in ('full','diagnostic'):
        raise RuntimeError('STALE_RTL_PROVENANCE invalid schema/kind')
    allowed=('SparseLUKernel',) if manifest['kind']=='full' else DIAGNOSTIC_TOPS
    if manifest.get('top') not in allowed: raise RuntimeError('WRONG_ARTIFACT_TOP '+str(manifest.get('top')))
    if manifest.get('sources') != source_fingerprint(root,manifest['kind']):
        raise RuntimeError('STALE_RTL_PROVENANCE compilation source/header changed')


def verify_graph(graph,kind,diagnostic_top=None):
    expected_top = 'SparseLUKernel' if kind == 'full' else diagnostic_top or 'SalutMemoryValidationTop'
    if graph.get('top') != expected_top: raise RuntimeError('WRONG_ARTIFACT_TOP '+str(graph.get('top')))
    top = graph['tasks'][expected_top]
    if kind == 'full':
        for task,count in FULL_INSTANCES.items():
            if len(top['tasks'].get(task,[])) != count:
                raise RuntimeError(f'FULL_ARCHITECTURE_MISSING {task} expected={count}')
        expected = {f'{kind}_{i}' for kind in ('A','L','U') for i in range(8)}
        expected |= {f'META_{name}_mmap' for name in ('SCHED','FRONT','SEGMENT')}
        ports = [p['name'] for p in top['ports'] if p['cat'] == 'mmap']
        if len(ports) != 27 or set(ports) != expected:
            raise RuntimeError('FULL_ABI_MISSING exact 27 HBM interfaces')
        if any(t in top['tasks'] for t in ('SM','ParallelLMLane','ParallelSMHBMLane')):
            raise RuntimeError('LEGACY_MEMORY_GRAPH_NOT_DEPLOYABLE')


def seal_provenance(manifest,hls_root,xo,graph_path,root=ROOT):
    verify_sources(manifest,root)
    graph_path, hls_root, xo = Path(graph_path),Path(hls_root),Path(xo)
    verify_graph(json.loads(graph_path.read_text()),manifest['kind'],manifest.get('top'))
    files = {str(p.relative_to(hls_root)):sha256(p) for pattern in ('*.v','*.dat')
             for p in sorted(hls_root.rglob(pattern))}
    modules = {}
    for module in TASK_MODULES if manifest['kind']=='full' else MEMORY_MODULES:
        candidates = [p for p in files if Path(p).name == module+'.v' and '/syn/verilog/' in '/'+p]
        if len(candidates) != 1: raise RuntimeError('RTL_MODULE_MISSING '+module)
        modules[module] = candidates[0]
    sealed = dict(manifest,phase='sealed',rtl_files=files,modules=modules,
                  graph={'path':str(graph_path.resolve()),'sha256':sha256(graph_path)},
                  xo={'path':str(xo.resolve()),'sha256':sha256(xo)})
    return sealed


def seal_task_provenance(manifest,hls_root,modules,root=ROOT):
    """Pin finished leaf RTL while unrelated HLS workers continue.

    This does not seal an XO, a full architecture or a deployment artifact.
    Full verify_provenance deliberately rejects phase=task-sealed.
    """
    verify_sources(manifest,root)
    hls_root=Path(hls_root)
    modules=tuple(sorted(set(modules)))
    if not modules or any(name not in TASK_MODULES for name in modules):
        raise RuntimeError('INVALID_TASK_RTL_SCOPE')
    files,paths,cpp={}, {}, {}
    for module in modules:
        directory=hls_root/module/'project'/module/'syn/verilog'
        main=directory/(module+'.v')
        generated=hls_root.parent/'cpp'/(module+'.cpp')
        if not main.is_file(): raise RuntimeError('RTL_MODULE_MISSING '+module)
        if not generated.is_file(): raise RuntimeError('GENERATED_CPP_MISSING '+module)
        paths[module]=str(main.relative_to(hls_root))
        cpp[module]={'path':str(generated.resolve()),'sha256':sha256(generated)}
        for pattern in ('*.v','*.dat'):
            files.update({str(p.relative_to(hls_root)):sha256(p) for p in directory.glob(pattern)})
    return dict(manifest,phase='task-sealed',task_scope=list(modules),
                rtl_files=files,modules=paths,generated_cpp=cpp)


def verify_task_provenance(manifest,hls_root,root=ROOT,require_modules=()):
    verify_sources(manifest,root)
    if manifest.get('phase') != 'task-sealed': raise RuntimeError('UNSEALED_TASK_RTL_PROVENANCE')
    modules=manifest.get('task_scope',[])
    if not modules or any(name not in TASK_MODULES for name in modules):
        raise RuntimeError('INVALID_TASK_RTL_SCOPE')
    hls_root=Path(hls_root)
    for module in require_modules:
        expected=hls_root/module/'project'/module/'syn/verilog'/(module+'.v')
        if (module not in modules or not expected.is_file() or
                manifest.get('modules',{}).get(module) != str(expected.relative_to(hls_root))):
            raise RuntimeError('RTL_MODULE_MISSING '+module)
    actual_files={str(p.relative_to(hls_root)) for module in modules
        for pattern in ('*.v','*.dat')
        for p in (hls_root/module/'project'/module/'syn/verilog').glob(pattern)}
    if actual_files != set(manifest.get('rtl_files',{})):
        raise RuntimeError('STALE_RTL_PROVENANCE task RTL inventory changed')
    for name,digest in manifest['rtl_files'].items():
        if sha256(hls_root/name) != digest:
            raise RuntimeError('STALE_RTL_PROVENANCE task RTL bytes changed '+name)
    for module in modules:
        identity=manifest.get('generated_cpp',{}).get(module,{})
        expected=hls_root.parent/'cpp'/(module+'.cpp')
        if (identity.get('path') != str(expected.resolve()) or not expected.is_file() or
                sha256(expected) != identity.get('sha256')):
            raise RuntimeError('STALE_RTL_PROVENANCE generated CPP changed/missing '+module)


def verify_provenance(manifest,hls_root,xo=None,root=ROOT,expected_kind=None,require_modules=()):
    verify_sources(manifest,root,expected_kind)
    if manifest.get('phase') != 'sealed': raise RuntimeError('UNSEALED_RTL_PROVENANCE')
    for module in require_modules:
        path = manifest.get('modules',{}).get(module)
        candidates = [p for p in manifest.get('rtl_files',{})
                      if Path(p).name == module+'.v' and '/syn/verilog/' in '/'+p]
        if (not path or candidates != [path] or not (Path(hls_root)/path).is_file()):
            raise RuntimeError('RTL_MODULE_MISSING '+module)
    # xvlog consumes all .v files and RAM models consume .dat images. Reject
    # additions as well as altered/deleted files: otherwise an old RTL file
    # could silently enter a supposedly hash-pinned elaboration.
    actual_files = {str(p.relative_to(hls_root)) for pattern in ('*.v','*.dat')
                    for p in Path(hls_root).rglob(pattern)}
    if actual_files != set(manifest.get('rtl_files',{})):
        raise RuntimeError('STALE_RTL_PROVENANCE RTL inventory changed')
    for name,digest in manifest.get('rtl_files',{}).items():
        path = Path(hls_root)/name
        if not path.is_file() or sha256(path) != digest:
            raise RuntimeError('STALE_RTL_PROVENANCE RTL bytes changed/missing '+name)
    if not manifest.get('rtl_files'): raise RuntimeError('RTL_MODULE_MISSING no RTL')
    graph = manifest.get('graph',{})
    if not graph.get('path') or not Path(graph['path']).is_file() or sha256(graph['path']) != graph.get('sha256'):
        raise RuntimeError('STALE_RTL_PROVENANCE task graph changed/missing')
    verify_graph(json.loads(Path(graph['path']).read_text()),manifest['kind'],manifest.get('top'))
    if xo is not None:
        identity = manifest.get('xo',{})
        if (not Path(xo).is_file() or Path(identity.get('path','')).resolve() != Path(xo).resolve()
                or sha256(xo) != identity.get('sha256')):
            raise RuntimeError('STALE_RTL_PROVENANCE XO changed/missing')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action',choices=('capture','seal','verify'))
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--kind',choices=('full','diagnostic'),default='full')
    parser.add_argument('--clock-mhz',type=float,default=200)
    parser.add_argument('--diagnostic-top',choices=DIAGNOSTIC_TOPS)
    parser.add_argument('--hls-root',type=Path)
    parser.add_argument('--xo',type=Path)
    parser.add_argument('--graph',type=Path)
    parser.add_argument('--require-module',action='append',default=[])
    args = parser.parse_args()
    if args.action == 'capture':
        if args.manifest.exists(): raise RuntimeError('PROVENANCE_EXISTS choose a fresh build directory')
        value = capture_provenance(kind=args.kind,clock_mhz=args.clock_mhz,diagnostic_top=args.diagnostic_top)
    else:
        value = json.loads(args.manifest.read_text())
        if args.hls_root is None: raise RuntimeError('--hls-root required')
        if args.action == 'seal':
            if args.xo is None or args.graph is None: raise RuntimeError('--xo and --graph required')
            value = seal_provenance(value,args.hls_root,args.xo,args.graph)
        else:
            verify_provenance(value,args.hls_root,args.xo,expected_kind=args.kind,
                              require_modules=args.require_module)
            print('BANKED_PROVENANCE_PASS '+args.kind)
            return
    args.manifest.parent.mkdir(parents=True,exist_ok=True)
    args.manifest.write_text(json.dumps(value,sort_keys=True,indent=2)+'\n')


if __name__ == '__main__':
    main()
