#!/usr/bin/env python3
"""Require fresh complete software and exact-XO RTL BEFORE physical builds.

This is a prerequisite gate, never a routed/bitstream acceptance claim.
"""
import argparse
import json
from pathlib import Path
from salut_banked_provenance import verify_provenance,TASK_MODULES,sha256
from verify_banked_artifacts import checked_file,_checked_json,_software,_rtl


def verify_functional_manifest(manifest,base_dir):
    if not isinstance(manifest,dict) or manifest.get('schema_version')!=1:
        raise RuntimeError('BANKED_FUNCTIONAL_SCHEMA_INVALID')
    if manifest.get('kind')!='full':
        raise RuntimeError('DIAGNOSTIC_ARTIFACT_NOT_DEPLOYABLE')
    provenance=_checked_json(manifest.get('provenance'),base_dir,'functional-provenance')
    if not isinstance(manifest.get('hls_root'),str): raise RuntimeError('HLS_ROOT_MISSING')
    hls=(base_dir/Path(manifest['hls_root'])).resolve()
    xo=checked_file(manifest.get('xo'),base_dir,'functional-XO')
    verify_provenance(provenance,hls,xo,expected_kind='full',require_modules=TASK_MODULES)
    if provenance['clock_mhz']<200: raise RuntimeError('HLS_CLOCK_BELOW_200MHZ')
    identities=_software(manifest.get('software'),base_dir,provenance['sources'])
    _rtl(manifest.get('rtl'),base_dir,provenance,hls,xo,identities)
    return provenance


def reference(path):
    path=Path(path).resolve()
    return {'path':str(path),'sha256':sha256(path)}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--create',action='store_true')
    parser.add_argument('--provenance',type=Path)
    parser.add_argument('--hls-root',type=Path)
    parser.add_argument('--software-dir',type=Path)
    parser.add_argument('--host',type=Path)
    parser.add_argument('--matrix',type=Path)
    parser.add_argument('--rtl-result',type=Path,action='append')
    parser.add_argument('--expect-xo',type=Path)
    parser.add_argument('--expect-hls-root',type=Path)
    parser.add_argument('--expect-clock-mhz',type=float)
    args=parser.parse_args();output=args.manifest.resolve()
    if args.create:
        if not all((args.provenance,args.hls_root,args.software_dir,args.host,args.matrix,args.rtl_result)):
            raise RuntimeError('FUNCTIONAL_CAPTURE_EXPLICIT_INPUTS_REQUIRED')
        provenance=json.loads(args.provenance.read_text())
        manifest={'schema_version':1,'kind':'full','stage':'functional-prerequisite',
            'provenance':reference(args.provenance),'hls_root':str(args.hls_root.resolve()),
            'xo':provenance['xo'],'software':{
                'log':reference(args.software_dir/'memory_full.log'),
                'inputs':reference(args.software_dir/'inputs.sha256'),
                'host':reference(args.host),'matrix':reference(args.matrix)},
            'rtl':[reference(path) for path in args.rtl_result]}
        verify_functional_manifest(manifest,output.parent)
        # Never adopt a former partial result or overwrite a successful record.
        with output.open('x') as stream:
            stream.write(json.dumps(manifest,sort_keys=True,indent=2)+'\n')
    else:
        if not output.is_file():
            raise RuntimeError('FUNCTIONAL_PREREQUISITES_REQUIRED '+str(output))
        manifest=json.loads(output.read_text())
        provenance=verify_functional_manifest(manifest,output.parent)
        if args.expect_xo and checked_file(manifest['xo'],output.parent,'functional-XO')!=args.expect_xo.resolve():
            raise RuntimeError('FUNCTIONAL_SELECTED_XO_MISMATCH')
        if args.expect_hls_root and (output.parent/Path(manifest['hls_root'])).resolve()!=args.expect_hls_root.resolve():
            raise RuntimeError('FUNCTIONAL_SELECTED_HLS_MISMATCH')
        if args.expect_clock_mhz and abs(provenance['clock_mhz']-args.expect_clock_mhz)>0.01:
            raise RuntimeError('FUNCTIONAL_SELECTED_CLOCK_MISMATCH')
    print('BANKED_FUNCTIONAL_PREREQUISITES_PASS '+str(output))


if __name__=='__main__': main()
