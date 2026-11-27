#!/usr/bin/env python3
"""Read-only routed checkpoint preflight before SDK packaging, not deployment."""
import argparse
import json
import math
from pathlib import Path

from salut_banked_provenance import FULL_INSTANCES, sha256
from verify_frequency_artifact import verify_routed_audit
from verify_packaged_memory import EXPECTED_HBM
from verify_banked_task_resources import verify_resource_census
from finalize_route_audit import SUPPORT_REPORTS
from verify_feedback_placement import verify_feedback_placement
from verify_close_fifo_placement import verify_close_fifo_placement
from verify_vivado_bus_skew import verify_bus_skew
from banked_floorplan_owners import build_owners
from verify_axi_control_placement import verify_axi_control_placement, verify_axi_control_constraints


def verify_checkpoint(audit, architecture, dcp, minimum_mhz, expected_graph=None):
    if isinstance(minimum_mhz, bool) or not isinstance(minimum_mhz, (int, float)) or \
            not math.isfinite(minimum_mhz) or minimum_mhz < 200:
        raise RuntimeError('ROUTED_CHECKPOINT_MINIMUM_CLOCK_INVALID')
    dcp = Path(dcp)
    if not dcp.is_file() or not dcp.stat().st_size:
        raise RuntimeError('ROUTED_CHECKPOINT_MISSING')
    digest = sha256(dcp)
    if not isinstance(audit, dict) or audit.get('origin') != 'vivado' or \
            audit.get('route_complete') is not True or audit.get('dcp_sha256') != digest:
        raise RuntimeError('ROUTED_CHECKPOINT_AUDIT_IDENTITY_MISMATCH')
    if not isinstance(architecture, dict) or architecture.get('origin') != 'vivado' or \
            architecture.get('dcp_sha256') != digest:
        raise RuntimeError('ROUTED_CHECKPOINT_ARCHITECTURE_IDENTITY_MISMATCH')
    instances = architecture.get('instances')
    if not isinstance(instances, dict) or any(type(instances.get(name)) is not int or
            instances[name] != count for name, count in FULL_INSTANCES.items()):
        raise RuntimeError('ROUTED_CHECKPOINT_FULL_ARCHITECTURE_MISSING')
    interfaces = architecture.get('hbm_interfaces')
    if not isinstance(interfaces, list) or len(interfaces) != 27 or \
            set(interfaces) != set(EXPECTED_HBM):
        raise RuntimeError('ROUTED_CHECKPOINT_27_HBM_MISSING')
    pins = audit.get('report_inputs')
    required = SUPPORT_REPORTS | {'salut_route_queries.json',
                'salut_timing_routed.rpt', 'salut_check_timing.rpt'}
    if not isinstance(pins, dict) or not required.issubset({Path(p).name for p in pins}):
        raise RuntimeError('ROUTED_CHECKPOINT_REPORT_INPUTS_MISSING')
    for name, expected in pins.items():
        path = Path(name)
        if not path.is_absolute() or path.parent != dcp.resolve().parent or \
                not path.is_file() or not path.stat().st_size or sha256(path) != expected:
            raise RuntimeError('ROUTED_CHECKPOINT_REPORT_CHANGED ' + name)
    if any(sum(Path(path).name == name for path in pins) != 1 for name in required):
        raise RuntimeError('ROUTED_CHECKPOINT_REPORT_SCOPE_AMBIGUOUS')
    graph_path = dcp.resolve().parent / 'salut_axi_control_graph.json'
    if architecture.get('axi_control_graph_sha256') != sha256(graph_path) or \
            expected_graph is not None and sha256(expected_graph) != sha256(graph_path):
        raise RuntimeError('ROUTED_AXI_GRAPH_IDENTITY_MISMATCH')
    owners = build_owners(json.loads(graph_path.read_text()))
    verify_close_fifo_placement(
        (dcp.resolve().parent / 'salut_close_fifo_routed.rpt').read_text(), owners, 'routed', digest)
    verify_axi_control_constraints(
        (dcp.resolve().parent / 'salut_axi_control_constraints.rpt').read_text(), owners)
    verify_axi_control_placement(
        (dcp.resolve().parent / 'salut_axi_control_routed.rpt').read_text(), owners, 'routed', digest)
    census = architecture.get('resource_census')
    verify_resource_census(census, digest)
    resource_paths = [Path(name) for name in pins if Path(name).name == 'salut_task_resources.json']
    if len(resource_paths) != 1 or json.loads(resource_paths[0].read_text()) != census:
        raise RuntimeError('ROUTED_RESOURCE_CENSUS_REPORT_MISMATCH')
    verify_feedback_placement(
        (dcp.resolve().parent / 'salut_fifo_feedback_constraints.rpt').read_text(),
        (dcp.resolve().parent / 'salut_feedback_routed.rpt').read_text(), digest)
    verify_bus_skew((dcp.resolve().parent / 'salut_bus_skew.rpt').read_text(),
                   (dcp.resolve().parent / 'salut_all_timing.xdc').read_text(),
                   report_name='salut_bus_skew.rpt', dcp_sha256=digest,
                   scope_inventory=json.loads((dcp.resolve().parent /
                       'salut_bus_skew_scope.json').read_text()))
    verify_routed_audit(audit, minimum_mhz)
    return digest


def main():
    parser = argparse.ArgumentParser()
    for name in ('audit', 'architecture', 'dcp'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--minimum-mhz', type=float, required=True)
    parser.add_argument('--expected-graph', type=Path)
    args = parser.parse_args()
    digest = verify_checkpoint(json.loads(args.audit.read_text()),
        json.loads(args.architecture.read_text()), args.dcp, args.minimum_mhz, args.expected_graph)
    print('ROUTED_CHECKPOINT_PREFLIGHT_PASS ' + digest + '; SDK packaging still required')


if __name__ == '__main__':
    main()
