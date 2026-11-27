"""Check real TAPA-analyzed diagnostic graph wiring (not RTL simulation).

CLI script: ``python3 test_diagnostic_graph.py <graph.json>``. The checks run
only under ``__main__`` so unittest discovery can import this module safely.
"""
import json
import sys
from pathlib import Path


def main(path):
    graph=json.loads(Path(path).read_text())
    top=graph['tasks'][graph['top']]
    assert all(port['cat']=='mmap' or port['cat']=='scalar' for port in top['ports'])
    for name,fifo in top['fifos'].items():
        assert 'depth' in fifo and fifo['depth']>0, 'diagnostic FIFO incorrectly external: '+name
        assert 'produced_by' in fifo and 'consumed_by' in fifo, 'unowned diagnostic FIFO: '+name
    print('DIAGNOSTIC_GRAPH_WIRING_PASS '+graph['top'])


if __name__ == '__main__':
    main(sys.argv[1])
