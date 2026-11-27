#!/usr/bin/env python3
"""v722: normal SDK packaging (v++ -l --reuse_impl) of the v720a routed DCP (150 MHz, all clocks met,
pr_verify vs F5 = 0 in v721), via the existing package_preserving_project (backs up and restores the SDK project)."""
import json, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]; sys.path.insert(0, str(ROOT / 'scripts'))
from postopt_capture_context import verify_context, digest
from run_postopt_link import package_preserving_project
HERE = Path(__file__).resolve().parent
CACHE = Path(sys.argv[1]).resolve(); ROUTED = HERE / 'routed_v720a.dcp'
EXPECT_ROUTED = '50b687d28c61918e70b9bb2036714d892738655ddb50a2a61335959aca467016'
context = json.loads((CACHE / 'capture_context.json').read_text())
xo = Path(context['xo']['path'])
verify_context(context, CACHE, xo, CACHE.name, 150.0)
if digest(ROUTED) != EXPECT_ROUTED: raise RuntimeError('ROUTED_SHA')
platform = Path('/opt/xilinx/platforms/xilinx_u280_gen3x16_xdma_1_202211_1/xilinx_u280_gen3x16_xdma_1_202211_1.xpfm')
run = HERE / 'run'; run.mkdir(); fpga = HERE / 'out'; fpga.mkdir()
rec = {'start': time.time(), 'routed_sha256': EXPECT_ROUTED, 'xo_sha256': context['xo']['sha256'], 'clock_mhz': 150, 'accepted': False}
(HERE / 'START.json').write_text(json.dumps(rec, indent=2))
package_preserving_project(context, run, ROUTED, platform, ROOT / 'hbm_config.ini', 150, fpga)
rec['end'] = time.time(); rec['xclbin_sha256'] = digest(fpga / 'SparseLUKernel_xilinx_u280_gen3x16_xdma_1_202211_1.xclbin')
(HERE / 'DONE.json').write_text(json.dumps(rec, indent=2)); print('PACKAGE_DONE', rec['xclbin_sha256'])
