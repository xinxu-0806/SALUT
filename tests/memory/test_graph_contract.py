"""Structural tests of the PRODUCT top, not the diagnostic memory top."""
from pathlib import Path
import re
import unittest
from graph_ownership import CONSTANTS, owners

ROOT = Path(__file__).resolve().parents[2]


class ProductGraph(unittest.TestCase):
    def setUp(self):
        source = (ROOT / 'src/lu_kernel.cpp').read_text()
        self.top = source[source.rindex('void SparseLUKernel('):]

    def test_full_math_and_external_abi(self):
        signature = self.top.split(') {', 1)[0]
        self.assertEqual(signature.count('tapa::mmap<'), 27)
        self.assertRegex(self.top, r'invoke<tapa::detach, PAR_NUM_PE>\(ParallelSegmentPE')
        self.assertRegex(self.top, r'invoke<tapa::detach, PAR_NUM_FRONT>\(ParallelColumnFront')
        for task in ['SalutLMLane', 'SalutSmBank', 'SalutPegWriteFork',
                     'SalutFrontReadSource', 'SalutFrontWriteSource', 'SalutResponseEgress']:
            self.assertRegex(self.top, rf'invoke<tapa::detach, 8>\({task}\b')
        self.assertEqual(self.top.count('>(SalutHBMWordPort,'), 16)
        self.assertEqual(self.top.count('>(SalutHBMReadWordPort,'), 8)
        self.assertNotRegex(self.top, r'invoke[^\n]*\((?:SM|ParallelLMLane|ParallelSMHBMLane)\b')

    def test_three_omegas_have_exact_edges(self):
        for prefix, task, packet in [('rd', 'SalutReadSwitch2', 'SalutReadEnvelope'),
                                     ('wr', 'SalutWriteSwitch2', 'SalutWriteBeat'),
                                     ('rsp', 'SalutResponseSwitch2', 'SalutResponseEnvelope')]:
            calls = re.findall(rf'\({task},\s*([012]),\s*([^)]*)\)', self.top)
            self.assertEqual(len(calls), 12, task)
            for stage in range(3):
                seen = [args for s, args in calls if int(s) == stage]
                self.assertEqual(len(seen), 4)
                for switch, args in enumerate(seen):
                    incoming = [f'{prefix}_omega_{stage}[{2*switch+i}]' for i in range(2)]
                    # The shuffle is independently specified, not copied from the graph.
                    outgoing = [f'{prefix}_omega_{stage+1}[{((j<<1)&7)|(j>>2) if stage<2 else j}]'
                                for j in [2*switch, 2*switch+1]]
                    self.assertEqual([x.strip() for x in args.split(',')], incoming+outgoing)
            for stage in range(4):
                self.assertIn(f'tapa::streams<{packet}, 8, 4> {prefix}_omega_{stage}', self.top)

    def test_completion_owned_by_physical_drain(self):
        self.assertRegex(self.top, r'invoke<tapa::detach>\(ParallelScheduler')
        self.assertRegex(self.top, r'invoke<tapa::join>\(SalutMemoryDrainJoin')
        self.assertEqual(self.top.count('>(SalutCompletionFanout,'), 1)
        self.assertIn('bank_drained, response_ended, publish_drained', self.top)
        source = (ROOT / 'src/lu_kernel.cpp').read_text()
        scheduler = source[source.index('void ParallelScheduler('):source.index('// SALUT Task Allocation Unit')]
        self.assertIn('tapa::ostream<ap_uint<1>>& compute_done_out', scheduler)
        scheduler = re.sub(r'//[^\n]*', '', scheduler)
        self.assertRegex(scheduler, r'ready_segment_id_out.write\(0xFFFFFFFFu\);\s*compute_done_out.write\(ap_uint<1>\(1\)\);')

    def test_every_fifo_has_one_producer_and_one_consumer(self):
        edges = owners()
        self.assertGreater(len(edges), 300)
        self.assertEqual(edges['factor_word_rd[3]']['writer'], 'SalutSmBank_1')
        self.assertEqual(edges['factor_word_rd[3]']['reader'], 'SalutHBMWordPort_3')
        last_pe = CONSTANTS['PAR_NUM_PE'] - 1
        last_peg = CONSTANTS['PAR_NUM_PEG'] - 1
        self.assertEqual(edges[f'pe_rd_req[{last_pe}]']['reader'], f'SalutLMLane_{last_peg}')
        self.assertEqual(edges[f'pe_wr_req[{last_pe}]']['reader'], f'SalutPegWriteFork_{last_peg}')
        # PEG-major ownership: PE p belongs to LM lane / write fork p // PAR_PE_PER_PEG.
        for pe in range(CONSTANTS['PAR_NUM_PE']):
            peg = pe // CONSTANTS['PAR_PE_PER_PEG']
            self.assertEqual(edges[f'pe_rd_req[{pe}]']['reader'], f'SalutLMLane_{peg}')
            self.assertEqual(edges[f'pe_wr_req[{pe}]']['reader'], f'SalutPegWriteFork_{peg}')
        self.assertNotIn(f'pe_rd_req[{last_pe + 1}]', edges)


if __name__ == '__main__':
    unittest.main()
