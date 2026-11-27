#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_fabric.inc"
#include <cassert>

// Breaks caught: terminating a merge/split with only one side drained,
// misrouting front vs PEG, fabricating zero-length data, stale read ticket.
static MemoryTrace Adapters() {
  SalutFrontReadControl source{};
  SalutFrontReadInputs source_in{}; SalutFrontReadOutputs source_out{};
  source_in.request_valid = true; source_in.request = {{2, pack_addr(23, 0), 8}, SALUT_FRONT_CLIENT0 + 7};  // front 7
  SalutFrontReadTick(source, source_in, source_out, 7); assert(source_out.consume);
  source_in.request_valid = false; source_in.close = true;
  SalutFrontReadTick(source, source_in, source_out, 7);
  assert(source_out.valid && !source_out.data.end && source_out.data.req.pe == SALUT_FRONT_CLIENT0 + 7);
  source_in.close = false; source_in.ready = true;
  SalutFrontReadTick(source, source_in, source_out, 7);
  assert(source_out.valid && !source_out.data.end);
  SalutFrontReadTick(source, source_in, source_out, 7);
  assert(source_out.valid && source_out.data.end);
  SalutFrontReadTick(source, source_in, source_out, 7); assert(!source_out.valid);
  SalutMergeState<SalutReadEnvelope> merge{};
  SalutSwitchInputs<SalutReadEnvelope> input{};
  SalutSwitchOutputs<SalutReadEnvelope> output{};
  input.input_valid[0] = true; input.input_data[0].end = true;
  SalutMergeTick(merge, input, output);
  assert(output.input_consume[0]); input.input_valid[0] = false;
  for (int cycle = 0; cycle < 4; ++cycle) {
    SalutMergeTick(merge, input, output); assert(!output.output_valid[0]);
  }
  input.input_valid[1] = true;
  input.input_data[1] = {{{2, pack_addr(16, 0), 8}, SALUT_FRONT_CLIENT0}, false};
  SalutMergeTick(merge, input, output); assert(output.input_consume[1]);
  input.input_data[1].end = true;
  SalutMergeTick(merge, input, output);
  assert(output.output_valid[0] && !output.output_data[0].end);
  input.output_ready[0] = true;
  SalutMergeTick(merge, input, output);
  assert(!output.output_data[0].end);
  if (!output.input_consume[1]) SalutMergeTick(merge, input, output);
  assert(output.input_consume[1]); input.input_valid[1] = false;
  SalutMergeTick(merge, input, output);
  assert(output.output_valid[0] && output.output_data[0].end);
  SalutMergeTick(merge, input, output); assert(!output.output_valid[0]);

  SalutBankSplitState split{};
  SalutBankSplitInputs si{}; SalutBankSplitOutputs so{};
  si.valid = true; si.data = {{{0, pack_addr(3, 0), 8}, SALUT_MAX_CLIENT}, false};
  SalutBankSplitTick(split, si, so, 3); assert(so.consume);
  si.valid = false; si.ready[0] = true;
  SalutBankSplitTick(split, si, so, 3);
  assert(so.valid[1] && !so.valid[0]);
  si.ready[1] = true; SalutBankSplitTick(split, si, so, 3);
  si.valid = true; si.data.end = true;
  SalutBankSplitTick(split, si, so, 3); assert(so.consume);
  si.valid = false; si.ready[0] = false;
  SalutBankSplitTick(split, si, so, 3);
  assert(so.valid[0] && so.valid[1] && so.data.end);
  SalutBankSplitTick(split, si, so, 3);
  assert(so.valid[0] && !so.valid[1]);
  si.ready[0] = true; SalutBankSplitTick(split, si, so, 3);
  SalutBankSplitTick(split, si, so, 3); assert(!so.valid[0] && !so.valid[1]);

  SalutABurstControl burst{};
  SalutABurstInputs bi{}; SalutABurstOutputs bo{};
  bi.request_valid = true; bi.request = {{{0, pack_addr(3, 3), 10}, SALUT_MAX_CLIENT}, false};
  bi.read_ready = bi.response_ready = true;
  SalutABurstTick(burst, bi, bo, 3); assert(bo.request_consume);
  bi.request_valid = false;
  SalutABurstTick(burst, bi, bo, 3);
  assert(bo.read_valid && bo.read.address == pack_addr(3, 0));
  const uint32_t ticket0 = bo.read.ticket;
  bi.fill_valid = true; bi.fill = {WORD512(0xaa), ticket0, false};
  SalutABurstTick(burst, bi, bo, 3); assert(bo.fill_consume);
  bi.fill_valid = false;
  SalutABurstTick(burst, bi, bo, 3);
  assert(bo.response_valid && bo.response.rsp.first == 3 && bo.response.rsp.count == 5);
  assert(bo.response.rsp.v == WORD512(0xaa) && bo.response.rsp.pe == SALUT_MAX_CLIENT);
  if (!bo.read_valid) SalutABurstTick(burst, bi, bo, 3);
  assert(bo.read_valid && bo.read.address == pack_addr(3, 8));
  bi.fill_valid = true; bi.fill = {WORD512(0xbb), bo.read.ticket, false};
  SalutABurstTick(burst, bi, bo, 3); bi.fill_valid = false;
  SalutABurstTick(burst, bi, bo, 3);
  assert(bo.response_valid && bo.response.rsp.first == 0 && bo.response.rsp.count == 5);
  bi.request_valid = true; bi.request.req.r.len = 0;
  SalutABurstTick(burst, bi, bo, 3); assert(bo.request_consume);
  bi.request.end = true;
  SalutABurstTick(burst, bi, bo, 3); assert(bo.request_consume && !bo.read_valid);
  bi.request_valid = false;
  SalutABurstTick(burst, bi, bo, 3); assert(bo.read_valid && bo.read.end);
  bi.fill_valid = true; bi.fill = {WORD512(0), 0, true};
  SalutABurstTick(burst, bi, bo, 3); assert(bo.fill_consume);
  bi.fill_valid = false; SalutABurstTick(burst, bi, bo, 3);
  assert(bo.response_valid && bo.response.end && !burst.fault);

  // First/last PE of PEG 0, first PE of PEG 1, last PE, first/last front.
  for (uint8_t client : {1, PAR_PE_PER_PEG, PAR_PE_PER_PEG + 1, PAR_NUM_PE,
                         SALUT_FRONT_CLIENT0, SALUT_MAX_CLIENT}) {
    SalutResponseEnvelope envelope{{WORD512(0xfeed), client, 0, 8}, false};
    const bool pe = client <= PAR_NUM_PE;
    const int lane = pe ? (client - 1) / PAR_PE_PER_PEG : client - SALUT_FRONT_CLIENT0;
    const auto egress = SalutEgressTick(envelope, true, pe, !pe, true, lane);
    assert(egress.consume && !egress.fault && !egress.end_valid);
    assert(egress.peg_valid == pe);
    assert(egress.front_valid == !pe && egress.data.v == WORD512(0xfeed));
  }
  SalutResponseEnvelope end{}; end.end = true;
  auto egress = SalutEgressTick(end, true, true, true, false, 7);
  assert(egress.end_valid && !egress.consume && !egress.peg_valid && !egress.front_valid);
  SalutResponseEnvelope invalid{{WORD512(0), 0, 0, 8}, false};
  assert(SalutEgressTick(invalid, true, true, true, true, 0).fault);
  return {};
}
static const bool registered = [] {
  RegisterMemoryScenario("fabric_adapters", Adapters); return true;
}();
