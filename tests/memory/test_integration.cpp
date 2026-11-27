#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_commit.inc"
#include "salut_sm_bank.h"
#include "salut_hbm_word_port.h"
#include "core_omega.h"
#include <cassert>
#include <iostream>
#include <map>
#include <memory>

namespace {
// Front b writes/reads its own bank; PEG 1's first PE and front 1 share
// response lane 1. Derived from the numbering contract, not hardcoded.
constexpr int kFront0 = SALUT_FRONT_CLIENT0, kFront1 = SALUT_FRONT_CLIENT0 + 1;
constexpr int kPeg1Pe = PAR_PE_PER_PEG + 1;
std::string ClientKey(int client) { return "client_" + std::to_string(client); }
template <typename T> bool Head(const std::deque<T>& q, T& value) {
  if (q.empty()) return false;
  value = q.front(); return true;
}
struct FakeAxi {
  std::map<uint32_t, WORD512> backing;
  std::deque<WORD512> read_data;
  WORD512 submitted{};
  uint32_t address = 0;
  bool has_address = false, has_data = false;
  unsigned physical_completions = 0;
  // This models ONLY the external memory controller, never cache behavior.
  void Accept(const SalutHBMWordInputs& in, const SalutHBMWordOutputs& out) {
    if (out.read_data_consume) read_data.pop_front();
    if (out.read_addr_valid && in.read_addr_ready) read_data.push_back(backing[out.read_addr]);
    if (out.write_addr_valid && in.write_addr_ready) {
      assert(!has_address); address = out.write_addr; has_address = true;
    }
    if (out.write_data_valid && in.write_data_ready) {
      assert(!has_data); submitted = out.write_data; has_data = true;
    }
    if (out.write_resp_consume) {
      assert(has_address && has_data);
      backing[address] = submitted; has_address = has_data = false;
      ++physical_completions;
    }
  }
};
struct BankedRig {
  std::unique_ptr<SalutSmBankStorage> ram[8];
  SalutSmBankControl bank[8]{};
  SalutHBMWordControl factor_engine[8][2]{}, a_engine[8]{};
  FakeAxi factor_axi[8][2], a_axi[8];
  SalutBankSplitState split[8]{};
  SalutABurstControl a_burst[8]{};
  SalutMergeState<SalutResponseEnvelope> merge[8]{};
  SalutPublishControl publisher{};
  SalutDrainControl drain{};
  CoreOmega<SalutResponseEnvelope> response_network;
  std::deque<SalutReadEnvelope> requests[8], factor_req[8], a_req[8];
  std::deque<SalutWriteBeat> writes[8];
  std::deque<SalutWriteBeat> deferred_writes[8];
  std::deque<SalutReadEnvelope> deferred_reads[8];
  unsigned writes_requested[8]{}, writes_retired[8]{};
  std::deque<SalutResponseEnvelope> factor_rsp[8], a_rsp[8];
  std::deque<SalutWordReadReq> reads[8][2], a_reads[8];
  std::deque<SalutWordReadRsp> fills[8][2], a_fills[8];
  std::deque<SalutWritebackWord> wb[8][2];
  std::deque<SalutWritebackDone> done[8][2];
  std::deque<SalutPublishAck> publications[8];
  std::deque<ap_uint<3>> bank_done[8], egress_done[8];
  bool egress_closed[8]{}, closed = false, publish_done_pending = false;
  unsigned expected_publications = 0, expected_reads = 0, published = 0, responses = 0;
  int delay_bank = -1, delay_port = 0, last_response_cycle = -1;
  MemoryTrace trace;
  BankedRig() {
    for (int b = 0; b < 8; ++b) ram[b] = std::make_unique<SalutSmBankStorage>();
    Initialize();
  }
  void Initialize() {
    for (int b = 0; b < 8; ++b) {
      bank[b] = {}; bank[b].bank = b;
      for (int i = 0; i < 4096; ++i) {
        SalutSmBankInputs in{}; SalutSmBankOutputs out{};
        SalutSmBankTick(bank[b], *ram[b], in, out);
      }
      assert(bank[b].initialized && !bank[b].fault);
    }
  }
  void Write(int b, int port, uint32_t seq, WORD512 word) {
    SalutWriteBeat beat{word, pack_addr((port ? 16 : 8)+b, port ? 8 : 0), seq, 0,
                       ap_uint<8>(SALUT_FRONT_CLIENT0+b), ap_uint<8>(port+1), true, false, false};
    if (writes_requested[b]++ == 0) writes[b].push_back(beat);
    else deferred_writes[b].push_back(beat);
    ++expected_publications;
  }
  void Read(int b, int port, int client) {
    SalutReadEnvelope request{{{uint8_t(port+1), pack_addr((port ? 16 : 8)+b, port ? 8 : 0), 8}, ap_uint<8>(client)}, false};
    if (writes_requested[b] != writes_retired[b]) deferred_reads[b].push_back(request);
    else requests[b].push_back(request);
    ++expected_reads;
  }
  void Close() {
    closed = true;
    for (int b = 0; b < 8; ++b) {
      SalutReadEnvelope end{}; end.end = true; requests[b].push_back(end);
      SalutWriteBeat write_end{}; write_end.end = true; writes[b].push_back(write_end);
    }
  }
  void Tick(int cycle) {
    for (int b = 0; b < 8; ++b) {
      SalutBankSplitInputs xi{}; SalutBankSplitOutputs xo{};
      xi.valid = Head(requests[b], xi.data);
      xi.ready[0] = factor_req[b].size() < 4; xi.ready[1] = a_req[b].size() < 4;
      SalutBankSplitTick(split[b], xi, xo, b); assert(!split[b].fault);
      if (xo.consume) requests[b].pop_front();
      if (xo.valid[0] && xi.ready[0]) factor_req[b].push_back(xo.data);
      if (xo.valid[1] && xi.ready[1]) a_req[b].push_back(xo.data);
      SalutSmBankInputs si{}; SalutSmBankOutputs so{};
      si.request_valid = Head(factor_req[b], si.request); si.write_valid = Head(writes[b], si.write);
      si.response_ready = factor_rsp[b].size() < 4;
      si.publish_ready = publications[b].size() < 4; si.drained_ready = bank_done[b].size() < 4;
      for (int p = 0; p < 2; ++p) {
        si.read_ready[p] = reads[b][p].size() < 4; si.writeback_ready[p] = wb[b][p].size() < 4;
        si.fill_valid[p] = Head(fills[b][p], si.fill[p]); si.done_valid[p] = Head(done[b][p], si.done[p]);
      }
      SalutSmBankTick(bank[b], *ram[b], si, so); assert(!bank[b].fault);
      if (so.request_consume) factor_req[b].pop_front();
      if (so.write_consume) writes[b].pop_front();
      if (so.response_valid && si.response_ready) factor_rsp[b].push_back(so.response);
      if (so.publish_valid && si.publish_ready) publications[b].push_back(so.publish);
      if (so.drained_valid && si.drained_ready) bank_done[b].push_back(so.drained);
      for (int p = 0; p < 2; ++p) {
        if (so.fill_consume[p]) fills[b][p].pop_front();
        if (so.done_consume[p]) done[b][p].pop_front();
        if (so.read_valid[p] && si.read_ready[p]) reads[b][p].push_back(so.read[p]);
        if (so.writeback_valid[p] && si.writeback_ready[p]) wb[b][p].push_back(so.writeback[p]);
        auto& engine = factor_engine[b][p]; auto& axi = factor_axi[b][p];
        SalutHBMWordInputs hi{}; SalutHBMWordOutputs ho{};
        hi.read_valid = Head(reads[b][p], hi.read); hi.write_valid = Head(wb[b][p], hi.write);
        hi.read_data_valid = Head(axi.read_data, hi.read_data);
        hi.read_addr_ready = hi.write_addr_ready = hi.write_data_ready = true;
        hi.response_ready = fills[b][p].size() < 4; hi.done_ready = done[b][p].size() < 4;
        hi.write_resp_valid = engine.write_phase == 2 && axi.has_address && axi.has_data &&
                            (b != delay_bank || p != delay_port || cycle >= 300);
        SalutHBMWordTick(engine, hi, ho); assert(!engine.fault);
        if (ho.read_consume) reads[b][p].pop_front();
        if (ho.write_consume) wb[b][p].pop_front();
        if (ho.response_valid && hi.response_ready) fills[b][p].push_back(ho.response);
        if (ho.done_valid && hi.done_ready) done[b][p].push_back(ho.done);
        if (ho.write_resp_consume) {
          last_response_cycle = cycle;
          ++trace.counts["physical_write_responses"];
          ++trace.counts["bank_"+std::to_string(b)+"_physical_completions"];
          if (cycle < 300 && b != delay_bank) ++trace.counts["other_bank_completed_before_delayed_bank"];
          if (cycle < 300 && b == delay_bank && p != delay_port) ++trace.counts["other_port_completed_before_delayed_port"];
        }
        axi.Accept(hi, ho);
      }
      SalutABurstInputs ai{}; SalutABurstOutputs ao{};
      ai.request_valid = Head(a_req[b], ai.request); ai.fill_valid = Head(a_fills[b], ai.fill);
      ai.read_ready = a_reads[b].size() < 4; ai.response_ready = a_rsp[b].size() < 4;
      SalutABurstTick(a_burst[b], ai, ao, b); assert(!a_burst[b].fault);
      if (ao.request_consume) a_req[b].pop_front();
      if (ao.fill_consume) a_fills[b].pop_front();
      if (ao.read_valid && ai.read_ready) a_reads[b].push_back(ao.read);
      if (ao.response_valid && ai.response_ready) a_rsp[b].push_back(ao.response);
      SalutHBMWordInputs hi{}; SalutHBMWordOutputs ho{};
      hi.read_valid = Head(a_reads[b], hi.read); hi.read_data_valid = Head(a_axi[b].read_data, hi.read_data);
      hi.read_addr_ready = true; hi.response_ready = a_fills[b].size() < 4;
      SalutHBMWordTick(a_engine[b], hi, ho); assert(!a_engine[b].fault);
      if (ho.read_consume) a_reads[b].pop_front();
      if (ho.response_valid && hi.response_ready) a_fills[b].push_back(ho.response);
      a_axi[b].Accept(hi, ho);
      SalutSwitchInputs<SalutResponseEnvelope> mi{}; SalutSwitchOutputs<SalutResponseEnvelope> mo{};
      mi.input_valid[0] = Head(factor_rsp[b], mi.input_data[0]);
      mi.input_valid[1] = Head(a_rsp[b], mi.input_data[1]);
      mi.output_ready[0] = response_network.links[0][b].size() < 4;
      SalutMergeTick(merge[b], mi, mo);
      if (mo.input_consume[0]) factor_rsp[b].pop_front();
      if (mo.input_consume[1]) a_rsp[b].pop_front();
      if (mo.output_valid[0] && mi.output_ready[0]) response_network.links[0][b].push_back(mo.output_data[0]);
    }
    response_network.Tick();
    for (int lane = 0; lane < 8; ++lane) if (!egress_closed[lane]) {
      SalutResponseEnvelope packet{};
      auto& queue = response_network.links[3][lane];
      auto output = SalutEgressTick(packet, false, true, true, true, lane);
      if (Head(queue, packet)) output = SalutEgressTick(packet, true, true, true, true, lane);
      assert(!output.fault);
      if (output.consume) {
        queue.pop_front();
        if (output.end_valid) { egress_closed[lane] = true; egress_done[lane].push_back(lane); }
        else {
          ++responses; ++trace.counts[output.peg_valid ? "pe_responses" : "front_responses"];
          trace.words["client_"+std::to_string(unsigned(output.data.pe))] = output.data.v;
          if (cycle < 300 && delay_bank >= 0) ++trace.counts["read_before_delayed_physical_ack"];
        }
      }
    }
    SalutPublishInputs pi{}; SalutPublishOutputs po{};
    for (int b = 0; b < 8; ++b) pi.sm_valid[b] = Head(publications[b], pi.sm[b]);
    pi.ack_ready = pi.drained_ready = true; pi.close = closed;
    SalutPublishTick(publisher, pi, po); assert(!publisher.fault);
    for (int b = 0; b < 8; ++b) if (po.sm_consume[b]) publications[b].pop_front();
    if (po.ack_valid) {
      ++published;
      const int b = int(unsigned(po.ack.client)) - SALUT_FRONT_CLIENT0;
      assert(b >= 0 && b < 8 && po.ack.seq == writes_retired[b]);
      ++writes_retired[b];
      if (!deferred_writes[b].empty()) {
        writes[b].push_back(deferred_writes[b].front()); deferred_writes[b].pop_front();
      } else while (!deferred_reads[b].empty()) {
        requests[b].push_back(deferred_reads[b].front()); deferred_reads[b].pop_front();
      }
    }
    if (po.drained_valid) publish_done_pending = true;
    SalutDrainInputs di{}; SalutDrainOutputs dout{};
    di.compute_valid = closed && !drain.compute_seen; di.compute = 1;
    di.publish_valid = publish_done_pending; di.publish = 1;
    for (int b = 0; b < 8; ++b) {
      di.bank_valid[b] = Head(bank_done[b], di.bank[b]);
      di.response_valid[b] = Head(egress_done[b], di.response[b]);
    }
    SalutDrainTick(drain, di, dout); assert(!drain.fault);
    if (dout.publish_consume) publish_done_pending = false;
    for (int b = 0; b < 8; ++b) {
      if (dout.bank_consume[b]) bank_done[b].pop_front();
      if (dout.response_consume[b]) egress_done[b].pop_front();
    }
    if (drain.completed && delay_bank >= 0 && cycle < 300)
      ++trace.counts["ap_done_before_last_physical_ack"];
    if (!closed && published == expected_publications && responses == expected_reads) Close();
  }
  MemoryTrace Run() {
    int cycle = 0;
    for (; cycle < 10000 && !drain.completed; ++cycle) Tick(cycle);
    assert(drain.completed && cycle < 10000);
    assert(published == expected_publications && responses == expected_reads);
    unsigned dirty = 0;
    for (int b = 0; b < 8; ++b) {
      dirty += bank[b].dirty_count;
      assert(bank[b].queue_count[0] == 0 && bank[b].queue_count[1] == 0);
      assert(factor_engine[b][0].write_phase == 5 && factor_engine[b][1].write_phase == 5);
      assert(a_engine[b].read_phase == 5 && drain.bank_seen[b] && drain.response_seen[b]);
    }
    trace.counts["dirty_at_done"] = dirty;
    trace.counts["ap_done_after_all_barriers"] = drain.completed;
    trace.counts["last_physical_ack_cycle"] = last_response_cycle < 0 ? 0 : last_response_cycle;
    trace.counts["completion_cycle"] = cycle;
    assert(dirty == 0);
    return trace;
  }
};
MemoryTrace Concurrency() {
  BankedRig rig; rig.delay_bank = 0;
  for (int b = 0; b < 8; ++b) { rig.Write(b, 0, 0, WORD512(0x100+b)); rig.Write(b, 1, 1, WORD512(0x200+b)); }
  // Dirty cache read (front) while bank0's L engine waits for a real response.
  rig.Read(0, 0, kFront0);
  // Two classes on the same output lane, sharing U but never misrouted.
  rig.Read(1, 1, kPeg1Pe); rig.Read(1, 1, kFront1);
  auto trace = rig.Run();
  assert(trace.counts["physical_write_responses"] == 16);
  assert(trace.counts["other_bank_completed_before_delayed_bank"] == 14);
  assert(trace.counts["other_port_completed_before_delayed_port"] == 1);
  assert(trace.counts["read_before_delayed_physical_ack"] == 3);
  assert(trace.counts["pe_responses"] == 1 && trace.counts["front_responses"] == 2);
  assert(trace.words[ClientKey(kFront0)] == WORD512(0x100));
  assert(trace.words[ClientKey(kPeg1Pe)] == WORD512(0x201) && trace.words[ClientKey(kFront1)] == WORD512(0x201));
  for (int b = 0; b < 8; ++b) {
    assert(rig.factor_axi[b][0].backing[0] == WORD512(0x100+b));
    assert(rig.factor_axi[b][1].backing[1] == WORD512(0x200+b));
    assert(trace.counts["bank_"+std::to_string(b)+"_physical_completions"] == 2);
  }
  std::cout << "CONCURRENCY_TRACE physical_responses=16 other_bank_before_bank0="
            << trace.counts["other_bank_completed_before_delayed_bank"]
            << " dirty_reads_before_ack=" << trace.counts["read_before_delayed_physical_ack"] << '\n';
  return trace;
}
MemoryTrace FinalAckDelayed() {
  BankedRig rig; rig.delay_bank = 0; rig.delay_port = 1;
  rig.Write(0, 1, 0, WORD512(0x456));
  auto trace = rig.Run();
  assert(trace.counts["ap_done_before_last_physical_ack"] == 0);
  assert(trace.counts["ap_done_after_all_barriers"] == 1 && trace.counts["dirty_at_done"] == 0);
  assert(trace.counts["physical_write_responses"] == 1);
  assert(trace.counts["completion_cycle"] > trace.counts["last_physical_ack_cycle"]);
  assert(rig.factor_axi[0][1].backing[1] == WORD512(0x456));
  std::cout << "DRAIN_PHYSICAL_TRACE last_ack=" << trace.counts["last_physical_ack_cycle"]
            << " completion=" << trace.counts["completion_cycle"] << " dirty=0\n";
  return trace;
}
MemoryTrace EmptyReadRepeat() {
  BankedRig empty; auto first = empty.Run();
  assert(first.counts["physical_write_responses"] == 0);
  BankedRig reads;
  reads.factor_axi[1][1].backing[1] = WORD512(0x789);
  reads.Read(1, 1, kPeg1Pe); reads.Read(1, 1, kFront1);
  auto result = reads.Run();
  assert(result.words[ClientKey(kPeg1Pe)] == WORD512(0x789) && result.words[ClientKey(kFront1)] == WORD512(0x789));
  // Reuse the exact same storage allocation AFTER the complete first drain.
  // Fresh task state initializes valid/tag metadata; stale RAM bytes remain.
  BankedRig next;
  for (int b = 0; b < 8; ++b) { next.ram[b].swap(reads.ram[b]); next.factor_axi[b][1].backing = reads.factor_axi[b][1].backing; }
  next.Initialize();
  assert(!next.ram[1]->valid[1]);
  next.factor_axi[1][1].backing[1] = WORD512(0xABC); next.Read(1, 1, kPeg1Pe);
  auto second = next.Run(); assert(second.words[ClientKey(kPeg1Pe)] == WORD512(0xABC));
  return second;
}
const bool registered = [] {
  RegisterMemoryScenario("concurrency", Concurrency);
  RegisterMemoryScenario("final_ack_delayed", FinalAckDelayed);
  RegisterMemoryScenario("empty_read_repeat", EmptyReadRepeat);
  return true;
}();
}  // namespace
