#include "fixture.h"
#include "salut_sm_bank.h"
#include <algorithm>
#include <cassert>
#include <deque>
#include <iostream>
#include <memory>
#include <cstring>
#include <new>
#include <vector>

// All events and traces here come from SalutSmBankTick, the production core.
// Fake HBM supplies external latency/data only; it does not simulate the cache.
struct BankFixture {
  SalutSmBankControl control{};
  std::unique_ptr<SalutSmBankStorage> storage = std::make_unique<SalutSmBankStorage>();
  SalutSmBankInputs input{}; SalutSmBankOutputs output{};
  std::deque<SalutWritebackWord> issued[2];
  std::deque<SalutWordReadReq> reads[2];
  std::vector<SalutPublishAck> publications;
  std::vector<SalutResponseEnvelope> responses;
  MemoryTrace trace;
  explicit BankFixture(int bank = 0) {
    control.bank = bank;
    input.response_ready = input.publish_ready = input.drained_ready = true;
    for (int p = 0; p < 2; ++p) input.read_ready[p] = input.writeback_ready[p] = true;
    for (int i = 0; i < 4200 && !control.initialized; ++i) Tick();
    assert(control.initialized && !control.fault);
  }
  void Tick() {
    SalutSmBankTick(control, *storage, input, output);
    if (output.request_consume) input.request_valid = false;
    if (output.write_consume) input.write_valid = false;
    for (int p = 0; p < 2; ++p) {
      if (output.fill_consume[p]) input.fill_valid[p] = false;
      if (output.done_consume[p]) input.done_valid[p] = false;
      if (output.read_valid[p] && input.read_ready[p]) reads[p].push_back(output.read[p]);
      if (output.writeback_valid[p] && input.writeback_ready[p]) {
        issued[p].push_back(output.writeback[p]);
        if (!output.writeback[p].end) {
          const uint32_t index = SalutSmIndex(output.writeback[p].address);
          assert(storage->dirty[index] && storage->valid[index]);
          assert(storage->data[index] == output.writeback[p].data);
          ++trace.counts["issued"];
        }
      }
      trace.counts[p == 0 ? "max_l_occupancy" : "max_u_occupancy"] =
          std::max(trace.counts[p == 0 ? "max_l_occupancy" : "max_u_occupancy"],
                   uint64_t(control.queue_count[p]));
      trace.counts["completed"] += output.done_consume[p] && !input.done[p].end;
    }
    if (output.publish_valid && input.publish_ready) publications.push_back(output.publish);
    if (output.response_valid && input.response_ready) responses.push_back(output.response);
    if (output.drained_valid && input.drained_ready) ++trace.counts["drained"];
  }
  void Ticks(int count = 12) { for (int i = 0; i < count; ++i) Tick(); }
  void Write(uint32_t address, uint8_t kind, WORD512 data, uint32_t seq = 0, uint8_t client = 1) {
    input.write = {data, address, seq, 0, client, kind, true, false, false};
    input.write_valid = true;
    for (int i = 0; i < 40 && input.write_valid && !control.fault; ++i) Tick();
    assert(!input.write_valid && !control.fault); Ticks();
  }
  void Read(uint32_t address, uint8_t kind, uint32_t len = 8, uint8_t client = SALUT_FRONT_CLIENT0) {
    input.request = {{{kind, address, len}, client}, false}; input.request_valid = true;
    for (int i = 0; i < 40 && input.request_valid; ++i) Tick();
    assert(!input.request_valid); Ticks();
  }
  void Fill(int port, WORD512 payload) {
    assert(!reads[port].empty());
    const auto request = reads[port].front(); reads[port].pop_front();
    input.fill[port] = {payload, request.ticket, request.end}; input.fill_valid[port] = true;
    for (int i = 0; i < 40 && input.fill_valid[port]; ++i) Tick();
    assert(!input.fill_valid[port]); Ticks();
  }
  void Complete(int port) {
    assert(!issued[port].empty());
    const auto write = issued[port].front(); issued[port].pop_front();
    input.done[port] = {write.ticket, write.end}; input.done_valid[port] = true;
    for (int i = 0; i < 40 && input.done_valid[port]; ++i) Tick();
    assert(!input.done_valid[port]); Ticks();
  }
};

// Breaks caught: L-only cache, reading old HBM while dirty, replacing dirty
// victim, late fill overwriting a newer SM install, kind aliasing, early WB.
static MemoryTrace SmBank() {
  // ap_uint's user-provided default constructor does not initialize bits.
  // Construct over nonzero backing bytes rather than relying on allocator
  // zero pages; every live queue pointer/count/generation MUST be explicit.
  {
    alignas(SalutSmBankControl) unsigned char backing[sizeof(SalutSmBankControl)];
    std::memset(backing,0xa5,sizeof(backing));
    auto* initial=new(backing) SalutSmBankControl{};
    for(int port=0;port<2;++port) {
      assert(initial->queue_head[port]==0 && initial->queue_tail[port]==0 &&
             initial->queue_issue[port]==0 && initial->queue_count[port]==0);
      for(int slot=0;slot<8;++slot) assert(initial->generations[port][slot]==0);
    }
    initial->~SalutSmBankControl();
  }
  // Initialization may clear at most ONE snapshot slot per physical L/U
  // queue per tick. Eight parallel clears on the last cache-init edge cannot
  // be implemented by the two-port, slot-indexed snapshot RAM.
  {
    SalutSmBankControl initial{};
    auto ram = std::make_unique<SalutSmBankStorage>();
    SalutSmBankInputs input{}; SalutSmBankOutputs output{};
    for (int port = 0; port < 2; ++port)
      for (int slot = 0; slot < 8; ++slot) {
        ram->wb[port][slot].valid = true;
        ram->wb_data[port][slot] = WORD512(0x12345678) + 16*port + slot;
      }
    for (int tick = 0; tick < 4096; ++tick) {
      if(tick<8) SalutSmInitialize(initial,*ram);
      else SalutSmBankTick(initial,*ram,input,output);
      assert(!initial.fault && initial.initialized == (tick == 4095));
      assert(!output.write_consume && !output.request_consume && !output.publish_valid);
      for (int port = 0; port < 2; ++port)
        for (int slot = 0; slot < 8; ++slot)
          {
            assert(ram->wb[port][slot].valid == (slot > tick));
            // Invalid snapshot payload is not metadata. Clearing it on init
            // adds unnecessary RAM writes and cannot affect visible traffic.
            assert(ram->wb_data[port][slot] == WORD512(0x12345678) + 16*port + slot);
          }
    }
  }
  assert(!RunMemoryScenario("sm_faults").fault);
  {
    BankFixture one_probe;
    one_probe.Write(pack_addr(8,0),1,WORD512(0x10),0,1);
    one_probe.Write(pack_addr(8,8),1,WORD512(0x20),1,1);
    // Two legitimate read contexts conflict with pinned dirty victims. A
    // single arbitration event probes ONE cache candidate, rotating fairly;
    // retrying both inline duplicates the entire RAM-dependence graph.
    for(int context=0;context<2;++context) {
      const uint32_t cursor=pack_addr(8,32768+8*context);
      one_probe.control.reads[context]={{{1,cursor,8},ap_uint<8>(SALUT_FRONT_CLIENT0+context)},cursor,8,true,false};
    }
    one_probe.control.prefer_read=true;one_probe.control.read_rr=false;
    one_probe.Tick();
    assert(one_probe.control.read_rr && !one_probe.control.fault);
    assert(one_probe.control.stage==0 && one_probe.responses.empty());
    one_probe.Tick();
    assert(!one_probe.control.read_rr && !one_probe.control.fault);
  }
  BankFixture f;
  f.Write(pack_addr(8, 0), 1, WORD512(0x11));
  assert(f.publications.size() == 1 && f.control.queue_count[0] == 1);
  assert(f.storage->dirty[0]);
  f.Read(pack_addr(8, 3), 1, 2);
  assert(f.responses.size() == 1 && f.responses.back().rsp.v == WORD512(0x11));
  assert(f.responses.back().rsp.first == 3 && f.responses.back().rsp.count == 2);
  assert(f.reads[0].empty() && f.trace.counts["completed"] == 0);
  f.trace.words["dirty_hit"] = f.responses.back().rsp.v;
  f.trace.counts["hbm_write_completions_at_hit"] = f.trace.counts["completed"];
  // U with the same index cannot displace the outstanding dirty L word.
  f.input.write = {WORD512(0x22), pack_addr(16, 0), 1, 0, 1, 2, true, false, false};
  f.input.write_valid = true; f.Ticks();
  assert(f.input.write_valid && f.storage->data[0] == WORD512(0x11));
  f.Complete(0);
  for (int i = 0; i < 40 && f.input.write_valid; ++i) f.Tick();
  assert(!f.input.write_valid); f.Ticks();
  f.Read(pack_addr(16, 0), 2);
  assert(f.responses.back().rsp.v == WORD512(0x22) && f.reads[1].empty());
  f.Complete(1);
  // Actual U miss then fill; the next U read is a shared hit.
  f.Read(pack_addr(16, 8), 2);
  assert(f.reads[1].size() == 1 && f.reads[1].front().address == pack_addr(16, 8));
  f.Fill(1, WORD512(0x33));
  const size_t responses = f.responses.size();
  f.Read(pack_addr(16, 8), 2);
  assert(f.responses.size() == responses + 1 && f.responses.back().rsp.v == WORD512(0x33));
  assert(f.reads[1].empty());
  // Counters must reflect real production events, not preset summary values.
  assert(f.control.stats.hits == 3 && f.control.stats.dirty_hits == 2);
  assert(f.control.stats.misses == 1 && f.control.stats.installs == 2);
  assert(f.control.stats.writeback_issued[0] == 1 && f.control.stats.writeback_issued[1] == 1);
  assert(f.control.stats.writeback_completed[0] == 1 && f.control.stats.writeback_completed[1] == 1);
  assert(f.control.stats.capacity_stall_ticks > 0 && f.control.stats.dirty_peak == 1);
  // Read tickets are 32 bits, unlike the separate 19-bit WB generation/slot.
  // Do not accidentally apply the write-ticket bound to physical reads.
  BankFixture wrap;
  wrap.control.next_ticket = (1u<<19)-1;
  wrap.Read(pack_addr(16,0),2);
  assert(wrap.reads[1].front().ticket == (1u<<19)-1);
  wrap.Fill(1,WORD512(0xa1));
  wrap.Read(pack_addr(16,8),2);
  assert(wrap.reads[1].front().ticket == (1u<<19));
  wrap.Fill(1,WORD512(0xa2));
  BankFixture software_sequence;
  software_sequence.Write(pack_addr(8,0),1,WORD512(0xa3),1u<<20);

  BankFixture late;
  late.Read(pack_addr(16, 0), 2); assert(late.reads[1].size() == 1);
  late.Write(pack_addr(16, 0), 2, WORD512(0x44));
  late.Fill(1, WORD512(0xdead));
  assert(late.responses.back().rsp.v == WORD512(0x44));
  assert(late.storage->data[0] == WORD512(0x44) && late.storage->dirty[0]);
  // Different key/version discards stale payload and safely re-looks up.
  BankFixture retry;
  retry.Read(pack_addr(8, 0), 1);
  retry.Write(pack_addr(16, 0), 2, WORD512(0x55));
  retry.Fill(0, WORD512(0xbad));
  assert(retry.responses.empty() && retry.reads[0].empty());
  retry.Complete(1); retry.Ticks();
  assert(retry.reads[0].size() == 1);
  retry.Fill(0, WORD512(0x66));
  assert(retry.responses.back().rsp.v == WORD512(0x66) && !retry.control.fault);
  // A multiword logical burst has exactly one publication, after last.
  BankFixture burst;
  // Producer: the last PE of PEG 0 (client PAR_PE_PER_PEG).
  burst.input.write = {WORD512(0x70), pack_addr(8, 0), 0, 0, PAR_PE_PER_PEG, 1, false, false, false};
  burst.input.write_valid = true; burst.Ticks();
  assert(!burst.input.write_valid && burst.publications.empty());
  burst.input.write = {WORD512(0x71), pack_addr(8, 8), 0, 1, PAR_PE_PER_PEG, 1, true, false, false};
  burst.input.write_valid = true; burst.Ticks();
  assert(burst.publications.size() == 1 && burst.publications.back().client == PAR_PE_PER_PEG);
  assert(burst.issued[0].size() == 2 && burst.issued[0][0].data == WORD512(0x70));
  burst.input.write = {WORD512(0), pack_addr(16, 0), 0, 0, SALUT_MAX_CLIENT, 2, true, true, false};
  burst.input.write_valid = true; burst.Ticks();
  assert(burst.publications.size() == 2 && burst.issued[1].empty());
  std::cout << "SM_TRACE dirty_hit_before_completion=" << f.trace.counts.at("hbm_write_completions_at_hit")
            << " U_hit=1 late_fill_forward=1 stale_fill_retry=1\n";
  return f.trace;
}

static MemoryTrace SmLocalDrain() {
  BankFixture f;
  f.Write(pack_addr(16, 0), 2, WORD512(0xd0));
  f.input.request_valid = f.input.write_valid = true;
  f.input.request = {}; f.input.request.end = true;
  f.input.write = {}; f.input.write.end = true;
  f.Ticks();
  assert(f.reads[0].size() == 1 && f.reads[0].front().end);
  f.Fill(0, WORD512(0)); f.Fill(1, WORD512(0));
  // L's ordered write End can close, but U still has an actual dirty word.
  assert(f.issued[0].size() == 1 && f.issued[0].front().end);
  f.Complete(0); f.Ticks();
  assert(!f.control.completed && f.trace.counts["drained"] == 0);
  assert(f.control.queue_count[1] == 1 && f.storage->dirty[0]);
  f.Complete(1);
  assert(f.issued[1].size() == 1 && f.issued[1].front().end);
  f.Complete(1);
  assert(f.control.completed && f.trace.counts["drained"] == 1);
  assert(f.responses.back().end && f.control.dirty_count == 0);
  assert(f.control.queue_count[0] == 0 && f.control.queue_count[1] == 0);
  // Fresh launch on the SAME cache RAM after a proven drain clears tags.
  f.control = {};
  for (int i = 0; i < 4200 && !f.control.initialized; ++i) f.Tick();
  f.Read(pack_addr(16, 0), 2);
  assert(f.reads[1].size() == 1 && !f.reads[1].front().end);
  f.Fill(1, WORD512(0xe0)); assert(f.responses.back().rsp.v == WORD512(0xe0));
  return f.trace;
}

// Breaks caught: ninth word accepted without retention, L full freezing U ACK
// or dirty hit, stale/duplicate ticket freeing live data, version-wrap ABA.
static MemoryTrace SmBackpressure() {
  assert(!SmLocalDrain().fault);
  BankFixture f;
  f.input.writeback_ready[0] = false;
  for (uint32_t word = 0; word < 8; ++word)
    f.Write(pack_addr(8, word * 8), 1, WORD512(0x100 + word), word);
  assert(f.control.queue_count[0] == 8 && f.issued[0].empty());
  f.Write(pack_addr(16, 64), 2, WORD512(0x200), 8);
  assert(f.control.queue_count[1] == 1);
  f.input.write = {WORD512(0x999), pack_addr(8, 72), 9, 0, 1, 1, true, false, false};
  f.input.write_valid = true; f.Ticks(); assert(f.input.write_valid);
  BankFixture other(1);
  other.Write(pack_addr(17, 0), 2, WORD512(0x301), 0, SALUT_FRONT_CLIENT0);
  other.Complete(1); other.Read(pack_addr(17, 0), 2);
  assert(other.publications.size() == 1 && other.responses.back().rsp.v == WORD512(0x301));
  assert(other.control.dirty_count == 0 && f.input.write_valid);
  f.trace.counts["other_bank_completed_while_l_full"] = other.publications.size();
  f.Read(pack_addr(8, 0), 1);
  assert(f.responses.back().rsp.v == WORD512(0x100) && f.input.write_valid);
  f.Complete(1); assert(f.control.queue_count[1] == 0 && f.input.write_valid);
  f.input.writeback_ready[0] = true; f.Ticks();
  assert(f.issued[0].size() == 8);
  for (uint32_t word = 0; word < 8; ++word)
    assert(f.issued[0][word].data == WORD512(0x100 + word));
  f.Complete(0); assert(!f.input.write_valid);
  f.Ticks(); assert(f.control.queue_count[0] == 8);

  BankFixture blocked_publish;
  blocked_publish.input.publish_ready = false;
  blocked_publish.Write(pack_addr(8, 0), 1, WORD512(0x77));
  assert(blocked_publish.publications.empty() && blocked_publish.issued[0].size() == 1);
  blocked_publish.Complete(0);
  assert(!blocked_publish.storage->dirty[0] && blocked_publish.publications.empty());
  blocked_publish.input.publish_ready = true; blocked_publish.Ticks();
  assert(blocked_publish.publications.size() == 1);
  blocked_publish.input.done[0] = {0, false}; blocked_publish.input.done_valid[0] = true;
  blocked_publish.Tick(); assert(blocked_publish.control.fault);

  BankFixture wrap;
  wrap.storage->version[0] = 65535;
  wrap.Read(pack_addr(8, 0), 1);
  wrap.input.write = {WORD512(0x88), pack_addr(16, 0), 0, 0, 1, 2, true, false, false};
  wrap.input.write_valid = true; wrap.Ticks();
  assert(wrap.input.write_valid && wrap.storage->version[0] == 65535);
  wrap.Fill(0, WORD512(0x89));
  for (int i = 0; i < 40 && wrap.input.write_valid; ++i) wrap.Tick();
  wrap.Ticks(); assert(!wrap.input.write_valid && !wrap.control.fault);
  assert(wrap.storage->version[0] == 1);

  BankFixture wrong;
  wrong.Write(pack_addr(8, 0), 1, WORD512(0xaa));
  wrong.input.done[0] = {wrong.issued[0].front().ticket ^ 8u, false};
  wrong.input.done_valid[0] = true; wrong.Tick();
  assert(wrong.control.fault && wrong.storage->dirty[0]);
  BankFixture invalid;
  invalid.input.write = {WORD512(0), pack_addr(8, 0), 0, 0, 65, 1, true, false, false};
  invalid.input.write_valid = true; invalid.Tick(); assert(invalid.control.fault);
  std::cout << "SM_QUEUE_TRACE max_L=" << f.trace.counts.at("max_l_occupancy")
            << " max_U=" << f.trace.counts.at("max_u_occupancy")
            << " issued=" << f.trace.counts.at("issued")
            << " completed=" << f.trace.counts.at("completed") << '\n';
  return f.trace;
}

static MemoryTrace SmFaultsAndBackpressure() {
  BankFixture f;
  f.Write(pack_addr(8, 0), 1, WORD512(0xa));
  f.Write(pack_addr(16, 8), 2, WORD512(0xb), 1);
  f.input.response_ready = false; f.Read(pack_addr(8, 0), 1);
  assert(f.responses.empty() && f.control.response_held);
  for (int port = 0; port < 2; ++port) {
    f.input.done[port] = {f.issued[port].front().ticket, false};
    f.issued[port].pop_front(); f.input.done_valid[port] = true;
  }
  f.Tick(); assert(f.output.done_consume[0] && f.output.done_consume[1]);
  f.Ticks(); assert(f.control.dirty_count == 0 && f.responses.empty());
  f.Write(pack_addr(16, 0), 2, WORD512(0xc), 2);
  f.input.response_ready = true; f.Ticks();
  assert(f.responses.size() == 1 && f.responses.back().rsp.v == WORD512(0xa));

  BankFixture fill_ticket;
  fill_ticket.Read(pack_addr(16, 0), 2);
  fill_ticket.input.fill[1] = {WORD512(0xd), fill_ticket.reads[1].front().ticket ^ 1u, false};
  fill_ticket.input.fill_valid[1] = true; fill_ticket.Tick(); assert(fill_ticket.control.fault);
  BankFixture duplicate;
  duplicate.Write(pack_addr(8, 0), 1, WORD512(0xe)); duplicate.Complete(0);
  duplicate.input.write = {WORD512(0xf), pack_addr(8, 0), 1, 0, 1, 1, true, false, false};
  duplicate.input.write_valid = true; duplicate.Tick(); assert(duplicate.control.fault);
  BankFixture seq;
  // This bank need not have observed seq0/1: they may target other banks.
  seq.Write(pack_addr(8, 8), 1, WORD512(0xa0), 2);
  seq.input.write = {WORD512(0), pack_addr(8, 0), 1, 0, 1, 1, true, false, false};
  seq.input.write_valid = true; seq.Tick(); assert(seq.control.fault);
  BankFixture ordinal;
  ordinal.input.write = {WORD512(0), pack_addr(8, 0), 0, 1, 1, 1, true, false, false};
  ordinal.input.write_valid = true; ordinal.Tick(); assert(ordinal.control.fault);
  BankFixture reused;
  for (uint32_t i = 0; i < 8; ++i) reused.Write(pack_addr(8, i * 8), 1, WORD512(i), i);
  const uint32_t old_ticket = reused.issued[0].front().ticket;
  for (int i = 0; i < 8; ++i) reused.Complete(0);
  assert(reused.control.queue_count[0] == 0 && reused.control.dirty_count == 0);
  reused.Write(pack_addr(8, 64), 1, WORD512(0x10), 8);
  assert(reused.issued[0].front().ticket == old_ticket + 8);
  reused.input.done[0] = {old_ticket, false}; reused.input.done_valid[0] = true;
  reused.Tick(); assert(reused.control.fault && reused.storage->dirty[8]);
  return {};
}
static const bool registered = [] {
  RegisterMemoryScenario("sm_bank", SmBank); RegisterMemoryScenario("sm_dirty_hit", SmBank);
  RegisterMemoryScenario("sm_backpressure", SmBackpressure);
  RegisterMemoryScenario("sm_local_drain", SmLocalDrain);
  RegisterMemoryScenario("sm_faults", SmFaultsAndBackpressure); return true;
}();
