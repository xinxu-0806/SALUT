#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_commit.inc"
#include "salut_sm_bank.h"
#include "salut_hbm_word_port.h"
#include <cassert>
#include <deque>
#include <iostream>
#include <memory>

// Breaks caught: publication before retained SM/visible LM, or retirement
// still waiting for physical HBM completion. Real production cores connected
// by bounded queues; the external AXI controller alone supplies delayed data.
static MemoryTrace DelayedHbm() {
  SalutSmBankControl sm{}; SalutLMControl lm{};
  auto shared = std::make_unique<SalutSmBankStorage>();
  auto local = std::make_unique<SalutLMStorage>();
  SalutSmBankInputs si{}; SalutSmBankOutputs so{};
  SalutLMInputs li{}; SalutLMOutputs lo{};
  for (int i = 0; i < 4096; ++i) {
    SalutSmBankTick(sm, *shared, si, so);
    if (!lm.initialized) SalutLMTick(lm, *local, li, lo);
  }
  assert(sm.initialized && lm.initialized);
  std::deque<SalutWritebackWord> writes;
  std::deque<SalutWritebackDone> completions;
  std::deque<SalutPublishAck> sm_publications, lm_publications;
  SalutHBMWordControl hbm{};
  SalutHBMWordInputs hi{}; SalutHBMWordOutputs ho{};
  SalutPublishControl publish{};
  SalutPublishInputs pi{}; SalutPublishOutputs po{};
  si.write_valid = true;
  si.write = {WORD512(0x123), pack_addr(8, 0), 0, 0, 1, 1, true, false, false};
  li.snoop_valid = li.data_valid = true;
  li.snoop = {{{1, pack_addr(8, 0), 8}, 1}, 0, false};
  li.data = {WORD512(0x123), 1, 8};
  WORD512 backing = 0, submitted = 0;
  bool physical_responded = false;
  int retirement_cycle = -1;
  MemoryTrace trace;
  for (int cycle = 0; cycle < 160; ++cycle) {
    si.publish_ready = sm_publications.size() < 4;
    si.writeback_ready[0] = writes.size() < 4;
    si.done_valid[0] = !completions.empty();
    if (si.done_valid[0]) si.done[0] = completions.front();
    // Delay the actual LM confirmation too: an implementation dropping the
    // required LM join would retire around cycle41 with no LM event observed.
    li.installed_ready = cycle >= 60 && lm_publications.size() < 4;
    hi = {}; hi.write_valid = !writes.empty();
    if (hi.write_valid) hi.write = writes.front();
    hi.write_addr_ready = hi.write_data_ready = hi.done_ready = true;
    // Keep the real word port waiting, long AFTER the observed retirement.
    if (retirement_cycle >= 0 && cycle > retirement_cycle + 20 && !physical_responded) {
      assert(hbm.write_phase == 2); hi.write_resp_valid = true;
    }
    pi = {}; pi.sm_valid[0] = !sm_publications.empty(); pi.lm_valid[0] = !lm_publications.empty();
    if (pi.sm_valid[0]) pi.sm[0] = sm_publications.front();
    if (pi.lm_valid[0]) pi.lm[0] = lm_publications.front();
    pi.ack_ready = true;
    SalutSmBankTick(sm, *shared, si, so);
    SalutLMTick(lm, *local, li, lo);
    SalutHBMWordTick(hbm, hi, ho);
    SalutPublishTick(publish, pi, po);
    assert(!sm.fault && !lm.fault && !hbm.fault && !publish.fault);
    if (so.write_consume) si.write_valid = false;
    if (lo.snoop_consume) li.snoop_valid = false;
    if (lo.data_consume) li.data_valid = false;
    if (so.done_consume[0]) completions.pop_front();
    if (ho.write_consume) writes.pop_front();
    if (po.sm_consume[0]) sm_publications.pop_front();
    if (po.lm_consume[0]) lm_publications.pop_front();
    if (so.writeback_valid[0] && si.writeback_ready[0]) writes.push_back(so.writeback[0]);
    if (so.publish_valid && si.publish_ready) {
      sm_publications.push_back(so.publish); ++trace.counts["sm_published"];
    }
    if (lo.installed_valid && li.installed_ready) {
      lm_publications.push_back(lo.installed); ++trace.counts["lm_installed"];
    }
    if (ho.write_data_valid && hi.write_data_ready) submitted = ho.write_data;
    if (ho.write_resp_consume) {
      physical_responded = true; backing = submitted; ++trace.counts["physical_responses"];
    }
    if (ho.done_valid && hi.done_ready) completions.push_back(ho.done);
    if (po.ack_valid && pi.ack_ready) {
      assert(retirement_cycle == -1 && po.ack.client == 1 && po.ack.seq == 0);
      retirement_cycle = cycle;
      trace.counts["retired_before_hbm"] += !physical_responded;
      trace.counts["retired_without_sm_or_required_lm"] +=
          trace.counts["sm_published"] == 0 || trace.counts["lm_installed"] == 0;
      assert(backing == WORD512(0) && shared->dirty[0] && sm.queue_count[0] == 1);
      assert(local->data[0] == WORD512(0x123) && shared->data[0] == WORD512(0x123));
    }
  }
  assert(trace.counts["retired_before_hbm"] == 1);
  assert(trace.counts["retired_without_sm_or_required_lm"] == 0);
  assert(trace.counts["physical_responses"] == 1 && backing == WORD512(0x123));
  assert(sm.queue_count[0] == 0 && sm.dirty_count == 0);
  std::cout << "PUBLISH_DELAY_TRACE retired_cycle=" << retirement_cycle
            << " before_hbm=" << trace.counts.at("retired_before_hbm")
            << " final_backing=" << backing.to_uint64() << '\n';
  return trace;
}
static const bool registered = [] {
  RegisterMemoryScenario("publish_delayed_hbm", DelayedHbm); return true;
}();
