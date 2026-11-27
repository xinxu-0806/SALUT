#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_commit.inc"
#include <cassert>
#include <iostream>

// Breaks caught: PE retirement without both caches, front incorrectly needing
// LM, stale/future/duplicate seq paired to the next burst, or early drain.
static MemoryTrace Publish() {
  assert(!RunMemoryScenario("write_flow").fault);
  assert(!RunMemoryScenario("write_merge").fault);
  assert(!RunMemoryScenario("publish_delayed_hbm").fault);
  SalutPublishControl state{};
  SalutPublishInputs in{}; SalutPublishOutputs out{};
  MemoryTrace trace;
  auto tick = [&] {
    SalutPublishTick(state, in, out);
    for (int lane = 0; lane < 8; ++lane) {
      if (out.sm_consume[lane]) in.sm_valid[lane] = false;
      if (out.lm_consume[lane]) in.lm_valid[lane] = false;
    }
    if (out.ack_valid && in.ack_ready) ++trace.counts["acks"];
  };
  in.ack_ready = true;
  in.sm_valid[0] = true; in.sm[0] = {1, 0}; tick();
  for (int cycle = 0; cycle < 80; ++cycle) { tick(); assert(!out.ack_valid); }
  in.lm_valid[0] = true; in.lm[0] = {1, 0};
  in.ack_ready = false;
  for (int cycle = 0; cycle < 80; ++cycle) tick();
  assert(out.ack_valid && out.ack.client == 1 && out.ack.seq == 0);
  for (int cycle = 0; cycle < 5; ++cycle) { tick(); assert(out.ack_valid && out.ack.seq == 0); }
  in.ack_ready = true; tick(); assert(trace.counts["acks"] == 1);
  in.lm_valid[0] = true; in.lm[0] = {1, 1}; tick();
  for (int cycle = 0; cycle < 80; ++cycle) { tick(); assert(!out.ack_valid); }
  in.sm_valid[3] = true; in.sm[3] = {1, 1};
  for (int cycle = 0; cycle < 80; ++cycle) tick();
  assert(trace.counts["acks"] == 2 && !state.fault);
  // SM owner can change; contiguous seq is enforced here globally, not by
  // requiring each individual SM bank to have observed every earlier burst.
  in.sm_valid[5] = in.lm_valid[7] = true;
  in.sm[5] = in.lm[7] = {PAR_NUM_PE, 0};
  for (int cycle = 0; cycle < 80; ++cycle) tick();
  assert(trace.counts["acks"] == 3);
  in.sm_valid[7] = true; in.sm[7] = {SALUT_MAX_CLIENT, 0};
  for (int cycle = 0; cycle < 80; ++cycle) tick();
  assert(trace.counts["acks"] == 4);
  in.sm_valid[2] = true; in.sm[2] = {SALUT_FRONT_CLIENT0, 0};
  for (int cycle = 0; cycle < 80; ++cycle) tick();
  assert(trace.counts["acks"] == 5);
  in.close = true; in.drained_ready = false; tick(); in.close = false;
  for (int cycle = 0; cycle < 10; ++cycle) tick();
  assert(out.drained_valid && !state.completed);
  in.drained_ready = true; tick(); assert(state.completed);

  for (const SalutPublishAck bad : {SalutPublishAck{0, 0}, {SALUT_MAX_CLIENT + 1, 0}, {1, 1}}) {
    state = {}; in = {}; in.sm_valid[0] = true; in.sm[0] = bad;
    for (int cycle = 0; cycle < 10; ++cycle) tick();
    assert(state.fault && !out.ack_valid);
  }
  state = {}; in = {}; in.lm_valid[0] = true; in.lm[0] = {SALUT_MAX_CLIENT, 0}; tick();
  assert(state.fault);
  state = {}; in = {}; in.sm_valid[0] = true; in.sm[0] = {1, 0}; tick();
  in.sm_valid[0] = true;
  for (int cycle = 0; cycle < 10; ++cycle) tick(); assert(state.fault);
  std::cout << "PUBLISH_TRACE acknowledgements=" << trace.counts.at("acks")
            << " missing_cache_ack=0\n";
  return trace;
}
static const bool registered = [] { RegisterMemoryScenario("publish", Publish); return true; }();
