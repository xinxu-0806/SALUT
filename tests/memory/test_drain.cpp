#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_commit.inc"
#include <cassert>

// Breaks caught: compute_done alone ending the kernel, omitting a bank/egress
// or publisher, duplicated/misidentified completion, sharing a close consumer.
static MemoryTrace Drain() {
  SalutDrainControl state{};
  SalutDrainInputs in{}; SalutDrainOutputs out{};
  MemoryTrace trace;
  in.compute_valid = true; in.compute = 1;
  SalutDrainTick(state, in, out); assert(out.compute_consume && !state.completed);
  in.compute_valid = false;
  for (int lane = 0; lane < 8; ++lane) {
    in.bank_valid[lane] = lane != 7; in.bank[lane] = lane;
    in.response_valid[lane] = true; in.response[lane] = lane;
  }
  in.publish_valid = true; in.publish = 1;
  SalutDrainTick(state, in, out); assert(!state.completed);
  for (int lane = 0; lane < 8; ++lane) {
    if (out.bank_consume[lane]) in.bank_valid[lane] = false;
    if (out.response_consume[lane]) in.response_valid[lane] = false;
  }
  in.publish_valid = false;
  for (int cycle = 0; cycle < 20; ++cycle) {
    SalutDrainTick(state, in, out);
    trace.counts["join_done_with_bank_missing"] += state.completed;
    assert(!state.completed);
  }
  in.bank_valid[7] = true;
  SalutDrainTick(state, in, out); assert(state.completed && !state.fault);
  trace.counts["join_done_after_last_bank"] += state.completed;
  for (int kind = 0; kind < 3; ++kind) {
    state = {}; in = {};
    if (kind == 0) { in.compute_valid = true; in.compute = 0; }
    if (kind == 1) { in.bank_valid[2] = true; in.bank[2] = 1; }
    if (kind == 2) { in.publish_valid = true; in.publish = 0; }
    SalutDrainTick(state, in, out); assert(state.fault && !state.completed);
  }
  state = {}; in = {}; in.response_valid[0] = true; in.response[0] = 0;
  SalutDrainTick(state, in, out);
  SalutDrainTick(state, in, out); assert(state.fault);

  // Validate the entire completion batch before retiring any FIFO head.
  // Besides fail-closed behavior, this removes a serial chain of early
  // returns across 16 unrolled lanes from the synthesized control path.
  state = {}; in = {};
  in.compute_valid = in.publish_valid = true; in.compute = in.publish = 1;
  for (int lane = 0; lane < 8; ++lane) {
    in.bank_valid[lane] = in.response_valid[lane] = true;
    in.bank[lane] = in.response[lane] = lane;
  }
  in.response[7] = 6;
  SalutDrainTick(state, in, out);
  assert(state.fault && !state.completed);
  assert(!out.compute_consume && !out.publish_consume);
  for (int lane = 0; lane < 8; ++lane)
    assert(!out.bank_consume[lane] && !out.response_consume[lane]);
  state = {}; in.response[7] = 7;
  SalutDrainTick(state, in, out);
  assert(state.completed && !state.fault);
  for (int lane = 0; lane < 8; ++lane)
    assert(out.bank_consume[lane] && out.response_consume[lane]);

  SalutCloseControl close{};
  SalutCloseInputs ci{}; SalutCloseOutputs co{};
  ci.compute_valid = true; ci.compute = 1; ci.publish_ready = ci.join_ready = true;
  for (int group = 0; group < 4; ++group)
    for (int lane = 0; lane < 8; ++lane) ci.ready[group][lane] = !(group == 3 && lane == 7);
  SalutCloseTick(close, ci, co); assert(co.compute_consume && !close.completed);
  ci.compute_valid = false;
  SalutCloseTick(close, ci, co);
  for (int group = 0; group < 4; ++group)
    for (int lane = 0; lane < 8; ++lane)
      assert(co.valid[group][lane] == (group == 3 && lane == 7));
  assert(!co.publish_valid && !co.join_valid);
  ci.ready[3][7] = true; SalutCloseTick(close, ci, co); assert(close.completed);
  const auto delayed = RunMemoryScenario("final_ack_delayed");
  assert(delayed.counts.at("ap_done_after_all_barriers") == 1);
  RunMemoryScenario("empty_read_repeat");
  return trace;
}
static const bool registered = [] {
  RegisterMemoryScenario("drain", Drain); return true;
}();
