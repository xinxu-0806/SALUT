#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_fabric.inc"
#include <cassert>

// Breaks caught: modifying globally assigned seq, consuming the other
// producer's payload during a burst, End overtaking the surviving input.
static MemoryTrace MergeWrites() {
  SalutWriteMergeControl state{};
  SalutWriteMergeInputs in{}; SalutWriteMergeOutputs out{};
  in.request_valid[0] = in.request_valid[1] = true;
  in.request0 = {{{1, pack_addr(8, 0), 8}, 1}, 3, false};
  in.request1 = {{{2, pack_addr(16, 0), 0}, SALUT_FRONT_CLIENT0}, 42, false};
  SalutWriteMergeTick(state, in, out);
  assert(out.request_consume[0] && !out.request_consume[1]);
  in.request_valid[0] = false; in.request_ready = true;
  SalutWriteMergeTick(state, in, out);
  assert(out.request_valid && out.request.seq == 3);
  in.data_valid[0] = true; in.data0 = {WORD512(0x11), 1, 8};
  in.data_valid[1] = true; in.data1 = {WORD512(0xffff), SALUT_FRONT_CLIENT0, 8};
  SalutWriteMergeTick(state, in, out);
  assert(out.data_consume[0] && !out.data_consume[1]);
  in.data_valid[0] = in.data_valid[1] = false;
  SalutWriteMergeTick(state, in, out); assert(out.data_valid && out.data.v == WORD512(0x11));
  in.data_ready = true; SalutWriteMergeTick(state, in, out);
  SalutWriteMergeTick(state, in, out); assert(out.request_consume[1]);
  in.request_valid[1] = false; SalutWriteMergeTick(state, in, out);
  assert(out.request_valid && out.request.seq == 42 && out.request.req.pe == SALUT_FRONT_CLIENT0);
  in.request_valid[0] = true; in.request0.end = true;
  SalutWriteMergeTick(state, in, out); assert(out.request_consume[0]);
  in.request_valid[0] = false;
  for (int cycle = 0; cycle < 5; ++cycle) {
    SalutWriteMergeTick(state, in, out); assert(!out.request_valid);
  }
  in.request_valid[1] = true; in.request1.end = true;
  SalutWriteMergeTick(state, in, out); assert(out.request_consume[1]);
  in.request_valid[1] = false; SalutWriteMergeTick(state, in, out);
  assert(out.request_valid && out.request.end && state.completed && !state.fault);
  return {};
}
static const bool registered = [] { RegisterMemoryScenario("write_merge", MergeWrites); return true; }();
