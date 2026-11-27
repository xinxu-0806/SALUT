#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_commit.inc"
#include "salut_memory_fabric.inc"
#include <cassert>

// Breaks caught: numbering independently after the fork, losing one fork
// branch under backpressure, partial WP accepted as full word, wrong last.
static MemoryTrace WriteFlow() {
  SalutWriteForkControl fork{};
  SalutWriteForkInputs fi{}; SalutWriteForkOutputs fo{};
  fi.request_valid[0] = true; fi.request[0] = {{1, pack_addr(8, 0), 16}, 1};
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20); assert(fo.request_consume[0]);
  fi.request_valid[0] = false; fi.request_ready[0] = true;
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20);
  assert(fo.request_valid[0] && fo.request_valid[1] && fo.request.seq == 0);
  fi.request_ready[1] = true;
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20);
  assert(!fo.request_valid[0] && fo.request_valid[1] && fo.request.seq == 0);
  fi.data_valid[0] = true; fi.data[0] = {WORD512(0x11), 1, 8};
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20); assert(fo.data_consume[0]);
  fi.data_valid[0] = false; fi.data_ready[0] = true;
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20);
  assert(fo.data_valid[0] && fo.data_valid[1] && fo.data.v == WORD512(0x11));
  fi.data_ready[1] = true;
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20);
  assert(!fo.data_valid[0] && fo.data_valid[1]);
  fi.data_valid[0] = true; fi.data[0].v = WORD512(0x22);
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20); assert(fo.data_consume[0]);
  fi.data_valid[0] = false; SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20);
  assert(fo.data_valid[0] && fo.data_valid[1] && fo.data.v == WORD512(0x22));
  fi.request_valid[0] = true; fi.request[0].r.len = 0;
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20); assert(fo.request_consume[0]);
  fi.request_valid[0] = false;
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20);
  assert(fo.request_valid[0] && fo.request_valid[1] && fo.request.seq == 1);
  fi.close = true;
  SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20); fi.close = false;
  if (!fo.request_valid[0]) SalutWriteForkTick(fork, fi, fo, 1, PAR_PE_PER_PEG, 1u << 20);
  assert(fo.request.end && fo.request_valid[0] && fo.request_valid[1]);
  assert(fork.completed && !fork.fault);
  fork = {}; fi = {}; fork.next_seq[0] = 65536;
  fi.request_valid[0] = true; fi.request[0] = {{2, pack_addr(16, 0), 0}, SALUT_FRONT_CLIENT0};
  SalutWriteForkTick(fork, fi, fo, SALUT_FRONT_CLIENT0, 1, 65536); assert(fork.fault);

  SalutBeatizerControl beat{};
  SalutBeatizerInputs bi{}; SalutBeatizerOutputs bo{};
  bi.request_valid = true; bi.request = {{{2, pack_addr(16, 16), 16}, SALUT_FRONT_CLIENT0}, 7, false};
  SalutBeatizerTick(beat, bi, bo); assert(bo.request_consume);
  bi.request_valid = false; bi.data_valid = true; bi.data = {WORD512(0xaa), SALUT_FRONT_CLIENT0, 8};
  SalutBeatizerTick(beat, bi, bo); assert(bo.data_consume); bi.data_valid = false;
  for (int cycle = 0; cycle < 5; ++cycle) {
    SalutBeatizerTick(beat, bi, bo);
    assert(bo.valid && bo.data.data == WORD512(0xaa) && bo.data.seq == 7 &&
           bo.data.ordinal == 0 && bo.data.address == pack_addr(16, 16) && !bo.data.last);
  }
  bi.ready = true; SalutBeatizerTick(beat, bi, bo);
  bi.data_valid = true; bi.data.v = WORD512(0xbb);
  SalutBeatizerTick(beat, bi, bo); assert(bo.data_consume); bi.data_valid = false;
  SalutBeatizerTick(beat, bi, bo);
  assert(bo.valid && bo.data.last && bo.data.ordinal == 1 && bo.data.address == pack_addr(16, 24));
  bi.request_valid = true; bi.request.req.r.len = 0; bi.request.seq = 8;
  SalutBeatizerTick(beat, bi, bo); assert(bo.request_consume); bi.request_valid = false;
  SalutBeatizerTick(beat, bi, bo);
  assert(bo.valid && bo.data.no_data && bo.data.last && !bo.data.end && bo.data.seq == 8);
  bi.request_valid = true; bi.request.end = true;
  SalutBeatizerTick(beat, bi, bo); bi.request_valid = false;
  SalutBeatizerTick(beat, bi, bo); assert(bo.valid && bo.data.end && beat.completed);
  beat = {}; bi = {}; bi.request_valid = true;
  bi.request = {{{1, pack_addr(8, 0), 8}, 1}, 0, false};
  SalutBeatizerTick(beat, bi, bo); bi.request_valid = false;
  bi.data_valid = true; bi.data = {WORD512(0), 1, 7};
  SalutBeatizerTick(beat, bi, bo); assert(beat.fault && !bo.valid);
  return {};
}
static const bool registered = [] { RegisterMemoryScenario("write_flow", WriteFlow); return true; }();
