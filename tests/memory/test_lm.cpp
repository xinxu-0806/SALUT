#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_commit.inc"
#include <cassert>
#include <deque>
#include <memory>
#include <vector>

struct LmFixture {
  SalutLMControl control{};
  std::unique_ptr<SalutLMStorage> storage = std::make_unique<SalutLMStorage>();
  SalutLMInputs input{}; SalutLMOutputs output{};
  std::deque<SalutReadEnvelope> lower;
  std::vector<RP> responses[PAR_PE_PER_PEG];
  std::vector<SalutPublishAck> installed;
  LmFixture() {
    input.lower_ready = input.installed_ready = true;
    for (int i = 0; i < PAR_PE_PER_PEG; ++i) input.response_ready[i] = true;
    for (int i = 0; i < 2100 && !control.initialized; ++i) Tick();
    assert(control.initialized && !control.fault);
  }
  void Tick() {
    SalutLMTick(control, *storage, input, output);
    for (int p = 0; p < PAR_PE_PER_PEG; ++p) {
      if (output.request_consume[p]) input.request_valid[p] = false;
      if (output.response_valid[p] && input.response_ready[p]) responses[p].push_back(output.response);
    }
    if (output.snoop_consume) input.snoop_valid = false;
    if (output.data_consume) input.data_valid = false;
    if (output.lower_response_consume) input.lower_response_valid = false;
    if (output.installed_valid && input.installed_ready) installed.push_back(output.installed);
    if (output.lower_valid && input.lower_ready) lower.push_back(output.lower);
  }
  void Ticks(int count = 16) { for (int i = 0; i < count; ++i) Tick(); }
  void Write(uint8_t client, uint32_t seq, uint8_t kind, uint32_t address, WORD512 data) {
    input.snoop = {{{kind, address, 8}, client}, seq, false}; input.snoop_valid = true;
    for (int i = 0; i < 50 && input.snoop_valid; ++i) Tick();
    assert(!input.snoop_valid && !control.fault);
    input.data = {data, client, 8}; input.data_valid = true;
    for (int i = 0; i < 50 && input.data_valid; ++i) Tick();
    Ticks(); assert(!control.fault && !input.data_valid);
    assert(installed.back().client == client && installed.back().seq == seq);
  }
  void Read(int local, uint32_t address, uint8_t kind, uint32_t len = 8) {
    input.request[local] = {{kind, address, len}, ap_uint<8>(local + 1)};
    input.request_valid[local] = true; Ticks(30);
    assert(!input.request_valid[local] && !control.fault);
  }
};

// Breaks caught: U omitted from LM, speculative whole-burst hit scan, stale
// lower fill overwriting a concurrent snoop, starvation, tags retained at launch.
static MemoryTrace Lm() {
  {
    SalutLMControl init{};
    auto ram=std::make_unique<SalutLMStorage>();
    for(int index=0;index<2048;++index) {
      ram->valid[index]=ram->written[index]=true;
      ram->tag[index]=ap_uint<33>(0x12345678);
      ram->data[index]=WORD512(0xabcdef);
    }
    for(int tick=0;tick<2048;++tick) {
      SalutLMInitialize(init,*ram,tick==0);
      assert(init.closed && init.initialized==(tick==2047) && !init.fault);
      assert(!ram->valid[tick] && !ram->written[tick]);
      assert(ram->tag[tick]==ap_uint<33>(0x12345678) && ram->data[tick]==WORD512(0xabcdef));
      if(tick<2047) assert(ram->valid[tick+1] && ram->written[tick+1]);
    }
  }
  // n=0 can deliver its close pulse BEFORE the RAM initialization completes.
  // The adapter pops this one token once, so the core must retain it.
  SalutLMControl cold{};
  auto cold_ram = std::make_unique<SalutLMStorage>();
  SalutLMInputs cold_in{}; SalutLMOutputs cold_out{};
  cold_in.close = true;
  SalutLMTick(cold,*cold_ram,cold_in,cold_out);
  assert(cold.closed && !cold.initialized);
  LmFixture f;
  f.Write(1, 0, 1, pack_addr(8, 0), WORD512(0x11));
  f.Read(0, pack_addr(8, 3), 1, 2);
  assert(f.responses[0].size() == 1 && f.responses[0].back().v == WORD512(0x11));
  assert(f.responses[0].back().first == 3 && f.responses[0].back().count == 2 && f.lower.empty());
  f.Write(1, 1, 2, pack_addr(16, 8), WORD512(0x22));
  f.Read(0, pack_addr(16, 8), 2); assert(f.responses[0].back().v == WORD512(0x22));
  f.Read(0, pack_addr(16, 40), 2);
  assert(f.lower.size() == 1 && f.lower.front().req.r.addr == pack_addr(16, 40));
  f.lower.pop_front(); f.input.lower_response = {WORD512(0x33), 1, 0, 8};
  f.input.lower_response_valid = true; f.Ticks();
  assert(f.responses[0].back().v == WORD512(0x33));
  f.Read(0, pack_addr(16, 40), 2); assert(f.lower.empty() && f.responses[0].back().v == WORD512(0x33));
  assert(f.control.stats.hits == 3 && f.control.stats.misses == 1);
  assert(f.control.stats.installs == 2 && f.control.stats.refill_words == 1);
  LmFixture software_sequence;
  software_sequence.control.next_seq[0] = 1u<<20;
  software_sequence.Write(1,1u<<20,1,pack_addr(8,0),WORD512(0xa4));
  f.Read(0, pack_addr(16, 80), 2); f.lower.pop_front();
  f.Write(1, 2, 2, pack_addr(16, 80), WORD512(0x44));
  f.input.lower_response = {WORD512(0xdead), 1, 0, 8}; f.input.lower_response_valid = true;
  f.Ticks(); assert(f.responses[0].back().v == WORD512(0x44));
  f.input.snoop = {{{1, pack_addr(8, 0), 0}, 1}, 3, false};
  f.input.snoop_valid = true; f.Ticks();
  assert(f.installed.back().seq == 3 && !f.input.data_valid);
  f.Read(0, pack_addr(0, 0), 0); assert(f.lower.front().req.r.kind == 0);
  f.lower.pop_front(); f.input.lower_response = {WORD512(0xb0), 1, 0, 8};
  f.input.lower_response_valid = true; f.Ticks();
  assert(f.responses[0].back().v == WORD512(0xb0));
  assert(f.storage->tag[0] == SalutFactorKey(1, pack_addr(8, 0)));
  f.Read(0, pack_addr(0, 0), 0); assert(f.lower.size() == 1); // A bypasses cache twice.

  LmFixture suffix;
  suffix.Write(1, 0, 1, pack_addr(8, 16), WORD512(0x55));
  suffix.Write(1, 1, 1, pack_addr(8, 24), WORD512(0x66));
  suffix.input.request[0] = {{1, pack_addr(8, 16), 16}, 1};
  suffix.input.request_valid[0] = true;
  for (int i = 0; i < 50 && suffix.responses[0].empty(); ++i) suffix.Tick();
  assert(suffix.responses[0].size() == 1 && suffix.responses[0].back().v == WORD512(0x55));
  // Replace the second line between first-word emission and its RAM read.
  // Both FIFO heads may already be available when this actor is scheduled;
  // this reserves the snoop before the next word's actual RAM read.
  // The writer is the last PE of the same PEG (client PAR_PE_PER_PEG).
  suffix.input.snoop = {{{2, pack_addr(16, 16384 + 24), 8}, ap_uint<8>(PAR_PE_PER_PEG)}, 0, false};
  suffix.input.snoop_valid = true;
  suffix.input.data = {WORD512(0x77), ap_uint<8>(PAR_PE_PER_PEG), 8}; suffix.input.data_valid = true;
  suffix.Ticks();
  assert(suffix.lower.size() == 1 && suffix.lower.front().req.r.addr == pack_addr(8, 24));
  assert(suffix.lower.front().req.r.len == 8);
  suffix.input.lower_response = {WORD512(0x66), 1, 0, 8}; suffix.input.lower_response_valid = true;
  suffix.Ticks(); assert(suffix.responses[0].size() == 2 && suffix.responses[0].back().v == WORD512(0x66));

  LmFixture fair;
  fair.Write(1, 0, 1, pack_addr(8, 0), WORD512(0x88));
  for (int local = 0; local < PAR_PE_PER_PEG; ++local) {
    fair.input.request[local] = {{1, pack_addr(8, 0), 8}, ap_uint<8>(local + 1)};
    fair.input.request_valid[local] = true;
  }
  fair.Ticks(80);
  for (int local = 0; local < PAR_PE_PER_PEG; ++local)
    assert(fair.responses[local].size() == 1 && fair.responses[local].back().pe == local + 1);
  fair.input.snoop_valid = true; fair.input.snoop = {}; fair.input.snoop.end = true;
  fair.input.close = true; fair.Ticks();
  assert(fair.control.completed && fair.lower.back().end);
  fair.input.close = false; fair.lower.clear(); fair.control = {};
  for (int i = 0; i < 2100 && !fair.control.initialized; ++i) fair.Tick();
  fair.Read(0, pack_addr(8, 0), 1); assert(fair.lower.size() == 1 && !fair.lower.front().end);
  LmFixture close_pending;
  close_pending.Read(0, pack_addr(16, 0), 2);
  close_pending.lower.clear(); close_pending.input.snoop = {}; close_pending.input.snoop.end = true;
  close_pending.input.snoop_valid = true; close_pending.input.close = true;
  close_pending.Ticks(); assert(!close_pending.control.completed && close_pending.lower.empty());
  close_pending.input.response_ready[0] = false;
  close_pending.input.lower_response = {WORD512(0xc0), 1, 0, 8};
  close_pending.input.lower_response_valid = true; close_pending.Ticks();
  assert(!close_pending.control.completed && close_pending.lower.empty());
  close_pending.input.response_ready[0] = true; close_pending.Ticks();
  assert(close_pending.control.completed && close_pending.lower.back().end);
  LmFixture high_client;
  // Highest PE client (last PEG, last local port).
  constexpr int last_local = PAR_PE_PER_PEG - 1;
  high_client.Write(PAR_NUM_PE, 0, 1, pack_addr(15, 64), WORD512(0xd0));
  high_client.input.request[last_local] = {{1, pack_addr(15, 64), 8}, ap_uint<8>(PAR_NUM_PE)};
  high_client.input.request_valid[last_local] = true; high_client.Ticks(40);
  assert(high_client.responses[last_local].size() == 1 &&
         high_client.responses[last_local].back().pe == PAR_NUM_PE);
  MemoryTrace trace; trace.counts["peg_local_pe_served"] = PAR_PE_PER_PEG;
  return trace;
}
static const bool registered = [] { RegisterMemoryScenario("lm", Lm); return true; }();
