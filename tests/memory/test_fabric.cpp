#include "fixture.h"
#define SALUT_MEMORY_CORE_ONLY
#include "salut_memory_fabric.inc"
#include <algorithm>
#include <array>
#include <cassert>
#include <deque>
#include <iostream>

// Breaks caught: wrong stage bit/shuffle, global scalar arbitration,
// unfair contention, changing blocked payload, early/duplicated End.
template <typename T> T Packet(int destination, int source, int ordinal);
template <> SalutReadEnvelope Packet(int dst, int src, int ordinal) {
  return {{{1, pack_addr(8 + dst, ordinal * 8), uint32_t(ordinal + 1)},
           ap_uint<8>(src + 1)}, false};
}
template <> SalutWriteBeat Packet(int dst, int src, int ordinal) {
  return {WORD512(0x1234 + src * 128 + ordinal), pack_addr(16 + dst, ordinal * 8),
          uint32_t(src), uint32_t(ordinal), ap_uint<8>(src + 1), 2,
          true, false, false};
}
template <> SalutResponseEnvelope Packet(int dst, int src, int ordinal) {
  return {{WORD512(0x9876 + src * 128 + ordinal), ap_uint<8>(SALUT_FRONT_CLIENT0 + dst),
           ap_uint<5>(src), ap_uint<7>(ordinal + 1)}, false};
}
template <typename T> bool Equal(const T& a, const T& b);
template <> bool Equal(const SalutReadEnvelope& a, const SalutReadEnvelope& b) {
  return a.end == b.end && a.req.pe == b.req.pe && a.req.r.addr == b.req.r.addr &&
         a.req.r.kind == b.req.r.kind && a.req.r.len == b.req.r.len;
}
template <> bool Equal(const SalutWriteBeat& a, const SalutWriteBeat& b) {
  return a.end == b.end && a.data == b.data && a.address == b.address &&
         a.seq == b.seq && a.ordinal == b.ordinal && a.client == b.client &&
         a.kind == b.kind && a.last == b.last && a.no_data == b.no_data;
}
template <> bool Equal(const SalutResponseEnvelope& a, const SalutResponseEnvelope& b) {
  return a.end == b.end && a.rsp.v == b.rsp.v && a.rsp.pe == b.rsp.pe &&
         a.rsp.first == b.rsp.first && a.rsp.count == b.rsp.count;
}

template <typename T> void TestSwitch() {
  SalutSwitchState<T> state{};
  SalutSwitchInputs<T> in{};
  SalutSwitchOutputs<T> out{};
  in.input_valid[0] = in.input_valid[1] = true;
  in.input_data[0] = Packet<T>(0, 0, 0);
  in.input_data[1] = Packet<T>(4, 1, 0);
  SalutSwitchTick(state, in, out, 0);
  assert(out.input_consume[0] && out.input_consume[1]);
  in.input_valid[0] = in.input_valid[1] = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    SalutSwitchTick(state, in, out, 0);
    assert(out.output_valid[0] && out.output_valid[1]);
    assert(Equal(out.output_data[0], Packet<T>(0, 0, 0)));
    assert(Equal(out.output_data[1], Packet<T>(4, 1, 0)));
  }
  in.output_ready[0] = in.output_ready[1] = true;
  SalutSwitchTick(state, in, out, 0);
  assert(out.output_valid[0] && out.output_valid[1]);
  SalutSwitchTick(state, in, out, 0);
  assert(!out.output_valid[0] && !out.output_valid[1]);

  // A newly arriving preferred input must not replace an already offered,
  // blocked packet from the other input.
  state = {}; in = {};
  in.input_valid[1] = true; in.input_data[1] = Packet<T>(0, 1, 0);
  SalutSwitchTick(state, in, out, 0);
  in.input_valid[1] = false;
  SalutSwitchTick(state, in, out, 0);
  assert(out.output_valid[0] && Equal(out.output_data[0], Packet<T>(0, 1, 0)));
  in.input_valid[0] = true; in.input_data[0] = Packet<T>(0, 0, 0);
  SalutSwitchTick(state, in, out, 0);
  in.input_valid[0] = false;
  SalutSwitchTick(state, in, out, 0);
  assert(out.output_valid[0] && Equal(out.output_data[0], Packet<T>(0, 1, 0)));
  in.output_ready[0] = true;
  SalutSwitchTick(state, in, out, 0);
  assert(Equal(out.output_data[0], Packet<T>(0, 1, 0)));
  SalutSwitchTick(state, in, out, 0);
  assert(Equal(out.output_data[0], Packet<T>(0, 0, 0)));

  // Continuously replenish two streams contending for the same output.
  state = {}; in = {}; in.output_ready[0] = in.output_ready[1] = true;
  int sent[2] = {0, 0}, received[2] = {0, 0};
  for (int cycle = 0; cycle < 120; ++cycle) {
    for (int i = 0; i < 2; ++i) {
      in.input_valid[i] = sent[i] < 24;
      in.input_data[i] = Packet<T>(0, i, sent[i]);
    }
    SalutSwitchTick(state, in, out, 0);
    if (out.output_valid[0]) {
      const int source = Equal(out.output_data[0], Packet<T>(0, 0, received[0])) ? 0 : 1;
      assert(Equal(out.output_data[0], Packet<T>(0, source, received[source])));
      ++received[source];
      assert(std::abs(received[0] - received[1]) <= 1);
    }
    for (int i = 0; i < 2; ++i) sent[i] += out.input_consume[i];
  }
  assert(received[0] == 24 && received[1] == 24);

  state = {}; in = {};
  in.input_valid[0] = in.input_valid[1] = true;
  in.input_data[0].end = true;
  in.input_data[1] = Packet<T>(0, 1, 0);
  SalutSwitchTick(state, in, out, 0);
  in.input_valid[0] = false;
  in.input_data[1].end = true;
  SalutSwitchTick(state, in, out, 0);
  assert(!out.input_consume[1]); // Held data must precede that input's End.
  in.input_valid[1] = false;
  in.output_ready[0] = true;
  SalutSwitchTick(state, in, out, 0);
  assert(out.output_valid[0] && !out.output_data[0].end);
  in.input_valid[1] = true;
  SalutSwitchTick(state, in, out, 0);
  assert(out.input_consume[1]);
  in.input_valid[1] = false;
  SalutSwitchTick(state, in, out, 0);
  assert(out.output_valid[0] && out.output_data[0].end);
  assert(out.output_valid[1] && out.output_data[1].end);
  // output 1 remains blocked; output 0's End must not repeat.
  for (int cycle = 0; cycle < 5; ++cycle) {
    SalutSwitchTick(state, in, out, 0);
    assert(!out.output_valid[0] && out.output_valid[1]);
  }
  in.output_ready[1] = true;
  SalutSwitchTick(state, in, out, 0);
  SalutSwitchTick(state, in, out, 0);
  assert(!out.output_valid[0] && !out.output_valid[1] && !state.fault);
}

template <typename T> MemoryTrace TestNetwork(bool simultaneous) {
  std::deque<T> link[4][8];
  SalutSwitchState<T> state[3][4]{};
  MemoryTrace trace;
  std::deque<T> expected[8];
  if (simultaneous) {
    for (int src = 0; src < 8; ++src) {
      link[0][src].push_back(Packet<T>(src, src, 0));
      expected[src].push_back(Packet<T>(src, src, 0));
    }
  } else {
    for (int src = 0; src < 8; ++src)
      for (int dst = 0; dst < 8; ++dst)
        link[0][src].push_back(Packet<T>(dst, src, 0));
  }
  for (int i = 0; i < 8; ++i) { T end{}; end.end = true; link[0][i].push_back(end); }
  bool received[8][8]{};
  int last_source[8]; std::fill_n(last_source, 8, -1);
  int ended[8]{};
  for (int cycle = 0; cycle < 500; ++cycle) {
    SalutSwitchOutputs<T> outputs[3][4]{};
    for (int stage = 0; stage < 3; ++stage)
      for (int sw = 0; sw < 4; ++sw) {
        SalutSwitchInputs<T> in{};
        for (int port = 0; port < 2; ++port) {
          const int lane = 2 * sw + port;
          in.input_valid[port] = !link[stage][lane].empty();
          if (in.input_valid[port]) in.input_data[port] = link[stage][lane].front();
          const int target = stage == 2 ? lane : ((lane << 1) & 7) | (lane >> 2);
          in.output_ready[port] = link[stage + 1][target].size() < 4;
        }
        SalutSwitchTick(state[stage][sw], in, outputs[stage][sw], stage);
        assert(!state[stage][sw].fault);
      }
    uint64_t advanced = 0;
    for (int stage = 0; stage < 3; ++stage)
      for (int sw = 0; sw < 4; ++sw)
        for (int port = 0; port < 2; ++port) {
          const int lane = sw * 2 + port;
          const int target = stage == 2 ? lane : ((lane << 1) & 7) | (lane >> 2);
          const auto& out = outputs[stage][sw];
          if (out.input_consume[port]) link[stage][lane].pop_front();
          if (out.output_valid[port] && link[stage + 1][target].size() < 4) {
            link[stage + 1][target].push_back(out.output_data[port]); ++advanced;
          }
        }
    trace.counts["max_links_per_cycle"] = std::max(trace.counts["max_links_per_cycle"], advanced);
    for (int dst = 0; dst < 8; ++dst) {
      // Stall output0 long enough to fill upstream elastic links.
      if (link[3][dst].empty() || (dst == 0 && cycle < 80)) continue;
      const T packet = link[3][dst].front(); link[3][dst].pop_front();
      if (packet.end) { assert(++ended[dst] == 1); continue; }
      assert(!ended[dst]);
      int source = -1;
      for (int src = 0; src < 8; ++src)
        if (Equal(packet, Packet<T>(dst, src, 0))) source = src;
      assert(source >= 0 && !received[dst][source]);
      received[dst][source] = true;
      // FIFO per-source/per-destination is covered additionally by TestSwitch.
      last_source[dst] = source;
      if (dst != 0 && cycle < 80) ++trace.counts["delivered_while_output0_blocked"];
      ++trace.counts["delivered"];
    }
  }
  for (int dst = 0; dst < 8; ++dst) assert(ended[dst] == 1);
  assert(trace.counts["delivered"] == (simultaneous ? 8 : 64));
  assert(trace.counts["max_links_per_cycle"] > 1);
  assert(trace.counts["delivered_while_output0_blocked"] > 0);
  trace.counts["wrong_destination"] = trace.counts["reordered"] = 0;
  return trace;
}

static MemoryTrace Fabric() {
  assert(!RunMemoryScenario("fabric_adapters").fault);
  TestSwitch<SalutReadEnvelope>(); TestSwitch<SalutWriteBeat>();
  TestSwitch<SalutResponseEnvelope>();
  TestNetwork<SalutWriteBeat>(false); TestNetwork<SalutResponseEnvelope>(false);
  TestNetwork<SalutReadEnvelope>(true);
  const auto trace = TestNetwork<SalutReadEnvelope>(false);
  std::cout << "FABRIC_TRACE delivered=" << trace.counts.at("delivered")
            << " max_links=" << trace.counts.at("max_links_per_cycle") << '\n';
  return trace;
}
static const bool registered = [] {
  RegisterMemoryScenario("fabric", Fabric);
  RegisterMemoryScenario("fabric64", Fabric);
  return true;
}();
