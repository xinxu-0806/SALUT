#pragma once
#include "salut_memory_fabric.inc"
#include <array>
#include <cassert>
#include <deque>

// The production 12 switch cores and the product top's depth-4 links.
// Snapshot all heads/readiness first: no same-cycle shortcut across stages.
template <typename T> struct CoreOmega {
  std::deque<T> links[4][8];
  SalutSwitchState<T> state[3][4]{};
  void Tick() {
    SalutSwitchOutputs<T> result[3][4]{};
    bool ready[3][4][2]{};
    for (int stage = 0; stage < 3; ++stage) for (int sw = 0; sw < 4; ++sw) {
      SalutSwitchInputs<T> input{};
      for (int port = 0; port < 2; ++port) {
        const int i = 2*sw + port;
        const int j = stage < 2 ? ((i << 1) & 7) | (i >> 2) : i;
        input.input_valid[port] = !links[stage][i].empty();
        if (input.input_valid[port]) input.input_data[port] = links[stage][i].front();
        ready[stage][sw][port] = input.output_ready[port] = links[stage+1][j].size() < 4;
      }
      SalutSwitchTick(state[stage][sw], input, result[stage][sw], stage);
      assert(!state[stage][sw].fault);
    }
    for (int stage = 0; stage < 3; ++stage) for (int sw = 0; sw < 4; ++sw)
      for (int port = 0; port < 2; ++port)
        if (result[stage][sw].input_consume[port]) links[stage][2*sw+port].pop_front();
    for (int stage = 0; stage < 3; ++stage) for (int sw = 0; sw < 4; ++sw)
      for (int port = 0; port < 2; ++port) {
        const int i = 2*sw+port, j = stage < 2 ? ((i << 1) & 7) | (i >> 2) : i;
        if (result[stage][sw].output_valid[port] && ready[stage][sw][port])
          links[stage+1][j].push_back(result[stage][sw].output_data[port]);
      }
  }
};
