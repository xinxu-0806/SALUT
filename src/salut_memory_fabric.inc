#pragma once
#include "salut_memory_protocol.h"

template <typename T> struct SalutSwitchState {
  T held[2]{};
  bool valid[2]{}, end[2]{}, rr[2]{}, end_sent[2]{};
  bool locked[2]{}, selected[2]{};
  bool fault = false;
};
template <typename T> struct SalutSwitchInputs {
  bool input_valid[2]{}, output_ready[2]{};
  T input_data[2]{};
};
template <typename T> struct SalutSwitchOutputs {
  bool input_consume[2]{}, output_valid[2]{};
  T output_data[2]{};
};

static inline uint32_t SalutDestination(const SalutReadEnvelope& p) {
#pragma HLS inline
  return SalutSmOwner(p.req.r.addr);
}
static inline uint32_t SalutDestination(const SalutWriteBeat& p) {
#pragma HLS inline
  return SalutSmOwner(p.address);
}
static inline uint32_t SalutDestination(const SalutResponseEnvelope& p) {
#pragma HLS inline
  const uint8_t client = uint8_t(p.rsp.pe);
  return SalutIsPeClient(client) ? uint32_t(SalutClientPeg(client)) : uint32_t(client) - uint32_t(SALUT_FRONT_CLIENT0);
}

// A bounded elastic switch; both outputs can transfer independently.
// Outputs describe pre-edge held state; input consumption refills at the edge.
template <typename T>
static inline void SalutSwitchTick(SalutSwitchState<T>& s,
                                  const SalutSwitchInputs<T>& in,
                                  SalutSwitchOutputs<T>& out, int stage) {
#pragma HLS inline
  out = {};
  if (stage < 0 || stage > 2) { s.fault = true; return; }
  if (s.fault) return;
  const bool draining = s.end[0] && s.end[1] && !s.valid[0] && !s.valid[1];
  for (int output = 0; output < 2; ++output) {
#pragma HLS unroll
    if (draining) {
      if (!s.end_sent[output]) {
        out.output_valid[output] = true;
        out.output_data[output].end = true;
        if (in.output_ready[output]) s.end_sent[output] = true;
      }
    } else {
      bool candidate[2]{};
      for (int input = 0; input < 2; ++input) {
#pragma HLS unroll
        candidate[input] = s.valid[input] &&
          (((SalutDestination(s.held[input]) >> (2 - stage)) & 1u) == uint32_t(output));
      }
      const int selected = s.locked[output] ? int(s.selected[output]) :
                           candidate[int(s.rr[output])] ? int(s.rr[output]) :
                           (candidate[0] ? 0 : (candidate[1] ? 1 : -1));
      if (selected >= 0) {
        out.output_valid[output] = true;
        out.output_data[output] = s.held[selected];
        s.locked[output] = true; s.selected[output] = selected;
        if (in.output_ready[output]) {
          s.valid[selected] = false;
          s.locked[output] = false;
          s.rr[output] = !selected;
        }
      }
    }
  }
  for (int input = 0; input < 2; ++input) {
#pragma HLS unroll
    if (s.end[input] || s.valid[input] || !in.input_valid[input]) continue;
    out.input_consume[input] = true;
    if (in.input_data[input].end) s.end[input] = true;
    else { s.held[input] = in.input_data[input]; s.valid[input] = true; }
  }
}

template <typename T> struct SalutMergeState {
  T held{};
  bool valid = false, end[2]{}, end_sent = false, rr = false;
};
template <typename T> static inline void SalutMergeTick(SalutMergeState<T>& s,
    const SalutSwitchInputs<T>& in, SalutSwitchOutputs<T>& out) {
#pragma HLS inline
  out = {};
  if (s.valid) {
    out.output_valid[0] = true; out.output_data[0] = s.held;
    if (in.output_ready[0]) s.valid = false;
  } else if (s.end[0] && s.end[1] && !s.end_sent) {
    out.output_valid[0] = true; out.output_data[0].end = true;
    if (in.output_ready[0]) s.end_sent = true;
  }
  if (s.valid || s.end_sent) return;
  // End is local to each input; consuming it cannot terminate the other.
  for (int i = 0; i < 2; ++i) {
#pragma HLS unroll
    if (!s.end[i] && in.input_valid[i] && in.input_data[i].end) {
      s.end[i] = true; out.input_consume[i] = true;
    }
  }
  const int preferred = int(s.rr);
  const int selected = !s.end[preferred] && in.input_valid[preferred] ? preferred :
                       (!s.end[!preferred] && in.input_valid[!preferred] ? !preferred : -1);
  if (selected >= 0) {
    s.held = in.input_data[selected]; s.valid = true;
    out.input_consume[selected] = true; s.rr = !selected;
  }
}

struct SalutBankSplitState {
  SalutReadEnvelope held{};
  bool valid = false, sent[2]{}, ended = false, fault = false;
};
struct SalutBankSplitInputs {
  SalutReadEnvelope data{}; bool valid = false, ready[2]{};
};
struct SalutBankSplitOutputs {
  SalutReadEnvelope data{}; bool consume = false, valid[2]{};
};
static inline void SalutBankSplitTick(SalutBankSplitState& s,
    const SalutBankSplitInputs& in, SalutBankSplitOutputs& out, int bank) {
#pragma HLS inline
  out = {};
  if (s.fault || s.ended) return;
  if (s.valid) {
    out.data = s.held;
    for (int port = 0; port < 2; ++port) {
#pragma HLS unroll
      const bool destined = s.held.end || int(s.held.req.r.kind == 0) == port;
      out.valid[port] = destined && !s.sent[port];
      if (out.valid[port] && in.ready[port]) s.sent[port] = true;
    }
    const bool sent = s.held.end ? s.sent[0] && s.sent[1] :
                      s.sent[int(s.held.req.r.kind == 0)];
    if (sent) { s.valid = false; if (s.held.end) s.ended = true; }
  }
  if (!s.valid && !s.ended && in.valid) {
    const auto& r = in.data.req.r;
    if (!in.data.end && (bank < 0 || bank > 7 || r.kind > 2 ||
        addr_bank(r.addr) != uint32_t(bank + 8 * r.kind) ||
        !SalutValidClient(uint8_t(in.data.req.pe)))) {
      s.fault = true; return;
    }
    s.held = in.data; s.valid = true; s.sent[0] = s.sent[1] = false;
    out.consume = true;
  }
}

struct SalutABurstControl {
  SalutReadEnvelope request{};
  SalutResponseEnvelope response{};
  uint32_t cursor = 0, remaining = 0, next_ticket = 0, expected_ticket = 0;
  ap_uint<3> phase = 0;
  bool fault = false;
};
struct SalutABurstInputs {
  SalutReadEnvelope request{}; SalutWordReadRsp fill{};
  bool request_valid = false, fill_valid = false, read_ready = false, response_ready = false;
};
struct SalutABurstOutputs {
  SalutWordReadReq read{}; SalutResponseEnvelope response{};
  bool request_consume = false, fill_consume = false, read_valid = false, response_valid = false;
};
static inline void SalutABurstTick(SalutABurstControl& s, const SalutABurstInputs& in,
    SalutABurstOutputs& out, int bank) {
#pragma HLS inline
  out = {};
  if (s.fault) return;
  switch (unsigned(s.phase)) {
    case 0:
      if (in.request_valid) {
        const auto& r = in.request.req.r;
        if (!in.request.end && (bank < 0 || bank > 7 || r.kind != 0 ||
            addr_bank(r.addr) != uint32_t(bank) || !SalutValidClient(uint8_t(in.request.req.pe)) ||
            uint64_t(addr_off(r.addr)) + r.len > BANK_SIZE)) {
          s.fault = true; break;
        }
        out.request_consume = true; s.request = in.request;
        s.cursor = r.addr; s.remaining = r.len;
        s.phase = in.request.end ? 4 : (r.len == 0 ? 0 : 1);
      }
      break;
    case 1:
      out.read_valid = true; out.read = {s.cursor & ~7u, s.next_ticket, false};
      if (in.read_ready) { s.expected_ticket = s.next_ticket++; s.phase = 2; }
      break;
    case 2:
      if (in.fill_valid) {
        out.fill_consume = true;
        if (in.fill.end || in.fill.ticket != s.expected_ticket) { s.fault = true; break; }
        const uint32_t first = s.cursor & 7u;
        const uint32_t count = s.remaining < 8 - first ? s.remaining : 8 - first;
        s.response = {{in.fill.data, s.request.req.pe, ap_uint<5>(first), ap_uint<7>(count)}, false};
        s.phase = 3;
      }
      break;
    case 3:
      out.response_valid = true; out.response = s.response;
      if (in.response_ready) {
        const uint32_t count = uint32_t(s.response.rsp.count);
        s.cursor += count; s.remaining -= count; s.phase = s.remaining == 0 ? 0 : 1;
      }
      break;
    case 4:
      out.read_valid = true; out.read.end = true;
      if (in.read_ready) s.phase = 5;
      break;
    case 5:
      if (in.fill_valid) {
        out.fill_consume = true;
        if (!in.fill.end) s.fault = true;
        else s.phase = 6;
      }
      break;
    case 6:
      out.response_valid = true; out.response.end = true;
      if (in.response_ready) s.phase = 7;
      break;
    default: break;
  }
}

struct SalutEgressOutputs {
  RP data{};
  bool consume = false, peg_valid = false, front_valid = false, end_valid = false, fault = false;
};
static inline SalutEgressOutputs SalutEgressTick(const SalutResponseEnvelope& p,
    bool valid, bool peg_ready, bool front_ready, bool end_ready, int lane) {
#pragma HLS inline
  SalutEgressOutputs out{};
  if (!valid) return out;
  if (p.end) { out.end_valid = true; out.consume = end_ready; return out; }
  if (lane < 0 || lane > 7 || !SalutValidClient(uint8_t(p.rsp.pe)) ||
      SalutDestination(p) != uint32_t(lane) || p.rsp.count == 0 ||
      uint32_t(p.rsp.first) + uint32_t(p.rsp.count) > 8) {
    out.fault = true; return out;
  }
  out.data = p.rsp; out.peg_valid = SalutIsPeClient(uint8_t(p.rsp.pe)); out.front_valid = !out.peg_valid;
  out.consume = out.peg_valid ? peg_ready : front_ready;
  return out;
}

struct SalutFrontReadControl {
  SalutReadEnvelope held{};
  bool valid = false, closed = false, end_sent = false, fault = false;
};
struct SalutFrontReadInputs {
  RR request{}; bool request_valid = false, close = false, ready = false;
};
struct SalutFrontReadOutputs {
  SalutReadEnvelope data{}; bool consume = false, valid = false;
};
static inline void SalutFrontReadTick(SalutFrontReadControl& s,
    const SalutFrontReadInputs& in, SalutFrontReadOutputs& out, int front) {
#pragma HLS inline
  out = {};
  if (s.fault || s.end_sent) return;
  s.closed = s.closed || in.close;
  if (s.valid) {
    out.valid = true; out.data = s.held;
    if (in.ready) s.valid = false;
  } else if (s.closed && !in.request_valid) {
    out.valid = true; out.data.end = true;
    if (in.ready) s.end_sent = true;
  }
  if (!s.valid && !s.end_sent && in.request_valid) {
    if (front < 0 || front > 7 || uint8_t(in.request.pe) != SALUT_FRONT_CLIENT0 + front) {
      s.fault = true; return;
    }
    s.held = {in.request, false}; s.valid = true; out.consume = true;
  }
}

struct SalutBeatizerControl {
  SalutCacheWriteReq request{};
  SalutWriteBeat held{};
  uint32_t remaining = 0, cursor = 0, ordinal = 0;
  bool active = false, held_valid = false, completed = false, fault = false;
};
struct SalutBeatizerInputs {
  SalutCacheWriteReq request{}; WP data{};
  bool request_valid = false, data_valid = false, ready = false;
};
struct SalutBeatizerOutputs {
  SalutWriteBeat data{};
  bool request_consume = false, data_consume = false, valid = false;
};
static inline void SalutBeatizerTick(SalutBeatizerControl& s,
    const SalutBeatizerInputs& in, SalutBeatizerOutputs& out) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  if (s.held_valid) {
    out.valid = true; out.data = s.held;
    if (in.ready) {
      s.held_valid = false;
      if (s.held.end) { s.completed = true; return; }
      if (s.held.last) s.active = false;
      else { s.cursor += 8; ++s.ordinal; }
    }
    return;
  }
  if (!s.active && in.request_valid) {
    if (!in.request.end && (!SalutValidFactorWrite(in.request.req) || in.request.seq >= (1u << 20))) {
      s.fault = true; return;
    }
    out.request_consume = true; s.request = in.request; s.active = true;
    s.remaining = in.request.req.r.len; s.cursor = in.request.req.r.addr; s.ordinal = 0;
    if (in.request.end || s.remaining == 0) {
      s.held = {WORD512(0), s.cursor, s.request.seq, 0, s.request.req.pe,
                s.request.req.r.kind, true, !in.request.end, in.request.end};
      s.held_valid = true;
    }
  } else if (s.active && in.data_valid) {
    if (in.data.pe != s.request.req.pe || in.data.count != 8 || s.remaining < 8) {
      s.fault = true; return;
    }
    s.held = {in.data.v, s.cursor, s.request.seq, s.ordinal, s.request.req.pe,
              s.request.req.r.kind, s.remaining == 8, false, false};
    s.remaining -= 8; s.held_valid = true; out.data_consume = true;
  }
}

struct SalutWriteMergeControl {
  SalutCacheWriteReq request{}; WP data{};
  uint32_t remaining = 0;
  ap_uint<2> phase = 0;
  bool end[2]{}, rr = false, active = false, completed = false, fault = false;
};
struct SalutWriteMergeInputs {
  // Keep the two physical input ports as separate packet latches. HLS 2022.2
  // emitted byte-offset zero writes into two-entry field RAMs for the nested
  // packet arrays, corrupting kind after the first front transaction.
  SalutCacheWriteReq request0{}, request1{}; WP data0{}, data1{};
  bool request_valid[2]{}, data_valid[2]{}, request_ready = false, data_ready = false;
};
struct SalutWriteMergeOutputs {
  SalutCacheWriteReq request{}; WP data{};
  bool request_consume[2]{}, data_consume[2]{}, request_valid = false, data_valid = false;
};
static inline void SalutWriteMergeTick(SalutWriteMergeControl& s,
    const SalutWriteMergeInputs& in, SalutWriteMergeOutputs& out) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  if (s.phase == 0) {
    for (int i = 0; i < 2; ++i) {
#pragma HLS unroll
      const bool end = i == 0 ? in.request0.end : in.request1.end;
      if (!s.end[i] && in.request_valid[i] && end) {
        s.end[i] = true; out.request_consume[i] = true;
      }
    }
    const int preferred = int(s.rr);
    const int selected = !s.end[preferred] && in.request_valid[preferred] ? preferred :
      (!s.end[!preferred] && in.request_valid[!preferred] ? !preferred : -1);
    if (selected >= 0) {
      const SalutCacheWriteReq request = selected == 0 ? in.request0 : in.request1;
      if (!SalutValidFactorWrite(request.req)) { s.fault = true; return; }
      s.request = request; s.remaining = s.request.req.r.len;
      s.active = selected; s.rr = !selected;
      out.request_consume[selected] = true; s.phase = 1;
    } else if (s.end[0] && s.end[1]) { s.request = {}; s.request.end = true; s.phase = 1; }
  } else if (s.phase == 1) {
    out.request_valid = true; out.request = s.request;
    if (in.request_ready) {
      if (s.request.end) s.completed = true;
      else s.phase = s.remaining == 0 ? 0 : 2;
    }
  } else if (s.phase == 2) {
    if (in.data_valid[int(s.active)]) {
      const WP p = s.active ? in.data1 : in.data0;
      if (p.pe != s.request.req.pe || p.count != 8 || s.remaining < 8) { s.fault = true; return; }
      s.data = p; s.remaining -= 8; out.data_consume[int(s.active)] = true; s.phase = 3;
    }
  } else {
    out.data_valid = true; out.data = s.data;
    if (in.data_ready) s.phase = s.remaining == 0 ? 0 : 2;
  }
}

#ifndef SALUT_MEMORY_CORE_ONLY
// Peeking avoids consuming an input until the production core can retain it.
template <typename T>
static void SalutSwitchAdapter(int stage, tapa::istream<T>& in0,
    tapa::istream<T>& in1, tapa::ostream<T>& out0, tapa::ostream<T>& out1) {
  SalutSwitchState<T> state{};
  while (!(state.end_sent[0] && state.end_sent[1])) {
#pragma HLS pipeline II=1
    SalutSwitchInputs<T> input{};
    input.input_valid[0] = in0.try_peek(input.input_data[0]);
    input.input_valid[1] = in1.try_peek(input.input_data[1]);
    input.output_ready[0] = !out0.full(); input.output_ready[1] = !out1.full();
    SalutSwitchOutputs<T> output{};
    SalutSwitchTick(state, input, output, stage);
    if (output.input_consume[0]) in0.read();
    if (output.input_consume[1]) in1.read();
    if (output.output_valid[0] && input.output_ready[0]) out0.write(output.output_data[0]);
    if (output.output_valid[1] && input.output_ready[1]) out1.write(output.output_data[1]);
  }
}
void SalutReadSwitch2(int stage, tapa::istream<SalutReadEnvelope>& in0,
    tapa::istream<SalutReadEnvelope>& in1, tapa::ostream<SalutReadEnvelope>& out0,
    tapa::ostream<SalutReadEnvelope>& out1) {
  SalutSwitchAdapter(stage, in0, in1, out0, out1);
}
void SalutWriteSwitch2(int stage, tapa::istream<SalutWriteBeat>& in0,
    tapa::istream<SalutWriteBeat>& in1, tapa::ostream<SalutWriteBeat>& out0,
    tapa::ostream<SalutWriteBeat>& out1) {
  SalutSwitchAdapter(stage, in0, in1, out0, out1);
}
void SalutResponseSwitch2(int stage, tapa::istream<SalutResponseEnvelope>& in0,
    tapa::istream<SalutResponseEnvelope>& in1, tapa::ostream<SalutResponseEnvelope>& out0,
    tapa::ostream<SalutResponseEnvelope>& out1) {
  SalutSwitchAdapter(stage, in0, in1, out0, out1);
}

template <typename T> static void SalutMergeAdapter(tapa::istream<T>& in0,
    tapa::istream<T>& in1, tapa::ostream<T>& out) {
  SalutMergeState<T> state{};
  while (!state.end_sent) {
#pragma HLS pipeline II=1
    SalutSwitchInputs<T> input{}; SalutSwitchOutputs<T> output{};
    input.input_valid[0] = in0.try_peek(input.input_data[0]);
    input.input_valid[1] = in1.try_peek(input.input_data[1]);
    input.output_ready[0] = !out.full();
    SalutMergeTick(state, input, output);
    if (output.input_consume[0]) in0.read();
    if (output.input_consume[1]) in1.read();
    if (output.output_valid[0] && input.output_ready[0]) out.write(output.output_data[0]);
  }
}
void SalutReadLaneMerge(tapa::istream<SalutReadEnvelope>& peg,
    tapa::istream<SalutReadEnvelope>& front, tapa::ostream<SalutReadEnvelope>& out) {
  SalutMergeAdapter(peg, front, out);
}
void SalutResponseLaneMerge(tapa::istream<SalutResponseEnvelope>& factor,
    tapa::istream<SalutResponseEnvelope>& a, tapa::ostream<SalutResponseEnvelope>& out) {
  SalutMergeAdapter(factor, a, out);
}
void SalutFrontReadSource(int front, tapa::istream<RR>& in,
    tapa::ostream<SalutReadEnvelope>& out, tapa::istream<ap_uint<1>>& close) {
  SalutFrontReadControl state{};
  while (!state.end_sent) {
#pragma HLS pipeline II=1
    SalutFrontReadInputs input{}; SalutFrontReadOutputs output{};
    ap_uint<1> signal;
    input.close = close.try_read(signal);
    input.request_valid = in.try_peek(input.request); input.ready = !out.full();
    SalutFrontReadTick(state, input, output, front);
    if (output.consume) in.read();
    if (output.valid && input.ready) out.write(output.data);
  }
}
void SalutRequestBankSplit(int bank, tapa::istream<SalutReadEnvelope>& in,
    tapa::ostream<SalutReadEnvelope>& factor, tapa::ostream<SalutReadEnvelope>& a) {
  SalutBankSplitState state{};
  while (!state.ended) {
#pragma HLS pipeline II=1
    SalutBankSplitInputs input{}; SalutBankSplitOutputs output{};
    input.valid = in.try_peek(input.data);
    input.ready[0] = !factor.full(); input.ready[1] = !a.full();
    SalutBankSplitTick(state, input, output, bank);
    if (output.consume) in.read();
    if (output.valid[0] && input.ready[0]) factor.write(output.data);
    if (output.valid[1] && input.ready[1]) a.write(output.data);
  }
}
void SalutABurstAdapter(int bank, tapa::istream<SalutReadEnvelope>& in,
    tapa::ostream<SalutWordReadReq>& rd, tapa::istream<SalutWordReadRsp>& rsp,
    tapa::ostream<SalutResponseEnvelope>& out) {
  SalutABurstControl state{};
  while (state.phase != 7) {
#pragma HLS pipeline II=1
    SalutABurstInputs input{}; SalutABurstOutputs output{};
    input.request_valid = in.try_peek(input.request);
    input.fill_valid = rsp.try_peek(input.fill);
    input.read_ready = !rd.full(); input.response_ready = !out.full();
    SalutABurstTick(state, input, output, bank);
    if (output.request_consume) in.read();
    if (output.fill_consume) rsp.read();
    if (output.read_valid && input.read_ready) rd.write(output.read);
    if (output.response_valid && input.response_ready) out.write(output.response);
  }
}
void SalutResponseEgress(int lane, tapa::istream<SalutResponseEnvelope>& in,
    tapa::ostream<RP>& peg, tapa::ostream<RP>& front, tapa::ostream<ap_uint<3>>& ended) {
  bool fault = false, closed = false;
  while (!closed) {
#pragma HLS pipeline II=1
    SalutResponseEnvelope input{};
    if (fault || closed) continue;
    const bool valid = in.try_peek(input);
    const auto output = SalutEgressTick(input, valid, !peg.full(), !front.full(), !ended.full(), lane);
    fault = output.fault;
    if (output.consume) {
      in.read();
      if (output.end_valid) { ended.write(ap_uint<3>(lane)); closed = true; }
      else if (output.peg_valid) peg.write(output.data);
      else if (output.front_valid) front.write(output.data);
    }
  }
}
void SalutWriteBeatizer(tapa::istream<SalutCacheWriteReq>& req, tapa::istream<WP>& data,
    tapa::ostream<SalutWriteBeat>& out) {
  SalutBeatizerControl state{};
  while (!state.completed) {
#pragma HLS pipeline II=1
    SalutBeatizerInputs input{}; SalutBeatizerOutputs output{};
    input.request_valid = req.try_peek(input.request); input.data_valid = data.try_peek(input.data);
    input.ready = !out.full(); SalutBeatizerTick(state, input, output);
    if (output.request_consume) req.read();
    if (output.data_consume) data.read();
    if (output.valid && input.ready) out.write(output.data);
  }
}
void SalutWriteLaneMerge(tapa::istream<SalutCacheWriteReq>& req0, tapa::istream<WP>& data0,
    tapa::istream<SalutCacheWriteReq>& req1, tapa::istream<WP>& data1,
    tapa::ostream<SalutCacheWriteReq>& req, tapa::ostream<WP>& data) {
  SalutWriteMergeControl state{};
  while (!state.completed) {
#pragma HLS pipeline II=4
    SalutWriteMergeInputs input{}; SalutWriteMergeOutputs output{};
    input.request_valid[0] = req0.try_peek(input.request0);
    input.request_valid[1] = req1.try_peek(input.request1);
    input.data_valid[0] = data0.try_peek(input.data0);
    input.data_valid[1] = data1.try_peek(input.data1);
    input.request_ready = !req.full(); input.data_ready = !data.full();
    SalutWriteMergeTick(state, input, output);
    if (output.request_consume[0]) req0.read();
    if (output.request_consume[1]) req1.read();
    if (output.data_consume[0]) data0.read();
    if (output.data_consume[1]) data1.read();
    if (output.request_valid && input.request_ready) req.write(output.request);
    if (output.data_valid && input.data_ready) data.write(output.data);
  }
}
#endif
