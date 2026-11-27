#pragma once
#include "salut_memory_protocol.h"
#include "salut_stream_handshake.h"
#include "salut_memory_fabric.inc"
#ifndef __SYNTHESIS__
#include <memory>
#endif

struct SalutPublishControl {
  uint32_t next_seq[SALUT_MAX_CLIENT + 1]{};
  bool sm_seen[SALUT_MAX_CLIENT + 1]{}, lm_seen[SALUT_MAX_CLIENT + 1]{};
  ap_uint<3> sm_rr = 0, lm_rr = 0;
  uint8_t client_rr = 1;
  SalutPublishAck held{};
  bool held_valid = false, closed = false, completed = false, fault = false;
};
struct SalutPublishInputs {
  SalutPublishAck sm[8]{}, lm[8]{};
  bool sm_valid[8]{}, lm_valid[8]{};
  bool ack_ready = false, close = false, drained_ready = false;
};
struct SalutPublishOutputs {
  SalutPublishAck ack{};
  bool sm_consume[8]{}, lm_consume[8]{}, ack_valid = false, drained_valid = false;
};
static inline void SalutPublishTick(SalutPublishControl& s,
    const SalutPublishInputs& in, SalutPublishOutputs& out) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  s.closed = s.closed || in.close;
  if (s.held_valid) {
    out.ack_valid = true; out.ack = s.held;
    if (in.ack_ready) {
      const uint8_t client = uint8_t(s.held.client);
      ++s.next_seq[client]; s.sm_seen[client] = s.lm_seen[client] = false;
      s.held_valid = false;
    }
  }
  const int sm = unsigned(s.sm_rr), lm = unsigned(s.lm_rr);
  s.sm_rr = s.sm_rr + 1; s.lm_rr = s.lm_rr + 1;
  if (in.sm_valid[sm]) {
    const auto& ack = in.sm[sm]; const uint8_t client = uint8_t(ack.client);
    if (!SalutValidClient(client) || ack.seq != s.next_seq[client] || s.sm_seen[client]) {
      s.fault = true; return;
    }
    s.sm_seen[client] = true; out.sm_consume[sm] = true;
  }
  if (in.lm_valid[lm]) {
    const auto& ack = in.lm[lm]; const uint8_t client = uint8_t(ack.client);
    if (!SalutIsPeClient(client) || SalutClientPeg(client) != lm ||
        ack.seq != s.next_seq[client] || s.lm_seen[client]) {
      s.fault = true; return;
    }
    s.lm_seen[client] = true; out.lm_consume[lm] = true;
  }
  const uint8_t client = s.client_rr;
  s.client_rr = client == SALUT_MAX_CLIENT ? 1 : client + 1;
  if (!s.held_valid && s.sm_seen[client] && (!SalutIsPeClient(client) || s.lm_seen[client])) {
    s.held = {client, s.next_seq[client]}; s.held_valid = true;
  }
  bool empty = !s.held_valid;
  for (int i = 1; i <= SALUT_MAX_CLIENT; ++i) empty = empty && !s.sm_seen[i] && !s.lm_seen[i];
  for (int i = 0; i < 8; ++i) empty = empty && !in.sm_valid[i] && !in.lm_valid[i];
  if (s.closed && empty) {
    out.drained_valid = true;
    if (in.drained_ready) s.completed = true;
  }
}

struct SalutWriteForkControl {
  uint32_t next_seq[PAR_PE_PER_PEG]{}, remaining = 0;
  SalutCacheWriteReq request{}; WP data{};
  ap_uint<3> phase = 0;
  uint8_t rr = 0, active = 0;
  bool sent[2]{}, closed = false, completed = false, fault = false;
};
struct SalutWriteForkInputs {
  WR request[PAR_PE_PER_PEG]{}; WP data[PAR_PE_PER_PEG]{};
  bool request_valid[PAR_PE_PER_PEG]{}, data_valid[PAR_PE_PER_PEG]{}, request_ready[2]{}, data_ready[2]{};
  bool close = false;
};
struct SalutWriteForkOutputs {
  SalutCacheWriteReq request{}; WP data{};
  bool request_consume[PAR_PE_PER_PEG]{}, data_consume[PAR_PE_PER_PEG]{}, request_valid[2]{}, data_valid[2]{};
};
static inline void SalutWriteForkTick(SalutWriteForkControl& s,
    const SalutWriteForkInputs& in, SalutWriteForkOutputs& out,
    int first_client, int clients, uint32_t seq_limit) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  s.closed = s.closed || in.close;
  if (s.phase == 0) {
    int selected = -1;
    for (int offset = 0; offset < PAR_PE_PER_PEG; ++offset) {
      const int local = (s.rr + offset) & (PAR_PE_PER_PEG - 1);
      if (selected < 0 && local < clients && in.request_valid[local]) selected = local;
    }
    if (selected >= 0) {
      const WR request = in.request[selected];
      if (!SalutValidFactorWrite(request) || uint8_t(request.pe) != first_client + selected ||
          s.next_seq[selected] >= seq_limit) { s.fault = true; return; }
      // Exactly one assignment BEFORE either branch sees the header.
      s.request = {request, s.next_seq[selected]++, false}; s.active = selected;
      s.remaining = request.r.len; s.rr = (selected + 1) & (PAR_PE_PER_PEG - 1);
      s.sent[0] = s.sent[1] = false; s.phase = 1; out.request_consume[selected] = true;
    } else if (s.closed) {
      s.request = {}; s.request.end = true; s.sent[0] = s.sent[1] = false; s.phase = 4;
    }
  } else if (s.phase == 1 || s.phase == 4) {
    out.request = s.request;
    for (int branch = 0; branch < 2; ++branch) {
#pragma HLS unroll
      out.request_valid[branch] = !s.sent[branch];
      if (out.request_valid[branch] && in.request_ready[branch]) s.sent[branch] = true;
    }
    if (s.sent[0] && s.sent[1]) {
      if (s.phase == 4) s.completed = true;
      else s.phase = s.remaining == 0 ? 0 : 2;
    }
  } else if (s.phase == 2) {
    if (in.data_valid[s.active]) {
      const WP p = in.data[s.active];
      if (p.pe != s.request.req.pe || p.count != 8 || s.remaining < 8) { s.fault = true; return; }
      s.data = p; s.remaining -= 8; s.sent[0] = s.sent[1] = false;
      out.data_consume[s.active] = true; s.phase = 3;
    }
  } else if (s.phase == 3) {
    out.data = s.data;
    for (int branch = 0; branch < 2; ++branch) {
#pragma HLS unroll
      out.data_valid[branch] = !s.sent[branch];
      if (out.data_valid[branch] && in.data_ready[branch]) s.sent[branch] = true;
    }
    if (s.sent[0] && s.sent[1]) s.phase = s.remaining == 0 ? 0 : 2;
  }
}

struct SalutLMStorage {
  WORD512 data[2048];
  ap_uint<33> tag[2048];
  bool valid[2048], written[2048];
};
struct SalutLMControl {
#ifndef __SYNTHESIS__
  SalutCacheStats stats{};
#endif
  uint32_t init_cursor = 0;
  int peg = -1;
  bool initialized = false, fault = false, closed = false, snoop_end = false, completed = false;
  uint32_t next_seq[PAR_PE_PER_PEG]{};
  SalutCacheWriteReq write{};
  WP write_packet{};
  uint32_t write_remaining = 0, write_cursor = 0, write_address = 0;
  bool write_active = false;
  ap_uint<2> write_phase = 0;
  SalutPublishAck installed{};
  bool installed_held = false;
  ap_uint<SALUT_PEG_LOCAL_BITS> rr = 0, active_local = 0;
  ap_uint<3> read_phase = 0;
  RR read{};
  uint32_t cursor = 0, remaining = 0;
  RP response{};
  bool from_lower = false;
};
struct SalutLMInputs {
  RR request[PAR_PE_PER_PEG]{};
  RP lower_response{};
  SalutCacheWriteReq snoop{}; WP data{};
  bool request_valid[PAR_PE_PER_PEG]{}, response_ready[PAR_PE_PER_PEG]{};
  bool lower_ready = false, lower_response_valid = false;
  bool snoop_valid = false, data_valid = false, installed_ready = false, close = false;
};
struct SalutLMOutputs {
  SalutReadEnvelope lower{};
  RP response{}; SalutPublishAck installed{};
  bool request_consume[PAR_PE_PER_PEG]{}, response_valid[PAR_PE_PER_PEG]{};
  bool lower_valid = false, lower_response_consume = false;
  bool snoop_consume = false, data_consume = false, installed_valid = false;
};
static inline bool SalutLMClient(SalutLMControl& s, uint8_t client, int local) {
#pragma HLS inline
  if (!SalutIsPeClient(client) || SalutClientLocal(client) != local) return false;
  const int peg = SalutClientPeg(client);
  if (s.peg < 0) s.peg = peg;
  return s.peg == peg;
}
static inline void SalutLMInitialize(SalutLMControl& s,SalutLMStorage& m,bool close) {
#pragma HLS inline
  s.closed=s.closed || close;
  if(s.initialized || s.fault) return;
  m.valid[s.init_cursor]=m.written[s.init_cursor]=false;
  if(++s.init_cursor==2048) s.initialized=true;
}
static inline void SalutLMTick(SalutLMControl& s, SalutLMStorage& m,
    const SalutLMInputs& in, SalutLMOutputs& out) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  // The close FIFO carries a one-shot pulse, including during cold init.
  // Never discard it while clearing RAM metadata (notably n=0 launches).
  s.closed = s.closed || in.close;
  if (!s.initialized) {
    SalutLMInitialize(s,m,in.close);
    return;
  }
  if (s.installed_held) {
    out.installed_valid = true; out.installed = s.installed;
    if (in.installed_ready) {
      s.installed_held = false;
#ifndef __SYNTHESIS__
      ++s.stats.publications;
#endif
    }
  }
  bool writing_ram = false;
  if (s.write_phase == 1) {
    const uint32_t index = (addr_off(s.write_address) >> 3) & 2047u;
    m.data[index] = s.write_packet.v;
    m.tag[index] = SalutFactorKey(s.write.req.r.kind, s.write_address);
    m.valid[index] = m.written[index] = true;
#ifndef __SYNTHESIS__
    ++s.stats.installs;
#endif
    writing_ram = true; s.write_phase = 2;
  } else if (s.write_phase == 2) {
    if (s.write_remaining != 0) s.write_phase = 0;
    else if (!s.installed_held) {
      s.installed = {s.write.req.pe, s.write.seq}; s.installed_held = true;
      ++s.next_seq[SalutClientLocal(uint8_t(s.write.req.pe))];
      s.write_active = false; s.write_phase = 0;
    }
  } else {
    if (!s.write_active && !s.snoop_end && in.snoop_valid) {
      if (in.snoop.end) { s.snoop_end = true; out.snoop_consume = true; }
      else {
        const WR& wr = in.snoop.req;
        const int local = SalutClientLocal(uint8_t(wr.pe));
        if (!SalutValidFactorWrite(wr) || !SalutLMClient(s, uint8_t(wr.pe), local) ||
            in.snoop.seq != s.next_seq[local] || in.snoop.seq >= uint32_t(PAR_MAX_SEG)) {
          s.fault = true; return;
        }
        if (wr.r.len != 0 || !s.installed_held) {
          s.write = in.snoop; s.write_remaining = wr.r.len; s.write_cursor = wr.r.addr;
          s.write_active = wr.r.len != 0; out.snoop_consume = true;
          if (wr.r.len == 0) {
            s.installed = {wr.pe, in.snoop.seq}; s.installed_held = true; ++s.next_seq[local];
          }
        }
      }
    }
    if (s.write_active && in.data_valid) {
      if (in.data.pe != s.write.req.pe || in.data.count != 8 || s.write_remaining < 8) {
        s.fault = true; return;
      }
      s.write_packet = in.data; s.write_address = s.write_cursor;
      s.write_cursor += 8; s.write_remaining -= 8; s.write_phase = 1; out.data_consume = true;
    }
  }

  switch (unsigned(s.read_phase)) {
    case 0: {
      const int local = unsigned(s.rr); s.rr = s.rr + 1;
      if (in.request_valid[local]) {
        const RR rr = in.request[local];
        if (!SalutLMClient(s, uint8_t(rr.pe), local) || rr.r.kind > 2 ||
            addr_bank(rr.r.addr) / 8 != rr.r.kind || addr_bank(rr.r.addr) >= 24 ||
            uint64_t(addr_off(rr.r.addr)) + rr.r.len > BANK_SIZE) { s.fault = true; return; }
        s.read = rr; s.active_local = local; s.cursor = rr.r.addr; s.remaining = rr.r.len;
        out.request_consume[local] = true; s.from_lower = false;
        s.read_phase = rr.r.len == 0 ? 0 : (rr.r.kind == 0 ? 2 : 1);
      }
      break;
    }
    case 1: {
      if (writing_ram) break;
      const uint32_t index = (addr_off(s.cursor) >> 3) & 2047u;
      s.read_phase = m.valid[index] && m.tag[index] == SalutFactorKey(s.read.r.kind, s.cursor) ? 3 : 2;
      break;
    }
    case 2:
      out.lower_valid = true; out.lower = {{{s.read.r.kind, s.cursor, s.remaining}, s.read.pe}, false};
      if (in.lower_ready) {
        s.read_phase = 4; s.from_lower = true;
#ifndef __SYNTHESIS__
        if (s.read.r.kind == 0) ++s.stats.bypass_requests;
        else ++s.stats.misses;
#endif
      }
      break;
    case 3: {
      if (writing_ram) break;
      const uint32_t index = (addr_off(s.cursor) >> 3) & 2047u;
      // Revalidate here, not just before an earlier word or snoop replacement.
      if (!m.valid[index] || m.tag[index] != SalutFactorKey(s.read.r.kind, s.cursor)) {
        s.read_phase = 2; break;
      }
      const uint32_t first = s.cursor & 7u;
      const uint32_t count = s.remaining < 8 - first ? s.remaining : 8 - first;
      s.response = {m.data[index], s.read.pe, ap_uint<5>(first), ap_uint<7>(count)};
#ifndef __SYNTHESIS__
      ++s.stats.hits;
#endif
      s.read_phase = 5;
      break;
    }
    case 4:
      if (in.lower_response_valid) {
        const RP p = in.lower_response;
        const uint32_t first = s.cursor & 7u;
        const uint32_t count = s.remaining < 8 - first ? s.remaining : 8 - first;
        if (p.pe != s.read.pe || p.first != first || p.count != count) { s.fault = true; return; }
        s.response = p; out.lower_response_consume = true;
#ifndef __SYNTHESIS__
        s.stats.refill_words += s.read.r.kind != 0;
#endif
        s.read_phase = s.read.r.kind == 0 ? 5 : 6;
      }
      break;
    case 6: {
      if (writing_ram) break;
      const uint32_t index = (addr_off(s.cursor) >> 3) & 2047u;
      const auto key = SalutFactorKey(s.read.r.kind, s.cursor);
      if (m.valid[index] && m.tag[index] == key && m.written[index]) s.response.v = m.data[index];
      else {
        m.data[index] = s.response.v; m.tag[index] = key;
        m.valid[index] = true; m.written[index] = false;
      }
      s.read_phase = 5;
      break;
    }
    case 5:
      out.response_valid[unsigned(s.active_local)] = true; out.response = s.response;
      if (in.response_ready[unsigned(s.active_local)]) {
        s.cursor += uint32_t(s.response.count); s.remaining -= uint32_t(s.response.count);
        s.read_phase = s.remaining == 0 ? 0 : (s.from_lower ? 4 : 1);
      }
      break;
    default: s.fault = true; return;
  }
  bool no_requests = true;
  for (int i = 0; i < PAR_PE_PER_PEG; ++i) no_requests = no_requests && !in.request_valid[i];
  if (s.closed && s.snoop_end && s.read_phase == 0 && no_requests &&
      !s.write_active && s.write_phase == 0 && !s.installed_held) {
    out.lower_valid = true; out.lower = {}; out.lower.end = true;
    if (in.lower_ready) s.completed = true;
  }
}

struct SalutDrainControl {
  bool compute_seen = false, publish_seen = false;
  bool bank_seen[8]{}, response_seen[8]{};
  bool completed = false, fault = false;
};
struct SalutDrainInputs {
  ap_uint<1> compute = 0, publish = 0;
  ap_uint<3> bank[8]{}, response[8]{};
  bool compute_valid = false, publish_valid = false, bank_valid[8]{}, response_valid[8]{};
};
struct SalutDrainOutputs {
  bool compute_consume = false, publish_consume = false, bank_consume[8]{}, response_consume[8]{};
};
static inline void SalutDrainTick(SalutDrainControl& s,
    const SalutDrainInputs& in, SalutDrainOutputs& out) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  // Parallel prevalidation avoids a chain of early returns through every
  // unrolled bank/response lane. No invalid batch may retire a FIFO head.
  bool invalid = (in.compute_valid & (s.compute_seen | (in.compute != 1))) |
                 (in.publish_valid & (s.publish_seen | (in.publish != 1)));
  for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
    // Bitwise boolean reduction deliberately has NO C++ short-circuit CFG.
    invalid |= (in.bank_valid[lane] &
                (s.bank_seen[lane] | (in.bank[lane] != lane))) |
               (in.response_valid[lane] &
                (s.response_seen[lane] | (in.response[lane] != lane)));
  }
  if (invalid) { s.fault = true; return; }
  if (in.compute_valid) {
    s.compute_seen = true; out.compute_consume = true;
  }
  if (in.publish_valid) {
    s.publish_seen = true; out.publish_consume = true;
  }
  bool all = s.compute_seen & s.publish_seen;
  for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
    if (in.bank_valid[lane]) {
      s.bank_seen[lane] = true; out.bank_consume[lane] = true;
    }
    if (in.response_valid[lane]) {
      s.response_seen[lane] = true; out.response_consume[lane] = true;
    }
    all &= s.bank_seen[lane] & s.response_seen[lane];
  }
  s.completed = all;
}
struct SalutCloseControl {
  bool started = false, sent[4][8]{}, publish_sent = false, join_sent = false;
  bool completed = false, fault = false;
};
struct SalutCloseInputs {
  ap_uint<1> compute = 0;
  bool compute_valid = false, ready[4][8]{}, publish_ready = false, join_ready = false;
};
struct SalutCloseOutputs {
  bool compute_consume = false, valid[4][8]{}, publish_valid = false, join_valid = false;
};
static inline void SalutCloseTick(SalutCloseControl& s,
    const SalutCloseInputs& in, SalutCloseOutputs& out) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  if (!s.started && in.compute_valid) {
    if (in.compute != 1) { s.fault = true; return; }
    s.started = true; out.compute_consume = true;
  }
  if (!s.started) return;
  bool all = true;
  for (int group = 0; group < 4; ++group)
    for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
      out.valid[group][lane] = !s.sent[group][lane];
      if (out.valid[group][lane] && in.ready[group][lane]) s.sent[group][lane] = true;
      all = all && s.sent[group][lane];
    }
  out.publish_valid = !s.publish_sent; out.join_valid = !s.join_sent;
  if (out.publish_valid && in.publish_ready) s.publish_sent = true;
  if (out.join_valid && in.join_ready) s.join_sent = true;
  s.completed = all && s.publish_sent && s.join_sent;
}

#ifndef SALUT_MEMORY_CORE_ONLY
void SalutPublishJoin(tapa::istreams<SalutPublishAck, 8>& sm,
    tapa::istreams<SalutPublishAck, 8>& lm, tapa::ostream<SalutPublishAck>& ack,
    tapa::istream<ap_uint<1>>& close, tapa::ostream<ap_uint<1>>& drained) {
  SalutPublishControl state{};
  while (!state.completed) {
#pragma HLS pipeline II=1
    SalutPublishInputs input{}; SalutPublishOutputs output{};
    ap_uint<1> signal; input.close = close.try_read(signal);
    for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
      input.sm_valid[lane] = sm[lane].try_peek(input.sm[lane]);
      input.lm_valid[lane] = lm[lane].try_peek(input.lm[lane]);
    }
    input.ack_ready = !ack.full(); input.drained_ready = !drained.full();
    SalutPublishTick(state, input, output);
    for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
      if (output.sm_consume[lane]) sm[lane].read();
      if (output.lm_consume[lane]) lm[lane].read();
    }
    if (output.ack_valid && input.ack_ready) ack.write(output.ack);
    if (output.drained_valid && input.drained_ready) drained.write(ap_uint<1>(1));
  }
}
void SalutAckClientAdapter(tapa::istream<SalutPublishAck>& in, tapa::ostream<ap_uint<8>>& out) {
  bool fault = false;
  while (true) {
#pragma HLS pipeline II=1
    SalutPublishAck ack{};
    if (!fault && !out.full() && in.try_peek(ack)) {
      if (!SalutValidClient(uint8_t(ack.client))) { fault = true; continue; }
      in.read(); out.write(ack.client);
    }
  }
}
void SalutPegWriteFork(int peg, tapa::istreams<WR, PAR_PE_PER_PEG>& req, tapa::istreams<WP, PAR_PE_PER_PEG>& data,
    tapa::ostream<SalutCacheWriteReq>& global_req, tapa::ostream<WP>& global_data,
    tapa::ostream<SalutCacheWriteReq>& lm_req, tapa::ostream<WP>& lm_data,
    tapa::istream<ap_uint<1>>& close) {
  SalutWriteForkControl state{};
  while (!state.completed) {
#pragma HLS pipeline II=1
    SalutWriteForkInputs input{}; SalutWriteForkOutputs output{};
    ap_uint<1> signal; input.close = close.try_read(signal);
    for (int local = 0; local < PAR_PE_PER_PEG; ++local) {
#pragma HLS unroll
      input.request_valid[local] = req[local].try_peek(input.request[local]);
      input.data_valid[local] = data[local].try_peek(input.data[local]);
    }
    input.request_ready[0] = !global_req.full(); input.request_ready[1] = !lm_req.full();
    input.data_ready[0] = !global_data.full(); input.data_ready[1] = !lm_data.full();
    SalutWriteForkTick(state, input, output, peg * PAR_PE_PER_PEG + 1, PAR_PE_PER_PEG, PAR_MAX_SEG);
    for (int local = 0; local < PAR_PE_PER_PEG; ++local) {
#pragma HLS unroll
      if (output.request_consume[local]) req[local].read();
      if (output.data_consume[local]) data[local].read();
    }
    if (output.request_valid[0] && input.request_ready[0]) global_req.write(output.request);
    if (output.request_valid[1] && input.request_ready[1]) lm_req.write(output.request);
    if (output.data_valid[0] && input.data_ready[0]) global_data.write(output.data);
    if (output.data_valid[1] && input.data_ready[1]) lm_data.write(output.data);
  }
}
void SalutFrontWriteSource(int front, tapa::istream<WR>& req, tapa::istream<WP>& data,
    tapa::ostream<SalutCacheWriteReq>& out_req, tapa::ostream<WP>& out_data,
    tapa::istream<ap_uint<1>>& close) {
  SalutWriteForkControl state{};
  while (!state.completed) {
#pragma HLS pipeline II=1
    SalutWriteForkInputs input{}; SalutWriteForkOutputs output{};
    ap_uint<1> signal; input.close = close.try_read(signal);
    input.request_valid[0] = req.try_peek(input.request[0]); input.data_valid[0] = data.try_peek(input.data[0]);
    input.request_ready[0] = !out_req.full(); input.data_ready[0] = !out_data.full();
    input.request_ready[1] = input.data_ready[1] = true; // no LM for column fronts
    SalutWriteForkTick(state, input, output, SALUT_FRONT_CLIENT0 + front, 1, PAR_RUNTIME_MAX_N);
    if (output.request_consume[0]) req.read();
    if (output.data_consume[0]) data.read();
    if (output.request_valid[0] && input.request_ready[0]) out_req.write(output.request);
    if (output.data_valid[0] && input.data_ready[0]) out_data.write(output.data);
  }
}
void SalutLMLane(tapa::istreams<RR, PAR_PE_PER_PEG>& req, tapa::ostreams<RP, PAR_PE_PER_PEG>& rsp,
    tapa::ostream<SalutReadEnvelope>& lower_req, tapa::istream<RP>& lower_rsp,
    tapa::istream<SalutCacheWriteReq>& snoop, tapa::istream<WP>& data,
    tapa::ostream<SalutPublishAck>& installed, tapa::istream<ap_uint<1>>& close) {
  SalutLMControl state{};
#ifdef __SYNTHESIS__
  SalutLMStorage storage;
#else
  auto storage_owner = std::make_unique<SalutLMStorage>();
  auto& storage = *storage_owner;
#endif
#pragma HLS disaggregate variable=storage
#pragma HLS bind_storage variable=storage.data type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=storage.tag type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=storage.valid type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=storage.written type=RAM_2P impl=BRAM
  // Initialization must not inherit the runtime cache event pipeline's II.
  // Use the SAME reset core; close still has its single real FIFO consumer.
  for(int index=0;index<2048;++index) {
#pragma HLS pipeline II=1
    ap_uint<1> signal;
    SalutLMInitialize(state,storage,close.try_read(signal));
  }
  state.initialized=true;
  state.init_cursor=2048;
  while (!state.completed) {
#pragma HLS pipeline II=1
    SalutLMInputs input{}; SalutLMOutputs output{};
    ap_uint<1> signal; input.close = close.try_read(signal);
    for (int local = 0; local < PAR_PE_PER_PEG; ++local) {
#pragma HLS unroll
      input.request_valid[local] = req[local].try_peek(input.request[local]);
      input.response_ready[local] = !rsp[local].full();
    }
    input.snoop_valid = snoop.try_peek(input.snoop); input.data_valid = data.try_peek(input.data);
    input.lower_response_valid = lower_rsp.try_peek(input.lower_response);
    input.lower_ready = !lower_req.full(); input.installed_ready = !installed.full();
    SalutLMTick(state, storage, input, output);
#ifndef __SYNTHESIS__
    if (state.completed && state.peg >= 0) salut_lm_stats[state.peg] = state.stats;
#endif
    for (int local = 0; local < PAR_PE_PER_PEG; ++local) {
#pragma HLS unroll
      if (output.request_consume[local]) req[local].read();
      if (output.response_valid[local] && input.response_ready[local]) rsp[local].write(output.response);
    }
    if (output.snoop_consume) snoop.read();
    if (output.data_consume) data.read();
    if (output.lower_response_consume) lower_rsp.read();
    if (output.lower_valid && input.lower_ready) lower_req.write(output.lower);
    if (output.installed_valid && input.installed_ready) installed.write(output.installed);
  }
}
void SalutCompletionFanout(tapa::istream<ap_uint<1>>& compute_done,
    tapa::ostreams<ap_uint<1>, 8>& peg_rd_close, tapa::ostreams<ap_uint<1>, 8>& peg_wr_close,
    tapa::ostreams<ap_uint<1>, 8>& front_rd_close, tapa::ostreams<ap_uint<1>, 8>& front_wr_close,
    tapa::ostream<ap_uint<1>>& publish_close, tapa::ostream<ap_uint<1>>& join_close) {
  SalutCloseControl state{};
  while (!state.completed) {
#pragma HLS pipeline II=1
    SalutCloseInputs input{}; SalutCloseOutputs output{};
    input.compute_valid = compute_done.try_peek(input.compute);
    for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
      input.ready[0][lane] = !peg_rd_close[lane].full();
      input.ready[1][lane] = !peg_wr_close[lane].full();
      input.ready[2][lane] = !front_rd_close[lane].full();
      input.ready[3][lane] = !front_wr_close[lane].full();
    }
    input.publish_ready = !publish_close.full(); input.join_ready = !join_close.full();
    SalutCloseTick(state, input, output);
    if (output.compute_consume) compute_done.read();
    for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
      if (output.valid[0][lane] && input.ready[0][lane]) peg_rd_close[lane].write(ap_uint<1>(1));
      if (output.valid[1][lane] && input.ready[1][lane]) peg_wr_close[lane].write(ap_uint<1>(1));
      if (output.valid[2][lane] && input.ready[2][lane]) front_rd_close[lane].write(ap_uint<1>(1));
      if (output.valid[3][lane] && input.ready[3][lane]) front_wr_close[lane].write(ap_uint<1>(1));
    }
    if (output.publish_valid && input.publish_ready) publish_close.write(ap_uint<1>(1));
    if (output.join_valid && input.join_ready) join_close.write(ap_uint<1>(1));
  }
}
void SalutMemoryDrainJoin(tapa::istream<ap_uint<1>>& compute_done,
    tapa::istreams<ap_uint<3>, 8>& bank_done, tapa::istreams<ap_uint<3>, 8>& response_end,
    tapa::istream<ap_uint<1>>& publish_done) {
  SalutDrainControl state{};
#pragma HLS disaggregate variable=state
#pragma HLS array_partition variable=state.bank_seen complete
#pragma HLS array_partition variable=state.response_seen complete
  while (!state.completed || state.fault) {
#pragma HLS pipeline II=1
    SalutDrainInputs input{}; SalutDrainOutputs output{};
    input.compute_valid = compute_done.try_peek(input.compute);
    input.publish_valid = publish_done.try_peek(input.publish);
    for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
      input.bank_valid[lane] = bank_done[lane].try_peek(input.bank[lane]);
      input.response_valid[lane] = response_end[lane].try_peek(input.response[lane]);
    }
    SalutDrainTick(state, input, output);
    bool failures[18];
#pragma HLS array_partition variable=failures complete
    failures[0] = SalutConsume(compute_done,output.compute_consume,input.compute);
    failures[1] = SalutConsume(publish_done,output.publish_consume,input.publish);
    for (int lane = 0; lane < 8; ++lane) {
#pragma HLS unroll
      failures[2+lane] = SalutConsume(bank_done[lane],output.bank_consume[lane],input.bank[lane]);
      failures[10+lane] = SalutConsume(response_end[lane],output.response_consume[lane],input.response[lane]);
    }
    state.fault |= SalutAnyHandshakeFailure(failures);
  }
}
#endif
