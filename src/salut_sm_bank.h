#pragma once
#include "salut_memory_protocol.h"

struct SalutSmWritebackSlot {
  ap_uint<33> key{};
  uint32_t address = 0, ticket = 0, index = 0;
  ap_uint<16> version = 0;
  bool valid = false, ready = false, issued = false;
};
struct SalutSmBankStorage {
  WORD512 data[4096];
  ap_uint<33> tag[4096];
  bool valid[4096], dirty[4096], written[4096];
  ap_uint<16> version[4096];
  SalutSmWritebackSlot wb[2][8];
  // One independent 1R/1W snapshot RAM for each physical L/U port. Metadata
  // is shallow register state; do not partition 512-bit payload into the
  // same multi-writer control/SSA graph as valid/ready/issued.
  WORD512 wb_data[2][8];
};
struct SalutSmReadContext {
  RR req{};
  uint32_t cursor = 0, remaining = 0;
  bool active = false, waiting = false;
};
struct SalutSmBankControl {
#ifndef __SYNTHESIS__
  SalutCacheStats stats{};
#endif
  int bank = 0;
  uint32_t init_cursor = 0, dirty_count = 0;
  bool initialized = false, fault = false;
  // {} default-constructs ap_uint elements; the synthesis constructor leaves
  // their bits UNDEFINED. These are live state even on a cold empty launch,
  // so give every pointer/count/generation an explicit integer initializer.
  ap_uint<3> queue_head[2]{0,0}, queue_tail[2]{0,0}, queue_issue[2]{0,0};
  ap_uint<4> queue_count[2]{0,0};
  ap_uint<16> generations[2][8]{{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
  uint32_t next_seq[SALUT_MAX_CLIENT + 1]{}, active_seq[SALUT_MAX_CLIENT + 1]{};
  uint32_t next_ordinal[SALUT_MAX_CLIENT + 1]{}, next_address[SALUT_MAX_CLIENT + 1]{};
  uint8_t burst_kind[SALUT_MAX_CLIENT + 1]{};
  SalutSmReadContext reads[2]{};
  bool read_rr = false, prefer_read = false;
  bool pending = false, pending_sent = false, fill_held = false;
  uint8_t pending_context = 0, pending_port = 0;
  uint32_t pending_index = 0, pending_address = 0, pending_ticket = 0, next_ticket = 0;
  ap_uint<33> pending_key{};
  ap_uint<16> pending_version = 0;
  WORD512 fill_data{};
  bool done_held[2]{}, done_rr = false;
  SalutWritebackDone held_done[2]{};
  SalutResponseEnvelope response{};
  SalutPublishAck publish{};
  uint8_t response_context = 0;
  bool response_held = false, publish_held = false;
  // 1 reserve->RAM, 2 RAM-visible->WB ready, 3 hit RAM read,
  // 4 registered data->response, 5 fill RAM install, 6 fill-visible response.
  ap_uint<3> stage = 0;
  SalutWriteBeat stage_write{};
  uint32_t stage_index = 0;
  uint8_t stage_port = 0, stage_slot = 0, stage_context = 0;
  ap_uint<33> stage_key{};
  ap_uint<16> stage_version = 0;
  WORD512 stage_data{};
  bool request_end = false, write_end = false;
  bool read_end_sent[2]{}, read_end_ack[2]{}, wb_end_sent[2]{}, wb_end_ack[2]{};
  bool response_end_sent = false, drained_sent = false, completed = false;
};
struct SalutSmBankInputs {
  SalutReadEnvelope request{};
  SalutWriteBeat write{};
  SalutWordReadRsp fill[2]{};
  SalutWritebackDone done[2]{};
  bool request_valid = false, write_valid = false, fill_valid[2]{}, done_valid[2]{};
  bool read_ready[2]{}, writeback_ready[2]{};
  bool response_ready = false, publish_ready = false, drained_ready = false;
};
struct SalutSmBankOutputs {
  SalutWordReadReq read[2]{};
  SalutWritebackWord writeback[2]{};
  SalutResponseEnvelope response{};
  SalutPublishAck publish{};
  ap_uint<3> drained = 0;
  bool request_consume = false, write_consume = false, fill_consume[2]{}, done_consume[2]{};
  bool read_valid[2]{}, writeback_valid[2]{};
  bool response_valid = false, publish_valid = false, drained_valid = false;
};

static inline void SalutSmMakeResponse(SalutSmBankControl& s, uint8_t context, WORD512 data) {
#pragma HLS inline
  const auto& r = s.reads[context];
  const uint32_t first = r.cursor & 7u;
  const uint32_t count = r.remaining < 8 - first ? r.remaining : 8 - first;
  s.response = {{data, r.req.pe, ap_uint<5>(first), ap_uint<7>(count)}, false};
  s.response_context = context; s.response_held = true;
}
static inline void SalutSmFinishWrite(SalutSmBankControl& s, const SalutWriteBeat& p) {
#pragma HLS inline
  const uint8_t client = uint8_t(p.client);
  if (p.last) {
    s.publish = {p.client, p.seq}; s.publish_held = true;
    s.next_seq[client] = p.seq + 1; s.next_ordinal[client] = 0;
  } else {
    s.active_seq[client] = p.seq;
    s.next_ordinal[client] = p.ordinal + 1; s.next_address[client] = p.address + 8;
    s.burst_kind[client] = uint8_t(p.kind);
  }
}
static inline bool SalutSmStartRead(SalutSmBankControl& s, SalutSmBankStorage& m) {
#pragma HLS inline
  const int ctx = int(s.read_rr); s.read_rr = !s.read_rr;
  auto& r = s.reads[ctx];
  if (!r.active || r.waiting || s.response_held) return false;
  const uint32_t index = SalutSmIndex(r.cursor);
  const auto key = SalutFactorKey(r.req.r.kind, r.cursor);
  if (m.valid[index] && m.tag[index] == key) {
#ifndef __SYNTHESIS__
    ++s.stats.hits; s.stats.dirty_hits += m.dirty[index];
#endif
    s.stage = 3; s.stage_index = index; s.stage_context = ctx;
    s.stage_key = key; r.waiting = true; s.prefer_read = false;
    return true;
  }
  if (m.dirty[index] || s.pending) {
#ifndef __SYNTHESIS__
    s.stats.capacity_stall_ticks += m.dirty[index];
#endif
    return false;
  }
#ifndef __SYNTHESIS__
  ++s.stats.misses;
#endif
  s.pending = true; s.pending_sent = false; s.pending_context = ctx;
  s.pending_port = r.req.r.kind - 1; s.pending_index = index;
  s.pending_address = r.cursor & ~7u; s.pending_key = key;
  s.pending_version = m.version[index]; s.pending_ticket = s.next_ticket++;
  r.waiting = true; s.prefer_read = false;
  return true;
}

static inline void SalutSmInitialize(SalutSmBankControl& s, SalutSmBankStorage& m) {
#pragma HLS inline
  if(s.initialized || s.fault) return;
  if(s.bank<0 || s.bank>7) {s.fault=true;return;}
  const uint32_t i=s.init_cursor++;
  m.valid[i]=m.dirty[i]=m.written[i]=false;m.version[i]=0;
  if(i<8) {
    for(int port=0;port<2;++port) {
#pragma HLS unroll
      m.wb[port][i]={};
    }
  }
  if(s.init_cursor==4096) s.initialized=true;
}

// Both software tests and the actual TAPA task call this exact event core.
// A RAM action has explicit registered stages; WB completion is retained
// separately and can be consumed despite blocked cache requests/outputs.
static inline void SalutSmBankTick(SalutSmBankControl& s, SalutSmBankStorage& m,
    const SalutSmBankInputs& in, SalutSmBankOutputs& out) {
#pragma HLS inline
  out = {};
  if (s.fault || s.completed) return;
  if (s.bank < 0 || s.bank > 7) { s.fault = true; return; }
  if (!s.initialized) {
    SalutSmInitialize(s,m);
    return;
  }

  if (s.response_held) {
    out.response_valid = true; out.response = s.response;
    if (in.response_ready) {
      auto& r = s.reads[s.response_context];
      const uint32_t count = uint32_t(s.response.rsp.count);
      r.cursor += count; r.remaining -= count; r.waiting = false;
      if (r.remaining == 0) r.active = false;
      s.response_held = false;
    }
  }
  if (s.publish_held) {
    out.publish_valid = true; out.publish = s.publish;
    if (in.publish_ready) {
      s.publish_held = false;
#ifndef __SYNTHESIS__
      ++s.stats.publications;
#endif
    }
  }

  // Independently capture both physical completion channels in narrow slots.
  for (int port = 0; port < 2; ++port) {
#pragma HLS unroll
    if (in.done_valid[port] && !s.done_held[port]) {
      const auto& done = in.done[port];
      if (done.end) {
        if (!s.wb_end_sent[port] || s.wb_end_ack[port] || s.queue_count[port] != 0) {
          s.fault = true; return;
        }
        out.done_consume[port] = true; s.wb_end_ack[port] = true;
      } else {
        const auto& entry = m.wb[port][unsigned(s.queue_head[port])];
        if (s.queue_count[port] == 0 || !entry.valid || !entry.issued ||
            entry.ticket != done.ticket || done.ticket >= (1u << 19)) {
          s.fault = true; return;
        }
        s.held_done[port] = done; s.done_held[port] = true;
        out.done_consume[port] = true;
#ifndef __SYNTHESIS__
        ++s.stats.writeback_completed[port];
#endif
      }
    }
    if (in.fill_valid[port]) {
      if (in.fill[port].end) {
        if (!s.read_end_sent[port] || s.read_end_ack[port]) { s.fault = true; return; }
        s.read_end_ack[port] = true; out.fill_consume[port] = true;
      } else if (!s.fill_held) {
        if (!s.pending || !s.pending_sent || s.pending_port != port ||
            s.pending_ticket != in.fill[port].ticket) { s.fault = true; return; }
        s.fill_data = in.fill[port].data; s.fill_held = true;
        out.fill_consume[port] = true;
#ifndef __SYNTHESIS__
        ++s.stats.refill_words;
#endif
      }
    }
  }

  // Snapshot dispatch is independent of cache RAM/publish availability.
  for (int port = 0; port < 2; ++port) {
#pragma HLS unroll
    auto& entry = m.wb[port][unsigned(s.queue_issue[port])];
    if (entry.valid && entry.ready && !entry.issued) {
      out.writeback_valid[port] = true;
      out.writeback[port] = {m.wb_data[port][unsigned(s.queue_issue[port])], entry.address, entry.ticket, false};
      if (in.writeback_ready[port]) {
        entry.issued = true; s.queue_issue[port] = s.queue_issue[port] + 1;
#ifndef __SYNTHESIS__
        ++s.stats.writeback_issued[port];
#endif
      }
    }
  }
  if (s.pending && !s.pending_sent) {
    const int port = s.pending_port;
    out.read_valid[port] = true; out.read[port] = {s.pending_address, s.pending_ticket, false};
    if (in.read_ready[port]) s.pending_sent = true;
  }

  if (!s.request_end && in.request_valid) {
    const auto& rr = in.request.req;
    if (in.request.end) { s.request_end = true; out.request_consume = true; }
    else if (!SalutValidClient(uint8_t(rr.pe)) || rr.r.kind < 1 || rr.r.kind > 2 ||
             addr_bank(rr.r.addr) != uint32_t(s.bank + 8 * rr.r.kind) ||
             uint64_t(addr_off(rr.r.addr)) + rr.r.len > BANK_SIZE) {
      s.fault = true; return;
    } else if (rr.r.len == 0) out.request_consume = true;
    else {
      const int free = !s.reads[0].active ? 0 : (!s.reads[1].active ? 1 : -1);
      if (free >= 0) {
        s.reads[free] = {rr, rr.r.addr, rr.r.len, true, false}; out.request_consume = true;
      }
    }
  }
  if (!s.write_end && in.write_valid && in.write.end) {
    s.write_end = true; out.write_consume = true;
  }

  // At most one cache RAM action per tick. Finite RAM stages cannot be
  // starved; held completions run before beginning another RAM operation.
  if (s.stage != 0) {
    const uint32_t index = s.stage_index;
    switch (unsigned(s.stage)) {
      case 1:
        m.data[index] = s.stage_write.data; m.tag[index] = s.stage_key;
        m.version[index] = s.stage_version;
        m.valid[index] = m.dirty[index] = m.written[index] = true;
        ++s.dirty_count; s.stage = 2;
#ifndef __SYNTHESIS__
        ++s.stats.installs;
        if (s.dirty_count > s.stats.dirty_peak) s.stats.dirty_peak = s.dirty_count;
#endif
        break;
      case 2:
        m.wb[s.stage_port][s.stage_slot].ready = true;
        SalutSmFinishWrite(s, s.stage_write); s.stage = 0;
        break;
      case 3:
        s.stage_data = m.data[index]; s.stage = 4;
        break;
      case 4:
        if (!s.response_held) { SalutSmMakeResponse(s, s.stage_context, s.stage_data); s.stage = 0; }
        break;
      case 5:
        m.data[index] = s.stage_data; m.tag[index] = s.stage_key;
        m.valid[index] = true; m.dirty[index] = m.written[index] = false;
        m.version[index] = s.stage_version; s.stage = 6;
        break;
      case 6:
        if (!s.response_held) { SalutSmMakeResponse(s, s.stage_context, s.stage_data); s.stage = 0; }
        break;
      default: s.fault = true; break;
    }
  } else if (s.done_held[0] || s.done_held[1]) {
    const int port = s.done_held[int(s.done_rr)] ? int(s.done_rr) : int(!s.done_rr);
    auto& entry = m.wb[port][unsigned(s.queue_head[port])];
    const uint32_t index = entry.index;
    if (!entry.valid || entry.ticket != s.held_done[port].ticket || !m.valid[index] ||
        !m.dirty[index] || m.tag[index] != entry.key || m.version[index] != entry.version) {
      s.fault = true; return;
    }
    m.dirty[index] = false; --s.dirty_count; entry = {};
    s.queue_head[port] = s.queue_head[port] + 1; --s.queue_count[port];
    s.done_held[port] = false; s.done_rr = !port;
  } else if (s.fill_held) {
    const uint32_t index = s.pending_index;
    const int ctx = s.pending_context;
    if (!s.response_held && m.valid[index] && m.tag[index] == s.pending_key) {
      s.stage = 3; s.stage_index = index; s.stage_context = ctx;
      s.pending = s.fill_held = false;
    } else if (!s.response_held && m.version[index] == s.pending_version && !m.dirty[index]) {
      // The physical reply was consumed; this fill's own reference is gone.
      s.stage = 5; s.stage_index = index; s.stage_context = ctx;
      s.stage_key = s.pending_key; s.stage_data = s.fill_data;
      s.stage_version = m.version[index] + 1;
      s.pending = s.fill_held = false;
    } else if (m.version[index] != s.pending_version || m.dirty[index]) {
      s.reads[ctx].waiting = false; s.pending = s.fill_held = false;
    }
  } else {
    // Precompute a legal write candidate, then arbitrate exactly ONCE. The
    // former preferred-read + fallback-read calls duplicated the cache RAM
    // access/loop-dependence graph, and could probe both pinned victims in
    // one event. One probe rotates fairly even when the chosen read stalls.
    const auto& p = in.write;
    const bool write_candidate=!s.write_end && in.write_valid && !p.end;
    bool write_ready=false;
    uint32_t index=0;
    ap_uint<33> key=0;
    int port=0;
    if (write_candidate) {
      const uint8_t kind = uint8_t(p.kind), client = uint8_t(p.client);
      const WR wr{{kind, p.address, p.no_data ? 0u : 8u}, p.client};
      if (!SalutValidFactorWrite(wr) || SalutSmOwner(p.address) != uint32_t(s.bank) ||
          p.seq < s.next_seq[client] || p.ordinal != s.next_ordinal[client] ||
          p.seq >= uint32_t(PAR_MAX_SEG) || (p.no_data && (!p.last || p.ordinal != 0)) ||
          (p.ordinal != 0 && (p.seq != s.active_seq[client] ||
                             p.address != s.next_address[client] || kind != s.burst_kind[client]))) {
        s.fault = true; return;
      }
      index = SalutSmIndex(p.address);
      key = SalutFactorKey(kind, p.address);
      if (!p.no_data && m.valid[index] && m.tag[index] == key && m.written[index]) {
        s.fault = true; return;
      }
      port = kind - 1;
      const bool refs = s.pending && s.pending_index == index;
#ifndef __SYNTHESIS__
      if (!p.no_data) {
        s.stats.capacity_stall_ticks += m.dirty[index];
        s.stats.queue_stall_ticks += s.queue_count[port] == 8;
      }
#endif
      write_ready=(!p.last || !s.publish_held) && (p.no_data ||
          (!m.dirty[index] && s.queue_count[port] < 8 && SalutVersionMayAdvance(m.version[index], refs)));
    }
    bool read_started=false;
    if(s.prefer_read || !write_ready || (write_candidate && p.no_data))
      read_started=SalutSmStartRead(s,m);
    if(!read_started && write_ready) {
        out.write_consume = true; s.prefer_read = true;
        if (p.no_data) SalutSmFinishWrite(s, p);
        else {
          const int slot = unsigned(s.queue_tail[port]);
          auto& entry = m.wb[port][slot];
          if (entry.valid) { s.fault = true; return; }
          const uint32_t ticket = (uint32_t(s.generations[port][slot]) << 3) | uint32_t(slot);
          s.generations[port][slot] = s.generations[port][slot] + 1;
          const ap_uint<16> version = m.version[index] + 1;
          m.wb_data[port][slot] = p.data;
          entry = {key, p.address, ticket, index, version, true, false, false};
          ++s.queue_count[port]; s.queue_tail[port] = s.queue_tail[port] + 1;
#ifndef __SYNTHESIS__
          if (uint64_t(s.queue_count[port]) > s.stats.queue_peak[port])
            s.stats.queue_peak[port] = uint64_t(s.queue_count[port]);
#endif
          s.stage = 1; s.stage_write = p; s.stage_index = index;
          s.stage_port = port; s.stage_slot = slot; s.stage_key = key; s.stage_version = version;
        }
    }
  }

  const bool reads_empty = !s.reads[0].active && !s.reads[1].active && !s.pending &&
                           !s.fill_held && !s.response_held && s.stage == 0;
  for (int port = 0; port < 2; ++port) {
#pragma HLS unroll
    if (s.request_end && reads_empty && !s.read_end_sent[port]) {
      out.read_valid[port] = true; out.read[port] = {}; out.read[port].end = true;
      if (in.read_ready[port]) s.read_end_sent[port] = true;
    }
    if (s.write_end && s.stage == 0 && !s.publish_held && s.queue_count[port] == 0 &&
        !s.done_held[port] && !s.wb_end_sent[port]) {
      out.writeback_valid[port] = true; out.writeback[port] = {}; out.writeback[port].end = true;
      if (in.writeback_ready[port]) s.wb_end_sent[port] = true;
    }
  }
  const bool drained = s.request_end && s.write_end && reads_empty && !s.publish_held &&
      !s.done_held[0] && !s.done_held[1] && s.dirty_count == 0 &&
      s.queue_count[0] == 0 && s.queue_count[1] == 0 &&
      s.read_end_ack[0] && s.read_end_ack[1] && s.wb_end_ack[0] && s.wb_end_ack[1];
  if (drained) {
    if (!s.response_end_sent) {
      out.response_valid = true; out.response = {}; out.response.end = true;
      if (in.response_ready) s.response_end_sent = true;
    }
    if (!s.drained_sent) {
      out.drained_valid = true; out.drained = s.bank;
      if (in.drained_ready) s.drained_sent = true;
    }
    s.completed = s.response_end_sent && s.drained_sent;
  }
}
