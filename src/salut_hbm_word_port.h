#pragma once
#include "salut_memory_protocol.h"

// Separate read and write state deliberately share no progress prerequisite.
// A write ticket is retained locally: AXI write_resp is count-minus-one,
// NOT a client or a ticket supplied by the memory controller.
struct SalutHBMWordControl {
  SalutWordReadReq read{};
  SalutWordReadRsp response{};
  SalutWritebackWord write{};
  SalutWritebackDone done{};
  ap_uint<3> read_phase = 0, write_phase = 0;
  bool address_sent = false, data_sent = false, fault = false;
};
struct SalutHBMWordInputs {
  SalutWordReadReq read{}; SalutWritebackWord write{};
  WORD512 read_data{};
  uint8_t write_resp = 0;
  bool read_valid = false, write_valid = false;
  bool response_ready = false, done_ready = false;
  bool read_addr_ready = false, write_addr_ready = false, write_data_ready = false;
  bool read_data_valid = false, write_resp_valid = false;
};
struct SalutHBMWordOutputs {
  SalutWordReadRsp response{}; SalutWritebackDone done{};
  WORD512 write_data{};
  uint32_t read_addr = 0, write_addr = 0;
  bool read_consume = false, write_consume = false;
  bool read_addr_valid = false, write_addr_valid = false, write_data_valid = false;
  bool read_data_consume = false, write_resp_consume = false;
  bool response_valid = false, done_valid = false;
};

static inline void SalutHBMWordTick(SalutHBMWordControl& s,
    const SalutHBMWordInputs& in, SalutHBMWordOutputs& out) {
#pragma HLS inline
  out = {};
  // Payload/address come directly from PRE-edge retained registers, never
  // from a mux chain controlled by current read/write FIFO heads. Valid bits
  // below still enforce the same independently stalled channel handshakes.
  out.read_addr = addr_off(s.read.address) >> 3;
  out.write_addr = addr_off(s.write.address) >> 3;
  out.write_data = s.write.data;
  out.response = s.response;
  out.done = s.done;
  if (s.fault) return;
  // Validate orphan completions against PRE-edge outstanding state.
  if (in.read_data_valid && s.read_phase != 2) {
    out.read_data_consume = true; s.fault = true; return;
  }
  if (in.write_resp_valid && s.write_phase != 2) {
    out.write_resp_consume = true; s.fault = true; return;
  }
  switch (unsigned(s.read_phase)) {
    case 0:
      if (in.read_valid) {
        if (!in.read.end && ((in.read.address & 7u) != 0 || addr_bank(in.read.address) >= 24)) {
          s.fault = true; break;
        }
        s.read = in.read; out.read_consume = true;
        if (in.read.end) { s.response = {}; s.response.end = true; s.read_phase = 4; }
        else s.read_phase = 1;
      }
      break;
    case 1:
      out.read_addr_valid = true;
      if (in.read_addr_ready) s.read_phase = 2;
      break;
    case 2:
      if (in.read_data_valid) {
        out.read_data_consume = true;
        s.response = {in.read_data, s.read.ticket, false}; s.read_phase = 3;
      }
      break;
    case 3:
    case 4:
      out.response_valid = true;
      if (in.response_ready) s.read_phase = s.response.end ? 5 : 0;
      break;
    default: break;
  }
  // Never nest this switch under read output readiness.
  switch (unsigned(s.write_phase)) {
    case 0:
      if (in.write_valid) {
        const uint32_t bank = addr_bank(in.write.address);
        if (!in.write.end && ((in.write.address & 7u) != 0 || bank < 8 || bank >= 24 ||
                             in.write.ticket >= (1u << 19))) {
          s.fault = true; break;
        }
        s.write = in.write; out.write_consume = true;
        s.address_sent = s.data_sent = false;
        if (in.write.end) { s.done = {}; s.done.end = true; s.write_phase = 4; }
        else s.write_phase = 1;
      }
      break;
    case 1:
      out.write_addr_valid = !s.address_sent; out.write_data_valid = !s.data_sent;
      if (out.write_addr_valid && in.write_addr_ready) s.address_sent = true;
      if (out.write_data_valid && in.write_data_ready) s.data_sent = true;
      if (s.address_sent && s.data_sent) s.write_phase = 2;
      break;
    case 2:
      if (in.write_resp_valid) {
        out.write_resp_consume = true;
        if (in.write_resp != 0) { s.fault = true; break; }
        s.done = {s.write.ticket, false}; s.write_phase = 3;
      }
      break;
    case 3:
    case 4:
      out.done_valid = true;
      if (in.done_ready) s.write_phase = s.done.end ? 5 : 0;
      break;
    default: break;
  }
}
