#include "fixture.h"
#include "salut_hbm_word_port.h"
#include "salut_stream_handshake.h"
#include <cassert>
#include <iostream>

// Breaks caught: ACK on submission instead of physical write_resp, coupling
// read output backpressure to write completion, duplicate AXI addr/data,
// dropping retained tickets or accepting an orphan/aggregated completion.
static MemoryTrace HbmProgress() {
  struct Endpoint {
    int reads=0,writes=0,value=0;
    bool available=false,ready=false;
    bool try_read(int& result) {
      ++reads; if (!available) return false;
      result=value; available=false; return true;
    }
    bool try_write(const int& input) {
      ++writes; if (!ready) return false;
      value=input; ready=false; return true;
    }
  } endpoint;
  int packet=123;
  assert(!SalutConsume(endpoint,false,packet) && endpoint.reads==0);
  assert(!SalutProduce(endpoint,false,packet) && endpoint.writes==0);
  assert(SalutConsume(endpoint,true,packet) && packet==123);
  assert(SalutProduce(endpoint,true,packet) && endpoint.value==0);
  // TAPA's streams[i] returns a short-lived stream view. The handshake helper
  // must accept that view without copying an underlying physical FIFO.
  assert(SalutConsume(Endpoint{},true,packet));
  assert(SalutProduce(Endpoint{},true,packet));
  endpoint.available=true; endpoint.value=456;
  assert(!SalutConsume(endpoint,true,packet) && packet==456 && !endpoint.available);
  endpoint.ready=true;
  assert(!SalutProduce(endpoint,true,packet) && endpoint.value==456 && !endpoint.ready);
  // Independent handshake failures must be reduced only after ALL ports were
  // sampled; never build a fault-state feedback chain across unrolled ports.
  bool failures[18]{};
  assert(!SalutAnyHandshakeFailure(failures));
  for (int channel=0; channel<18; ++channel) {
    failures[channel]=true;
    assert(SalutAnyHandshakeFailure(failures));
    failures[channel]=false;
  }
  SalutHBMWordControl state{};
  SalutHBMWordInputs in{}; SalutHBMWordOutputs out{};
  MemoryTrace trace;
  in.read_valid = in.write_valid = true;
  in.read = {pack_addr(8, 16), 77, false};
  in.write = {WORD512(0xcafe), pack_addr(8, 24), 0x12345, false};
  in.done_ready = true;
  SalutHBMWordTick(state, in, out);
  assert(out.read_consume && out.write_consume);
  in.read_valid = in.write_valid = false;
  SalutHBMWordTick(state, in, out);
  assert(out.read_addr_valid && out.read_addr == 2);
  assert(out.write_addr_valid && out.write_addr == 3);
  assert(out.write_data_valid && out.write_data == WORD512(0xcafe));
  in.read_addr_ready = in.write_addr_ready = true;
  SalutHBMWordTick(state, in, out);
  trace.counts["write_addr_handshakes"] += out.write_addr_valid && in.write_addr_ready;
  in.read_data_valid = true; in.read_data = WORD512(0xbeef);
  SalutHBMWordTick(state, in, out);
  assert(out.read_data_consume && !out.write_addr_valid && out.write_data_valid);
  in.read_data_valid = false;
  for (int cycle = 0; cycle < 6; ++cycle) {
    SalutHBMWordTick(state, in, out);
    assert(out.response_valid && out.response.data == WORD512(0xbeef));
    assert(!out.done_valid && !out.write_addr_valid);
    trace.counts["done_before_write_resp"] += out.done_valid;
  }
  in.write_data_ready = true;
  SalutHBMWordTick(state, in, out);
  assert(out.write_data_valid); ++trace.counts["write_data_handshakes"];
  in.write_resp_valid = true; in.write_resp = 0; // one actual AXI word completed
  SalutHBMWordTick(state, in, out);
  assert(out.write_resp_consume && !out.done_valid && out.response_valid);
  ++trace.counts["physical_write_responses"];
  in.write_resp_valid = false;
  SalutHBMWordTick(state, in, out);
  assert(out.done_valid && out.done.ticket == 0x12345 && !out.done.end);
  assert(out.response_valid && !in.response_ready);
  ++trace.counts["write_done_while_read_blocked"];
  assert(out.response.ticket == 77);

  // Closing write can finish independently; read End cannot overtake payload.
  in.read_valid = in.write_valid = true; in.read.end = in.write.end = true;
  in.done_ready = false;
  SalutHBMWordTick(state, in, out);
  assert(out.write_consume && !out.read_consume);
  in.write_valid = false;
  SalutHBMWordTick(state, in, out);
  assert(out.done_valid && out.done.end && out.response_valid && !out.response.end);
  in.done_ready = in.response_ready = true;
  SalutHBMWordTick(state, in, out);
  assert(out.done_valid && out.done.end);
  // The core may accept the ordered End on the edge that emits the last word.
  if (!out.read_consume) SalutHBMWordTick(state, in, out);
  assert(out.read_consume); in.read_valid = false;
  SalutHBMWordTick(state, in, out);
  assert(out.response_valid && out.response.end);
  SalutHBMWordTick(state, in, out);
  assert(!out.done_valid && !out.response_valid && !state.fault);

  // Independent physical port state: delaying L does not freeze U.
  SalutHBMWordControl ports[2]{};
  SalutHBMWordInputs pi[2]{}; SalutHBMWordOutputs po[2]{};
  for (int p = 0; p < 2; ++p) {
    pi[p].write_valid = true;
    pi[p].write = {WORD512(0x10 + p), pack_addr(8 + 8 * p, 0), uint32_t(p + 1), false};
    pi[p].write_addr_ready = pi[p].write_data_ready = pi[p].done_ready = true;
    SalutHBMWordTick(ports[p], pi[p], po[p]); pi[p].write_valid = false;
    SalutHBMWordTick(ports[p], pi[p], po[p]);
    assert(po[p].write_addr_valid && po[p].write_data_valid);
  }
  pi[1].write_resp_valid = true;
  SalutHBMWordTick(ports[0], pi[0], po[0]); SalutHBMWordTick(ports[1], pi[1], po[1]);
  pi[1].write_resp_valid = false;
  SalutHBMWordTick(ports[0], pi[0], po[0]); SalutHBMWordTick(ports[1], pi[1], po[1]);
  assert(!po[0].done_valid && po[1].done_valid && po[1].done.ticket == 2);

  // Data may handshake before address as well, without duplication.
  state = {}; in = {};
  in.write_valid = true; in.write = {WORD512(0x22), pack_addr(16, 0), 6, false};
  SalutHBMWordTick(state, in, out); in.write_valid = false;
  in.write_data_ready = true; SalutHBMWordTick(state, in, out);
  assert(out.write_data_valid && out.write_addr_valid);
  SalutHBMWordTick(state, in, out); assert(!out.write_data_valid && out.write_addr_valid);
  in.write_addr_ready = true; SalutHBMWordTick(state, in, out);
  in.write_resp_valid = true; in.write_resp = 1; // illegal two-word aggregation
  SalutHBMWordTick(state, in, out); assert(state.fault && !out.done_valid);
  state = {}; in = {}; in.write_resp_valid = true;
  SalutHBMWordTick(state, in, out); assert(state.fault && !out.done_valid);
  state = {}; in = {}; in.read_data_valid = true;
  SalutHBMWordTick(state, in, out); assert(state.fault && !out.response_valid);
  std::cout << "HBM_TRACE addr=" << trace.counts.at("write_addr_handshakes")
            << " data=" << trace.counts.at("write_data_handshakes")
            << " physical_resp=" << trace.counts.at("physical_write_responses")
            << " write_done_while_read_blocked=" << trace.counts.at("write_done_while_read_blocked") << '\n';
  return trace;
}
static const bool registered = [] {
  RegisterMemoryScenario("hbm_word", HbmProgress);
  RegisterMemoryScenario("hbm_response_backpressure", HbmProgress);
  return true;
}();
