#pragma once
#include "salut_sm_bank.h"
#include "salut_stream_handshake.h"
#ifndef __SYNTHESIS__
#include <memory>
#endif

void SalutSmBank(int bank, tapa::istream<SalutReadEnvelope>& req,
    tapa::istream<SalutWriteBeat>& wr, tapa::ostream<SalutResponseEnvelope>& rsp,
    tapa::ostreams<SalutWordReadReq, 2>& rd, tapa::istreams<SalutWordReadRsp, 2>& fill,
    tapa::ostreams<SalutWritebackWord, 2>& wb, tapa::istreams<SalutWritebackDone, 2>& done,
    tapa::ostream<SalutPublishAck>& published, tapa::ostream<ap_uint<3>>& drained) {
  SalutSmBankControl control{};
#ifdef __SYNTHESIS__
  SalutSmBankStorage storage;
#else
  // TAPA uses small coroutine stacks. Keep the exact same RAM object/core,
  // but give each software task its own heap allocation, not a >300KB frame.
  auto storage_owner = std::make_unique<SalutSmBankStorage>();
  auto& storage = *storage_owner;
#endif
  control.bank = bank;
#pragma HLS disaggregate variable=storage
#pragma HLS bind_storage variable=storage.data type=ram_t2p impl=uram
#pragma HLS bind_storage variable=storage.tag type=ram_t2p impl=bram
#pragma HLS bind_storage variable=storage.version type=ram_t2p impl=bram
#pragma HLS bind_storage variable=storage.valid type=ram_t2p impl=bram
#pragma HLS bind_storage variable=storage.dirty type=ram_t2p impl=bram
#pragma HLS bind_storage variable=storage.written type=ram_t2p impl=bram
// Partition only narrow control metadata. Each wide snapshot bank has one
// reserve writer and one dispatch reader; init/completion never clear its
// invalid payload. This avoids a massive 512-bit multi-writer mux graph.
#pragma HLS array_partition variable=storage.wb complete dim=0
#pragma HLS array_partition variable=storage.wb_data complete dim=1
#pragma HLS bind_storage variable=storage.wb_data type=ram_2p impl=bram
#pragma HLS array_partition variable=control.queue_head complete
#pragma HLS array_partition variable=control.queue_tail complete
#pragma HLS array_partition variable=control.queue_issue complete
#pragma HLS array_partition variable=control.queue_count complete
#pragma HLS array_partition variable=control.generations complete dim=0
#pragma HLS array_partition variable=control.reads complete
#pragma HLS array_partition variable=control.done_held complete
#pragma HLS array_partition variable=control.held_done complete
#pragma HLS array_partition variable=control.read_end_sent complete
#pragma HLS array_partition variable=control.read_end_ack complete
#pragma HLS array_partition variable=control.wb_end_sent complete
#pragma HLS array_partition variable=control.wb_end_ack complete
  // A separate bounded metadata sweep is II=1 even if the event core's real
  // RAM dependence requires a larger runtime II. No request is consumed here.
  for(int index=0;index<4096;++index) {
#pragma HLS pipeline II=1
    SalutSmInitialize(control,storage);
  }
  control.initialized=!control.fault;
  control.init_cursor=4096;
  while (!control.completed || control.fault) {
    // Each of the eight bank actors remains independent. Use a multicycle
    // event FSM here: forcing II=1 makes Vitis 2022.2's recurrence analysis
    // stall. The verified baseline schedules 4..25 cycles per iteration;
    // this is NOT an II=1 throughput claim. The metadata sweep above stays
    // pipelined and both L/U queues retain their independent progress paths.
#pragma HLS pipeline off
    SalutSmBankInputs input{}; SalutSmBankOutputs output{};
    input.request_valid = req.try_peek(input.request);
    input.write_valid = wr.try_peek(input.write);
    input.response_ready = !rsp.full(); input.publish_ready = !published.full();
    input.drained_ready = !drained.full();
    for (int port = 0; port < 2; ++port) {
#pragma HLS unroll
      input.fill_valid[port] = fill[port].try_peek(input.fill[port]);
      input.done_valid[port] = done[port].try_peek(input.done[port]);
      input.read_ready[port] = !rd[port].full(); input.writeback_ready[port] = !wb[port].full();
    }
    SalutSmBankTick(control, storage, input, output);
#ifndef __SYNTHESIS__
    // Publish the final snapshot BEFORE the bank_done FIFO handshake so the
    // finite join cannot read a stale stats snapshot after observing drain.
    if (output.drained_valid && input.drained_ready) {
      control.stats.dirty_final = control.dirty_count;
      for (int port = 0; port < 2; ++port) control.stats.queue_final[port] = uint64_t(control.queue_count[port]);
      salut_sm_stats[bank] = control.stats;
    }
#endif
    bool failures[12];
#pragma HLS array_partition variable=failures complete
    failures[0] = SalutConsume(req,output.request_consume,input.request);
    failures[1] = SalutConsume(wr,output.write_consume,input.write);
    for (int port = 0; port < 2; ++port) {
#pragma HLS unroll
      failures[2+port] = SalutConsume(fill[port],output.fill_consume[port],input.fill[port]);
      failures[4+port] = SalutConsume(done[port],output.done_consume[port],input.done[port]);
      failures[6+port] = SalutProduce(rd[port],output.read_valid[port] && input.read_ready[port],output.read[port]);
      failures[8+port] = SalutProduce(wb[port],output.writeback_valid[port] && input.writeback_ready[port],output.writeback[port]);
    }
    failures[10] = SalutProduce(rsp,output.response_valid && input.response_ready,output.response);
    failures[11] = SalutProduce(published,output.publish_valid && input.publish_ready,output.publish);
    control.fault |= SalutAnyHandshakeFailure(failures);
    control.fault |= SalutProduce(drained,output.drained_valid && input.drained_ready && !control.fault,output.drained);
  }
}
