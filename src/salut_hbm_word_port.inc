#pragma once
#include "salut_hbm_word_port.h"
#include "salut_stream_handshake.h"

void SalutHBMWordPort(tapa::async_mmap<WORD512>& port,
    tapa::istream<SalutWordReadReq>& rd, tapa::ostream<SalutWordReadRsp>& rsp,
    tapa::istream<SalutWritebackWord>& wr, tapa::ostream<SalutWritebackDone>& done) {
  SalutHBMWordControl state{};
  while (state.read_phase != 5 || state.write_phase != 5 || state.fault) {
#pragma HLS pipeline II=1
    SalutHBMWordInputs input{}; SalutHBMWordOutputs output{};
    input.read_valid = rd.try_peek(input.read);
    input.write_valid = wr.try_peek(input.write);
    input.read_data_valid = port.read_data.try_peek(input.read_data);
    input.write_resp_valid = port.write_resp.try_peek(input.write_resp);
    input.read_addr_ready = !port.read_addr.full();
    input.write_addr_ready = !port.write_addr.full();
    input.write_data_ready = !port.write_data.full();
    input.response_ready = !rsp.full(); input.done_ready = !done.full();
    SalutHBMWordTick(state, input, output);
    bool failures[9];
#pragma HLS array_partition variable=failures complete
    failures[0] = SalutConsume(rd,output.read_consume,input.read);
    failures[1] = SalutConsume(wr,output.write_consume,input.write);
    failures[2] = SalutConsume(port.read_data,output.read_data_consume,input.read_data);
    failures[3] = SalutConsume(port.write_resp,output.write_resp_consume,input.write_resp);
    failures[4] = SalutProduce(port.read_addr,output.read_addr_valid && input.read_addr_ready,output.read_addr);
    failures[5] = SalutProduce(port.write_addr,output.write_addr_valid && input.write_addr_ready,output.write_addr);
    failures[6] = SalutProduce(port.write_data,output.write_data_valid && input.write_data_ready,output.write_data);
    failures[7] = SalutProduce(rsp,output.response_valid && input.response_ready,output.response);
    failures[8] = SalutProduce(done,output.done_valid && input.done_ready,output.done);
    state.fault |= SalutAnyHandshakeFailure(failures);
  }
}

void SalutHBMReadWordPort(tapa::async_mmap<WORD512>& port,
    tapa::istream<SalutWordReadReq>& rd, tapa::ostream<SalutWordReadRsp>& rsp) {
  SalutHBMWordControl state{};
  while (state.read_phase != 5 || state.fault) {
#pragma HLS pipeline II=1
    SalutHBMWordInputs input{}; SalutHBMWordOutputs output{};
    input.read_valid = rd.try_peek(input.read);
    input.read_data_valid = port.read_data.try_peek(input.read_data);
    input.read_addr_ready = !port.read_addr.full(); input.response_ready = !rsp.full();
    SalutHBMWordTick(state, input, output);
    bool failures[4];
#pragma HLS array_partition variable=failures complete
    failures[0] = SalutConsume(rd,output.read_consume,input.read);
    failures[1] = SalutConsume(port.read_data,output.read_data_consume,input.read_data);
    failures[2] = SalutProduce(port.read_addr,output.read_addr_valid && input.read_addr_ready,output.read_addr);
    failures[3] = SalutProduce(rsp,output.response_valid && input.response_ready,output.response);
    state.fault |= SalutAnyHandshakeFailure(failures);
  }
}
