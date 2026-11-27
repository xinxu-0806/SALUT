// DIAGNOSTIC ONLY: one physical SM owner, one PEG, one front, three HBM data
// engines. Empty lanes below are explicitly virtual test endpoints, NOT eight
// banks/PAR_NUM_PE math PEs. Full-platform/provenance gates must reject this XO.
// Include the production translation unit, not only its header: tapacc's
// RewriteOtherFunc dereferences declaration-only legacy task bodies. This
// supplies their real definitions without stubbing or changing production.
// Only tasks reachable from SalutMemoryValidationTop are synthesized here.
#include "../../src/lu_kernel.cpp"

// DIAGNOSTIC ONLY: synthesize the three retained PRODUCTION task bodies
// together without waiting for the unrelated SM frontend. No mathematical
// PE or HBM factor bank is present; this is never a deployable product top.
void SalutRetainedValidationDriver(tapa::mmap<uint32_t> results,
    tapa::istream<uint32_t>& front_ready,tapa::ostream<uint32_t>& front_done,
    tapa::istream<uint32_t>& pivot_ready,tapa::istream<uint32_t>& segment_ready,
    tapa::ostream<uint32_t>& segment_done,tapa::istream<ap_uint<1>>& compute_done,
    tapa::ostreams<ap_uint<8>,PAR_NUM_PEG>& ack_in,tapa::istreams<ap_uint<1>,PAR_NUM_PE>& ack_out) {
  for(int ordinal=0;ordinal<PAR_PE_PER_PEG;++ordinal)
    for(int peg=0;peg<PAR_NUM_PEG;++peg) ack_in[peg].write(ap_uint<8>(peg*PAR_PE_PER_PEG+ordinal+1));
  bool fault=false;
  for(int pe=0;pe<PAR_NUM_PE;++pe) fault |= ack_out[pe].read()!=1;
  ap_uint<1> finished=0;
  uint32_t fronts=0,segments=0;
  while(!compute_done.try_read(finished)) {
    uint32_t id;
    if(front_ready.try_read(id) && id!=UINT32_MAX) {front_done.write(id);++fronts;}
    if(segment_ready.try_read(id) && id!=UINT32_MAX) {segment_done.write(id);++segments;}
    pivot_ready.try_read(id);
  }
  if(finished!=1) fault=true;
  while(fault) {
#pragma HLS pipeline II=1
  }
  results[0]=fronts;results[1]=segments;results[2]=PAR_NUM_PE;
}
void SalutRetainedValidationTop(tapa::mmap<uint32_t> meta,
    tapa::mmap<uint32_t> results) {
  // tapacc records FIFO depth only for the FIRST declarator in a declaration.
  // Keep one stream per declaration so internal channels cannot be mistaken
  // for externally connected, bidirectional FIFOs during top instrumentation.
  tapa::stream<uint32_t,4> front_ready("retained_front_ready");
  tapa::stream<uint32_t,4> front_done("retained_front_done");
  tapa::stream<uint32_t,4> pivot_ready("retained_pivot_ready");
  tapa::stream<uint32_t,4> segment_ready("retained_segment_ready");
  tapa::stream<uint32_t,4> segment_done("retained_segment_done");
  tapa::stream<ap_uint<1>,2> compute_done("retained_compute_done");
  tapa::streams<ap_uint<8>,PAR_NUM_PEG,4> ack_in("retained_ack_in");
  tapa::streams<ap_uint<1>,PAR_NUM_PE,4> ack_out("retained_ack_out");
  tapa::stream<ParDegreeRequest,4> degree_request("retained_degree_request");
  tapa::stream<ParDegreeResponse,4> degree_response("retained_degree_response");
  tapa::task()
    .invoke<tapa::detach>(ParallelDegreeStore,degree_request,degree_response)
    .invoke<tapa::detach>(ParallelPegAckDemux,ack_in,ack_out)
    .invoke<tapa::detach>(ParallelScheduler,meta,front_ready,front_done,
        pivot_ready,segment_ready,segment_done,degree_request,degree_response,compute_done)
    .invoke<tapa::join>(SalutRetainedValidationDriver,results,front_ready,front_done,
        pivot_ready,segment_ready,segment_done,compute_done,ack_in,ack_out);
}

void SalutValidationBeatMerge(tapa::istream<SalutWriteBeat>& in0,
    tapa::istream<SalutWriteBeat>& in1, tapa::ostream<SalutWriteBeat>& out) {
  SalutMergeAdapter(in0,in1,out);
}
void SalutValidationPublishRoute(int bank, tapa::istream<SalutPublishAck>& sm,
    tapa::istream<SalutPublishAck>& lm, tapa::ostreams<SalutPublishAck,8>& sm_out,
    tapa::ostreams<SalutPublishAck,8>& lm_out) {
  for (;;) {
#pragma HLS pipeline II=1
    SalutPublishAck ack{};
    if (sm.try_peek(ack) && bank >= 0 && bank < 8 && !sm_out[bank].full())
      sm_out[bank].write(sm.read());
    if (lm.try_peek(ack)) {
      const int peg = SalutClientPeg(uint8_t(ack.client));
      if (SalutIsPeClient(uint8_t(ack.client)) && !lm_out[peg].full())
        lm_out[peg].write(lm.read());
    }
  }
}

// Two WORD512 entries per command/result. Header: op[7:0] (0=read,1=write),
// kind[15:8], client[23:16], packed address[63:32], elements[95:64].
// This driver deliberately supports only one physical beat/command; standalone
// RTL benches exercise the production cores' longer/nonaligned bursts.
void SalutValidationDriver(tapa::mmap<WORD512> commands, tapa::mmap<WORD512> results,
    uint32_t n_commands, uint32_t bank,
    tapa::ostreams<RR,PAR_PE_PER_PEG>& pe_read, tapa::istreams<RP,PAR_PE_PER_PEG>& pe_response,
    tapa::ostreams<WR,PAR_PE_PER_PEG>& pe_write, tapa::ostreams<WP,PAR_PE_PER_PEG>& pe_data,
    tapa::ostream<RR>& front_read, tapa::istream<RP>& front_response,
    tapa::ostream<WR>& front_write, tapa::ostream<WP>& front_data,
    tapa::istream<SalutPublishAck>& published, tapa::ostream<ap_uint<1>>& compute_done,
    tapa::istreams<ap_uint<1>,8>& peg_rd_close, tapa::istreams<ap_uint<1>,8>& peg_wr_close,
    tapa::istreams<ap_uint<1>,8>& front_rd_close, tapa::istreams<ap_uint<1>,8>& front_wr_close,
    tapa::ostream<ap_uint<1>>& lm_close, tapa::ostream<ap_uint<1>>& fork_close,
    tapa::ostream<ap_uint<1>>& rd_close, tapa::ostream<ap_uint<1>>& wr_close,
    tapa::ostream<SalutReadEnvelope>& read_unused,
    tapa::ostream<SalutWriteBeat>& write_unused,
    tapa::ostream<SalutResponseEnvelope>& response_unused,
    tapa::istream<ap_uint<3>>& real_bank_done, tapa::ostreams<ap_uint<3>,8>& banks_done,
    tapa::istream<ap_uint<3>>& real_response_end, tapa::ostreams<ap_uint<3>,8>& responses_end,
    tapa::istream<ap_uint<1>>& real_publish_done, tapa::ostream<ap_uint<1>>& publish_done) {
  bool fault = bank >= 8 || n_commands > 64;
  for (uint32_t i = 0; i < n_commands && !fault; ++i) {
    WORD512 header = commands[2*i];
    const uint8_t op = uint8_t(header.range(7,0));
    const uint8_t kind = uint8_t(header.range(15,8));
    const uint8_t client = uint8_t(header.range(23,16));
    const uint32_t address = uint32_t(header.range(63,32));
    const uint32_t length = uint32_t(header.range(95,64));
    // Only PEG 0's PEs (clients 1..PAR_PE_PER_PEG) and front 0 exist here.
    const bool front = client == SALUT_FRONT_CLIENT0;
    const int local = SalutClientLocal(client);
    if ((client < 1 || client > PAR_PE_PER_PEG) && !front) fault = true;
    if (kind > 2 || addr_bank(address) != bank+8*kind || op > 1 ||
        uint64_t(addr_off(address))+length > BANK_SIZE) fault = true;
    WORD512 value = 0;
    if (op == 1) {
      WR request{{kind,address,length},client};
      if (kind == 0 || length > 8 || !SalutValidFactorWrite(request)) fault = true;
      if (!fault) {
        if (front) front_write.write(request); else pe_write[local].write(request);
        if (length) {
          WP packet{commands[2*i+1],client,8};
          if (front) front_data.write(packet); else pe_data[local].write(packet);
        }
        const auto ack = published.read();
        if (ack.client != client) fault = true;
        header.range(159,128) = ack.seq;
      }
    } else {
      if (length > 8-(address&7u)) fault = true;
      if (!fault) {
        RR request{{kind,address,length},client};
        if (front) front_read.write(request); else pe_read[local].write(request);
        if (length) {
          const RP packet = front ? front_response.read() : pe_response[local].read();
          if (packet.pe != client || packet.first != (address&7u) || packet.count != length) fault = true;
          value = packet.v;
          header.range(103,96) = packet.first; header.range(111,104) = packet.count;
        }
      }
    }
    if (!fault) { results[2*i] = header; results[2*i+1] = value; }
  }
  // Invalid diagnostic input is fail-closed, never a fabricated success marker.
  while (fault) {
#pragma HLS pipeline II=1
  }
  compute_done.write(ap_uint<1>(1));
  for (int lane = 0; lane < 8; ++lane) {
    const auto peg_rd = peg_rd_close[lane].read();
    const auto peg_wr = peg_wr_close[lane].read();
    const auto front_rd = front_rd_close[lane].read();
    const auto front_wr = front_wr_close[lane].read();
    if (lane == 0) { lm_close.write(peg_rd); fork_close.write(peg_wr); rd_close.write(front_rd); wr_close.write(front_wr); }
  }
  SalutReadEnvelope r_end{}; r_end.end = true; read_unused.write(r_end);
  SalutWriteBeat w_end{}; w_end.end = true; write_unused.write(w_end);
  SalutResponseEnvelope p_end{}; p_end.end = true; response_unused.write(p_end);
  // These are explicitly ABSENT owners/lanes in this diagnostic graph. The
  // real owner/egress must STILL return their actual End/drain tokens below.
  for (int lane = 0; lane < 8; ++lane) {
    if (uint32_t(lane) != bank) banks_done[lane].write(ap_uint<3>(lane));
    if (lane != 0) responses_end[lane].write(ap_uint<3>(lane));
  }
  const auto b = real_bank_done.read(); const auto e = real_response_end.read();
  const auto p = real_publish_done.read();
  if (b != bank || e != 0 || p != 1) {
    for (;;) {
#pragma HLS pipeline II=1
    }
  }
  banks_done[bank].write(b); responses_end[0].write(e);
  // Marker is written BEFORE the final drain token, never after top ap_done.
  results[2*n_commands] = WORD512(0x534D56414C504153ULL);
  publish_done.write(p);
}

void SalutMemoryValidationTop(tapa::mmap<WORD512> L, tapa::mmap<WORD512> U,
    tapa::mmap<WORD512> A, tapa::mmap<WORD512> commands, tapa::mmap<WORD512> results,
    uint32_t n_commands, uint32_t bank) {
  tapa::streams<RR,PAR_PE_PER_PEG,4> pe_read("pe_read"); tapa::streams<RP,PAR_PE_PER_PEG,4> pe_response("pe_response");
  tapa::streams<WR,PAR_PE_PER_PEG,4> pe_write("pe_write"); tapa::streams<WP,PAR_PE_PER_PEG,4> pe_data("pe_data");
  tapa::stream<RR,4> front_read("front_read"); tapa::stream<RP,4> front_response("front_response");
  tapa::stream<WR,4> front_write("front_write"); tapa::stream<WP,4> front_data("front_data");
  tapa::stream<SalutReadEnvelope,4> lm_req("lm_req");
  tapa::stream<SalutReadEnvelope,4> front_req("front_req");
  tapa::stream<SalutReadEnvelope,4> lane_rd("lane_rd");
  tapa::stream<SalutReadEnvelope,4> read_unused("read_unused");
  tapa::stream<SalutReadEnvelope,4> rs0("rs0");
  tapa::stream<SalutReadEnvelope,4> rs1("rs1");
  tapa::stream<SalutReadEnvelope,4> rs_merged("rs_merged");
  tapa::stream<SalutReadEnvelope,4> factor_req("factor_req");
  tapa::stream<SalutReadEnvelope,4> a_req("a_req");
  tapa::stream<RP,4> lm_rsp("lm_rsp");
  tapa::stream<SalutCacheWriteReq,4> peg_wr("peg_wr");
  tapa::stream<SalutCacheWriteReq,4> lm_wr("lm_wr");
  tapa::stream<SalutCacheWriteReq,4> front_wr("front_wr");
  tapa::stream<SalutCacheWriteReq,4> lane_wr("lane_wr");
  tapa::stream<WP,4> peg_wp("peg_wp");
  tapa::stream<WP,4> lm_wp("lm_wp");
  tapa::stream<WP,4> front_wp("front_wp");
  tapa::stream<WP,4> lane_wp("lane_wp");
  tapa::stream<SalutWriteBeat,4> beat("beat");
  tapa::stream<SalutWriteBeat,4> write_unused("write_unused");
  tapa::stream<SalutWriteBeat,4> ws0("ws0");
  tapa::stream<SalutWriteBeat,4> ws1("ws1");
  tapa::stream<SalutWriteBeat,4> bank_wr("bank_wr");
  tapa::stream<SalutResponseEnvelope,4> factor_rsp("factor_rsp");
  tapa::stream<SalutResponseEnvelope,4> a_rsp("a_rsp");
  tapa::stream<SalutResponseEnvelope,4> lane_rsp("lane_rsp");
  tapa::stream<SalutResponseEnvelope,4> response_unused("response_unused");
  tapa::stream<SalutResponseEnvelope,4> ps0("ps0");
  tapa::stream<SalutResponseEnvelope,4> ps1("ps1");
  tapa::stream<SalutResponseEnvelope,4> egress_rsp("egress_rsp");
  tapa::streams<SalutWordReadReq,2,4> word_rd("word_rd"); tapa::streams<SalutWordReadRsp,2,4> word_rsp("word_rsp");
  tapa::streams<SalutWritebackWord,2,4> wb("wb"); tapa::streams<SalutWritebackDone,2,4> done("done");
  tapa::stream<SalutWordReadReq,4> a_word_rd("a_word_rd"); tapa::stream<SalutWordReadRsp,4> a_word_rsp("a_word_rsp");
  tapa::stream<SalutPublishAck,4> sm_ack("sm_ack");
  tapa::stream<SalutPublishAck,4> lm_ack("lm_ack");
  tapa::stream<SalutPublishAck,4> published("published");
  tapa::streams<SalutPublishAck,8,4> sm_acks("sm_acks");
  tapa::streams<SalutPublishAck,8,4> lm_acks("lm_acks");
  tapa::stream<ap_uint<1>,2> compute_done("compute_done");
  tapa::stream<ap_uint<1>,2> publisher_close("publisher_close");
  tapa::stream<ap_uint<1>,2> join_close("join_close");
  tapa::stream<ap_uint<1>,2> real_publish_done("real_publish_done");
  tapa::stream<ap_uint<1>,2> publish_done("publish_done");
  tapa::stream<ap_uint<1>,2> lm_close("lm_close");
  tapa::stream<ap_uint<1>,2> fork_close("fork_close");
  tapa::stream<ap_uint<1>,2> rd_close("rd_close");
  tapa::stream<ap_uint<1>,2> wr_close("wr_close");
  tapa::streams<ap_uint<1>,8,2> peg_rd_close("peg_rd_close");
  tapa::streams<ap_uint<1>,8,2> peg_wr_close("peg_wr_close");
  tapa::streams<ap_uint<1>,8,2> front_rd_close("front_rd_close");
  tapa::streams<ap_uint<1>,8,2> front_wr_close("front_wr_close");
  tapa::stream<ap_uint<3>,4> real_bank_done("real_bank_done");
  tapa::stream<ap_uint<3>,4> real_response_end("real_response_end");
  tapa::streams<ap_uint<3>,8,4> banks_done("banks_done");
  tapa::streams<ap_uint<3>,8,4> responses_end("responses_end");
  tapa::task()
    .invoke<tapa::detach>(SalutValidationDriver,commands,results,n_commands,bank,
        pe_read,pe_response,pe_write,pe_data,front_read,front_response,front_write,front_data,published,compute_done,
        peg_rd_close,peg_wr_close,front_rd_close,front_wr_close,lm_close,fork_close,rd_close,wr_close,
        read_unused,write_unused,response_unused,real_bank_done,banks_done,real_response_end,responses_end,real_publish_done,publish_done)
    .invoke<tapa::detach>(SalutLMLane,pe_read,pe_response,lm_req,lm_rsp,lm_wr,lm_wp,lm_ack,lm_close)
    .invoke<tapa::detach>(SalutPegWriteFork,0,pe_write,pe_data,peg_wr,peg_wp,lm_wr,lm_wp,fork_close)
    .invoke<tapa::detach>(SalutFrontReadSource,0,front_read,front_req,rd_close)
    .invoke<tapa::detach>(SalutFrontWriteSource,0,front_write,front_data,front_wr,front_wp,wr_close)
    .invoke<tapa::detach>(SalutReadLaneMerge,lm_req,front_req,lane_rd)
    .invoke<tapa::detach>(SalutReadSwitch2,2,lane_rd,read_unused,rs0,rs1)
    .invoke<tapa::detach>(SalutReadLaneMerge,rs0,rs1,rs_merged)
    .invoke<tapa::detach>(SalutWriteLaneMerge,peg_wr,peg_wp,front_wr,front_wp,lane_wr,lane_wp)
    .invoke<tapa::detach>(SalutWriteBeatizer,lane_wr,lane_wp,beat)
    .invoke<tapa::detach>(SalutWriteSwitch2,2,beat,write_unused,ws0,ws1)
    .invoke<tapa::detach>(SalutValidationBeatMerge,ws0,ws1,bank_wr)
    .invoke<tapa::detach>(SalutRequestBankSplit,bank,rs_merged,factor_req,a_req)
    .invoke<tapa::detach>(SalutSmBank,bank,factor_req,bank_wr,factor_rsp,word_rd,word_rsp,wb,done,sm_ack,real_bank_done)
    .invoke<tapa::detach>(SalutABurstAdapter,bank,a_req,a_word_rd,a_word_rsp,a_rsp)
    .invoke<tapa::detach>(SalutHBMWordPort,L,word_rd[0],word_rsp[0],wb[0],done[0])
    .invoke<tapa::detach>(SalutHBMWordPort,U,word_rd[1],word_rsp[1],wb[1],done[1])
    .invoke<tapa::detach>(SalutHBMReadWordPort,A,a_word_rd,a_word_rsp)
    .invoke<tapa::detach>(SalutResponseLaneMerge,factor_rsp,a_rsp,lane_rsp)
    .invoke<tapa::detach>(SalutResponseSwitch2,2,lane_rsp,response_unused,ps0,ps1)
    .invoke<tapa::detach>(SalutResponseLaneMerge,ps0,ps1,egress_rsp)
    .invoke<tapa::detach>(SalutResponseEgress,0,egress_rsp,lm_rsp,front_response,real_response_end)
    .invoke<tapa::detach>(SalutValidationPublishRoute,bank,sm_ack,lm_ack,sm_acks,lm_acks)
    .invoke<tapa::detach>(SalutPublishJoin,sm_acks,lm_acks,published,publisher_close,real_publish_done)
    .invoke<tapa::detach>(SalutCompletionFanout,compute_done,peg_rd_close,peg_wr_close,front_rd_close,front_wr_close,publisher_close,join_close)
    .invoke<tapa::join>(SalutMemoryDrainJoin,join_close,banks_done,responses_end,publish_done);
}
