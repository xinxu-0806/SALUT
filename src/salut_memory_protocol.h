#pragma once
#include "lu_kernel.h"

// Memory-client numbering shared by the fabric, LM, publish and write paths.
// PEs are clients 1..PAR_NUM_PE in PEG-major order (client-1 = peg*PAR_PE_PER_PEG+local);
// column fronts follow as PAR_NUM_PE+1..PAR_NUM_PE+PAR_NUM_FRONT. Client 0 is invalid.
constexpr int SALUT_FRONT_CLIENT0 = PAR_NUM_PE + 1;
constexpr int SALUT_MAX_CLIENT = PAR_NUM_PE + PAR_NUM_FRONT;
constexpr int SALUT_PEG_LOCAL_BITS = PAR_PE_PER_PEG <= 2 ? 1 : PAR_PE_PER_PEG <= 4 ? 2 : 3;
static_assert((PAR_PE_PER_PEG & (PAR_PE_PER_PEG - 1)) == 0 && PAR_PE_PER_PEG >= 2 &&
              PAR_PE_PER_PEG <= (1 << SALUT_PEG_LOCAL_BITS),
              "PEG-local PE indices are power-of-two ring positions");
static_assert(SALUT_MAX_CLIENT < 256, "clients travel in 8-bit fields");
static inline bool SalutIsPeClient(unsigned client) {
#pragma HLS inline
  return client >= 1 && client <= unsigned(PAR_NUM_PE);
}
static inline int SalutClientPeg(unsigned client) {
#pragma HLS inline
  return int((client - 1u) / unsigned(PAR_PE_PER_PEG));
}
static inline int SalutClientLocal(unsigned client) {
#pragma HLS inline
  return int((client - 1u) & unsigned(PAR_PE_PER_PEG - 1));
}

#ifndef __SYNTHESIS__
// Per-task software evidence; NOT extra hardware counters or a shared hot path.
struct SalutCacheStats {
  uint64_t hits = 0, dirty_hits = 0, misses = 0, installs = 0, refill_words = 0;
  uint64_t publications = 0, bypass_requests = 0;
  uint64_t writeback_issued[2]{}, writeback_completed[2]{}, queue_peak[2]{};
  uint64_t dirty_peak = 0, capacity_stall_ticks = 0, queue_stall_ticks = 0;
  uint64_t dirty_final = 0, queue_final[2]{};
};
inline SalutCacheStats salut_sm_stats[8]{}, salut_lm_stats[8]{};
#endif

// The wire ABI remains the existing packed 64-bit-element address ABI.
struct SalutReadEnvelope { RR req; bool end; };
struct SalutResponseEnvelope { RP rsp; bool end; };
struct SalutCacheWriteReq { WR req; uint32_t seq; bool end; };
struct SalutWriteBeat {
  WORD512 data;
  uint32_t address, seq, ordinal;
  ap_uint<8> client, kind;
  bool last, no_data, end;
};
struct SalutPublishAck { ap_uint<8> client; uint32_t seq; };
struct SalutWordReadReq { uint32_t address, ticket; bool end; };
struct SalutWordReadRsp { WORD512 data; uint32_t ticket; bool end; };
struct SalutWritebackWord { WORD512 data; uint32_t address, ticket; bool end; };
struct SalutWritebackDone { uint32_t ticket; bool end; };

static inline uint32_t SalutSmOwner(uint32_t address) {
#pragma HLS inline
  return addr_bank(address) & 7u;
}
static inline uint32_t SalutSmIndex(uint32_t address) {
#pragma HLS inline
  return (addr_off(address) >> 3) & 4095u;
}
static inline ap_uint<33> SalutFactorKey(uint8_t kind, uint32_t address) {
#pragma HLS inline
  ap_uint<33> key = address & ~7u;
  key[32] = kind == 2;
  return key;
}
static inline bool SalutValidClient(uint8_t client) {
#pragma HLS inline
  return client >= 1 && client <= SALUT_MAX_CLIENT;
}
static inline bool SalutValidFactorWrite(const WR& wr) {
#pragma HLS inline
  const uint32_t bank = addr_bank(wr.r.addr);
  const bool bank_matches = (wr.r.kind == 1 && bank >= 8 && bank < 16) ||
                            (wr.r.kind == 2 && bank >= 16 && bank < 24);
  return bank_matches && SalutValidClient(uint8_t(wr.pe)) &&
         (wr.r.addr & 7u) == 0 && (wr.r.len & 7u) == 0 &&
         uint64_t(addr_off(wr.r.addr)) + uint64_t(wr.r.len) <= BANK_SIZE;
}
static inline bool SalutVersionMayAdvance(ap_uint<16> version, bool has_refs) {
#pragma HLS inline
  return version != 65535 || !has_refs;
}
