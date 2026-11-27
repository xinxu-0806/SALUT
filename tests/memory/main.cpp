#include "fixture.h"
#include "salut_memory_protocol.h"
#include <cassert>
#include <iostream>
#include <stdexcept>

// Breaks caught: wrong bank/index/key derivation, truncated range arithmetic,
// accepting invalid producer/kind, or wrapping a version with live references.
static void TestProtocolContract() {
  assert(SalutSmOwner(pack_addr(11, 32800)) == 3);
  assert(SalutSmIndex(pack_addr(11, 32800)) == 4);
  assert(SalutSmIndex(pack_addr(19, 32800)) == 4);
  assert(SalutFactorKey(1, pack_addr(11, 32803)) ==
         SalutFactorKey(1, pack_addr(11, 32800)));
  assert(SalutFactorKey(1, pack_addr(11, 32800)) !=
         SalutFactorKey(2, pack_addr(19, 32800)));
  // First PE, last PE, first front, last front are valid; 0 and past the last
  // front are not (65/255: arbitrary larger out-of-range clients).
  for (uint8_t client : {1, PAR_NUM_PE, SALUT_FRONT_CLIENT0, SALUT_MAX_CLIENT})
    assert(SalutValidClient(client));
  for (uint8_t client : {0, SALUT_MAX_CLIENT + 1, 65, 255}) assert(!SalutValidClient(client));
  assert(SalutIsPeClient(1) && SalutIsPeClient(PAR_NUM_PE));
  assert(!SalutIsPeClient(0) && !SalutIsPeClient(SALUT_FRONT_CLIENT0));
  assert(SalutClientPeg(1) == 0 && SalutClientLocal(1) == 0);
  assert(SalutClientPeg(PAR_PE_PER_PEG) == 0 && SalutClientLocal(PAR_PE_PER_PEG) == PAR_PE_PER_PEG - 1);
  assert(SalutClientPeg(PAR_PE_PER_PEG + 1) == 1 && SalutClientLocal(PAR_PE_PER_PEG + 1) == 0);
  assert(SalutClientPeg(PAR_NUM_PE) == PAR_NUM_PEG - 1 &&
         SalutClientLocal(PAR_NUM_PE) == PAR_PE_PER_PEG - 1);
  WR wr{{1, pack_addr(8, 0), 8}, 1};
  assert(SalutValidFactorWrite(wr));
  wr.r.addr = pack_addr(8, 1); assert(!SalutValidFactorWrite(wr));
  wr.r.addr = pack_addr(8, 0); wr.r.len = 7;
  assert(!SalutValidFactorWrite(wr));
  wr.r.len = 0; assert(SalutValidFactorWrite(wr));
  wr.r.addr = pack_addr(8, BANK_SIZE - 8); wr.r.len = 8;
  assert(SalutValidFactorWrite(wr));
  wr.r.len = 16; assert(!SalutValidFactorWrite(wr));
  wr.r.len = 0xfffffff8u; assert(!SalutValidFactorWrite(wr));
  wr = WR{{2, pack_addr(16, 8), 16}, SALUT_MAX_CLIENT}; assert(SalutValidFactorWrite(wr));
  wr.r.kind = 1; assert(!SalutValidFactorWrite(wr));
  wr.r.kind = 0; assert(!SalutValidFactorWrite(wr));
  wr = WR{{1, pack_addr(15, 0), 8}, SALUT_MAX_CLIENT + 1}; assert(!SalutValidFactorWrite(wr));
  assert(SalutVersionMayAdvance(ap_uint<16>(65534), true));
  assert(!SalutVersionMayAdvance(ap_uint<16>(65535), true));
  assert(SalutVersionMayAdvance(ap_uint<16>(65535), false));
}

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("one case argument required");
    const std::string name = argv[1];
    if (name == "protocol") TestProtocolContract();
    else if (name == "all") {
      TestProtocolContract();
      // Fail closed until every required production-core case exists.
      for (const char* test : {"fabric", "hbm_word", "sm_bank", "sm_backpressure",
                             "publish", "lm", "write_merge", "concurrency", "drain", "host_layout"}) {
        const auto trace = RunMemoryScenario(test);
        if (trace.fault) throw std::runtime_error(std::string("DUT fault: ") + test);
      }
    } else {
      const auto trace = RunMemoryScenario(name);
      if (trace.fault) throw std::runtime_error("DUT fault: " + name);
    }
    std::cout << "MEMORY_TEST_PASS " << name << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "MEMORY_TEST_FAIL " << error.what() << '\n';
    return 1;
  }
}
