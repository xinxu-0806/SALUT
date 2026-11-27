#include "fixture.h"
#include "../../host/salut_layout_validation.h"
#include <cassert>
#include <stdexcept>

// Use the SAME launch validators as host/main.cpp, including BEFORE packing.
static MemoryTrace Layout() {
  assert(SalutValidateFactorRange(1, pack_addr(8, 0), 8));
  assert(SalutValidateFactorRange(2, pack_addr(23, BANK_SIZE-8), 8));
  assert(!SalutValidateFactorRange(2, pack_addr(23, BANK_SIZE-8), 16));
  assert(!SalutValidateFactorRange(2, pack_addr(23, BANK_SIZE-8), 0xfffffff8u));
  assert(!SalutValidateFactorRange(1, pack_addr(16, 0), 8));
  assert(!SalutValidateFactorRange(0, pack_addr(8, 0), 8));
  assert(!SalutValidateFactorRange(1, pack_addr(8, 1), 8));
  assert(!SalutValidateFactorRange(1, pack_addr(8, 0), 7));
  assert(SalutValidateFactorRange(1, pack_addr(8, 0), 0));
  assert(SalutValidateMetadataLimits(65536, 1u<<20, true));
  assert(!SalutValidateMetadataLimits(65537, 1u<<20, true));
  assert(!SalutValidateMetadataLimits(65536, (1u<<20)+1, true));
  assert(SalutValidateMetadataLimits(1u<<20, 1u<<21, false));
  assert(!SalutValidateMetadataLimits((1u<<20)+1, 1u<<21, false));
  assert(SalutValidateMetadataLimits(0, 0, true));
  SalutValidateOutputOwnership({{1,pack_addr(8,0),8,0}, {1,pack_addr(8,8),16,1},
                                {2,pack_addr(16,0),8,0}, {1,pack_addr(8,0),0,2}});
  for (const auto& ranges : {
         std::vector<SalutFactorRange>{{1,pack_addr(8,0),16,0}, {1,pack_addr(8,8),8,1}},
         std::vector<SalutFactorRange>{{1,pack_addr(8,0),16,0}, {1,pack_addr(8,8),8,0}},
         std::vector<SalutFactorRange>{{1,pack_addr(8,0),8,UINT32_MAX}},
         std::vector<SalutFactorRange>{{2,pack_addr(8,0),8,0}}}) {
    bool rejected = false;
    try { SalutValidateOutputOwnership(ranges); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  for (const auto offset : {uint64_t(BANK_SIZE), uint64_t(1)<<32, UINT64_MAX}) {
    bool rejected = false;
    try { (void)SalutCheckedPackAddress(8,offset); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  assert(SalutCheckedPackAddress(23, BANK_SIZE-1) == pack_addr(23,BANK_SIZE-1));
  return {};
}
static const bool registered = [] {
  RegisterMemoryScenario("host_layout", Layout); return true;
}();
