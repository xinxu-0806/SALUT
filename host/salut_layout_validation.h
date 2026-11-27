#pragma once
#include "../src/lu_kernel.h"
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

struct SalutFactorRange {
  uint8_t kind;
  uint32_t address, elements, producer;
};
static inline bool SalutValidateFactorRange(uint8_t kind, uint32_t address, uint32_t elements) {
  const uint32_t bank = addr_bank(address), offset = addr_off(address);
  return ((kind == 1 && bank >= 8 && bank < 16) ||
          (kind == 2 && bank >= 16 && bank < 24)) &&
         (offset & 7u) == 0 && (elements & 7u) == 0 &&
         uint64_t(offset) + uint64_t(elements) <= uint64_t(BANK_SIZE);
}
static inline bool SalutValidateMetadataLimits(uint32_t n, uint32_t segments, bool hardware) {
  return n <= uint32_t(hardware ? 65536 : 1u<<20) &&
         segments <= uint32_t(hardware ? 1u<<20 : 1u<<21);
}
// Check the RAW offset before pack_addr can mask away overflow bits.
static inline uint32_t SalutCheckedPackAddress(uint32_t bank, uint64_t offset) {
  if (bank >= 24 || offset >= uint64_t(BANK_SIZE))
    throw std::invalid_argument("SALUT bank/offset exceeds packed address capacity");
  return pack_addr(bank, uint32_t(offset));
}
static inline void SalutCheckBankAllocation(uint32_t bank, uint64_t offset, uint64_t elements) {
  if (bank >= 24 || offset >= uint64_t(BANK_SIZE) || elements > uint64_t(BANK_SIZE)-offset)
    throw std::invalid_argument("SALUT allocation exceeds physical bank capacity before packing");
}
static inline void SalutValidateOutputOwnership(const std::vector<SalutFactorRange>& ranges) {
  auto sorted = ranges;
  for (const auto& range : sorted)
    if (range.producer == UINT32_MAX || !SalutValidateFactorRange(range.kind, range.address, range.elements))
      throw std::invalid_argument("SALUT invalid factor output range/producer");
  std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
    if (addr_bank(a.address) != addr_bank(b.address)) return addr_bank(a.address) < addr_bank(b.address);
    return addr_off(a.address) < addr_off(b.address);
  });
  uint64_t end = 0;
  uint32_t bank = UINT32_MAX;
  for (const auto& range : sorted) {
    if (!range.elements) continue; // Empty producer owns no physical word.
    const uint32_t next_bank = addr_bank(range.address);
    if (next_bank != bank) { bank = next_bank; end = 0; }
    const uint64_t begin = addr_off(range.address);
    if (begin < end) throw std::invalid_argument("SALUT overlapping padded factor outputs");
    end = begin + uint64_t(range.elements);
  }
}
