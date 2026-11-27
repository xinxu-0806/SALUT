#include "lu_kernel.h"
#include <cassert>
#include <iostream>
#include <vector>

int main() {
  using Bank = std::vector<WORD512,tapa::aligned_allocator<WORD512>>;
  Bank banks[24];
  for (auto& bank : banks) bank.resize(1);
  std::vector<uint32_t,tapa::aligned_allocator<uint32_t>> meta(32,16);
  meta[0] = PAR_META_MAGIC; meta[1] = meta[2] = 0; meta[3] = 2048;
  for (int launch = 0; launch < 2; ++launch) {
    tapa::invoke(SparseLUKernel, "",
      tapa::read_only_mmap<WORD512>(banks[0]), tapa::read_only_mmap<WORD512>(banks[1]),
      tapa::read_only_mmap<WORD512>(banks[2]), tapa::read_only_mmap<WORD512>(banks[3]),
      tapa::read_only_mmap<WORD512>(banks[4]), tapa::read_only_mmap<WORD512>(banks[5]),
      tapa::read_only_mmap<WORD512>(banks[6]), tapa::read_only_mmap<WORD512>(banks[7]),
      tapa::read_write_mmap<WORD512>(banks[8]), tapa::read_write_mmap<WORD512>(banks[9]),
      tapa::read_write_mmap<WORD512>(banks[10]), tapa::read_write_mmap<WORD512>(banks[11]),
      tapa::read_write_mmap<WORD512>(banks[12]), tapa::read_write_mmap<WORD512>(banks[13]),
      tapa::read_write_mmap<WORD512>(banks[14]), tapa::read_write_mmap<WORD512>(banks[15]),
      tapa::read_write_mmap<WORD512>(banks[16]), tapa::read_write_mmap<WORD512>(banks[17]),
      tapa::read_write_mmap<WORD512>(banks[18]), tapa::read_write_mmap<WORD512>(banks[19]),
      tapa::read_write_mmap<WORD512>(banks[20]), tapa::read_write_mmap<WORD512>(banks[21]),
      tapa::read_write_mmap<WORD512>(banks[22]), tapa::read_write_mmap<WORD512>(banks[23]),
      tapa::read_only_mmap<uint32_t>(meta), tapa::read_only_mmap<uint32_t>(meta),
      tapa::read_only_mmap<uint32_t>(meta));
    for (const auto& bank : banks) assert(bank[0] == WORD512(0));
    std::cout << "PRODUCT_EMPTY_PASS launch=" << launch << std::endl;
  }
}
