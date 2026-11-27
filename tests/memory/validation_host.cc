#include "lu_kernel.h"
#include "salut_memory_protocol.h"
#include <cassert>
#include <iostream>
#include <vector>

void SalutMemoryValidationTop(tapa::mmap<WORD512>,tapa::mmap<WORD512>,tapa::mmap<WORD512>,
    tapa::mmap<WORD512>,tapa::mmap<WORD512>,uint32_t,uint32_t);

int main() {
  using Buffer=std::vector<WORD512,tapa::aligned_allocator<WORD512>>;
  for (uint32_t bank : {0u,1u}) {
    Buffer l(4),u(4),a(4),commands(16),results(17);
    a[0] = WORD512(0x33);
    auto add = [&](int i,uint8_t op,uint8_t kind,uint8_t client,uint32_t offset,uint32_t length,WORD512 word) {
      WORD512 header=0;
      header.range(7,0)=op; header.range(15,8)=kind; header.range(23,16)=client;
      header.range(63,32)=pack_addr(bank+8*kind,offset); header.range(95,64)=length;
      commands[2*i]=header; commands[2*i+1]=word;
    };
    add(0,1,1,1,0,8,WORD512(0x11)); add(1,0,1,1,3,2,0);
    // PEs are PEG 0's clients 1..PAR_PE_PER_PEG; the front is front 0.
    add(2,1,2,SALUT_FRONT_CLIENT0,8,8,WORD512(0x22)); add(3,0,2,SALUT_FRONT_CLIENT0,8,8,0);
    add(4,0,2,2,8,8,0); add(5,0,0,1,0,8,0);
    add(6,1,1,PAR_PE_PER_PEG,16,0,0); add(7,1,2,SALUT_FRONT_CLIENT0,16,0,0);
    tapa::invoke(SalutMemoryValidationTop,"",tapa::read_write_mmap<WORD512>(l),
        tapa::read_write_mmap<WORD512>(u),tapa::read_only_mmap<WORD512>(a),
        tapa::read_only_mmap<WORD512>(commands),tapa::read_write_mmap<WORD512>(results),8u,bank);
    assert(results[3]==WORD512(0x11) && results[7]==WORD512(0x22));
    assert(results[9]==WORD512(0x22) && results[11]==WORD512(0x33));
    assert(l[0]==WORD512(0x11) && u[1]==WORD512(0x22));
    assert(l[2]==WORD512(0) && u[2]==WORD512(0));
    assert(results[16]==WORD512(0x534D56414C504153ULL));
    std::cout << "VALIDATION_DIAGNOSTIC_SOFTWARE_PASS bank=" << bank << std::endl;
  }
}
