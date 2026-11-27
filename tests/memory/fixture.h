#pragma once
#include "lu_kernel.h"
#include <cstdint>
#include <map>
#include <string>

struct MemoryTrace {
  std::map<std::string, uint64_t> counts;
  std::map<std::string, WORD512> words;
  bool fault = false;
};
using MemoryScenario = MemoryTrace (*)();
void RegisterMemoryScenario(const std::string& name, MemoryScenario run);
MemoryTrace RunMemoryScenario(const std::string& name);
