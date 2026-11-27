#include "fixture.h"
#include <stdexcept>

static std::map<std::string, MemoryScenario>& Scenarios() {
  static std::map<std::string, MemoryScenario> scenarios;
  return scenarios;
}
void RegisterMemoryScenario(const std::string& name, MemoryScenario run) {
  if (!run || !Scenarios().emplace(name, run).second)
    throw std::runtime_error("invalid/duplicate scenario: " + name);
}
MemoryTrace RunMemoryScenario(const std::string& name) {
  const auto found = Scenarios().find(name);
  if (found == Scenarios().end())
    throw std::runtime_error("missing/unknown scenario: " + name);
  return found->second();
}
