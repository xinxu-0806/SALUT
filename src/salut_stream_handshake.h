#pragma once
#include <cstddef>

// Event adapters have already checked a FIFO head or retained a ready output.
// NEVER introduce a blocking HLS read/write here: independent ports otherwise
// acquire a serial empty/full control chain and a very large scheduling graph.
// Return true on an impossible handshake mismatch so the owner can latch a
// sticky fault and withhold successful completion. No retry/drop-as-success.
template <typename Stream, typename Value>
static inline bool SalutConsume(Stream&& stream, bool transfer, Value& value) {
#pragma HLS inline
  return transfer ? !stream.try_read(value) : false;
}
template <typename Stream, typename Value>
static inline bool SalutProduce(Stream&& stream, bool transfer, const Value& value) {
#pragma HLS inline
  return transfer ? !stream.try_write(value) : false;
}

// Each port produces its OWN error predicate. Updating the owner's fault
// after each conditional FIFO operation creates serial SSA phi/control muxes
// across the unrolled ports. Reduce independently, then latch fault ONCE.
template <std::size_t N>
static inline bool SalutAnyHandshakeFailure(const bool (&failures)[N]) {
#pragma HLS inline
  bool failed = false;
  for (std::size_t channel=0; channel<N; ++channel) {
#pragma HLS unroll
    failed |= failures[channel];
  }
  return failed;
}
