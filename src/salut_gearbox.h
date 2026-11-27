#pragma once
#include "salut_memory_protocol.h"

// Two-beat gearbox for wide memory-datapath links that cross an SLR boundary.
// A wide record is bit-packed, sent as two half-width beats (low half first)
// and reassembled on the far side. Order, content and end tokens are preserved;
// the link carries one record every two cycles. Scheduling streams never use it.
template <typename T> struct SalutGearCodec;

template <> struct SalutGearCodec<RP> {
  static constexpr int W = 512 + 8 + 5 + 7;
  static ap_uint<W> pack(const RP& x) { return (x.count, x.first, x.pe, x.v); }
  static RP unpack(const ap_uint<W>& b) {
    RP x;
    x.v = b(511, 0); x.pe = b(519, 512); x.first = b(524, 520); x.count = b(531, 525);
    return x;
  }
  static bool end(const RP&) { return false; }
};

template <> struct SalutGearCodec<WP> {
  static constexpr int W = 512 + 8 + 4;
  static ap_uint<W> pack(const WP& x) { return (x.count, x.pe, x.v); }
  static WP unpack(const ap_uint<W>& b) {
    WP x;
    x.v = b(511, 0); x.pe = b(519, 512); x.count = b(523, 520);
    return x;
  }
  static bool end(const WP&) { return false; }
};

template <> struct SalutGearCodec<SalutResponseEnvelope> {
  static constexpr int W = SalutGearCodec<RP>::W + 1;
  static ap_uint<W> pack(const SalutResponseEnvelope& x) {
    return (ap_uint<1>(x.end), SalutGearCodec<RP>::pack(x.rsp));
  }
  static SalutResponseEnvelope unpack(const ap_uint<W>& b) {
    SalutResponseEnvelope x;
    x.rsp = SalutGearCodec<RP>::unpack(b(W - 2, 0)); x.end = b[W - 1];
    return x;
  }
  static bool end(const SalutResponseEnvelope& x) { return x.end; }
};

template <> struct SalutGearCodec<SalutWordReadRsp> {
  static constexpr int W = 512 + 32 + 1;
  static ap_uint<W> pack(const SalutWordReadRsp& x) {
    return (ap_uint<1>(x.end), ap_uint<32>(x.ticket), x.data);
  }
  static SalutWordReadRsp unpack(const ap_uint<W>& b) {
    SalutWordReadRsp x;
    x.data = b(511, 0); x.ticket = b(543, 512).to_uint(); x.end = b[544];
    return x;
  }
  static bool end(const SalutWordReadRsp& x) { return x.end; }
};

template <> struct SalutGearCodec<SalutWritebackWord> {
  static constexpr int W = 512 + 32 + 32 + 1;
  static ap_uint<W> pack(const SalutWritebackWord& x) {
    return (ap_uint<1>(x.end), ap_uint<32>(x.ticket), ap_uint<32>(x.address), x.data);
  }
  static SalutWritebackWord unpack(const ap_uint<W>& b) {
    SalutWritebackWord x;
    x.data = b(511, 0); x.address = b(543, 512).to_uint(); x.ticket = b(575, 544).to_uint(); x.end = b[576];
    return x;
  }
  static bool end(const SalutWritebackWord& x) { return x.end; }
};

template <> struct SalutGearCodec<SalutWriteBeat> {
  static constexpr int W = 512 + 32 + 32 + 32 + 8 + 8 + 3;
  static ap_uint<W> pack(const SalutWriteBeat& x) {
    return (ap_uint<1>(x.end), ap_uint<1>(x.no_data), ap_uint<1>(x.last), x.kind, x.client,
            ap_uint<32>(x.ordinal), ap_uint<32>(x.seq), ap_uint<32>(x.address), x.data);
  }
  static SalutWriteBeat unpack(const ap_uint<W>& b) {
    SalutWriteBeat x;
    x.data = b(511, 0); x.address = b(543, 512).to_uint(); x.seq = b(575, 544).to_uint(); x.ordinal = b(607, 576).to_uint();
    x.client = b(615, 608); x.kind = b(623, 616); x.last = b[624]; x.no_data = b[625]; x.end = b[626];
    return x;
  }
  static bool end(const SalutWriteBeat& x) { return x.end; }
};

// N-beat gearbox: one beat per cycle, so one record every N cycles. Beats are sent
// least-significant first; the last beat may carry zero padding above bit W-1.
// Both loops are non-blocking (try_read / full checks), like the other fabric tasks:
// a blocking read inside a pipelined loop with a data-dependent exit lets HLS start the
// next iteration's read early, which stalled the last beat of a lone record in RTL.
template <typename T, int N>
using SalutGearBeat = ap_uint<(SalutGearCodec<T>::W + N - 1) / N>;

template <typename T, int N>
void SalutGearSplit(tapa::istream<T>& in, tapa::ostream<SalutGearBeat<T, N>>& out) {
  constexpr int W = SalutGearCodec<T>::W, H = (W + N - 1) / N;
  ap_uint<N * H> buf = 0;
  bool have = false, last = false, done = false;
  int k = 0;
  while (!done) {
#pragma HLS pipeline II=1
    if (!have) {
      T x;
      if (in.try_read(x)) {
        buf = SalutGearCodec<T>::pack(x);
        last = SalutGearCodec<T>::end(x);
        have = true;
        k = 0;
      }
    }
    if (have && !out.full()) {
      out.write(buf(H - 1, 0));
      buf >>= H;
      if (k == N - 1) { have = false; done = last; } else { ++k; }
    }
  }
}

template <typename T, int N>
void SalutGearJoin(tapa::istream<SalutGearBeat<T, N>>& in, tapa::ostream<T>& out) {
  constexpr int W = SalutGearCodec<T>::W, H = (W + N - 1) / N;
  ap_uint<N * H> buf = 0;
  T held{};
  bool have_out = false, done = false;
  int k = 0;
  while (!done) {
#pragma HLS pipeline II=1
    if (have_out && !out.full()) {
      out.write(held);
      have_out = false;
      done = SalutGearCodec<T>::end(held);
    }
    if (!have_out && !done) {
      ap_uint<H> beat;
      if (in.try_read(beat)) {
        buf = (beat, ap_uint<(N - 1) * H>(buf(N * H - 1, H)));
        if (k == N - 1) {
          held = SalutGearCodec<T>::unpack(buf(W - 1, 0));
          have_out = true;
          k = 0;
        } else {
          ++k;
        }
      }
    }
  }
}
