#include <ap_int.h>
#include <tapa.h>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <memory>
#include "lu_kernel.h"   // 类型/常量/宏定义接口
#include "salut_memory_fabric.inc"
#include "salut_hbm_word_port.inc"
#include "salut_sm_bank.inc"
#include "salut_memory_commit.inc"
#include "salut_gearbox_tasks.inc"
// 注意：不包含 type.h，因为它是主机端头文件，会导致 TAPA 展平时出错

// 启用全局LU缓存（覆盖任何外部定义），以减少重复访问延迟
#undef DISABLE_GLOBAL_CACHE
#define DISABLE_GLOBAL_CACHE 0

// 调试宏 - 使用 do-while 包装确保安全
// 收敛日志：关闭普通与重要日志，仅保留汇总日志
#ifdef __SYNTHESIS__
#define DEBUG_PRINT(msg) do { } while(0)
#define DEBUG_PRINT_IMPORTANT(msg) do { } while(0)
#define SUMMARY_PRINT(msg) do { } while(0)
#else
#define DEBUG_PRINT(msg) do { std::cout << "[DETAIL] " << msg << std::endl; } while(0)
#define DEBUG_PRINT_IMPORTANT(msg) do { std::cout << "[DEBUG] " << msg << std::endl; } while(0)
#define SUMMARY_PRINT(msg) do { std::cout << "[INFO] " << msg << std::endl; } while(0)
#endif

#ifndef __SYNTHESIS__
static uint64_t g_front_sm_hits = 0;
static uint64_t g_front_sm_misses = 0;
static uint64_t g_hbm_req_issued = 0;
static uint64_t g_hbm_rsp_routed = 0;
static uint64_t g_hbm_bank_rsp = 0;
static uint64_t g_hbm_lane_rsp = 0;
static uint64_t g_sm_fill_rsp = 0;
static uint64_t g_lm_rsp = 0;
static uint32_t g_hbm_req_max_parallel = 0;
static uint32_t g_hbm_rsp_max_parallel = 0;
static bool g_sim_direct_dependencies = false;
static META_T* g_sim_meta_ptr = nullptr;
#endif

// 正常模式：处理所有任务 
//======================================================================
//  工具函数：512b <-> 64b 打包解包
//======================================================================
static inline WORD64 get_word64(WORD512 w, int k) {
#pragma HLS inline
  // Avoid a 512-bit variable barrel shifter.  Vitis 2022.2 otherwise lowers
  // range(k*64+63, k*64) to a ~2K-LUT dynamic shift, which was the routed
  // critical path at 160 MHz.  Static slices synthesize as a small 8:1 mux.
  switch (ap_uint<3>(k)) {
    case 0: return w.range(63, 0);
    case 1: return w.range(127, 64);
    case 2: return w.range(191, 128);
    case 3: return w.range(255, 192);
    case 4: return w.range(319, 256);
    case 5: return w.range(383, 320);
    case 6: return w.range(447, 384);
    default: return w.range(511, 448);
  }
}

static inline WORD64 get_read_packet_element(
    const ReadPacketTagged& packet, const uint32_t lane) {
#pragma HLS inline
  const uint32_t absolute_lane = uint32_t(packet.first) + lane;
  return get_word64(packet.v, absolute_lane & 7u);
}

static inline void set_word64(WORD512 &w, int k, WORD64 v) {
#pragma HLS inline
  // Use fixed slice writes for the same reason as get_word64 above.
  switch (ap_uint<3>(k)) {
    case 0: w.range(63, 0) = v; break;
    case 1: w.range(127, 64) = v; break;
    case 2: w.range(191, 128) = v; break;
    case 3: w.range(255, 192) = v; break;
    case 4: w.range(319, 256) = v; break;
    case 5: w.range(383, 320) = v; break;
    case 6: w.range(447, 384) = v; break;
    default: w.range(511, 448) = v; break;
  }
}

//======================================================================
//  稀疏格式槽位解析函数
//======================================================================
// DUMMY元素标记: 0xFFFF在高位(63-48位)
const ap_uint<16> DUMMY_MARK = 0xFFFF;
// 从64位元素中提取row_offset(低16位)和浮点值(高32位)
static inline bool is_dummy_element(WORD64 elem) {
#pragma HLS inline
  // 检查63-48位是否为0xFFFF
  ap_uint<16> high_16 = elem.range(63, 48);
  return (high_16 == DUMMY_MARK);
}

static inline uint16_t extract_row_offset(WORD64 elem) {
#pragma HLS inline
  // 提取低16位作为row_offset
  return elem.range(15, 0);
}

static inline REAL extract_value(WORD64 elem) {
#pragma HLS inline
  // 提取63-32位作为浮点值
  ap_uint<32> val_bits = elem.range(63, 32);
  return tapa::bit_cast<REAL>(val_bits);
}

//======================================================================
//  ReadySegBroadcaster：将ReadySeg任务广播到两个输出流
//  使用单一模块但明确的输出路径，避免TAPA FIFO方向推断问题
//======================================================================
void ReadySegBroadcaster(
  tapa::istreams<ReadySeg, NUM_PE>& ready_in,
  tapa::ostreams<ReadySeg, NUM_PE>& ready_to_pe,
  tapa::ostreams<ReadySeg, NUM_PE>& ready_to_depexp
) {
  DEBUG_PRINT_IMPORTANT("ReadySegBroadcaster: Started");
  uint32_t tasks_broadcasted = 0;
  uint32_t iterations = 0;
  ap_uint<4> rr = 0;  // 轮询指针
  
  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000
    iterations++;
    
    if (iterations % 1000000 == 0) {
      DEBUG_PRINT_IMPORTANT("ReadySegBroadcaster: iterations=" << iterations);
    }
    
    // 使用安全的索引访问，避免引用计数问题
    uint8_t i = (uint8_t)(rr % NUM_PE);
    
    // 使用empty()检查而非直接读取，减少引用计数
    // 在软件仿真中，避免频繁访问流数组可以减少引用计数问题
    if (i < NUM_PE && !ready_in[i].empty()) {
      ReadySeg task = ready_in[i].read();
      // 同时写入两个输出流（广播）
      if (i < NUM_PE) {
        ready_to_pe[i].write(task);
      }
      if (i < NUM_PE) {
        ready_to_depexp[i].write(task);
      }
      rr = (ap_uint<4>)((i + 1) % NUM_PE);
      tasks_broadcasted++;
      
      if (tasks_broadcasted <= 20) {
        DEBUG_PRINT_IMPORTANT("ReadySegBroadcaster: ✅ Broadcasted task #" << tasks_broadcasted 
                              << " from PEG" << (int)i << " seg_id=" << task.seg_id);
      }
    } else {
      // 如果当前流为空，更新轮询指针
      rr = (ap_uint<4>)((rr + 1) % NUM_PE);
    }
  }
}

//======================================================================
//  MetaLoader：读取 Host 打包的 META 表并分流到下游
//  使用普通 mmap（与顶层函数签名匹配）
//======================================================================
void MetaLoader(
  tapa::mmap<META_T> meta,                              // 主机端 META 区映射
  tapa::ostream<uint32_t>& meta_idx_out,                // 输出头部参数（给 DegScheduler）- 逐字段传输
  tapa::ostream<uint32_t>& meta_idx_poster_out,         // 输出头部参数（给 Poster）- 逐字段传输
  tapa::ostream<META_T>& seg_table_out,                 // 段表流
  tapa::ostream<META_T>& dep_ptr_out_deg,               // 依赖 ptr 流（到 Deg）
  tapa::ostream<META_T>& dep_ptr_out_poster,            // 依赖 ptr 流（到 Poster）
  tapa::ostream<META_T>& dep_flat_out_deg,              // 依赖 flat 流（到 Deg）
  tapa::ostream<META_T>& dep_flat_out_poster,           // 依赖 flat 流（到 Poster）
  tapa::ostream<META_T>& deg_init_out,                  // 初始入度流
  tapa::ostream<META_T>& posting_ptr_out,               // posting ptr 流
  tapa::ostream<META_T>& posting_flat_out               // posting flat 流
) {
  DEBUG_PRINT_IMPORTANT("MetaLoader: 启动 Started (using mmap)");
  META_T hdr[12];                                       // 暂存 12 项头部
#pragma HLS bind_storage variable=hdr type=RAM_1P impl=BRAM

  // 使用普通 mmap 方式读取头部
  DEBUG_PRINT_IMPORTANT("MetaLoader: 开始读取头部 Reading header (12 words)...");
  for (int i = 0; i < 12; i++) {
#pragma HLS pipeline II=1
    hdr[i] = meta[i];
    if (i == 0) DEBUG_PRINT_IMPORTANT("MetaLoader: 第一个头部字 First header word, value=" << hdr[i]);
  }
  DEBUG_PRINT_IMPORTANT("MetaLoader: 头部读取完成 Header read complete");
  
  // 打印原始头部数据用于调试
  DEBUG_PRINT("MetaLoader: Raw header data:");
  for (int i = 0; i < 12; i++) {
    DEBUG_PRINT("  hdr[" << i << "] = " << hdr[i]);
  }

  MetaIndex mi;                                         // 解析 META 头
  mi.n = hdr[0]; mi.seg_rows = hdr[1]; mi.nSeg = hdr[2];
  mi.off_seg_table = hdr[3]; mi.off_seg_of_col_ptr = hdr[4];
  mi.off_seg_of_col_flat = hdr[5]; mi.off_dep_ptr = hdr[6];
  mi.off_dep_flat = hdr[7]; mi.off_deg_init = hdr[8];
  mi.off_post_ptr = hdr[9]; mi.off_post_flat = hdr[10];
  mi.off_dummy = hdr[11];
  
  DEBUG_PRINT("MetaLoader:  META头部信息 Header info:");
  DEBUG_PRINT("  n=" << mi.n << " nSeg=" << mi.nSeg << " seg_rows=" << mi.seg_rows);
  DEBUG_PRINT("  off_seg_table=" << mi.off_seg_table << " off_dep_ptr=" << mi.off_dep_ptr);
  DEBUG_PRINT("  off_dep_flat=" << mi.off_dep_flat << " off_deg_init=" << mi.off_deg_init);
  DEBUG_PRINT("  off_post_ptr=" << mi.off_post_ptr << " off_post_flat=" << mi.off_post_flat);
  DEBUG_PRINT("  off_dummy=" << mi.off_dummy);
  
  // 验证偏移量的合理性
  DEBUG_PRINT("MetaLoader: 🔍 Validating offsets...");
  DEBUG_PRINT("  Checking: off_seg_table=" << mi.off_seg_table << " should be 12");
  DEBUG_PRINT("  Checking: off_dep_ptr=" << mi.off_dep_ptr << " should be " << (12 + mi.nSeg * 6));
  DEBUG_PRINT("  Checking: off_dummy=" << mi.off_dummy);
  
  if (mi.off_seg_table != 12) {
    DEBUG_PRINT("MetaLoader: ❌ ERROR: off_seg_table=" << mi.off_seg_table << " != 12");
  }
  if (mi.off_dummy > 1000000) {
    DEBUG_PRINT("MetaLoader: ❌ ERROR: off_dummy=" << mi.off_dummy << " seems too large!");
  }
  
  // 逐字段发送 MetaIndex 到 DegScheduler（12个字段）
  DEBUG_PRINT_IMPORTANT("MetaLoader: 开始逐字段发送MetaIndex到DegScheduler Sending MetaIndex fields to DegScheduler...");
  meta_idx_out.write(mi.n);
  DEBUG_PRINT_IMPORTANT("MetaLoader: 已发送 n=" << mi.n);
  meta_idx_out.write(mi.seg_rows);
  meta_idx_out.write(mi.nSeg);
  meta_idx_out.write(mi.off_seg_table);
  meta_idx_out.write(mi.off_seg_of_col_ptr);
  meta_idx_out.write(mi.off_seg_of_col_flat);
  meta_idx_out.write(mi.off_dep_ptr);
  meta_idx_out.write(mi.off_dep_flat);
  meta_idx_out.write(mi.off_deg_init);
  meta_idx_out.write(mi.off_post_ptr);
  meta_idx_out.write(mi.off_post_flat);
  meta_idx_out.write(mi.off_dummy);
  DEBUG_PRINT_IMPORTANT("MetaLoader: MetaIndex已发送到DegScheduler (12 fields) MetaIndex sent to DegScheduler");
  
  // 逐字段发送 MetaIndex 到 Poster（12个字段）
  DEBUG_PRINT_IMPORTANT("MetaLoader: 开始逐字段发送MetaIndex到Poster Sending MetaIndex fields to Poster...");
  meta_idx_poster_out.write(mi.n);
  meta_idx_poster_out.write(mi.seg_rows);
  meta_idx_poster_out.write(mi.nSeg);
  meta_idx_poster_out.write(mi.off_seg_table);
  meta_idx_poster_out.write(mi.off_seg_of_col_ptr);
  meta_idx_poster_out.write(mi.off_seg_of_col_flat);
  meta_idx_poster_out.write(mi.off_dep_ptr);
  meta_idx_poster_out.write(mi.off_dep_flat);
  meta_idx_poster_out.write(mi.off_deg_init);
  meta_idx_poster_out.write(mi.off_post_ptr);
  meta_idx_poster_out.write(mi.off_post_flat);
  meta_idx_poster_out.write(mi.off_dummy);
  DEBUG_PRINT_IMPORTANT("MetaLoader: MetaIndex已发送到Poster (12 fields) MetaIndex sent to Poster");

  // 数据完整性验证计数器
  uint32_t total_words_sent = 0;
  
  // Send all META data tables sequentially (inline without lambda)
  // 1. seg_table
  DEBUG_PRINT_IMPORTANT("MetaLoader: 开始传输seg_table Starting seg_table transfer...");
  {
    uint32_t lo = mi.off_seg_table;
    uint32_t hi = mi.off_dep_ptr;
    uint32_t total = hi - lo;
    DEBUG_PRINT_IMPORTANT("MetaLoader: seg_table range: lo=" << lo << " hi=" << hi << " total=" << total);
    for (uint32_t i = 0; i < total; i++) {
#pragma HLS pipeline II=1
      META_T data = meta[lo + i];
      seg_table_out.write(data);
      if (i > 0 && i % 1000 == 0) {
        DEBUG_PRINT_IMPORTANT("MetaLoader: seg_table progress=" << i << "/" << total);
      }
    }
    total_words_sent += total;
    DEBUG_PRINT_IMPORTANT("MetaLoader: seg_table传输完成 drain complete, words=" << total);
  }
  
  // 2. dep_ptr (直接双写)
  DEBUG_PRINT("MetaLoader: 开始传输dep_ptr Starting dep_ptr transfer...");
  {
    uint32_t lo = mi.off_dep_ptr;
    uint32_t hi = mi.off_dep_flat;
    uint32_t total = hi - lo;
    for (uint32_t i = 0; i < total; i++) {
#pragma HLS pipeline II=1
      META_T data = meta[lo + i];
      dep_ptr_out_deg.write(data);
      // 使用阻塞写，确保 Poster 完整接收，避免元数据丢失导致死锁
      dep_ptr_out_poster.write(data);
    }
    total_words_sent += total;
    DEBUG_PRINT("MetaLoader: dep_ptr传输完成 drain complete, words=" << total);
  }
  
  // 3. dep_flat (直接双写)
  DEBUG_PRINT("MetaLoader: 开始传输dep_flat Starting dep_flat transfer...");
  {
    uint32_t lo = mi.off_dep_flat;
    uint32_t hi = mi.off_deg_init;
    uint32_t total = hi - lo;
    for (uint32_t i = 0; i < total; i++) {
#pragma HLS pipeline II=1
      META_T data = meta[lo + i];
      dep_flat_out_deg.write(data);
      // 使用阻塞写，确保 Poster 完整接收，避免元数据丢失导致死锁
      dep_flat_out_poster.write(data);
    }
    total_words_sent += total;
    DEBUG_PRINT("MetaLoader: dep_flat传输完成 drain complete, words=" << total);
  }
  
  // 4. deg_init
  DEBUG_PRINT("MetaLoader: 开始传输deg_init Starting deg_init transfer...");
  {
    uint32_t lo = mi.off_deg_init;
    uint32_t hi = mi.off_post_ptr;
    uint32_t total = hi - lo;
    for (uint32_t i = 0; i < total; i++) {
#pragma HLS pipeline II=1
      META_T data = meta[lo + i];
      deg_init_out.write(data);
    }
    total_words_sent += total;
    DEBUG_PRINT("MetaLoader: deg_init传输完成 drain complete, words=" << total);
  }
  
  // 5. posting_ptr
  DEBUG_PRINT("MetaLoader: 开始传输posting_ptr Starting posting_ptr transfer...");
  {
    uint32_t lo = mi.off_post_ptr;
    uint32_t hi = mi.off_post_flat;
    uint32_t total = hi - lo;
    for (uint32_t i = 0; i < total; i++) {
#pragma HLS pipeline II=1
      META_T data = meta[lo + i];
      posting_ptr_out.write(data);
    }
    total_words_sent += total;
    DEBUG_PRINT("MetaLoader: posting_ptr传输完成 drain complete, words=" << total);
  }
  
  // 6. posting_flat
  DEBUG_PRINT("MetaLoader: 开始传输posting_flat Starting posting_flat transfer...");
  {
    uint32_t lo = mi.off_post_flat;
    uint32_t hi = mi.off_dummy;
    uint32_t total = hi - lo;
    for (uint32_t i = 0; i < total; i++) {
#pragma HLS pipeline II=1
      META_T data = meta[lo + i];
      posting_flat_out.write(data);
    }
    total_words_sent += total;
    DEBUG_PRINT("MetaLoader: posting_flat传输完成 drain complete, words=" << total);
  }
  
  // 数据完整性验证
  uint32_t expected_total = mi.off_dummy - mi.off_seg_table;
  DEBUG_PRINT("MetaLoader: 🔍 数据完整性验证 Data integrity check:");
  DEBUG_PRINT("  Expected total words: " << expected_total);
  DEBUG_PRINT("  Actual sent words: " << total_words_sent);
  
  if (total_words_sent == expected_total) {
    DEBUG_PRINT("MetaLoader: 数据完整性验证通过 Data integrity check PASSED");
  } else {
    DEBUG_PRINT("MetaLoader: 数据完整性验证失败 Data integrity check FAILED!");
  }
  
  DEBUG_PRINT("MetaLoader: 所有META数据已发送，准备退出 All META data sent, exiting");
  DEBUG_PRINT("MetaLoader: 完成并退出 Complete and exiting");
}

//======================================================================
//  DegScheduler：图驱动入度递减调度（col_done → ready）
//======================================================================
void DegScheduler(
  tapa::istream<uint32_t>& meta_idx_in,                 // 输入 META 头 - 逐字段接收
  tapa::istream<META_T>& seg_table_in,                  // 段表流
  tapa::istream<META_T>& dep_ptr_in,                    // 依赖 ptr 流
  tapa::istream<META_T>& dep_flat_in,                   // 依赖 flat 流
  tapa::istream<META_T>& deg_init_in,                   // 初始入度
  tapa::istream<META_T>& posting_ptr_in,                // posting ptr
  tapa::istream<META_T>& posting_flat_in,               // posting flat
  tapa::ostreams<ReadySeg, PEGS>& ready_local_out,      // 每 PEG 的 ready 输出
  tapa::ostream<ReadySeg>& ready_global_out,            // 全局 ready 输出
  tapa::istream<uint32_t>& seg_done_in,                 // segment 写回 ACK 后的完成事件
  tapa::ostream<bool>& done_out                         // 完成信号输出
) {
  DEBUG_PRINT_IMPORTANT("DegScheduler: Started");
  
  // 逐字段接收 MetaIndex（12个字段）
  DEBUG_PRINT("DegScheduler: 📥 开始逐字段接收MetaIndex Receiving MetaIndex fields...");
  MetaIndex mi;
  mi.n = meta_idx_in.read();
  mi.seg_rows = meta_idx_in.read();
  mi.nSeg = meta_idx_in.read();
  mi.off_seg_table = meta_idx_in.read();
  mi.off_seg_of_col_ptr = meta_idx_in.read();
  mi.off_seg_of_col_flat = meta_idx_in.read();
  mi.off_dep_ptr = meta_idx_in.read();
  mi.off_dep_flat = meta_idx_in.read();
  mi.off_deg_init = meta_idx_in.read();
  mi.off_post_ptr = meta_idx_in.read();
  mi.off_post_flat = meta_idx_in.read();
  mi.off_dummy = meta_idx_in.read();
  
  const uint32_t nSeg = mi.nSeg, nCol = mi.n;
  DEBUG_PRINT("DegScheduler: MetaIndex接收完成 MetaIndex received (12 fields)");
  DEBUG_PRINT("DegScheduler: Received values:");
  DEBUG_PRINT("  n=" << mi.n << " nSeg=" << nSeg << " seg_rows=" << mi.seg_rows);
  DEBUG_PRINT("  off_seg_table=" << mi.off_seg_table << " off_dep_ptr=" << mi.off_dep_ptr);
  DEBUG_PRINT("  off_dep_flat=" << mi.off_dep_flat << " off_deg_init=" << mi.off_deg_init);
  DEBUG_PRINT("  off_post_ptr=" << mi.off_post_ptr << " off_post_flat=" << mi.off_post_flat);
  DEBUG_PRINT("  off_dummy=" << mi.off_dummy);

  // 优化：减小段表大小
  static SegRec seg_table[65536];                        // 64K (减小16倍)
#pragma HLS bind_storage variable=seg_table type=RAM_1P impl=BRAM
  DEBUG_PRINT_IMPORTANT("DegScheduler: 开始加载seg_table Loading seg_table, nSeg=" << nSeg << " (需要 " << (nSeg*9) << " words)");
  DEBUG_PRINT_IMPORTANT("DegScheduler: 准备进入seg_table读取循环 About to enter seg_table read loop");
  uint32_t seg_words_read = 0;
  for (uint32_t i = 0; i < nSeg; i++) {
#pragma HLS pipeline II=1
    if (i == 0) DEBUG_PRINT_IMPORTANT("DegScheduler:   读取第一个segment Reading first segment...");
    SegRec r;
    r.j = seg_table_in.read(); seg_words_read++;
    r.s = seg_table_in.read(); seg_words_read++;
    r.row_lo = seg_table_in.read(); seg_words_read++;
    r.row_hi = seg_table_in.read(); seg_words_read++;
    r.A_addr = seg_table_in.read(); seg_words_read++;
    r.word_count = seg_table_in.read(); seg_words_read++;
    r.L_base_addr = seg_table_in.read(); seg_words_read++;
    r.U_diag_addr = seg_table_in.read(); seg_words_read++;
    r.is_last = seg_table_in.read(); seg_words_read++;
    seg_table[i] = r;
    if (i == 0) {
      DEBUG_PRINT_IMPORTANT("DegScheduler:   第一个segment读取完成 First segment read complete");
      DEBUG_PRINT_IMPORTANT("DegScheduler:     j=" << r.j << " s=" << r.s << " row_lo=" << r.row_lo << " row_hi=" << r.row_hi << " is_last=" << r.is_last);
    }
    // 统计 is_last=1 的段数
    if (r.is_last && i < 100) {
      DEBUG_PRINT("DegScheduler:   seg_id=" << i << " is_last=1 j=" << r.j);
    }
    if (i > 0 && i % 250 == 0) { // 更频繁地报告进度
      DEBUG_PRINT_IMPORTANT("DegScheduler: seg_table 加载进度 progress=" << i << "/" << nSeg);
    }
  }
  DEBUG_PRINT_IMPORTANT("DegScheduler: seg_table加载完成 loaded, nSeg=" << nSeg << " total_words_read=" << seg_words_read);


  // 优化：减小依赖指针数组大小
  static uint32_t dep_ptr[65536 + 1];                    // 64K (减小16倍)
#pragma HLS bind_storage variable=dep_ptr type=RAM_1P impl=BRAM
  DEBUG_PRINT("DegScheduler: 📋 开始加载dep_ptr Loading dep_ptr, entries=" << (nSeg+1));
  uint32_t deg_dep_ptr_words = 0;
  for (uint32_t i = 0; i <= nSeg; i++) {
#pragma HLS pipeline II=1
    if (i == 0) DEBUG_PRINT("DegScheduler:   读取第一个dep_ptr Reading first dep_ptr...");
    dep_ptr[i] = dep_ptr_in.read();
    deg_dep_ptr_words++;
    if (i == 0) DEBUG_PRINT("DegScheduler:   第一个dep_ptr读取完成 First dep_ptr read, value=" << dep_ptr[i]);
    if (i > 0 && i % 250 == 0) { // 更频繁地报告进度
      DEBUG_PRINT_IMPORTANT("DegScheduler: dep_ptr 加载进度 progress=" << i << "/" << (nSeg+1));
    }
  }
  const uint32_t dep_entries = dep_ptr[nSeg];
  DEBUG_PRINT("DegScheduler: dep_ptr加载完成 loaded, dep_entries=" << dep_entries << " total_words=" << deg_dep_ptr_words);

  // 优化：减小依赖flat数组大小
  static DepEntry dep_flat[262144];                      // 256K (减小16倍)
#pragma HLS bind_storage variable=dep_flat type=RAM_1P impl=BRAM
  DEBUG_PRINT("DegScheduler: 📋 开始加载dep_flat Loading dep_flat, entries=" << dep_entries << " (需要 " << (dep_entries*4) << " words)");
  uint32_t deg_dep_flat_words = 0;
  for (uint32_t i = 0; i < dep_entries; i++) {
#pragma HLS pipeline II=1
    if (i == 0) DEBUG_PRINT("DegScheduler:   读取第一个dep_flat Reading first dep_flat...");
    DepEntry d;
    d.k      = dep_flat_in.read();
    d.L_addr = dep_flat_in.read();
    d.len    = dep_flat_in.read();
    d.U_addr = dep_flat_in.read();
    dep_flat[i] = d;
    if (i == 0) {
      DEBUG_PRINT("DegScheduler:   第一个dep_flat读取完成 First dep_flat read");
      DEBUG_PRINT("DegScheduler:     k=" << d.k << " L_addr=" << d.L_addr << " len=" << d.len << " U_addr=" << d.U_addr);
    }
    if (i % 1000 == 0 && i > 0) {
      DEBUG_PRINT("DegScheduler:   dep_flat进度 progress=" << i << "/" << dep_entries << " (已读 " << deg_dep_flat_words << " words)");
    }
  }
  DEBUG_PRINT("DegScheduler: dep_flat加载完成 loaded, total_words=" << deg_dep_flat_words);

  // 优化：减小入度数组大小
  static uint32_t deg_cur[65536];                        // 64K (减小16倍)
#pragma HLS bind_storage variable=deg_cur type=RAM_1P impl=BRAM
  DEBUG_PRINT("DegScheduler: 📋 开始加载deg_init Loading deg_init, entries=" << nSeg);
  uint32_t deg_init_words = 0;
  for (uint32_t i = 0; i < nSeg; i++) {
#pragma HLS pipeline II=1
    if (i == 0) DEBUG_PRINT("DegScheduler:   读取第一个deg_init Reading first deg_init...");
    deg_cur[i] = deg_init_in.read();
    deg_init_words++;
    if (i == 0) DEBUG_PRINT("DegScheduler:   第一个deg_init读取完成 First deg_init read, value=" << deg_cur[i]);
    if (i > 0 && i % 250 == 0) { // 更频繁地报告进度
      DEBUG_PRINT_IMPORTANT("DegScheduler: deg_init 加载进度 progress=" << i << "/" << nSeg);
    }
  }
  DEBUG_PRINT("DegScheduler: deg_init加载完成 loaded, total_words=" << deg_init_words);

  // 优化：减小posting指针数组大小
  static uint32_t post_ptr[131072 + 1];                 // 128K (减小16倍)
#pragma HLS bind_storage variable=post_ptr type=RAM_1P impl=BRAM
  DEBUG_PRINT("DegScheduler: 📋 开始加载post_ptr Loading post_ptr, entries=" << (nCol+1));
  uint32_t post_ptr_words = 0;
  for (uint32_t i = 0; i <= nCol; i++) {
#pragma HLS pipeline II=1
    if (i == 0) DEBUG_PRINT("DegScheduler:   读取第一个post_ptr Reading first post_ptr...");
    post_ptr[i] = posting_ptr_in.read();
    post_ptr_words++;
    if (i == 0) DEBUG_PRINT("DegScheduler:   第一个post_ptr读取完成 First post_ptr read, value=" << post_ptr[i]);
    if (i > 0 && i % 250 == 0) { // 更频繁地报告进度
      DEBUG_PRINT_IMPORTANT("DegScheduler: post_ptr 加载进度 progress=" << i << "/" << (nCol+1));
    }
  }
  const uint32_t post_entries = post_ptr[nCol];
  DEBUG_PRINT("DegScheduler: post_ptr加载完成 loaded, post_entries=" << post_entries << " total_words=" << post_ptr_words);

  // 优化：减小posting flat数组大小
  static uint32_t post_flat[524288];                     // 512K (减小16倍)
#pragma HLS bind_storage variable=post_flat type=RAM_1P impl=BRAM
  DEBUG_PRINT("DegScheduler: 📋 开始加载post_flat Loading post_flat, entries=" << post_entries);
  uint32_t post_flat_words = 0;
  for (uint32_t i = 0; i < post_entries; i++) {
#pragma HLS pipeline II=1
    if (i == 0) DEBUG_PRINT("DegScheduler:   读取第一个post_flat Reading first post_flat...");
    post_flat[i] = posting_flat_in.read();
    post_flat_words++;
    if (i == 0) DEBUG_PRINT("DegScheduler:   第一个post_flat读取完成 First post_flat read, value=" << post_flat[i]);
    if (i % 1000 == 0 && i > 0) {
      DEBUG_PRINT("DegScheduler: post_flat进度 progress=" << i << "/" << post_entries << " (已读 " << post_flat_words << " words)");
    }
  }
  DEBUG_PRINT("DegScheduler: post_flat加载完成 loaded, total_words=" << post_flat_words);
  DEBUG_PRINT("DegScheduler: 所有表加载完成 All tables loaded");
  
  // 数据完整性验证
  DEBUG_PRINT("DegScheduler: Final check:");
  DEBUG_PRINT("  nSeg=" << nSeg << " nCol=" << nCol);
  DEBUG_PRINT("  dep_entries=" << dep_entries << " post_entries=" << post_entries);

  // segment 与 column 的完成必须分开计数。只有同一列的所有 segment
  // 都在 HBM 写回 ACK 后报告 seg_done，才能产生唯一的 col_done 事件。
  static uint32_t col_segments_remaining[65536];
  static ap_uint<1> segment_completed[65536];
  static ap_uint<1> column_completed[65536];
#pragma HLS bind_storage variable=col_segments_remaining type=RAM_1P impl=BRAM
#pragma HLS bind_storage variable=segment_completed type=RAM_1P impl=BRAM
#pragma HLS bind_storage variable=column_completed type=RAM_1P impl=BRAM
  for (uint32_t j = 0; j < nCol; ++j) {
#pragma HLS pipeline II=1
    col_segments_remaining[j] = 0;
    column_completed[j] = 0;
  }
  for (uint32_t sid = 0; sid < nSeg; ++sid) {
#pragma HLS pipeline II=1
    segment_completed[sid] = 0;
    const uint32_t j = seg_table[sid].j;
    if (j < nCol) col_segments_remaining[j]++;
  }

  uint32_t initial_ready_count = 0;
  uint32_t initial_ready_local[PEGS] = {0};
  uint32_t initial_ready_global = 0;
  DEBUG_PRINT("DegScheduler: Scanning dependency-free initial segments, nSeg=" << nSeg);
  for (uint32_t sid = 0; sid < nSeg; ++sid) {
#pragma HLS pipeline II=1
    if (deg_cur[sid] != 0) continue;
    const SegRec r = seg_table[sid];
    const uint32_t dep_lo_val = dep_ptr[sid];
    const uint32_t dep_hi_val = dep_ptr[sid + 1];
    const ReadySeg t{sid, r.j, r.row_lo, r.row_hi, r.A_addr,
                     r.word_count, r.L_base_addr, r.U_diag_addr,
                     dep_lo_val, dep_hi_val, r.is_last};
    const int peg = addr_bank(r.A_addr) % PEGS;
    if (!ready_local_out[peg].full()) {
      ready_local_out[peg].write(t);
      initial_ready_local[peg]++;
    } else {
      ready_global_out.write(t);
      initial_ready_global++;
    }
    initial_ready_count++;
  }
  DEBUG_PRINT_IMPORTANT("DegScheduler: dependency-free initial segments=" << initial_ready_count);

  uint32_t col_done_count = 0;                          // 已完成列计数
  uint64_t triggered_total = 0;                         // 触发总数
  uint32_t col_done_received_total = 0;                 // 聚合统计：收到的col_done总数
  SUMMARY_PRINT("DegScheduler: Entering main loop");
  uint32_t wait_iterations = 0;
  for (;;) {                                            // 动态触发循环
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    if (col_done_count >= nCol) {
      DEBUG_PRINT_IMPORTANT("DegScheduler: All columns done, exiting main loop");
      break;                  // 所有列完成→退出
    }
    
    if (seg_done_in.empty()) {
      wait_iterations++;
      // 注释掉频繁的等待日志以减少输出
      // if (wait_iterations % 100000 == 0) {
      //   DEBUG_PRINT("DegScheduler: Waiting for col_done, iterations=" << wait_iterations << " received=" << col_done_count);
      // }
      continue;
    }
    
    wait_iterations = 0;
    const uint32_t completed_sid = seg_done_in.read();  // 只接收 HBM 写 ACK 后的段完成
    if (completed_sid >= nSeg || segment_completed[completed_sid]) {
      // 非法/重复 ACK 不得二次触发列完成。
      continue;
    }
    segment_completed[completed_sid] = 1;
    const uint32_t k = seg_table[completed_sid].j;
    if (k >= nCol || col_segments_remaining[k] == 0) continue;

    col_segments_remaining[k]--;
    if (col_segments_remaining[k] != 0) {
      // 同列还有段未写回，不允许释放任何后继段。
      continue;
    }
    if (column_completed[k]) continue;
    column_completed[k] = 1;
    col_done_count++;
    col_done_received_total++;
    
    // 检查 post_ptr 范围
    uint32_t post_start = post_ptr[k];
    uint32_t post_end = post_ptr[k + 1];
    uint32_t post_count = post_end - post_start;
    
    // 在前几次和关键里程碑打印
    if (col_done_count <= 10 || col_done_count % 50 == 0 || col_done_count == nCol) {
      DEBUG_PRINT("DegScheduler: ✅ col_done received #" << col_done_count << "/" << nCol << " for column k=" << k << " post_count=" << post_count);
    }
    
    uint32_t triggered_count = 0;
    uint32_t deg_before_first = 0;
    uint32_t deg_after_first = 0;
    
    for (uint32_t p = post_start; p < post_end; p++) {
#pragma HLS pipeline II=1
      uint32_t sid = post_flat[p];
      uint32_t d = deg_cur[sid];
      
      // 记录第一个段的入度变化
      if (p == post_start) {
        deg_before_first = d;
      }
      
      if (d > 0) {
        d--; 
        deg_cur[sid] = d;                               // 入度减一并更新
        
        if (p == post_start) {
          deg_after_first = d;
        }
        
        if (d == 0) {                                   // 入度归零→ready
          triggered_count++;
          SegRec r = seg_table[sid];
          uint32_t dep_lo_val = dep_ptr[sid];
          uint32_t dep_hi_val = dep_ptr[sid + 1];
          ReadySeg t{sid, r.j, r.row_lo, r.row_hi, r.A_addr,
                     r.word_count, r.L_base_addr, r.U_diag_addr,
                     dep_lo_val, dep_hi_val, r.is_last};
          // 打印触发的段的依赖信息
          if (triggered_count < 20) {
            DEBUG_PRINT("DegScheduler: 🔓 Triggered seg_id=" << sid << " j=" << r.j 
                        << " dep_lo=" << dep_lo_val << " dep_hi=" << dep_hi_val 
                        << " dep_cnt=" << (dep_hi_val - dep_lo_val));
          }
          int peg = addr_bank(r.A_addr) % PEGS;
          if (triggered_count <= 5 && col_done_count <= 50) {
            DEBUG_PRINT("度调度器:     ✅ 触发段 seg_id=" << sid << " j=" << r.j << " peg=" << peg);
          }
          
          // 添加依赖关系验证
          if (r.j == k) {
            DEBUG_PRINT("DegScheduler: ⚠️ 检测到自依赖段 seg_id=" << sid << " j=" << r.j << " 依赖列=" << k);
          }
          
          uint32_t dep_cnt = dep_hi_val - dep_lo_val;
          uint32_t seg_len = r.row_hi - r.row_lo;
          bool high_prio = (dep_cnt > 0) && (seg_len >= 64);
          if (high_prio && !ready_local_out[peg].full())
            ready_local_out[peg].write(t);
          else
            ready_global_out.write(t);
        } else {
          // 记录未触发的段，用于分析死锁
          if (col_done_count <= 10 && p < post_start + 3) {
            DEBUG_PRINT("DegScheduler: 段未触发 seg_id=" << sid << " j=" << seg_table[sid].j << " 剩余入度=" << d);
          }
        }
      }
    }
    
    if (col_done_count <= 20 || triggered_count > 0) {
      DEBUG_PRINT("度调度器: 列" << k << "完成，触发了 " << triggered_count << " 个新段 (post_count=" << post_count << ")");
      if (post_count > 0 && col_done_count <= 20) {
        DEBUG_PRINT("度调度器:   第一个依赖段入度: " << deg_before_first << " -> " << deg_after_first);
      }
    }
    triggered_total += triggered_count;
    if ((col_done_count & 0xFF) == 0) {
      DEBUG_PRINT_IMPORTANT("DegScheduler: progress col_done_received=" << col_done_count << "/" << nCol << ", triggered_total=" << triggered_total);
    }
  }
  
  // --- 软件仿真专用：添加延迟以确保下游模块能处理完所有剩余请求 ---
#ifndef __SYNTHESIS__
  // 在DegScheduler完成所有列的调度后，等待一段时间
  // 以确保所有PE的写请求能被ReqMux和HBM_Write_Service完全处理
  // 避免因调度器过早退出导致下游模块提前终止，从而无法观察到完整的写回流程
  DEBUG_PRINT_IMPORTANT("DegScheduler: [SW_EMU] Main loop done, starting drain wait...");
  for (int i = 0; i < 2000000; ++i) {
    // 这个循环在软件仿真中消耗时间，但在HLS综合中会被优化掉
    // 从而为下游模块提供处理未决请求的时间窗口
    if (i % 500000 == 0) {
      DEBUG_PRINT_IMPORTANT("DegScheduler: [SW_EMU] Draining... (" << i << "/2000000)");
    }
  }
  DEBUG_PRINT_IMPORTANT("DegScheduler: [SW_EMU] Drain wait finished.");
#endif
  // ------------------------------------------------------------------

  SUMMARY_PRINT("DegScheduler: SUMMARY col_done_received_total=" << col_done_received_total
                << " triggered_total=" << triggered_total);
  // 发送终止信号到所有 PEG
  ReadySeg terminator{0xFFFFFFFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF};
  for (int peg = 0; peg < PEGS; peg++) {
#pragma HLS unroll
    ready_local_out[peg].write(terminator);
  }
  
  // 发送完成信号到 GlobalLUCache
  DEBUG_PRINT("DegScheduler: Sending done signal to GlobalLUCache");
  done_out.write(true);
  
  DEBUG_PRINT("DegScheduler: Exiting");
}

//======================================================================
//  WorkDistributor: 工作分发器
//======================================================================
void WorkDistributor(
  tapa::istreams<ReadySeg, PEGS>& ready_local_in,       // 每 PEG 输入
  tapa::istream<ReadySeg>& ready_global_in,             // 全局输入
  tapa::ostreams<ReadySeg, NUM_PE>& ready_to_pe         // 输出到每 PE
) {
  DEBUG_PRINT_IMPORTANT("WorkDistributor_Optimized: Started");
  
  // 简化的轮询指针
  ap_uint<4> rr_peg = 0;      // 当前PEG (0-7)
  ap_uint<6> rr_pe = 0;       // 全局任务的 PE 轮询指针
  ap_uint<3> rr_local[PEGS] = {0};
#pragma HLS array_partition variable=rr_local complete
  
  uint32_t terminators_received = 0;
  uint32_t tasks_distributed = 0;
  uint32_t iterations = 0;
  
  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    iterations++;
    
    // 定期报告
    if (iterations % 1000000 == 0) {
      DEBUG_PRINT_IMPORTANT("WorkDistributor_Optimized: iterations=" << iterations << " tasks_distributed=" << tasks_distributed);
    }
    
    // 策略1: 优先处理local队列（顺序轮询，不展开）
    bool found_local = false;
    
    // 顺序检查当前PEG（不使用unroll）
    if (!ready_local_in[rr_peg].empty()) {
      ReadySeg local_task = ready_local_in[rr_peg].read();
      found_local = true;
      
      // 检测终止信号
      if (local_task.seg_id == 0xFFFFFFFF) {
        terminators_received++;
        if (terminators_received >= PEGS) {
          // 向所有PE发送终止信号
          for (int pe = 0; pe < NUM_PE; pe++) {
#pragma HLS unroll
            ready_to_pe[pe].write(local_task);
          }
          DEBUG_PRINT_IMPORTANT("WorkDistributor_Optimized: Exiting, distributed " << tasks_distributed << " tasks");
          return;
        }
      } else {
        // PEG i 只使用它的 PE_PER_PEG 个真实 PE，且在 PEG 内逐个轮询。
        const int local_slot = rr_local[rr_peg];
        const int pe = (int)rr_peg * PE_PER_PEG + local_slot;
        ready_to_pe[pe].write(local_task);  // 阻塞写入（依赖FIFO深度）
        rr_local[rr_peg] = (local_slot + 1) % PE_PER_PEG;
        tasks_distributed++;
        
        if (tasks_distributed < 100 || (local_task.dep_hi > local_task.dep_lo)) {
          DEBUG_PRINT_IMPORTANT("WorkDistributor_Optimized: ✅ PEG" << rr_peg << " → PE" << pe << " seg_id=" << local_task.seg_id << " dep_cnt=" << (local_task.dep_hi - local_task.dep_lo));
        }
      }
      
      // 移动到下一个PEG
      rr_peg = (rr_peg + 1) & 0x7;  // 0-7循环
    }
    
    // 策略2: 如果local队列空，检查global队列
    if (!found_local && !ready_global_in.empty()) {
      ReadySeg global_task = ready_global_in.read();
      // 分配到当前PE（若目标满则顺延一次）
      int pe_try = rr_pe;
      // 确保 pe_try 在有效范围内
      if (pe_try >= NUM_PE) pe_try = pe_try % NUM_PE;
      if (!ready_to_pe[pe_try].full()) {
        ready_to_pe[pe_try].write(global_task);
      } else {
        int pe_next = (rr_pe + 1) % NUM_PE;  // 使用模运算确保在有效范围内
        if (!ready_to_pe[pe_next].full()) {
          ready_to_pe[pe_next].write(global_task);
          pe_try = pe_next;
        } else {
          // 两个候选都满，放弃本周期，避免阻塞
          if ((iterations & 0xFFFFF) == 0) {
            DEBUG_PRINT_IMPORTANT("WorkDistributor_Optimized: ready_to_pe FULL at PE" << rr_pe << " and PE" << pe_next << ", stall global task seg_id=" << global_task.seg_id);
          }
          // 回滚读到的global_task不可行，这里简单选择下周期重试：将其写回不支持，只能丢弃是不正确的。
          // 为避免丢失，这里直接写入当前pe（阻塞），但已有full检查通常不会发生在硬件；sw_emu下也会尽快释放。
          if (rr_pe >= NUM_PE) rr_pe = rr_pe % NUM_PE;
          ready_to_pe[rr_pe].write(global_task);
        }
      }
      tasks_distributed++;
      if (tasks_distributed < 100) {
        DEBUG_PRINT_IMPORTANT("WorkDistributor_Optimized: Global task → PE" << pe_try << " seg_id=" << global_task.seg_id);
      }
      rr_pe = (rr_pe + 1) % NUM_PE;  // 使用模运算确保在有效范围内
    }
    
    // 定期报告
    if (tasks_distributed > 0 && tasks_distributed % 10000 == 0) {
      DEBUG_PRINT_IMPORTANT("WorkDistributor_Optimized: Distributed " << tasks_distributed << " tasks");
    }
  }
}

//======================================================================




//======================================================================
//  ColDoneMux：合并来自所有 PE 的 col_done 信号
//======================================================================
void ColDoneMux(
  tapa::istreams<uint32_t, NUM_PE>& col_done_in,        // 各 PE 的 col_done 输入
  tapa::ostream<uint32_t>& col_done_out                 // 合并后的 col_done 输出
) {
  DEBUG_PRINT_IMPORTANT("ColDoneMux: Started");
  ap_uint<8> rr = 0;                                     // 轮转指针（NUM_PE<=256）
  uint32_t total_col_done = 0;

  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    // 每周期只检查一路。避免在 pipeline 内扫描并部分展开 64 路
    // stream；后者会使 Vitis HLS 前端产生超大的动态选择逻辑。
    const ap_uint<6> k = rr;
    if (!col_done_in[k].empty()) {
      uint32_t col_id = col_done_in[k].read();
      col_done_out.write(col_id);
      total_col_done++;
      if (total_col_done <= 10 || (total_col_done & 0x3F) == 0) {
        DEBUG_PRINT("ColDoneMux: ✅ Forwarded col_done #" << total_col_done << " from PE" << (int)k << " col_id=" << col_id);
      }
    }
    rr = (k + 1 < NUM_PE) ? (ap_uint<8>)(k + 1) : (ap_uint<8>)0;
  }
}

//======================================================================
//  WriteChannelArbiter: 协调的写通道仲裁器
//  确保一个PE的写请求及其所有数据被原子地转发，防止死锁。
//======================================================================
void WriteChannelArbiter(
  tapa::istreams<WriteReqTagged, NUM_PE>& wr_req_in,
  tapa::ostream<WriteReqTagged>& wr_req_out,
  tapa::istreams<WriteElemTagged, NUM_PE>& wr_elem_in,
  tapa::ostream<WriteElemTagged>& wr_elem_out
) {
  enum State {IDLE, FORWARD_DATA};
  State state = IDLE;
  ap_uint<6> rr = 0;
  ap_uint<6> locked_pe = 0;
  uint32_t remaining_len = 0;
  uint32_t total_reqs = 0;
  uint32_t total_elems = 0;

  DEBUG_PRINT_IMPORTANT("WriteChannelArbiter: Started");

  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    if (state == IDLE) {
      // 每周期检查一路，保持轮转公平性并避免 HLS 展开 64 路扫描。
      const ap_uint<6> pe_idx = rr;
      if (!wr_req_in[pe_idx].empty()) {
        WriteReqTagged req = wr_req_in[pe_idx].read();
        wr_req_out.write(req);

        locked_pe = pe_idx;
        remaining_len = req.r.len;
        state = FORWARD_DATA;

        total_reqs++;
        if (total_reqs <= 20 || total_reqs % 100 == 0) {
          DEBUG_PRINT_IMPORTANT("WriteChannelArbiter: 锁定了 PE" << (int)locked_pe
                                << "，准备转发 " << remaining_len << " 个数据元素 (请求 #" << total_reqs << ")");
        }
      }
      rr = (pe_idx + 1 < NUM_PE) ? (ap_uint<6>)(pe_idx + 1) : (ap_uint<6>)0;
    } else { // FORWARD_DATA
      if (remaining_len > 0) {
        if (!wr_elem_in[locked_pe].empty()) {
          WriteElemTagged elem = wr_elem_in[locked_pe].read();
          wr_elem_out.write(elem);
          remaining_len--;
          total_elems++;
        }
      } else {
        if (total_reqs <= 20) {
          DEBUG_PRINT_IMPORTANT("WriteChannelArbiter: PE" << (int)locked_pe << " 的事务完成，释放锁定");
        }
        state = IDLE;
      }
    }
  }
}

//======================================================================
//  ReqMux：优化版 - 简化仲裁逻辑，加速HLS综合
//  
//  优化策略：
//  1. 移除完全展开循环 - 使用顺序轮询
//  2. 减少try_read/try_write调用 - 每周期只处理一个请求
//  3. 使用阻塞写入 - 简化控制逻辑
//======================================================================
void ReadReqMux(
  tapa::istreams<ReadReqTagged, NUM_PE>& in_rd,
  tapa::ostream<ReadReqTagged>& out_rd
) {
  DEBUG_PRINT_IMPORTANT("ReadReqMux: Started");
  ap_uint<6> rr_rd = 0;
  uint32_t total_rd = 0;
  uint32_t iterations = 0;

  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    iterations++;
    if (iterations % 10000 == 0) {
      DEBUG_PRINT_IMPORTANT("ReadReqMux: iterations=" << iterations << " total_rd=" << total_rd);
    }

    // 每周期检查一路，最坏仲裁延迟为 NUM_PE 周期。
    const ap_uint<6> idx = rr_rd;
    if (!in_rd[idx].empty()) {
      ReadReqTagged req = in_rd[idx].read();
      while (out_rd.full()) { /* spin until downstream ready */ }
      out_rd.write(req);
      total_rd++;
      if (total_rd <= 30 || (total_rd % 1024) == 0) {
        SUMMARY_PRINT("ReadReqMux: fwd #" << total_rd << " from PE" << (int)idx << " kind=" << (int)req.r.kind << " len=" << req.r.len);
      }
    }
    rr_rd = (idx + 1 < NUM_PE) ? (ap_uint<6>)(idx + 1) : (ap_uint<6>)0;
  }
}

//======================================================================
//  RespDemux is split into RdRespDemux and WrAckDemux to simplify logic
//  and avoid Vitis HLS crash.
//======================================================================

// RdRespDemux: Demultiplexes read responses.
void RdRespDemux(
  tapa::istream<ReadElemTagged>& in_rd_elem,
  tapa::ostreams<WORD64, NUM_PE>& out_rd_elem
) {
  DEBUG_PRINT_IMPORTANT("RdRespDemux: Started");
  uint32_t total_demuxed = 0;
  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    if (!in_rd_elem.empty()) {
      ReadElemTagged t = in_rd_elem.read();
      // 添加边界检查，确保 pe 在有效范围内
      ap_uint<8> pe = t.pe;
      if (pe < NUM_PE) {
        while (out_rd_elem[pe].full()) { /* spin until ready */ }
        out_rd_elem[pe].write(t.v);
        total_demuxed++;
        if (total_demuxed <= 16 || (total_demuxed % 2048) == 0) {
          SUMMARY_PRINT("RdRespDemux: demux #" << total_demuxed << " → PE" << (int)pe);
        }
      } else {
        DEBUG_PRINT_IMPORTANT("RdRespDemux: ❌ 无效的 PE 索引 " << (int)pe << " (max=" << NUM_PE << ")，跳过");
      }
    }
  }
}

// WrAckDemux: Demultiplexes write acknowledgements.
void WrAckDemux(
  tapa::istream<ap_uint<8>>& in_wr_ack,
  tapa::ostreams<ap_uint<1>, NUM_PE>& out_wr_ack
) {
  DEBUG_PRINT_IMPORTANT("WrAckDemux: Started");
  uint32_t total_acks = 0;
  for (;;) {
#pragma HLS pipeline II=1
    if (!in_wr_ack.empty()) {
      ap_uint<8> pe = in_wr_ack.read();
      // 添加边界检查，确保 pe 在有效范围内
      if (pe < NUM_PE) {
        // This is a blocking write.
        out_wr_ack[pe].write(1);
        total_acks++;
        if (total_acks <= 10 || total_acks % 100 == 0) {
          DEBUG_PRINT_IMPORTANT("WrAckDemux: ✅ Forwarded ACK #" << total_acks << " to PE" << (int)pe);
        }
      } else {
        DEBUG_PRINT_IMPORTANT("WrAckDemux: ❌ 无效的 PE 索引 " << (int)pe << " (max=" << NUM_PE << ")，跳过");
      }
    }
  }
}

 

//======================================================================
//  GlbReqMux: 合并来自所有 PE 的全局缓存请求流
//======================================================================
void GlbReqMux(
  tapa::istreams<LUReq, NUM_PE>& glb_req_in,
  tapa::ostream<LUReq>& glb_req_out
) {
  DEBUG_PRINT_IMPORTANT("GlbReqMux: Started");
  ap_uint<6> rr = 0;
  uint32_t total_req = 0;

  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000
    // 每周期检查一路，避免在流水线内展开 64 路 stream 扫描。
    const ap_uint<6> idx = rr;
    if (!glb_req_in[idx].empty()) {
      LUReq req = glb_req_in[idx].read();
      while (glb_req_out.full()) { /* spin until downstream ready */ }
      glb_req_out.write(req);
      total_req++;
      if (total_req <= 10 || (total_req % 1024) == 0) {
        DEBUG_PRINT("GlbReqMux: fwd #" << total_req << " from PE" << (int)idx << " is_write=" << req.is_write << " addr=" << req.addr);
      }
    }
    rr = (idx + 1 < NUM_PE) ? (ap_uint<6>)(idx + 1) : (ap_uint<6>)0;
  }
}

//======================================================================
//  GlbRspDemux: 将全局缓存响应流分发到各个 PE
//======================================================================
void GlbRspDemux(
  tapa::istream<LURsp>& glb_rsp_in,
  tapa::ostreams<LURsp, NUM_PE>& glb_rsp_out
) {
  DEBUG_PRINT_IMPORTANT("GlbRspDemux: Started");
  uint32_t total_rsp = 0;
  
  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000
    if (!glb_rsp_in.empty()) {
      LURsp rsp = glb_rsp_in.read();
      // 从响应中提取 PE 索引
      ap_uint<8> pe = rsp.pe;
      if (pe < NUM_PE) {
        // 阻塞写入到对应的 PE
        glb_rsp_out[pe].write(rsp);
        total_rsp++;
        if (total_rsp <= 10 || (total_rsp % 1024) == 0) {
          DEBUG_PRINT("GlbRspDemux: fwd #" << total_rsp << " to PE" << (int)pe);
        }
      } else {
        DEBUG_PRINT("GlbRspDemux: ❌ 无效的 PE 索引 " << (int)pe << " (max=" << NUM_PE << ")，跳过");
      }
    }
  }
}

//======================================================================
//  WriteElemMux：合并来自所有 PE 的写数据流
//======================================================================
void WriteElemMux(
  tapa::istreams<WriteElemTagged, NUM_PE>& wr_elem_in,   // 各 PE 的写数据输入
  tapa::ostream<WriteElemTagged>& wr_elem_out            // 合并后的写数据输出
) {
  DEBUG_PRINT_IMPORTANT("WriteElemMux: Started");
  ap_uint<8> rr = 0;                                     // 轮转指针
  uint32_t total_wr_elem = 0;
  
  for (;;) {
#pragma HLS pipeline II=1
    bool has_data = false;
    
    // 轮询所有 PE 的写数据流
    for (int i = 0; i < NUM_PE; i++) {
#pragma HLS unroll
      int k = (rr + i) % NUM_PE;
      if (!wr_elem_in[k].empty()) {
        WriteElemTagged elem = wr_elem_in[k].read();
        wr_elem_out.write(elem);
        rr = (k + 1) % NUM_PE;  // 从下一个 PE 开始轮询
        total_wr_elem++;
        has_data = true;
        if (total_wr_elem <= 10 || total_wr_elem % 1000 == 0) {
          DEBUG_PRINT("WriteElemMux: ✅ Forwarded wr_elem #" << total_wr_elem << " from PE" << k);
        }
        break;  // 每个周期只处理一个
      }
    }
  }
}

//======================================================================
//  地址解码工具结构：512b 块对齐、lane/offset 提取
//======================================================================
struct PortSel { uint8_t lane; uint32_t word_off; uint8_t head_pad; };
static inline PortSel decode_addr(uint32_t addr, uint32_t lanes, bool is_A = true) {
  PortSel s;                                             // bank→lane，offset→word_off
  const uint32_t bank = addr_bank(addr), elem = addr_off(addr);
  // A/L/U 分别使用 8/8/4 个 bank，均为 2 的幂。位掩码不仅等价，
  // 还避免 Vitis HLS 在这个永久服务 FSM 中插入多周期 urem 子模块；
  // 后者会把请求取走后卡在地址发射之前。
  s.lane = bank & (lanes - 1);                           // lane 号
  // 修复：直接使用elem作为偏移，不需要加上bank的偏移
  // 因为每个lane对应一个独立的缓冲区，bank编号只用于选择lane
  s.word_off = elem / VEC_64_PER_512;                    // 512位字地址
  s.head_pad = elem % VEC_64_PER_512;                    // 对齐偏移
  return s;
}
// 由于HLS disaggregate的限制，我们不能使用任何形式的动态选择
// (switch/if-else/三元运算符/数组索引都会导致Phi节点或指针的指针问题)
// 因此我们必须完全展开所有可能的kind和lane组合

// 合并读请求 → 发突发地址 → 收 512b → 拆 64b → 输出                             // 功能说明
void HBM_Read_Service_LegacyBlocking(
  tapa::async_mmap<ap_uint<512>>& A_0, tapa::async_mmap<ap_uint<512>>& A_1, tapa::async_mmap<ap_uint<512>>& A_2, tapa::async_mmap<ap_uint<512>>& A_3,
  tapa::async_mmap<ap_uint<512>>& A_4, tapa::async_mmap<ap_uint<512>>& A_5, tapa::async_mmap<ap_uint<512>>& A_6, tapa::async_mmap<ap_uint<512>>& A_7,
  tapa::async_mmap<ap_uint<512>>& L_0, tapa::async_mmap<ap_uint<512>>& L_1, tapa::async_mmap<ap_uint<512>>& L_2, tapa::async_mmap<ap_uint<512>>& L_3,
  tapa::async_mmap<ap_uint<512>>& L_4, tapa::async_mmap<ap_uint<512>>& L_5, tapa::async_mmap<ap_uint<512>>& L_6, tapa::async_mmap<ap_uint<512>>& L_7,
  tapa::async_mmap<ap_uint<512>>& U_0, tapa::async_mmap<ap_uint<512>>& U_1, tapa::async_mmap<ap_uint<512>>& U_2, tapa::async_mmap<ap_uint<512>>& U_3,
  tapa::istream<ReadReqTagged>& req_in,   // 读请求（带 pe）
  tapa::ostream<ReadElemTagged>& elem_out,// 读回 64b（带 pe）
  tapa::istream<WriteReqTagged>& wr_req_in,
  tapa::istream<WriteElemTagged>& wr_elem_in,
  tapa::ostream<ap_uint<8>>& wr_ack_out
) {
  bool prefer_write = false;
  
  for (;;) {                                                                     // 永久服务
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    // 注意：移除pipeline pragma以便在软件仿真中正确执行
    // #pragma HLS pipeline II=1                                                        // 单周期处理
    // L/U 的读写必须由同一个 TAPA 任务持有 mmap。否则同一
    // 顶层 mmap 会连到两个子任务，生成 RTL 时触发双向 FIFO 断言。
    // 读写同时到达时交替优先，避免任一方饥饿。
    if (!wr_req_in.empty() && (prefer_write || req_in.empty())) {
      WriteReqTagged wr = wr_req_in.read();
      const uint8_t wr_kind = wr.r.kind;
      if (wr_kind == 0) {
        wr_ack_out.write(wr.pe);
        prefer_write = false;
        continue;
      }

      const PortSel ws = (wr_kind == 1)
                             ? decode_addr(wr.r.addr, 8, false)
                             : decode_addr(wr.r.addr, 4, false);
      ap_uint<512> acc = 0;
      uint32_t filled = ws.head_pad;
      uint32_t word_idx = 0;

      for (uint32_t i = 0; i < wr.r.len; ++i) {
        WriteElemTagged elem;
        while (!wr_elem_in.try_read(elem)) {
        }
        set_word64(acc, filled++, elem.v);
        if (filled == VEC_64_PER_512) {
          if (wr_kind == 1) {
            if (ws.lane == 0) { L_0.write_addr.write(ws.word_off + word_idx); L_0.write_data.write(acc); (void)L_0.write_resp.read(); }
            else if (ws.lane == 1) { L_1.write_addr.write(ws.word_off + word_idx); L_1.write_data.write(acc); (void)L_1.write_resp.read(); }
            else if (ws.lane == 2) { L_2.write_addr.write(ws.word_off + word_idx); L_2.write_data.write(acc); (void)L_2.write_resp.read(); }
            else if (ws.lane == 3) { L_3.write_addr.write(ws.word_off + word_idx); L_3.write_data.write(acc); (void)L_3.write_resp.read(); }
            else if (ws.lane == 4) { L_4.write_addr.write(ws.word_off + word_idx); L_4.write_data.write(acc); (void)L_4.write_resp.read(); }
            else if (ws.lane == 5) { L_5.write_addr.write(ws.word_off + word_idx); L_5.write_data.write(acc); (void)L_5.write_resp.read(); }
            else if (ws.lane == 6) { L_6.write_addr.write(ws.word_off + word_idx); L_6.write_data.write(acc); (void)L_6.write_resp.read(); }
            else if (ws.lane == 7) { L_7.write_addr.write(ws.word_off + word_idx); L_7.write_data.write(acc); (void)L_7.write_resp.read(); }
          } else {
            if (ws.lane == 0) { U_0.write_addr.write(ws.word_off + word_idx); U_0.write_data.write(acc); (void)U_0.write_resp.read(); }
            else if (ws.lane == 1) { U_1.write_addr.write(ws.word_off + word_idx); U_1.write_data.write(acc); (void)U_1.write_resp.read(); }
            else if (ws.lane == 2) { U_2.write_addr.write(ws.word_off + word_idx); U_2.write_data.write(acc); (void)U_2.write_resp.read(); }
            else if (ws.lane == 3) { U_3.write_addr.write(ws.word_off + word_idx); U_3.write_data.write(acc); (void)U_3.write_resp.read(); }
          }
          acc = 0;
          filled = 0;
          ++word_idx;
        }
      }
      if (filled != 0) {
        if (wr_kind == 1) {
          if (ws.lane == 0) { L_0.write_addr.write(ws.word_off + word_idx); L_0.write_data.write(acc); (void)L_0.write_resp.read(); }
          else if (ws.lane == 1) { L_1.write_addr.write(ws.word_off + word_idx); L_1.write_data.write(acc); (void)L_1.write_resp.read(); }
          else if (ws.lane == 2) { L_2.write_addr.write(ws.word_off + word_idx); L_2.write_data.write(acc); (void)L_2.write_resp.read(); }
          else if (ws.lane == 3) { L_3.write_addr.write(ws.word_off + word_idx); L_3.write_data.write(acc); (void)L_3.write_resp.read(); }
          else if (ws.lane == 4) { L_4.write_addr.write(ws.word_off + word_idx); L_4.write_data.write(acc); (void)L_4.write_resp.read(); }
          else if (ws.lane == 5) { L_5.write_addr.write(ws.word_off + word_idx); L_5.write_data.write(acc); (void)L_5.write_resp.read(); }
          else if (ws.lane == 6) { L_6.write_addr.write(ws.word_off + word_idx); L_6.write_data.write(acc); (void)L_6.write_resp.read(); }
          else if (ws.lane == 7) { L_7.write_addr.write(ws.word_off + word_idx); L_7.write_data.write(acc); (void)L_7.write_resp.read(); }
        } else {
          if (ws.lane == 0) { U_0.write_addr.write(ws.word_off + word_idx); U_0.write_data.write(acc); (void)U_0.write_resp.read(); }
          else if (ws.lane == 1) { U_1.write_addr.write(ws.word_off + word_idx); U_1.write_data.write(acc); (void)U_1.write_resp.read(); }
          else if (ws.lane == 2) { U_2.write_addr.write(ws.word_off + word_idx); U_2.write_data.write(acc); (void)U_2.write_resp.read(); }
          else if (ws.lane == 3) { U_3.write_addr.write(ws.word_off + word_idx); U_3.write_data.write(acc); (void)U_3.write_resp.read(); }
        }
      }
      wr_ack_out.write(wr.pe);
      prefer_write = false;
      continue;
    }

    // 改为短自旋阻塞读取，确保请求被及时取走
    ReadReqTagged rq;
    if (req_in.empty()) {
      continue;
    }
    rq = req_in.read();
    prefer_write = true;
    const uint8_t kind = rq.r.kind;                                              // 0:A 1:L 2:U
    
    // 使用原始地址解码，不做PE调整
    const PortSel s = (kind == 0) ? decode_addr(rq.r.addr, /*lanes*/8, true)     // 解码 A
                     : (kind == 1) ? decode_addr(rq.r.addr, /*lanes*/8, false)   // 解码 L（8个bank）
                                   : decode_addr(rq.r.addr, /*lanes*/4, false);  // 解码 U（4个bank）
    const uint32_t words =                                                       // 需要的 512b 块数
        (rq.r.len + s.head_pad + VEC_64_PER_512 - 1) / VEC_64_PER_512;           // 向上取整
    // 使用流水线方式：先发送所有地址，然后接收所有数据
    // 这样可以避免阻塞在elem_out.write()时无法处理新请求
    uint32_t remain = rq.r.len;                                                  // 尚需 64b 数
    uint8_t  pad    = s.head_pad;                                                // 头部对齐空槽
    for (uint32_t i = 0; i < words; i++) {
      ap_uint<512> w = 0;
      if (kind == 0) {
        if (s.lane == 0) { A_0.read_addr.write(s.word_off + i); w = A_0.read_data.read(); }
        else if (s.lane == 1) { A_1.read_addr.write(s.word_off + i); w = A_1.read_data.read(); }
        else if (s.lane == 2) { A_2.read_addr.write(s.word_off + i); w = A_2.read_data.read(); }
        else if (s.lane == 3) { A_3.read_addr.write(s.word_off + i); w = A_3.read_data.read(); }
        else if (s.lane == 4) { A_4.read_addr.write(s.word_off + i); w = A_4.read_data.read(); }
        else if (s.lane == 5) { A_5.read_addr.write(s.word_off + i); w = A_5.read_data.read(); }
        else if (s.lane == 6) { A_6.read_addr.write(s.word_off + i); w = A_6.read_data.read(); }
        else if (s.lane == 7) { A_7.read_addr.write(s.word_off + i); w = A_7.read_data.read(); }
      } else if (kind == 1) {
        if (s.lane == 0) { L_0.read_addr.write(s.word_off + i); w = L_0.read_data.read(); }
        else if (s.lane == 1) { L_1.read_addr.write(s.word_off + i); w = L_1.read_data.read(); }
        else if (s.lane == 2) { L_2.read_addr.write(s.word_off + i); w = L_2.read_data.read(); }
        else if (s.lane == 3) { L_3.read_addr.write(s.word_off + i); w = L_3.read_data.read(); }
        else if (s.lane == 4) { L_4.read_addr.write(s.word_off + i); w = L_4.read_data.read(); }
        else if (s.lane == 5) { L_5.read_addr.write(s.word_off + i); w = L_5.read_data.read(); }
        else if (s.lane == 6) { L_6.read_addr.write(s.word_off + i); w = L_6.read_data.read(); }
        else if (s.lane == 7) { L_7.read_addr.write(s.word_off + i); w = L_7.read_data.read(); }
      } else {
        if (s.lane == 0) { U_0.read_addr.write(s.word_off + i); w = U_0.read_data.read(); }
        else if (s.lane == 1) { U_1.read_addr.write(s.word_off + i); w = U_1.read_data.read(); }
        else if (s.lane == 2) { U_2.read_addr.write(s.word_off + i); w = U_2.read_data.read(); }
        else if (s.lane == 3) { U_3.read_addr.write(s.word_off + i); w = U_3.read_data.read(); }
      }
      for (int k = 0; k < VEC_64_PER_512; k++) {
        if (i == 0 && k < pad) continue;
        if (!remain) break;
        WORD64 v = get_word64(w, k);
        // 优化：保持阻塞写入确保数据完整性，但减少自旋检查（在硬件中会被优化）
        elem_out.write(ReadElemTagged{v, rq.pe});
        remain--;
      }
    }
  }
}

// PLU2 memory service.  Each async-mmap handshake occupies its own explicit
// state.  In particular, never pipeline an address write together with the
// corresponding data/response read: that creates a cyclic ready dependency in
// RTL because the response cannot exist until the address handshake commits.
void HBM_Read_Service(
  tapa::async_mmap<ap_uint<512>>& A_0, tapa::async_mmap<ap_uint<512>>& A_1, tapa::async_mmap<ap_uint<512>>& A_2, tapa::async_mmap<ap_uint<512>>& A_3,
  tapa::async_mmap<ap_uint<512>>& A_4, tapa::async_mmap<ap_uint<512>>& A_5, tapa::async_mmap<ap_uint<512>>& A_6, tapa::async_mmap<ap_uint<512>>& A_7,
  tapa::async_mmap<ap_uint<512>>& L_0, tapa::async_mmap<ap_uint<512>>& L_1, tapa::async_mmap<ap_uint<512>>& L_2, tapa::async_mmap<ap_uint<512>>& L_3,
  tapa::async_mmap<ap_uint<512>>& L_4, tapa::async_mmap<ap_uint<512>>& L_5, tapa::async_mmap<ap_uint<512>>& L_6, tapa::async_mmap<ap_uint<512>>& L_7,
  tapa::async_mmap<ap_uint<512>>& U_0, tapa::async_mmap<ap_uint<512>>& U_1, tapa::async_mmap<ap_uint<512>>& U_2, tapa::async_mmap<ap_uint<512>>& U_3,
  tapa::istream<ReadReqTagged>& req_in,
  tapa::ostream<ReadElemTagged>& elem_out,
  tapa::istream<WriteReqTagged>& wr_req_in,
  tapa::istream<WriteElemTagged>& wr_elem_in,
  tapa::ostream<ap_uint<8>>& wr_ack_out
) {
  enum State : uint8_t {
    IDLE,
    READ_ADDR,
    READ_DATA,
    READ_EMIT,
    WRITE_GATHER,
    WRITE_ADDR,
    WRITE_DATA,
    WRITE_RESP,
    WRITE_ACK
  };

  State state = IDLE;
  bool prefer_write = false;
  ReadReqTagged rd{};
  WriteReqTagged wr{};
  PortSel port{};
  uint32_t word_idx = 0;
  uint32_t remaining = 0;
  uint8_t elem_lane = 0;
  uint8_t filled = 0;
  WORD512 word = 0;
  WORD512 acc = 0;

  for (;;) {
#pragma HLS pipeline II=1
    switch (state) {
      case IDLE: {
        if (!wr_req_in.empty() && (prefer_write || req_in.empty())) {
          if (wr_req_in.try_read(wr)) {
            prefer_write = false;
            if (wr.r.kind == 0 || wr.r.len == 0) {
              state = WRITE_ACK;
            } else {
              port = (wr.r.kind == 1) ? decode_addr(wr.r.addr, 8, false)
                                      : decode_addr(wr.r.addr, 4, false);
              word_idx = 0;
              remaining = wr.r.len;
              filled = port.head_pad;
              acc = 0;
              state = WRITE_GATHER;
            }
          }
        } else if (req_in.try_read(rd)) {
          prefer_write = true;
          port = (rd.r.kind == 0) ? decode_addr(rd.r.addr, 8, true)
               : (rd.r.kind == 1) ? decode_addr(rd.r.addr, 8, false)
                                  : decode_addr(rd.r.addr, 4, false);
          word_idx = 0;
          remaining = rd.r.len;
          elem_lane = port.head_pad;
          state = (remaining == 0) ? IDLE : READ_ADDR;
        }
        break;
      }

      case READ_ADDR: {
        const uint32_t addr = port.word_off + word_idx;
        bool sent = false;
        if (rd.r.kind == 0) {
          if (port.lane == 0) sent = A_0.read_addr.try_write(addr);
          else if (port.lane == 1) sent = A_1.read_addr.try_write(addr);
          else if (port.lane == 2) sent = A_2.read_addr.try_write(addr);
          else if (port.lane == 3) sent = A_3.read_addr.try_write(addr);
          else if (port.lane == 4) sent = A_4.read_addr.try_write(addr);
          else if (port.lane == 5) sent = A_5.read_addr.try_write(addr);
          else if (port.lane == 6) sent = A_6.read_addr.try_write(addr);
          else sent = A_7.read_addr.try_write(addr);
        } else if (rd.r.kind == 1) {
          if (port.lane == 0) sent = L_0.read_addr.try_write(addr);
          else if (port.lane == 1) sent = L_1.read_addr.try_write(addr);
          else if (port.lane == 2) sent = L_2.read_addr.try_write(addr);
          else if (port.lane == 3) sent = L_3.read_addr.try_write(addr);
          else if (port.lane == 4) sent = L_4.read_addr.try_write(addr);
          else if (port.lane == 5) sent = L_5.read_addr.try_write(addr);
          else if (port.lane == 6) sent = L_6.read_addr.try_write(addr);
          else sent = L_7.read_addr.try_write(addr);
        } else {
          if (port.lane == 0) sent = U_0.read_addr.try_write(addr);
          else if (port.lane == 1) sent = U_1.read_addr.try_write(addr);
          else if (port.lane == 2) sent = U_2.read_addr.try_write(addr);
          else sent = U_3.read_addr.try_write(addr);
        }
        if (sent) state = READ_DATA;
        break;
      }

      case READ_DATA: {
        bool received = false;
        if (rd.r.kind == 0) {
          if (port.lane == 0) received = A_0.read_data.try_read(word);
          else if (port.lane == 1) received = A_1.read_data.try_read(word);
          else if (port.lane == 2) received = A_2.read_data.try_read(word);
          else if (port.lane == 3) received = A_3.read_data.try_read(word);
          else if (port.lane == 4) received = A_4.read_data.try_read(word);
          else if (port.lane == 5) received = A_5.read_data.try_read(word);
          else if (port.lane == 6) received = A_6.read_data.try_read(word);
          else received = A_7.read_data.try_read(word);
        } else if (rd.r.kind == 1) {
          if (port.lane == 0) received = L_0.read_data.try_read(word);
          else if (port.lane == 1) received = L_1.read_data.try_read(word);
          else if (port.lane == 2) received = L_2.read_data.try_read(word);
          else if (port.lane == 3) received = L_3.read_data.try_read(word);
          else if (port.lane == 4) received = L_4.read_data.try_read(word);
          else if (port.lane == 5) received = L_5.read_data.try_read(word);
          else if (port.lane == 6) received = L_6.read_data.try_read(word);
          else received = L_7.read_data.try_read(word);
        } else {
          if (port.lane == 0) received = U_0.read_data.try_read(word);
          else if (port.lane == 1) received = U_1.read_data.try_read(word);
          else if (port.lane == 2) received = U_2.read_data.try_read(word);
          else received = U_3.read_data.try_read(word);
        }
        if (received) {
          elem_lane = (word_idx == 0) ? port.head_pad : 0;
          state = READ_EMIT;
        }
        break;
      }

      case READ_EMIT: {
        const WORD64 value = get_word64(word, elem_lane);
        if (elem_out.try_write(ReadElemTagged{value, rd.pe})) {
          --remaining;
          ++elem_lane;
          if (remaining == 0) {
            state = IDLE;
          } else if (elem_lane == VEC_64_PER_512) {
            ++word_idx;
            state = READ_ADDR;
          }
        }
        break;
      }

      case WRITE_GATHER: {
        WriteElemTagged elem;
        if (wr_elem_in.try_read(elem)) {
          set_word64(acc, filled, elem.v);
          ++filled;
          --remaining;
          if (filled == VEC_64_PER_512 || remaining == 0) state = WRITE_ADDR;
        }
        break;
      }

      case WRITE_ADDR: {
        const uint32_t addr = port.word_off + word_idx;
        bool sent = false;
        if (wr.r.kind == 1) {
          if (port.lane == 0) sent = L_0.write_addr.try_write(addr);
          else if (port.lane == 1) sent = L_1.write_addr.try_write(addr);
          else if (port.lane == 2) sent = L_2.write_addr.try_write(addr);
          else if (port.lane == 3) sent = L_3.write_addr.try_write(addr);
          else if (port.lane == 4) sent = L_4.write_addr.try_write(addr);
          else if (port.lane == 5) sent = L_5.write_addr.try_write(addr);
          else if (port.lane == 6) sent = L_6.write_addr.try_write(addr);
          else sent = L_7.write_addr.try_write(addr);
        } else {
          if (port.lane == 0) sent = U_0.write_addr.try_write(addr);
          else if (port.lane == 1) sent = U_1.write_addr.try_write(addr);
          else if (port.lane == 2) sent = U_2.write_addr.try_write(addr);
          else sent = U_3.write_addr.try_write(addr);
        }
        if (sent) state = WRITE_DATA;
        break;
      }

      case WRITE_DATA: {
        bool sent = false;
        if (wr.r.kind == 1) {
          if (port.lane == 0) sent = L_0.write_data.try_write(acc);
          else if (port.lane == 1) sent = L_1.write_data.try_write(acc);
          else if (port.lane == 2) sent = L_2.write_data.try_write(acc);
          else if (port.lane == 3) sent = L_3.write_data.try_write(acc);
          else if (port.lane == 4) sent = L_4.write_data.try_write(acc);
          else if (port.lane == 5) sent = L_5.write_data.try_write(acc);
          else if (port.lane == 6) sent = L_6.write_data.try_write(acc);
          else sent = L_7.write_data.try_write(acc);
        } else {
          if (port.lane == 0) sent = U_0.write_data.try_write(acc);
          else if (port.lane == 1) sent = U_1.write_data.try_write(acc);
          else if (port.lane == 2) sent = U_2.write_data.try_write(acc);
          else sent = U_3.write_data.try_write(acc);
        }
        if (sent) state = WRITE_RESP;
        break;
      }

      case WRITE_RESP: {
        uint8_t response = 0;
        bool received = false;
        if (wr.r.kind == 1) {
          if (port.lane == 0) received = L_0.write_resp.try_read(response);
          else if (port.lane == 1) received = L_1.write_resp.try_read(response);
          else if (port.lane == 2) received = L_2.write_resp.try_read(response);
          else if (port.lane == 3) received = L_3.write_resp.try_read(response);
          else if (port.lane == 4) received = L_4.write_resp.try_read(response);
          else if (port.lane == 5) received = L_5.write_resp.try_read(response);
          else if (port.lane == 6) received = L_6.write_resp.try_read(response);
          else received = L_7.write_resp.try_read(response);
        } else {
          if (port.lane == 0) received = U_0.write_resp.try_read(response);
          else if (port.lane == 1) received = U_1.write_resp.try_read(response);
          else if (port.lane == 2) received = U_2.write_resp.try_read(response);
          else received = U_3.write_resp.try_read(response);
        }
        if (received) {
          if (remaining == 0) {
            state = WRITE_ACK;
          } else {
            ++word_idx;
            filled = 0;
            acc = 0;
            state = WRITE_GATHER;
          }
        }
        break;
      }

      case WRITE_ACK:
        if (wr_ack_out.try_write(wr.pe)) state = IDLE;
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// Bank-parallel HBM fabric used by the SALUT path.
//
// The packed address chooses one of 8 A, 8 L, or 4 U physical ports.  Requests
// are first dispatched to a private queue for that port, then twenty independent
// bank engines overlap AXI latency.  Responses and write acknowledgements retain
// the original client tag and may therefore complete out of order across banks.
// ---------------------------------------------------------------------------
static inline uint32_t ParallelPhysicalBank(const uint8_t kind,
                                            const uint32_t addr) {
#pragma HLS inline
  const uint32_t bank = addr_bank(addr);
  if (kind == 0) return bank & (PAR_NUM_A_BANK - 1);
  if (kind == 1)
    return PAR_NUM_A_BANK + (bank & (PAR_NUM_L_BANK - 1));
  return PAR_NUM_A_BANK + PAR_NUM_L_BANK +
         (bank & (PAR_NUM_U_BANK - 1));
}

void ParallelHBMReadDispatch(
    tapa::istream<ReadReqTagged>& in,
    tapa::ostreams<ReadReqTagged, PAR_NUM_HBM_BANK>& bank_out) {
  ReadReqTagged held{};
  ap_uint<1> valid = 0;
  for (;;) {
#pragma HLS pipeline II=1
    if (!valid && in.try_read(held)) valid = 1;
    if (valid) {
      const uint32_t bank = ParallelPhysicalBank(held.r.kind, held.r.addr);
      if (!bank_out[bank].full()) {
        bank_out[bank].write(held);
        valid = 0;
      }
    }
  }
}

// Write request/data pairs are kept burst-atomic while they are steered to a
// bank.  This allows independent banks to execute concurrently without ever
// mixing the element streams belonging to two requests.
void ParallelHBMWriteDispatch(
    tapa::istream<WriteReqTagged>& req_in,
    tapa::istream<WritePacketTagged>& elem_in,
    tapa::ostreams<WriteReqTagged, PAR_NUM_HBM_WRITE_BANK>& bank_req_out,
    tapa::ostreams<WritePacketTagged, PAR_NUM_HBM_WRITE_BANK>& bank_elem_out) {
  WriteReqTagged held{};
  uint32_t target = 0;
  uint32_t remaining = 0;
  ap_uint<2> state = 0;  // 0: acquire, 1: send request, 2: send elements
  for (;;) {
#pragma HLS pipeline II=1
    if (state == 0) {
      if (req_in.try_read(held)) {
        // A is read-only, so write-bank zero corresponds to physical L bank 0.
        target = ParallelPhysicalBank(held.r.kind, held.r.addr) - PAR_NUM_A_BANK;
        remaining = held.r.len;
        state = 1;
      }
    } else if (state == 1) {
      if (!bank_req_out[target].full()) {
        bank_req_out[target].write(held);
        state = remaining == 0 ? 0 : 2;
      }
    } else if (!elem_in.empty() && !bank_elem_out[target].full()) {
      const WritePacketTagged packet = elem_in.read();
      bank_elem_out[target].write(packet);
      const uint32_t count = uint32_t(packet.count);
      if (remaining == count) {
        remaining = 0;
        state = 0;
      } else {
        remaining -= count;
      }
    }
  }
}

void ParallelHBMReadBank(
    tapa::async_mmap<WORD512>& port,
    tapa::istream<ReadReqTagged>& req_in,
    tapa::ostream<HBMReadPacketTagged>& rsp_out) {
  enum State : uint8_t { IDLE, SEND_ADDR, GET_DATA, EMIT };
  State state = IDLE;
  ReadReqTagged req{};
  WORD512 word = 0;
  uint32_t address = 0;
  uint32_t remaining = 0;
  for (;;) {
#pragma HLS pipeline II=1
    if (state == IDLE) {
      if (req_in.try_read(req)) {
        address = req.r.addr;
        remaining = req.r.len;
        state = remaining == 0 ? IDLE : SEND_ADDR;
      }
    } else if (state == SEND_ADDR) {
      const uint32_t addr = addr_off(address) / VEC_64_PER_512;
      if (port.read_addr.try_write(addr)) state = GET_DATA;
    } else if (state == GET_DATA) {
      WORD512 value;
      if (port.read_data.try_read(value)) {
        word = value;
        state = EMIT;
      }
    } else {
      const uint32_t first = addr_off(address) & (VEC_64_PER_512 - 1);
      const uint32_t capacity = VEC_64_PER_512 - first;
      const uint32_t count = remaining < capacity ? remaining : capacity;
      if (rsp_out.try_write(HBMReadPacketTagged{
              word, req.pe, ap_uint<5>(first), ap_uint<7>(count)})) {
#ifndef __SYNTHESIS__
        ++g_hbm_bank_rsp;
#endif
        address += count;
        remaining -= count;
        state = remaining == 0 ? IDLE : SEND_ADDR;
      }
    }
  }
}

// Column fronts dominate the dense U tail and consume L only.  A dedicated
// 2048-bit read master maps to each physical L bank and returns four adjacent
// 512-bit beats per mmap transaction.  Segment PEs keep using the coherent
// 512-bit read/write hierarchy; both masters address the same HBM storage.
void ParallelWideLReadBank(
    tapa::async_mmap<WORD2048>& port,
    tapa::istream<ReadReqTagged>& req_in,
    tapa::ostream<ReadPacketTagged>& rsp_out) {
  enum State : uint8_t { IDLE, SEND_ADDR, GET_DATA, EMIT };
  State state = IDLE;
  ReadReqTagged req{};
  WORD2048 word = 0;
  uint32_t address = 0;
  uint32_t remaining = 0;
  for (;;) {
#pragma HLS pipeline II=1
    if (state == IDLE) {
      if (req_in.try_read(req)) {
        address = req.r.addr;
        remaining = req.r.len;
        state = remaining == 0 ? IDLE : SEND_ADDR;
      }
    } else if (state == SEND_ADDR) {
      const uint32_t addr = addr_off(address) / 32u;
      if (port.read_addr.try_write(addr)) state = GET_DATA;
    } else if (state == GET_DATA) {
      if (port.read_data.try_read(word)) state = EMIT;
    } else {
      const uint32_t wide_lane = addr_off(address) & 31u;
      const uint32_t first = wide_lane & 7u;
      const uint32_t available = 8u - first;
      const uint32_t count = remaining < available ? remaining : available;
      WORD512 selected = 0;
      switch ((wide_lane >> 3) & 3u) {
        case 0: selected = WORD512(word.range(511, 0)); break;
        case 1: selected = WORD512(word.range(1023, 512)); break;
        case 2: selected = WORD512(word.range(1535, 1024)); break;
        default: selected = WORD512(word.range(2047, 1536)); break;
      }
      if (rsp_out.try_write(ReadPacketTagged{
              selected, req.pe, ap_uint<5>(first), ap_uint<7>(count)})) {
        address += count;
        remaining -= count;
        state = remaining == 0 ? IDLE : SEND_ADDR;
      }
    }
  }
}

void ParallelFrontLReadDispatch(
    tapa::istreams<ReadReqTagged, PAR_NUM_FRONT>& front_in,
    tapa::ostreams<ReadReqTagged, PAR_NUM_L_BANK>& bank_out) {
  ap_uint<3> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t front = rr;
    ReadReqTagged req;
    if (front_in[front].try_peek(req)) {
      const uint32_t physical = addr_bank(req.r.addr);
      const uint32_t bank = physical - PAR_NUM_A_BANK;
      if (physical >= PAR_NUM_A_BANK &&
          bank < PAR_NUM_L_BANK && !bank_out[bank].full()) {
        (void)front_in[front].read();
        bank_out[bank].write(req);
      }
    }
    rr = (front + 1) & (PAR_NUM_FRONT - 1);
  }
}

void ParallelFrontLReadResponseRoute(
    tapa::istreams<ReadPacketTagged, PAR_NUM_L_BANK>& bank_in,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_FRONT>& front_out) {
  ap_uint<3> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t bank = rr;
    ReadPacketTagged rsp;
    if (bank_in[bank].try_peek(rsp) &&
        rsp.pe > PAR_NUM_PE && rsp.pe <= PAR_NUM_CLIENT) {
      const uint32_t front = uint32_t(rsp.pe) - PAR_NUM_PE - 1;
      if (!front_out[front].full()) {
        (void)bank_in[bank].read();
        front_out[front].write(rsp);
      }
    }
    rr = (bank + 1) & (PAR_NUM_L_BANK - 1);
  }
}

void ParallelHBMRWBank(
    tapa::async_mmap<WORD512>& port,
    tapa::istream<ReadReqTagged>& rd_req_in,
    tapa::ostream<HBMReadPacketTagged>& rd_rsp_out,
    tapa::istream<WriteReqTagged>& wr_req_in,
    tapa::istream<WritePacketTagged>& wr_elem_in,
    tapa::ostream<ap_uint<8>>& wr_ack_out) {
  enum State : uint8_t {
    IDLE,
    READ_ADDR,
    READ_DATA,
    READ_EMIT,
    WRITE_GATHER,
    WRITE_ADDR,
    WRITE_DATA,
    WRITE_RESP,
    WRITE_ACK
  };
  State state = IDLE;
  ap_uint<1> prefer_write = 0;
  ReadReqTagged rd{};
  WriteReqTagged wr{};
  WORD512 word = 0;
  WORD512 acc = 0;
  uint32_t word_idx = 0;
  uint32_t rd_address = 0;
  uint32_t remaining = 0;

  for (;;) {
#pragma HLS pipeline II=1
    switch (state) {
      case IDLE:
        if (!wr_req_in.empty() && (prefer_write || rd_req_in.empty())) {
          wr = wr_req_in.read();
          prefer_write = 0;
          word_idx = 0;
          remaining = wr.r.len;
          acc = 0;
          state = remaining == 0 ? WRITE_ACK : WRITE_GATHER;
        } else if (rd_req_in.try_read(rd)) {
          prefer_write = 1;
          word_idx = 0;
          rd_address = rd.r.addr;
          remaining = rd.r.len;
          state = remaining == 0 ? IDLE : READ_ADDR;
        }
        break;

      case READ_ADDR: {
        const uint32_t addr = addr_off(rd_address) / VEC_64_PER_512;
        if (port.read_addr.try_write(addr)) state = READ_DATA;
        break;
      }
      case READ_DATA: {
        WORD512 value;
        if (port.read_data.try_read(value)) {
          word = value;
          state = READ_EMIT;
        }
        break;
      }
      case READ_EMIT: {
        const uint32_t first = addr_off(rd_address) & (VEC_64_PER_512 - 1);
        const uint32_t capacity = VEC_64_PER_512 - first;
        const uint32_t count = remaining < capacity ? remaining : capacity;
        if (rd_rsp_out.try_write(HBMReadPacketTagged{
                word, rd.pe, ap_uint<5>(first), ap_uint<7>(count)})) {
#ifndef __SYNTHESIS__
          ++g_hbm_bank_rsp;
#endif
          rd_address += count;
          remaining -= count;
          state = remaining == 0 ? IDLE : READ_ADDR;
        }
        break;
      }

      case WRITE_GATHER: {
        WritePacketTagged packet;
        if (wr_elem_in.try_read(packet)) {
          acc = packet.v;
          remaining -= uint32_t(packet.count);
          state = WRITE_ADDR;
        }
        break;
      }
      case WRITE_ADDR: {
        const uint32_t addr = (addr_off(wr.r.addr) / VEC_64_PER_512) +
                              word_idx;
        if (port.write_addr.try_write(addr)) state = WRITE_DATA;
        break;
      }
      case WRITE_DATA:
        if (port.write_data.try_write(acc)) state = WRITE_RESP;
        break;
      case WRITE_RESP: {
        uint8_t response = 0;
        if (port.write_resp.try_read(response)) {
          if (remaining == 0) {
            state = WRITE_ACK;
          } else {
            ++word_idx;
            acc = 0;
            state = WRITE_GATHER;
          }
        }
        break;
      }
      case WRITE_ACK:
        if (wr_ack_out.try_write(wr.pe)) state = IDLE;
        break;
    }
  }
}

void ParallelHBMReadResponseMux(
    tapa::istreams<ReadPacketTagged, PAR_NUM_HBM_BANK>& bank_in,
    tapa::ostream<ReadPacketTagged>& out) {
  ap_uint<5> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t bank = rr;
    if (!bank_in[bank].empty() && !out.full()) out.write(bank_in[bank].read());
    rr = bank == PAR_NUM_HBM_BANK - 1 ? 0 : bank + 1;
  }
}

void ParallelHBMWriteAckMux(
    tapa::istreams<ap_uint<8>, PAR_NUM_HBM_WRITE_BANK>& bank_in,
    tapa::ostream<ap_uint<8>>& out) {
  ap_uint<5> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t bank = rr;
    if (!bank_in[bank].empty() && !out.full()) out.write(bank_in[bank].read());
    rr = bank == PAR_NUM_HBM_WRITE_BANK - 1 ? 0 : bank + 1;
  }
}

//======================================================================
//  DepPtrBroadcaster/DepFlatBroadcaster: 将单输入依赖表复制给 Deg 与 Poster
//======================================================================
void DepPtrBroadcaster(
  tapa::istream<META_T>& dep_ptr_in,
  tapa::ostream<META_T>& dep_ptr_out_deg,
  tapa::ostream<META_T>& dep_ptr_out_poster
) {
#pragma HLS interface ap_ctrl_none port=return
  for (;;) {
#pragma HLS pipeline II=1
    if (!dep_ptr_in.empty() && !dep_ptr_out_deg.full() &&
        !dep_ptr_out_poster.full()) {
      META_T v = dep_ptr_in.read();
      dep_ptr_out_deg.write(v);
      dep_ptr_out_poster.write(v);
    }
  }
}

void DepFlatBroadcaster(
  tapa::istream<META_T>& dep_flat_in,
  tapa::ostream<META_T>& dep_flat_out_deg,
  tapa::ostream<META_T>& dep_flat_out_poster
) {
#pragma HLS interface ap_ctrl_none port=return
  for (;;) {
#pragma HLS pipeline II=1
    if (!dep_flat_in.empty() && !dep_flat_out_deg.full() &&
        !dep_flat_out_poster.full()) {
      META_T v = dep_flat_in.read();
      dep_flat_out_deg.write(v);
      dep_flat_out_poster.write(v);
    }
  }
}

//======================================================================
//  Poster: per-PE ready 透传 + 展开依赖到 per-PE 流
//  输入 meta_idx 为逐字段 12 个 uint32_t
//======================================================================
void Poster(
  tapa::istream<uint32_t>& meta_idx_in,
  tapa::istreams<ReadySeg, NUM_PE>& ready_in_per_pe,
  tapa::ostreams<ReadySeg, NUM_PE>& ready_passthrough,
  tapa::istream<META_T>& dep_ptr_in,
  tapa::istream<META_T>& dep_flat_in,
  tapa::ostreams<DepEntry, NUM_PE>& dep_stream_out
) {
  // 读取 12 项 MetaIndex 字段
  MetaIndex mi;
  mi.n                 = meta_idx_in.read();
  mi.seg_rows          = meta_idx_in.read();
  mi.nSeg              = meta_idx_in.read();
  mi.off_seg_table     = meta_idx_in.read();
  mi.off_seg_of_col_ptr= meta_idx_in.read();
  mi.off_seg_of_col_flat= meta_idx_in.read();
  mi.off_dep_ptr       = meta_idx_in.read();
  mi.off_dep_flat      = meta_idx_in.read();
  mi.off_deg_init      = meta_idx_in.read();
  mi.off_post_ptr      = meta_idx_in.read();
  mi.off_post_flat     = meta_idx_in.read();
  mi.off_dummy         = meta_idx_in.read();

  const uint32_t nSeg = mi.nSeg;

  // 加载 dep_ptr（nSeg+1 个）
  static uint32_t dep_ptr_local[
#if FAST_SYNTH
    1 << 16
#else
    1 << 20
#endif
  ];
#if FAST_SYNTH
#pragma HLS bind_storage variable=dep_ptr_local type=RAM_1P impl=BRAM
#else
#pragma HLS bind_storage variable=dep_ptr_local type=RAM_1P impl=URAM
#endif
  for (uint32_t i = 0; i <= nSeg; ++i) {
#pragma HLS pipeline II=1
    dep_ptr_local[i] = dep_ptr_in.read();
  }
  const uint32_t dep_entries = dep_ptr_local[nSeg];

  // 加载 dep_flat（dep_entries 个 DepEntry，每个 4 个 uint32）
  static DepEntry dep_flat_local[
#if FAST_SYNTH
    1 << 18
#else
    1 << 22
#endif
  ];
#if FAST_SYNTH
#pragma HLS bind_storage variable=dep_flat_local type=RAM_1P impl=BRAM
#else
#pragma HLS bind_storage variable=dep_flat_local type=RAM_1P impl=URAM
#endif
  for (uint32_t i = 0; i < dep_entries; ++i) {
#pragma HLS pipeline II=1
    DepEntry d;
    d.k      = dep_flat_in.read();
    d.L_addr = dep_flat_in.read();
    d.len    = dep_flat_in.read();
    d.U_addr = dep_flat_in.read();
    dep_flat_local[i] = d;
  }

  // 主循环：透传 ready，并展开依赖到对应 PE
  DEBUG_PRINT_IMPORTANT("Poster: Starting main loop, nSeg=" << nSeg << " dep_entries=" << dep_entries);
  uint32_t tasks_processed = 0;
  uint32_t iterations = 0;
  
  for (;;) {
#pragma HLS pipeline II=1
    iterations++;
    
    if (iterations % 1000000 == 0) {
      DEBUG_PRINT_IMPORTANT("Poster: iterations=" << iterations << " tasks_processed=" << tasks_processed);
    }
    
    for (int pe = 0; pe < NUM_PE; ++pe) {
#pragma HLS unroll
      if (!ready_in_per_pe[pe].empty()) {
        ReadySeg r = ready_in_per_pe[pe].read();
        tasks_processed++;
        
        if (tasks_processed <= 20) {
          DEBUG_PRINT_IMPORTANT("Poster: ✅ Received task #" << tasks_processed 
                                << " from PE" << pe << " seg_id=" << r.seg_id 
                                << " j=" << r.j << " dep_range=[" << r.dep_lo << "," << r.dep_hi << ")"
                                << " dep_cnt=" << (r.dep_hi - r.dep_lo));
        }
        
        if (r.seg_id != 0xFFFFFFFF) {
          // 调试：打印前若干个段的依赖展开统计
          if (r.seg_id < 32 || tasks_processed <= 20) {
            DEBUG_PRINT_IMPORTANT("Poster: 🔄 Expanding deps for seg_id=" << r.seg_id
                        << " pe=" << pe
                        << " dep_range=[" << r.dep_lo << "," << r.dep_hi << ")"
                        << " dep_cnt=" << (r.dep_hi - r.dep_lo));
          }
          
          uint32_t deps_sent = 0;
          for (uint32_t t = r.dep_lo; t < r.dep_hi; ++t) {
#pragma HLS pipeline II=1
            dep_stream_out[pe].write(dep_flat_local[t]);
            deps_sent++;
          }
          // 更明确：无条件在前若干次输出 summary，或每隔一段输出
          if (tasks_processed <= 64 || (tasks_processed & 0xFF) == 0) {
            DEBUG_PRINT_IMPORTANT("Poster: ✅ Sent deps seg_id=" << r.seg_id << " pe=" << pe
                                  << " deps_sent=" << deps_sent);
          }
        }
        // 必须在该段的所有依赖已写入 PE 依赖 FIFO 之后才发任务。
        ready_passthrough[pe].write(r);
      }
    }
  }
}


//======================================================================
//  HBM_Write_Service：处理合并的写请求并回写 ACK
//======================================================================
// 合并写请求 → 发突发地址 → 聚合 64b 成 512b → 写 → 等待 resp → 回 ACK            // 功能说明
void HBM_Write_Service(
  // A矩阵不需要写入，只写L和U
  tapa::mmap<ap_uint<512>> L_0, tapa::mmap<ap_uint<512>> L_1, tapa::mmap<ap_uint<512>> L_2, tapa::mmap<ap_uint<512>> L_3,
  tapa::mmap<ap_uint<512>> L_4, tapa::mmap<ap_uint<512>> L_5, tapa::mmap<ap_uint<512>> L_6, tapa::mmap<ap_uint<512>> L_7,
  tapa::mmap<ap_uint<512>> U_0, tapa::mmap<ap_uint<512>> U_1, tapa::mmap<ap_uint<512>> U_2, tapa::mmap<ap_uint<512>> U_3,
  tapa::istream<WriteReqTagged>& req_in,  // 写请求（带 pe）
  tapa::istream<WriteElemTagged>& elem_in,// 写入 64b 数据
  tapa::ostream<ap_uint<8>>& wr_ack_pe    // 写完成 ACK(pe)
) {
  // 将单独的mmap参数存储到数组中以便索引访问
  tapa::mmap<ap_uint<512>> L_arr[8] = {L_0, L_1, L_2, L_3, L_4, L_5, L_6, L_7};
  tapa::mmap<ap_uint<512>> U_arr[4] = {U_0, U_1, U_2, U_3};
  DEBUG_PRINT_IMPORTANT("HBM_Write_Service: Started, waiting for requests");
  uint32_t requests_processed = 0;                                               // 已处理请求数
  uint32_t iterations = 0;
  uint32_t idle_cycles = 0;
  
  for (;;) {                                                                     // 永久服务
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    // 注意：移除pipeline pragma以便在软件仿真中正确执行
    // #pragma HLS pipeline II=1                                                        // 单周期处理
    iterations++;
    if (iterations % 1000000 == 0) {
      DEBUG_PRINT_IMPORTANT("HBM_Write_Service: iterations=" << iterations << " requests_processed=" << requests_processed);
    }
    
    // 使用非阻塞读取避免软件仿真中的死锁
    if (req_in.empty()) {
      idle_cycles++;
      if (idle_cycles % 10000 == 0) {
        DEBUG_PRINT_IMPORTANT("HBM_Write_Service: Waiting for requests, idle_cycles=" << idle_cycles << " iterations=" << iterations);
      }
      continue;
    }
    
    WriteReqTagged rq = req_in.read();
    DEBUG_PRINT_IMPORTANT("HBM_Write_Service: ✅ 收到写请求 #" << (requests_processed + 1) << " from PE" << (int)rq.pe << " kind=" << (int)rq.r.kind << " len=" << rq.r.len);
    
    // 成功读取到请求
    requests_processed++;
    idle_cycles = 0;
    
    const uint8_t kind = rq.r.kind;                                              // 1:L, 2:U (A矩阵不写入)
    // 写服务只处理L和U，不处理A
    if (kind == 0) {
      DEBUG_PRINT("HBM_Write_Service: 警告：收到A矩阵写请求，跳过（A矩阵只读）");
      // 跳过A矩阵写请求，直接发送ACK
      wr_ack_pe.write(rq.pe);
      continue;
    }
    const PortSel s = (kind == 1) ? decode_addr(rq.r.addr, /*lanes*/8, false)
                                  : decode_addr(rq.r.addr, /*lanes*/4, false);

    if (requests_processed <= 10) {
      DEBUG_PRINT("HBM写服务: 收到写请求 #" << requests_processed << " from PE" << (int)rq.pe
                  << " kind=" << (kind == 1 ? "L" : "U") << " len=" << rq.r.len
                  << " addr=" << rq.r.addr << " lane=" << (int)s.lane);
    }

    // 直接从数组中选择对应的端口（只处理L和U）
    auto& p_port = (kind == 1) ? L_arr[s.lane] : U_arr[s.lane];

    const uint32_t total64 = rq.r.len + s.head_pad;                              // 需要写的 64b 数（含 pad）
    const uint32_t words   = (total64 + VEC_64_PER_512 - 1) / VEC_64_PER_512;    // 需要的 512b 数

#ifdef __SYNTHESIS__
    // 综合路径：完全展开端口选择，无数组
    ap_uint<512> acc = 0;
    uint32_t filled = 0;
    uint32_t word_idx = 0;
    for (uint8_t i = 0; i < s.head_pad; i++) {
#pragma HLS pipeline II=1
      set_word64(acc, filled++, 0);
      if (filled == VEC_64_PER_512) {
        if (kind == 1) {
          if (s.lane == 0) L_0[s.word_off + word_idx] = acc;
          else if (s.lane == 1) L_1[s.word_off + word_idx] = acc;
          else if (s.lane == 2) L_2[s.word_off + word_idx] = acc;
          else if (s.lane == 3) L_3[s.word_off + word_idx] = acc;
          else if (s.lane == 4) L_4[s.word_off + word_idx] = acc;
          else if (s.lane == 5) L_5[s.word_off + word_idx] = acc;
          else if (s.lane == 6) L_6[s.word_off + word_idx] = acc;
          else if (s.lane == 7) L_7[s.word_off + word_idx] = acc;
        } else {
          if (s.lane == 0) U_0[s.word_off + word_idx] = acc;
          else if (s.lane == 1) U_1[s.word_off + word_idx] = acc;
          else if (s.lane == 2) U_2[s.word_off + word_idx] = acc;
          else if (s.lane == 3) U_3[s.word_off + word_idx] = acc;
        }
        acc = 0; filled = 0; word_idx++;
      }
    }
    for (uint32_t i = 0; i < rq.r.len; i++) {
#pragma HLS pipeline II=1
      WriteElemTagged elem;
      while (!elem_in.try_read(elem)) {
        // Wait for data to become available
      }
      set_word64(acc, filled++, elem.v);
      if (filled == VEC_64_PER_512) {
        if (kind == 1) {
          if (s.lane == 0) L_0[s.word_off + word_idx] = acc;
          else if (s.lane == 1) L_1[s.word_off + word_idx] = acc;
          else if (s.lane == 2) L_2[s.word_off + word_idx] = acc;
          else if (s.lane == 3) L_3[s.word_off + word_idx] = acc;
          else if (s.lane == 4) L_4[s.word_off + word_idx] = acc;
          else if (s.lane == 5) L_5[s.word_off + word_idx] = acc;
          else if (s.lane == 6) L_6[s.word_off + word_idx] = acc;
          else if (s.lane == 7) L_7[s.word_off + word_idx] = acc;
        } else {
          if (s.lane == 0) U_0[s.word_off + word_idx] = acc;
          else if (s.lane == 1) U_1[s.word_off + word_idx] = acc;
          else if (s.lane == 2) U_2[s.word_off + word_idx] = acc;
          else if (s.lane == 3) U_3[s.word_off + word_idx] = acc;
        }
        acc = 0; filled = 0; word_idx++;
      }
    }
    if (filled) {
      for (; filled < VEC_64_PER_512; ++filled) set_word64(acc, filled, 0);
      if (kind == 1) {
        if (s.lane == 0) L_0[s.word_off + word_idx] = acc;
        else if (s.lane == 1) L_1[s.word_off + word_idx] = acc;
        else if (s.lane == 2) L_2[s.word_off + word_idx] = acc;
        else if (s.lane == 3) L_3[s.word_off + word_idx] = acc;
        else if (s.lane == 4) L_4[s.word_off + word_idx] = acc;
        else if (s.lane == 5) L_5[s.word_off + word_idx] = acc;
        else if (s.lane == 6) L_6[s.word_off + word_idx] = acc;
        else if (s.lane == 7) L_7[s.word_off + word_idx] = acc;
      } else {
        if (s.lane == 0) U_0[s.word_off + word_idx] = acc;
        else if (s.lane == 1) U_1[s.word_off + word_idx] = acc;
        else if (s.lane == 2) U_2[s.word_off + word_idx] = acc;
        else if (s.lane == 3) U_3[s.word_off + word_idx] = acc;
      }
    }
    // 综合路径：发送ACK
    while (wr_ack_pe.full()) {
#pragma HLS pipeline II=1
      // wait
    }
    wr_ack_pe.write(rq.pe);
#else
    // 软件模拟模式：使用同步写入（非综合路径）
    ap_uint<512>* ptr = p_port.get();
    ap_uint<512> acc = 0;
    uint32_t filled = 0;
    uint32_t word_idx = 0;
    for (uint8_t i = 0; i < s.head_pad; i++) {
#pragma HLS pipeline II=1
      set_word64(acc, filled++, 0);
      if (filled == VEC_64_PER_512) {
        ptr[s.word_off + word_idx++] = acc;
        acc = 0; filled = 0;
      }
    }
    for (uint32_t i = 0; i < rq.r.len; i++) {
#pragma HLS pipeline II=1
      WriteElemTagged elem = elem_in.read();
      if (requests_processed <= 3 && i < 3) {
        DEBUG_PRINT_IMPORTANT("HBM写服务: 收到数据元素 #" << i << "/" << rq.r.len << " from PE" << (int)elem.pe << " value=" << elem.v);
      }
      set_word64(acc, filled++, elem.v);
      if (filled == VEC_64_PER_512) {
        ptr[s.word_off + word_idx++] = acc;
        acc = 0; filled = 0;
      }
    }
    if (filled) {
      for (; filled < VEC_64_PER_512; ++filled) set_word64(acc, filled, 0);
      ptr[s.word_off + word_idx++] = acc;
    }
    
    if (requests_processed <= 10) {
      DEBUG_PRINT_IMPORTANT("HBM写服务: ✅ 写入完成，发送 ACK 给 PE" << (int)rq.pe);
    }
    while (wr_ack_pe.full()) {
#pragma HLS pipeline II=1
      // wait
    }
    wr_ack_pe.write(rq.pe); // 回 ACK(pe)
    if (requests_processed <= 10) {
      DEBUG_PRINT_IMPORTANT("HBM写服务: ✅ ACK 已发送给 PE" << (int)rq.pe);
    }
#endif
    
    if (requests_processed % 1000 == 0) {
      DEBUG_PRINT("HBM写服务: 📊 已处理 " << requests_processed << " 个写请求");
    }
    // ACK 已在综合路径或软件仿真路径中发送，此处不再重复发送
  }
}

//======================================================================
//  GlobalLUCache：共享 LU cache（NUM_PE 端口）
//  V1 块缓存版本：缓存连续8个元素作为一个块
//======================================================================

void GlobalLUCache(
  tapa::istream<LUReq>& req_in,                       // 输入请求（单个流）
  tapa::ostream<LURsp>& rsp_out,                      // 输出响应（单个流）
  tapa::istream<bool>& done_in                        // 结束信号
) {
#pragma HLS interface ap_ctrl_none port=return

#if DISABLE_GLOBAL_CACHE
  // 禁用缓存：提供空实现以维持单向FIFO连接
  // 读请求立即返回 miss=0、data=0；写请求直接丢弃
  for (;;) {
#pragma HLS pipeline II=1
    // 消费完成信号，保持接口活跃
    if (!done_in.empty()) (void)done_in.read();

    if (!req_in.empty()) {
      auto rq = req_in.read();
      if (!rq.is_write) {
        LURsp rsp{rq.addr, (WORD64)0, false, rq.pe};
        rsp_out.write(rsp);
      }
    }
  }

#elif USE_BLOCK_CACHE
  DEBUG_PRINT_IMPORTANT("GlobalLUCache: Started (V1 Block Cache - 8 elements per block, single stream mode)");
  
  // 块缓存存储
  static CacheBlock cache[CACHE_BLOCKS];
#pragma HLS bind_storage variable=cache type=RAM_2P impl=URAM
  // cache 是结构体数组，Vitis HLS 2022.2 不支持对 cache.data 使用
  // member-level array_partition，会将 cache 误解析为单个结构体。
  // 每周期只访问一个 data[offset]，因此保留整体双口 URAM 即可。
  
  // ========== 响应缓冲（参考 LevelST 设计）==========
  // 用于解耦请求和响应，避免响应队列满导致的死锁
  static LURsp pending_rsp;
  static bool has_pending = false;
  
  // 统计信息
  static CacheStats stats = {0, 0, 0, 0, 0, 0};
  
  uint32_t idle_cycles = 0;
  const uint32_t IDLE_THRESHOLD = 100000;
  const uint32_t STATS_REPORT_INTERVAL = 10000;
  uint32_t cycles_since_last_report = 0;
  uint32_t total_cycles = 0;
  bool main_done = false;
  
  for (;;) {
#pragma HLS pipeline II=1
    total_cycles++;
    bool has_activity = false;
    
    // 检查结束信号
    if (!done_in.empty()) {
      main_done = done_in.read();
      if (main_done) {
        DEBUG_PRINT("GlobalLUCache_Block: Received done signal");
      }
    }
    
    // 定期状态报告
    if (total_cycles % 50000 == 0) {
      uint64_t total_ops = stats.total_requests + stats.total_writes;
      DEBUG_PRINT("GlobalLUCache_Block: Cycle " << total_cycles 
                  << " reads=" << stats.total_requests 
                  << " writes=" << stats.total_writes 
                  << " hits=" << stats.total_hits
                  << " partial_hits=" << stats.partial_hits
                  << " block_fills=" << stats.block_fills);
    }
    
    // ========== 优先发送挂起的响应（关键！）==========
    if (has_pending && !rsp_out.full()) {
      rsp_out.write(pending_rsp);
      has_pending = false;
      has_activity = true;
    }
    
    // ========== 处理新请求（只有在没有挂起响应时）==========
    if (!has_pending && !req_in.empty()) {
      auto rq = req_in.read();
      has_activity = true;
      
      // 地址分解
      uint32_t addr = rq.addr;
      uint32_t block_index = get_block_index(addr);
      uint32_t offset = get_block_offset(addr);
      ap_uint<CACHE_TAG_BITS> tag = get_block_tag(addr);
      
      if (rq.is_write) {
        // ========== 写操作 ==========
        stats.total_writes++;
        
        // 检查块是否存在
        bool block_exists = cache[block_index].valid && 
                           (cache[block_index].tag == tag);
        
        if (!block_exists) {
          // 新块：初始化
          cache[block_index].valid = true;
          cache[block_index].tag = tag;
          cache[block_index].valid_mask = 0;
          stats.block_fills++;
        }
        
        // 写入元素
        cache[block_index].data[offset] = rq.data;
        cache[block_index].valid_mask[offset] = 1;
        
        if (stats.total_writes <= 10) {
          DEBUG_PRINT("GlobalLUCache_Block[PE" << (int)rq.pe << "]: WRITE addr=" << addr 
                      << " block=" << block_index << " offset=" << offset 
                      << " tag=" << tag << " new_block=" << !block_exists);
        }
        
      } else {
        // ========== 读操作 ==========
        stats.total_requests++;
        
        // 检查块命中
        bool block_hit = cache[block_index].valid && 
                        (cache[block_index].tag == tag);
        
        // 检查元素有效
        bool element_valid = block_hit && 
                            cache[block_index].valid_mask[offset];
        
        WORD64 val = element_valid ? cache[block_index].data[offset] : WORD64(0);
        
        if (element_valid) {
          stats.total_hits++;
          if (stats.total_hits <= 10) {
            DEBUG_PRINT("GlobalLUCache_Block[PE" << (int)rq.pe << "]: READ HIT addr=" << addr 
                        << " block=" << block_index << " offset=" << offset);
          }
        } else {
          stats.total_misses++;
          if (block_hit && !element_valid) {
            stats.partial_hits++;  // 块存在但元素无效
          }
          if (stats.total_misses <= 10) {
            DEBUG_PRINT("GlobalLUCache_Block[PE" << (int)rq.pe << "]: READ MISS addr=" << addr 
                        << " block=" << block_index << " offset=" << offset
                        << " block_hit=" << block_hit 
                        << " element_valid=" << element_valid);
          }
        }
        
        // ========== 尝试立即发送，失败则缓存（参考 LevelST）==========
        LURsp rsp{rq.addr, val, element_valid, rq.pe};
        if (!rsp_out.full()) {
          rsp_out.write(rsp);
        } else {
          pending_rsp = rsp;
          has_pending = true;
        }
      }
    }
    
    // 定期报告统计
    cycles_since_last_report++;
    if (cycles_since_last_report >= STATS_REPORT_INTERVAL && stats.total_requests > 0) {
      double hit_rate = (double)stats.total_hits / stats.total_requests * 100.0;
      double partial_rate = (double)stats.partial_hits / stats.total_requests * 100.0;
      DEBUG_PRINT("GlobalLUCache_Block Stats: Requests=" << stats.total_requests 
                  << " Hits=" << stats.total_hits 
                  << " Misses=" << stats.total_misses 
                  << " Writes=" << stats.total_writes
                  << " PartialHits=" << stats.partial_hits
                  << " BlockFills=" << stats.block_fills
                  << " HitRate=" << hit_rate << "%"
                  << " PartialRate=" << partial_rate << "%");
      cycles_since_last_report = 0;
    }
    
    // 检测空闲超时
    if (has_activity) {
      idle_cycles = 0;
    } else {
      idle_cycles++;
      if (main_done && idle_cycles >= IDLE_THRESHOLD) {
        DEBUG_PRINT("GlobalLUCache_Block: Exiting after " << total_cycles << " cycles");
        DEBUG_PRINT("GlobalLUCache_Block Final Stats:");
        DEBUG_PRINT("  Total Requests: " << stats.total_requests);
        DEBUG_PRINT("  Total Hits: " << stats.total_hits);
        DEBUG_PRINT("  Total Misses: " << stats.total_misses);
        DEBUG_PRINT("  Total Writes: " << stats.total_writes);
        DEBUG_PRINT("  Partial Hits: " << stats.partial_hits);
        DEBUG_PRINT("  Block Fills: " << stats.block_fills);
        if (stats.total_requests > 0) {
          double hit_rate = (double)stats.total_hits / stats.total_requests * 100.0;
          DEBUG_PRINT("  Hit Rate: " << hit_rate << "%");
        }
        return;
      }
    }
  }
  
#else
  // 原有直接映射缓存实现
  DEBUG_PRINT_IMPORTANT("GlobalLUCache: Started (Original Direct Mapped Version)");
  
  static CacheEntry cache[GLOBAL_LU_CACHE_SIZE];
#pragma HLS bind_storage variable=cache type=RAM_2P impl=URAM
  
  // ========== 响应缓冲（参考 LevelST 设计）==========
  static LURsp pending_rsp[NUM_PE];
  static bool has_pending[NUM_PE] = {false};
#pragma HLS array_partition variable=pending_rsp complete
#pragma HLS array_partition variable=has_pending complete
  
  static CacheStats stats = {0, 0, 0, 0};
  
  uint32_t idle_cycles = 0;
  const uint32_t IDLE_THRESHOLD = 100000;
  const uint32_t STATS_REPORT_INTERVAL = 10000;
  uint32_t cycles_since_last_report = 0;
  uint32_t total_cycles = 0;
  bool main_done = false;
  
  for (;;) {
#pragma HLS pipeline II=1
    total_cycles++;
    bool has_activity = false;
    
    if (!done_in.empty()) {
      main_done = done_in.read();
      if (main_done) {
        DEBUG_PRINT("GlobalLUCache: Received done signal");
      }
    }
    
    if (total_cycles % 50000 == 0) {
      uint64_t total_ops = stats.total_requests + stats.total_writes;
      DEBUG_PRINT("GlobalLUCache: Cycle " << total_cycles << " idle=" << idle_cycles 
                  << " reads=" << stats.total_requests << " writes=" << stats.total_writes 
                  << " total=" << total_ops << " hits=" << stats.total_hits);
    }
    
    for (int p = 0; p < NUM_PE; p++) {
#pragma HLS unroll
      
      // ========== 优先发送挂起的响应（关键！）==========
      if (has_pending[p] && !rsp_out[p].full()) {
        rsp_out[p].try_write(pending_rsp[p]);
        has_pending[p] = false;
        has_activity = true;
      }
      
      // ========== 处理新请求（只有在没有挂起响应时）==========
      if (!has_pending[p] && !req_in[p].empty()) {
        auto rq = req_in[p].read();
        has_activity = true;
        
        ap_uint<32> addr = rq.addr;
        uint32_t index = addr & ((1 << CACHE_INDEX_BITS) - 1);
        ap_uint<20> tag = addr >> CACHE_INDEX_BITS;
        
        if (rq.is_write) {
          // Write request: directly write to cache and finish.
          // The previous logic of waiting for a read was causing deadlock.
          cache[index].valid = true;
          cache[index].tag = tag;
          cache[index].data = rq.data;
          stats.total_writes++;
          
          if (stats.total_writes < 10) {
            DEBUG_PRINT("GlobalLUCache[PE" << p << "]: WRITE addr=" << addr 
                        << " index=" << index << " tag=" << tag);
          }
        } else {
          stats.total_requests++;
          
          bool hit = cache[index].valid && (cache[index].tag == tag);
          WORD64 val = hit ? cache[index].data : WORD64(0);
          
          if (hit) {
            stats.total_hits++;
            if (stats.total_hits < 10) {
              DEBUG_PRINT("GlobalLUCache[PE" << p << "]: READ HIT addr=" << addr 
                          << " index=" << index << " tag=" << tag);
            }
          } else {
            stats.total_misses++;
            if (stats.total_misses < 10) {
              DEBUG_PRINT("GlobalLUCache[PE" << p << "]: READ MISS addr=" << addr 
                          << " index=" << index << " tag=" << tag 
                          << " (valid=" << cache[index].valid 
                          << " cached_tag=" << cache[index].tag << ")");
            }
          }
          
          // ========== 尝试立即发送，失败则缓存（参考 LevelST）==========
          LURsp rsp = LURsp{addr, val, hit};
          if (!rsp_out[p].full()) {
            rsp_out[p].try_write(rsp);
          } else {
            pending_rsp[p] = rsp;
            has_pending[p] = true;
          }
        }
      }
    }
    
    cycles_since_last_report++;
    if (cycles_since_last_report >= STATS_REPORT_INTERVAL && stats.total_requests > 0) {
      double hit_rate = (double)stats.total_hits / stats.total_requests * 100.0;
      DEBUG_PRINT("GlobalLUCache Stats: Requests=" << stats.total_requests 
                  << " Hits=" << stats.total_hits 
                  << " Misses=" << stats.total_misses 
                  << " Writes=" << stats.total_writes 
                  << " HitRate=" << hit_rate << "%");
      cycles_since_last_report = 0;
    }
    
    if (has_activity) {
      idle_cycles = 0;
    } else {
      idle_cycles++;
      if (main_done && idle_cycles >= IDLE_THRESHOLD) {
        DEBUG_PRINT("GlobalLUCache: Exiting after " << total_cycles << " cycles");
        DEBUG_PRINT("GlobalLUCache Final Stats: Requests=" << stats.total_requests 
                    << " Hits=" << stats.total_hits 
                    << " Misses=" << stats.total_misses 
                    << " Writes=" << stats.total_writes);
        if (stats.total_requests > 0) {
          double hit_rate = (double)stats.total_hits / stats.total_requests * 100.0;
          DEBUG_PRINT("GlobalLUCache Hit Rate: " << hit_rate << "%");
        }
        return;
      }
    }
  }
#endif
}

//======================================================================
//  SegmentExecutor_withDepStream：细粒度段执行核（带 LU Cache）
//  【状态机版本】：完全非阻塞处理，支持多任务并发
//======================================================================
void SegmentExecutor_withDepStream(
  const ap_uint<8> pe_id_raw,
  tapa::istream<ReadySeg>& task_in,
  tapa::istream<DepEntry>& dep_stream_in,
  tapa::ostream<ReadReqTagged>& rd_req_q,
  tapa::istream<WORD64>& rd_elem_q,
  tapa::ostream<WriteReqTagged>& wr_req_q,
  tapa::ostream<WriteElemTagged>& wr_elem_q,
  tapa::istream<ap_uint<1>>& wr_ack_in,
  tapa::ostream<uint32_t>& seg_done_out,
  tapa::ostream<LUReq>& glb_req_out,
  tapa::istream<LURsp>& glb_rsp_in
) {
  // 确保 pe_id 在有效范围内
  const ap_uint<8> pe_id = (pe_id_raw < NUM_PE) ? pe_id_raw : (ap_uint<8>)(pe_id_raw % NUM_PE);
  
  // 验证 pe_id 的有效性（在软件仿真中添加检查）
  if ((uint32_t)pe_id >= NUM_PE) {
    DEBUG_PRINT_IMPORTANT("SegmentExecutor: ❌ 警告：pe_id_raw=" << (uint32_t)pe_id_raw << " pe_id=" << (uint32_t)pe_id << " 超出范围，已归一化");
  }
  
  static WORD64 A_buf[A_WIN_MAX];
#if FAST_SYNTH
#pragma HLS bind_storage variable=A_buf type=RAM_2P impl=BRAM
#else
#pragma HLS bind_storage variable=A_buf type=RAM_2P impl=URAM
#endif
  static WORD64 Out_buf[OUT_BUF_MAX];
#if FAST_SYNTH
#pragma HLS bind_storage variable=Out_buf type=RAM_2P impl=BRAM
#else
#pragma HLS bind_storage variable=Out_buf type=RAM_2P impl=URAM
#endif
  static WORD64 L_buf[L_SLICE_MAX];
#if FAST_SYNTH
#pragma HLS bind_storage variable=L_buf type=RAM_2P impl=BRAM
#else
#pragma HLS bind_storage variable=L_buf type=RAM_2P impl=URAM
#endif
  // 依赖列表本地缓冲：按段预取一次，后续每个A小块复用
  static DepEntry dep_list[
#if FAST_SYNTH
    512
#else
    2048
#endif
  ];
#if FAST_SYNTH
#pragma HLS bind_storage variable=dep_list type=RAM_2P impl=BRAM
#else
#pragma HLS bind_storage variable=dep_list type=RAM_2P impl=URAM
#endif

  // 状态机状态定义
  enum State {
    IDLE,              // 0: 等待新任务
    READ_A_REQ,        // 1: 发送A段读请求（按tile）
    READ_A_DATA,       // 2: 接收A段数据（按tile）
    PREFETCH_DEPS,     // 3: 首次读取该段全部依赖到本地缓冲
    PROC_DEP,          // 4: 处理依赖（从本地列表逐条应用到当前tile）
    QUERY_L_CACHE,     // 4: 查询L缓存
    WAIT_L_CACHE,      // 5: 等待L缓存响应
    READ_L_DATA,       // 6: 读取L数据（cache miss）
    QUERY_U_CACHE,     // 7: 查询U缓存
    WAIT_U_CACHE,      // 8: 等待U缓存响应
    READ_U_DATA,       // 9: 读取U数据（cache miss）
    COMPUTE,           // 10: 计算 A -= L×U
    WRITE_REQ,         // 11: 发送写请求（批量版本）
    WRITE_DATA_BATCH,  // 12: 批量发送写数据（优化版本）
    WRITE_DATA,        // 13: 发送写数据（保留用于兼容）
    WAIT_WRITE_ACK,    // 14: 等待写确认
    SEND_COL_DONE,     // 14: 发送列完成信号
    DONE               // 15: 任务完成
  };

  // 状态变量
  State state = IDLE;
  ReadySeg seg;
  uint32_t win_len = 0;
  uint32_t a_recv_cnt = 0;
  uint32_t dep_cnt = 0;
  uint32_t dep_idx = 0;              // 当前处理的依赖下标（在本地dep_list内）
  uint32_t dep_list_len = 0;         // 实际预取的依赖数
  bool deps_prefetched = false;       // 是否已将该段的依赖预取到本地
  DepEntry cur_dep;
  uint32_t l_recv_cnt = 0;
  uint32_t l_req_issued = 0;
  bool l_cache_hit = false;
  WORD64 U64 = 0;
  bool u_cache_hit = false;
  bool u_read_req_sent = false;
  uint32_t write_offset = 0;
  uint32_t write_batch_size = 0;
  uint32_t write_data_cnt = 0;
  uint32_t batch_count = 0;
  // 批量写回优化：用于收集连续的 L 元素
  uint32_t write_batch_start = 0;      // 当前批次的起始偏移
  uint32_t write_batch_end = 0;       // 当前批次的结束偏移
  uint32_t write_batch_base_addr = 0; // 当前批次的基地址
  uint8_t write_batch_kind = 0;       // 批次类型：1=L, 2=U
  // 整段进度计数：已完成写回的总元素数（64b 元素）

  // tile 控制
  uint32_t tile_len = 0;             // 本次tile长度（按64b元素）
  uint32_t a_req_issued = 0;         // 已下发的A读请求元素数
  // 针对当前依赖在本tile的投影
  uint32_t cur_len_tile = 0;         // 当前依赖在本tile参与的长度
  uint32_t cur_L_addr_tile = 0;      // 当前依赖对应的L起始地址（带read_offset平移）
  
  // 稀疏格式读取控制变量
  uint32_t word_count = 0;            // 段占用的512位字数（稀疏格式）
  uint32_t words_received = 0;       // 已接收的512位字数
  WORD512 hbm_word_buffer = 0;       // 临时缓冲：从8个64位元素组装512位字
  uint32_t word_buffer_cnt = 0;       // 当前512位字的64位元素计数
  
  uint32_t segments_processed = 0;
  uint32_t timeout_cnt = 0;
  const uint32_t MAX_TIMEOUT = 1000000;  // 恢复较大超时，减少日志刷屏
  // 统计：本 PE 发送的段完成次数。列聚合只能由 DegScheduler 完成。
  uint32_t seg_done_sent = 0;
  bool lu_started_reported = false;  // 首次真实LU计算标记

  DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: Starting state machine");

  // 主状态机循环
  for (;;) {
#pragma HLS pipeline II=1
    
    switch (state) {
      case IDLE:
        // 降低IDLE打印频率：仅在少量前期或长周期间隔打印
        if (segments_processed < 4) {
          DEBUG_PRINT("STATE: IDLE");
        } else if ((timeout_cnt & 0x7FFFF) == 0) { // 每 ~524k 周期
          DEBUG_PRINT("STATE: IDLE");
        }
        // 尝试读取新任务
        if (task_in.try_read(seg)) {
          // 减少调试输出频率，仅在关键任务时输出
          if (segments_processed < 10 || segments_processed % 1000 == 0) {
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: TASK RECV seg_id=" << seg.seg_id
                                      << " j=" << seg.j
                                      << " win_len=" << (seg.row_hi - seg.row_lo)
                                      << " dep_cnt=" << (seg.dep_hi - seg.dep_lo)
                                      << " is_last=" << (uint32_t)seg.is_last);
          }
          if (segments_processed < 100 || segments_processed % 1000 == 0) {
            SUMMARY_PRINT("PE" << (int)pe_id << ": TASK_RECV seg_id=" << seg.seg_id << " j=" << seg.j << " len=" << (seg.row_hi - seg.row_lo) << " dep_cnt=" << (seg.dep_hi - seg.dep_lo));
          }
          if (seg.seg_id == 0xFFFFFFFF) {
            SUMMARY_PRINT("SegmentExecutor[" << pe_id << "]: Terminator received, processed=" << segments_processed
                                  << " seg_done_sent=" << seg_done_sent);
            return;
          }
          win_len = seg.row_hi - seg.row_lo;
          dep_cnt = seg.dep_hi - seg.dep_lo;
          dep_idx = 0;
          a_recv_cnt = 0;
          a_req_issued = 0;
          timeout_cnt = 0;
          u_read_req_sent = false;
          // 初始化tile与依赖

          deps_prefetched = false;
          dep_list_len = 0;
          
          // 初始化稀疏格式读取变量
          word_count = seg.word_count;
          words_received = 0;
          word_buffer_cnt = 0;
          hbm_word_buffer = 0;
          
          // 初始化窗口缓冲区为0
          for (uint32_t i = 0; i < win_len && i < A_WIN_MAX; i++) {
#pragma HLS pipeline II=1
            A_buf[i] = 0;
            Out_buf[i] = 0;
          }
          
          // 减少调试输出频率
          if (segments_processed < 10 || (dep_cnt > 0 && segments_processed % 100 == 0)) {
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 📋 新任务 #" << segments_processed 
                                  << " seg_id=" << seg.seg_id << " j=" << seg.j 
                                  << " win_len=" << win_len << " dep_cnt=" << dep_cnt 
                                  << " dep_range=[" << seg.dep_lo << "," << seg.dep_hi << ") is_last=" << (uint32_t)seg.is_last);
          }
          
          // 调整优先级：优先预取依赖，避免A读等待导致长时间IDLE
          if (dep_cnt > 0) {
            state = PREFETCH_DEPS;
          } else if (win_len > 0) {
            state = READ_A_REQ;
          } else {
            state = SEND_COL_DONE;
          }
        }
        break;

      case READ_A_REQ:
          DEBUG_PRINT("STATE: READ_A_REQ");
        // 发送A段读请求（稀疏格式：读取512位字）
        {
          // 稀疏格式：A_addr现在表示512位字偏移，需要读取word_count个512位字
          // 但HBM_Read_Service期望len为64位元素数，所以需要转换
          // 每个512位字包含8个64位元素，所以需要读取 word_count * 8 个64位元素
          tile_len = word_count * VEC_64_PER_512;  // 需要读取的总64位元素数
          
          // 计算64位元素偏移：word_offset * 8
          uint32_t bank = addr_bank(seg.A_addr);
          uint32_t word_off_512 = addr_off(seg.A_addr);  // 512位字偏移
          uint32_t elem_off_64 = word_off_512 * VEC_64_PER_512;  // 转换为64位元素偏移
          uint32_t addr_elem = pack_addr(bank, elem_off_64);

          // 动态并行度：根据未回收数据量自适应并发
          uint32_t outstanding_a = (a_req_issued > words_received * VEC_64_PER_512) ? (a_req_issued - words_received * VEC_64_PER_512) : 0;
          uint32_t MAX_OUTSTANDING_A = (outstanding_a > 4096) ? 4u : 8u;
          const uint32_t CHUNK_LEN = 2048; // 每突发最多元素数（64b元素）
          ap_uint<2> issued_now = 0;
          while (issued_now < MAX_OUTSTANDING_A && a_req_issued < tile_len) {
            uint32_t remain = tile_len - a_req_issued;
            uint32_t burst_len = (remain > CHUNK_LEN) ? CHUNK_LEN : remain;
            uint32_t burst_addr = addr_elem + a_req_issued;
            // 改为阻塞写，确保发出请求
            rd_req_q.write(ReadReqTagged{ReadBurstReq{0, burst_addr, burst_len}, pe_id});
              if (segments_processed < 16) {
                DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: RD A req (sparse) seg_id=" << seg.seg_id
                                      << " word_count=" << word_count << " part_len=" << burst_len << " issued=" << (a_req_issued + burst_len)
                                      << "/" << tile_len);
              }
              a_req_issued += burst_len;
              issued_now++;
              timeout_cnt = 0;
            SUMMARY_PRINT("PE" << (int)pe_id << ": READ_A_REQ (sparse) issued len=" << burst_len << " issued_total=" << a_req_issued);
          }
          state = READ_A_DATA;
        }
        break;

      case READ_A_DATA:
          DEBUG_PRINT("STATE: READ_A_DATA");
        // 接收A段数据（稀疏格式：解析槽位并重构窗口）
        {
          WORD64 elem;
          if (rd_elem_q.try_read(elem)) {
            // 稀疏格式：每8个64位元素组装成一个512位字，然后解析槽位
            set_word64(hbm_word_buffer, word_buffer_cnt, elem);
            word_buffer_cnt++;
            a_recv_cnt++;
            timeout_cnt = 0;
            
            // 每收到8个64位元素，组装成一个完整的512位字并解析
            if (word_buffer_cnt >= VEC_64_PER_512) {
              // 解析512位字的8个槽位
              for (int slot = 0; slot < VEC_64_PER_512; slot++) {
#pragma HLS unroll
                WORD64 slot_elem = get_word64(hbm_word_buffer, slot);
                
                // 检查是否为DUMMY元素
                if (!is_dummy_element(slot_elem)) {
                  // 提取row_offset和value
                  uint16_t row_offset = extract_row_offset(slot_elem);
                  REAL val = extract_value(slot_elem);
                  
                  // 映射到窗口缓冲区（row_offset是相对于row_lo的偏移）
                  if (row_offset < win_len && row_offset < A_WIN_MAX) {
                    // REAL是float(32位),需要先转换为ap_uint<32>,然后扩展到64位
                    ap_uint<32> val_uint32 = tapa::bit_cast<ap_uint<32>>(val);
                    WORD64 val_word64 = (WORD64)val_uint32;  // 扩展到64位
                    A_buf[row_offset] = val_word64;
                    Out_buf[row_offset] = val_word64;
                    
                    if (segments_processed < 16 && words_received == 0 && slot == 0) {
                      DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: READ_A_DATA (sparse) recv word=" << words_received
                                        << " slot=" << slot << " row_offset=" << row_offset << " val=" << val);
                    }
                  }
                }
              }
              
              // 重置缓冲区，准备接收下一个512位字
              word_buffer_cnt = 0;
              hbm_word_buffer = 0;
              words_received++;
              
              if (words_received <= 4 || words_received == word_count) {
                DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: READ_A_DATA (sparse) recv words_received=" << words_received << "/" << word_count << " seg_id=" << seg.seg_id);
              }
              
              // 检查是否接收完所有512位字
              if (words_received >= word_count) {
                // 本tile数据接收完成，窗口已重构
                DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ✅ A tile完成 (sparse) seg_id=" << seg.seg_id 
                                      << " word_count=" << word_count << " dep_cnt=" << dep_cnt);
                // 更新tile_len为实际窗口长度（用于后续计算）
                tile_len = win_len;
                if (dep_cnt > 0) {
                  DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: → 进入 PROC_DEP（若未预取将由PREFETCH流程引导）");
                  dep_idx = 0; state = PROC_DEP;   // 优先进入依赖处理（PREFETCH完成后会到此）
                } else {
                  // 无依赖：为打通流水，先进行一次"热身计算"进入 COMPUTE，再写回
                  DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: → 无依赖，执行一次热身COMPUTE后 WRITE_REQ");
                  dep_list_len = 0;      // 作为热身标记（COMPUTE中识别）
                  cur_len_tile = tile_len;
                  state = COMPUTE;
                }
              }
            }
          } else {
            // 若尚有未下发的A突发，尝试继续下发，提升并行度
            if (a_req_issued < tile_len) {
              uint32_t outstanding_a2 = (a_req_issued > words_received * VEC_64_PER_512) ? (a_req_issued - words_received * VEC_64_PER_512) : 0;
              uint32_t MAX_OUTSTANDING_A = (outstanding_a2 > 4096) ? 3u : 6u;
              const uint32_t CHUNK_LEN = 2048;
              ap_uint<2> issued_now = 0;
              
              // 计算64位元素偏移
              uint32_t bank = addr_bank(seg.A_addr);
              uint32_t word_off_512 = addr_off(seg.A_addr);
              uint32_t elem_off_64 = word_off_512 * VEC_64_PER_512;
              uint32_t addr_elem = pack_addr(bank, elem_off_64);
              
              while (issued_now < MAX_OUTSTANDING_A && a_req_issued < tile_len) {
                uint32_t remain = tile_len - a_req_issued;
                uint32_t burst_len = (remain > CHUNK_LEN) ? CHUNK_LEN : remain;
                uint32_t burst_addr = addr_elem + a_req_issued;
                rd_req_q.write(ReadReqTagged{ReadBurstReq{0, burst_addr, burst_len}, pe_id});
                a_req_issued += burst_len;
                issued_now++;
                timeout_cnt = 0;
                SUMMARY_PRINT("PE" << (int)pe_id << ": READ_A_REQ(cont) len=" << burst_len << " issued_total=" << a_req_issued);
              }
            }
            timeout_cnt++;
            if (words_received >= word_count) {
              // 与正常完成路径一致
              tile_len = win_len;  // 更新为实际窗口长度
              if (dep_cnt > 0 && !deps_prefetched) { state = PREFETCH_DEPS; }
              else if (dep_cnt > 0) { dep_idx = 0; state = PROC_DEP; }
              else { dep_list_len = 0; cur_len_tile = tile_len; state = COMPUTE; }
            }
          }
        }
        break;

      case PREFETCH_DEPS:
          DEBUG_PRINT("STATE: PREFETCH_DEPS");
        // 首次：从dep_stream中读取本段的所有依赖到本地列表
        if (dep_list_len < dep_cnt) {
          DepEntry d;
          if (dep_stream_in.try_read(d)) {
            dep_list[dep_list_len] = d;
            dep_list_len++;
            timeout_cnt = 0;
            if (dep_list_len <= 4) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: PREFETCH_DEPS got dep#" << (dep_list_len-1)
                                        << " k=" << d.k << " L_addr=" << d.L_addr << " U_addr=" << d.U_addr << " len=" << d.len);
            }
          } else {
            timeout_cnt++;
          }
        } else {
          deps_prefetched = true;
          dep_idx = 0;
          DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: PREFETCH_DEPS 完成 dep_list_len=" << dep_list_len
                                << " a_recv_cnt=" << a_recv_cnt << "/" << tile_len
                                << " → 下一步=" << ((win_len > 0 && a_recv_cnt < tile_len) ? "READ_A_REQ" : "PROC_DEP"));
          // 如果A还未读完，先去读A；否则直接处理依赖
          if (win_len > 0 && words_received < word_count) {
            state = READ_A_REQ;
          } else {
            state = PROC_DEP;  // 进入依赖处理（针对当前tile）
          }
        }
        break;

      case PROC_DEP:
          DEBUG_PRINT("STATE: PROC_DEP");
        // 使用本地依赖列表，逐条应用到当前tile
        SUMMARY_PRINT("PE" << (int)pe_id << ": ENTER PROC_DEP dep_idx=" << dep_idx << "/" << dep_list_len);
        if (dep_idx >= dep_list_len) {
          // 本tile的依赖全部应用，进入写回
          state = WRITE_REQ;
          write_offset = 0;
          batch_count = 0;
          break;
        }

        cur_dep = dep_list[dep_idx];
        // 🔧 修复：处理 len==0 的情况（不需要读L，但仍需要U）
        if (cur_dep.len == 0) {
          // len==0 表示不需要读取L元素，但仍需要等待U(k,j)被计算
          // 直接跳到读取U，不需要读取L
          cur_len_tile = 0;
          timeout_cnt = 0;
          DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 📥 PROC_DEP (len=0) dep_idx=" << dep_idx << "/" << dep_list_len
                                << " k=" << cur_dep.k << " U_addr=" << cur_dep.U_addr << " (skip L read)");
          state = QUERY_U_CACHE;  // 直接查询U缓存
          break;
        }
        
        // 简化：直接使用整个依赖的长度
        cur_len_tile = cur_dep.len;
        cur_L_addr_tile = cur_dep.L_addr;

        timeout_cnt = 0;
        DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 📥 PROC_DEP tile dep_idx=" << dep_idx << "/" << dep_list_len
                              << " k=" << cur_dep.k << " L_addr_tile=" << cur_L_addr_tile
                              << " U_addr=" << cur_dep.U_addr << " len_tile=" << cur_len_tile);
        state = QUERY_L_CACHE;
        break;

      case QUERY_L_CACHE:
          DEBUG_PRINT("STATE: QUERY_L_CACHE");
        // 查询L缓存
        SUMMARY_PRINT("PE" << (int)pe_id << ": QUERY_L addr=" << cur_L_addr_tile << " len=" << cur_len_tile);
        if (glb_req_out.try_write(LUReq{false, cur_L_addr_tile, 0, pe_id})) {
          state = WAIT_L_CACHE;
          timeout_cnt = 0;
        }
        break;

      case WAIT_L_CACHE:
          DEBUG_PRINT("STATE: WAIT_L_CACHE");
        // 等待L缓存响应
        {
          LURsp rsp;
          if (glb_rsp_in.try_read(rsp)) {
            l_cache_hit = rsp.hit;
            if (l_cache_hit) {
              L_buf[0] = rsp.data;
              l_recv_cnt = 1;
            } else {
              l_recv_cnt = 0;
            }
            timeout_cnt = 0;
            
            if (!l_cache_hit && cur_len_tile > 0) {
              l_req_issued = 0;
              state = READ_L_DATA;
            } else if (l_cache_hit && cur_len_tile > 1) {
              l_req_issued = 1; // 命中1个元素，剩余继续读
              state = READ_L_DATA;
            } else {
              state = QUERY_U_CACHE;
            }
          } else {
            timeout_cnt++;
          }
        }
        break;

      case READ_L_DATA:
          DEBUG_PRINT("STATE: READ_L_DATA");
        // 读取L数据（cache miss或剩余数据），分块下发，提升并行度
        {
          const uint32_t CHUNK_LEN = 2048;
          uint32_t outstanding_l = (l_req_issued > l_recv_cnt) ? (l_req_issued - l_recv_cnt) : 0;
          uint32_t MAX_OUTSTANDING_L = (outstanding_l > 2048) ? 2u : 4u;
          ap_uint<2> issued_now = 0;
          while (issued_now < MAX_OUTSTANDING_L && l_req_issued < cur_len_tile) {
            uint32_t remain = cur_len_tile - l_req_issued;
            uint32_t burst_len = (remain > CHUNK_LEN) ? CHUNK_LEN : remain;
            uint32_t burst_addr = cur_L_addr_tile + l_req_issued;
            rd_req_q.write(ReadReqTagged{ReadBurstReq{1, burst_addr, burst_len}, pe_id});
            l_req_issued += burst_len;
            issued_now++;
            timeout_cnt = 0;
            SUMMARY_PRINT("PE" << (int)pe_id << ": READ_L_REQ part_len=" << burst_len << " issued_total=" << l_req_issued);
          }

          // 接收数据
          WORD64 elem;
          if (rd_elem_q.try_read(elem)) {
            L_buf[l_recv_cnt] = elem;
            // 写入cache（best-effort）
            glb_req_out.try_write(LUReq{true, cur_L_addr_tile + l_recv_cnt, elem, pe_id});
            l_recv_cnt++;
            timeout_cnt = 0;
            if (l_recv_cnt >= cur_len_tile) {
              state = QUERY_U_CACHE;
            }
          } else {
            timeout_cnt++;
          }
        }
        break;

      case QUERY_U_CACHE:
          DEBUG_PRINT("STATE: QUERY_U_CACHE");
        // 查询U缓存
        if (glb_req_out.try_write(LUReq{false, cur_dep.U_addr, 0, pe_id})) {
          state = WAIT_U_CACHE;
          timeout_cnt = 0;
          SUMMARY_PRINT("PE" << (int)pe_id << ": QUERY_U addr=" << cur_dep.U_addr);
        }
        break;

      case WAIT_U_CACHE:
          DEBUG_PRINT("STATE: WAIT_U_CACHE");
        // 等待U缓存响应
        {
          LURsp rsp;
          if (glb_rsp_in.try_read(rsp)) {
            u_cache_hit = rsp.hit;
            if (u_cache_hit) {
              U64 = rsp.data;
              state = COMPUTE;
            } else {
              state = READ_U_DATA;
            }
            timeout_cnt = 0;
          } else {
            timeout_cnt++;
          }
        }
        break;

      case READ_U_DATA: {
          DEBUG_PRINT("STATE: READ_U_DATA");
        // 读取U数据（cache miss）
        // 同一个 miss 只能有一个 HBM 请求在途，避免重复响应污染下一个任务。
        if (!u_read_req_sent &&
            rd_req_q.try_write(ReadReqTagged{ReadBurstReq{2, cur_dep.U_addr, 1}, pe_id})) {
          u_read_req_sent = true;
          SUMMARY_PRINT("PE" << (int)pe_id << ": READ_U_REQ addr=" << cur_dep.U_addr);
        }
        WORD64 elem;
        if (u_read_req_sent && rd_elem_q.try_read(elem)) {
          U64 = elem;
          // 尝试写入cache
          glb_req_out.try_write(LUReq{true, cur_dep.U_addr, elem, pe_id});
          u_read_req_sent = false;
          state = COMPUTE;
          timeout_cnt = 0;
        }
        timeout_cnt++;
        break;
      }

      case COMPUTE:
        // 计算 A -= L×U
        {
          DEBUG_PRINT("<<<<<<<<<<<<<<<<<<<<<<< COMPUTE STAGE ENTERED >>>>>>>>>>>>>>>>>>>>>>>>>");
          SUMMARY_PRINT("PE" << (int)pe_id << ": ENTER COMPUTE seg_id=" << seg.seg_id << " dep_idx=" << dep_idx << "/" << dep_list_len << " len=" << cur_len_tile);
          // dep_list_len==0 作为无依赖“热身计算”信号，不读取L/U，仅做一次空计算以打通流水
          bool warmup = (dep_list_len == 0);
          if (!warmup && !lu_started_reported) {
            SUMMARY_PRINT("LU_START: PE" << (int)pe_id << " seg_id=" << seg.seg_id << " j=" << seg.j);
            lu_started_reported = true;
          }
          REAL Uval;
          if (warmup) {
            Uval = 0;  // 不改变数据，仅进入计算分支
            if (cur_len_tile == 0) cur_len_tile = win_len;
            // 减少调试输出频率
            if (segments_processed < 10 || segments_processed % 1000 == 0) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 🚀🚀🚀 热身计算 seg_id=" << seg.seg_id << " j=" << seg.j << " len=" << cur_len_tile);
            }
          } else {
            // 减少调试输出频率
            if (segments_processed < 10 || segments_processed % 1000 == 0) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 🚀🚀🚀 开始LU计算 seg_id=" << seg.seg_id << " j=" << seg.j << " dep_idx=" << dep_idx << "/" << dep_list_len << " dep_k=" << cur_dep.k << " cur_len_tile=" << cur_len_tile);
            }
            REAL Utmp = *reinterpret_cast<const REAL*>(&U64);
            Uval = Utmp;
            if (segments_processed < 10 || segments_processed % 1000 == 0) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]:   U[" << cur_dep.k << "][" << seg.j << "] = " << Uval);
            }
          }
          
          // 🔧 修复：如果 cur_len_tile==0（即len==0），跳过计算，只是等待U
          if (cur_len_tile > 0) {
            // 打印前几个元素的计算过程
            for (uint32_t i = 0; i < cur_len_tile; i++) {
#pragma HLS pipeline II=1
              REAL Lval = warmup ? (REAL)0 : *reinterpret_cast<const REAL*>(&L_buf[i]);
              REAL acc = *reinterpret_cast<REAL*>(&Out_buf[i]);
              REAL nv = acc - Lval * Uval;
              
              // 减少调试输出：仅在最早几个任务时打印详细计算
              if (i < 3 && (segments_processed < 5 || segments_processed % 10000 == 0)) {
                  DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]:   [" << i << "] A=" << acc << " - L=" << Lval << " * U=" << Uval << " = " << nv);
              }
              Out_buf[i] = *reinterpret_cast<WORD64*>(&nv);
            }
          } else {
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]:   (len=0, skip computation, only wait for U)");
          }
          
          // 减少调试输出频率
          if (segments_processed < 10 || segments_processed % 1000 == 0) {
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ✅ LU计算完成 dep_idx=" << dep_idx << "/" << dep_list_len << (warmup?" (warmup)":""));
          }

          if (!warmup) {
            dep_idx++;
            if (dep_idx < dep_list_len) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: → 继续下一个依赖 PROC_DEP");
              state = PROC_DEP;          // 下一条依赖作用到当前tile
            } else {
              // 所有依赖作用完毕，写回本tile
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: → 所有依赖完成，进入 WRITE_REQ");
              state = WRITE_REQ;
              write_offset = 0;
              batch_count = 0;
            }
          } else {
            // 热身计算后直接写回
            state = WRITE_REQ;
            write_offset = 0;
            batch_count = 0;
          }
        }
        break;

      case WRITE_REQ:
          DEBUG_PRINT("STATE: WRITE_REQ");
        // 写回 L 和 U 元素（批量优化版本）
        // 收集连续的 L 元素，批量发送写请求以提升性能
        {
          const uint32_t MAX_WRITE_BURST = WRITE_BATCH_SIZE;  // 统一使用宏配置批量大小
          uint32_t seg_j = seg.j;
          
          // 收集连续的 L 元素形成批次
          write_batch_start = write_offset;
          write_batch_end = write_offset;
          write_batch_kind = 0;
          write_batch_base_addr = 0;
          
          while (write_batch_end < win_len && (write_batch_end - write_batch_start) < MAX_WRITE_BURST) {
            uint32_t global_row = seg.row_lo + write_batch_end;
            
            if (global_row > seg_j) {
              // L 元素: L(global_row, seg_j)
              uint32_t L_off = global_row - seg_j - 1;
              uint32_t L_addr = seg.L_base_addr + L_off;
              
              if (write_batch_kind == 0) {
                // 开始新批次
                write_batch_kind = 1;
                write_batch_base_addr = L_addr;
                write_batch_start = write_batch_end;
                write_batch_end++;
              } else if (write_batch_kind == 1 && L_addr == write_batch_base_addr + (write_batch_end - write_batch_start)) {
                // 地址连续，继续添加到批次
                write_batch_end++;
              } else {
                // 地址不连续，结束当前批次
                break;
              }
            } else if (global_row == seg_j) {
              // U 对角线元素（单个元素，单独处理）
              if (write_batch_kind == 0) {
                write_batch_kind = 2;
                write_batch_base_addr = seg.U_diag_addr;
                write_batch_start = write_batch_end;
                write_batch_end++;
                break;  // U 只有一个元素，批次结束
              } else {
                // 类型不匹配，结束当前批次
                break;
              }
            } else {
              // global_row < seg_j: 跳过上三角部分
              write_batch_end++;
            }
          }
          
          // 发送批量写请求
          if (write_batch_kind > 0 && write_batch_end > write_batch_start) {
            uint32_t batch_len = write_batch_end - write_batch_start;
            wr_req_q.write(WriteReqTagged{WriteBurstReq{write_batch_kind, write_batch_base_addr, batch_len}, pe_id});
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 📤 WRITE_REQ batch kind=" << (int)write_batch_kind 
                                  << " base_addr=" << write_batch_base_addr << " len=" << batch_len);
            state = WRITE_DATA_BATCH;
            write_data_cnt = 0;
            timeout_cnt = 0;
          } else if (write_offset >= win_len) {
            // 所有行都处理完了
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 🧮 L/U Writeback done for seg_id=" << seg.seg_id);
            state = SEND_COL_DONE;
          } else {
            // 没有找到可写的元素，跳过
            write_offset = write_batch_end;
          }
        }
        break;

      case WRITE_DATA_BATCH:
          DEBUG_PRINT("STATE: WRITE_DATA_BATCH");
        // 批量发送写数据
        {
          // 批量发送数据元素
          for (uint32_t i = write_batch_start; i < write_batch_end; i++) {
#pragma HLS pipeline II=1
            WORD64 data = Out_buf[i];
            wr_elem_q.write(WriteElemTagged{data, pe_id});
            
            if (write_batch_kind == 1) {
              // L 元素：计算地址并发送到全局缓存
              uint32_t L_addr = write_batch_base_addr + (i - write_batch_start);
              glb_req_out.try_write(LUReq{true, L_addr, data, pe_id});
              if (i < write_batch_start + 3 || i == write_batch_end - 1) {
                DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 📨 WRITE_DATA_BATCH L[" << (seg.row_lo + i) << "][" << seg.j << "] = " << data);
              }
            } else if (write_batch_kind == 2) {
              // U 对角线元素（单个）
              glb_req_out.try_write(LUReq{true, write_batch_base_addr, data, pe_id});
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 📨 WRITE_DATA_BATCH U[" << seg.j << "][" << seg.j << "] = " << data);
            }
          }
          
          // 更新写偏移量
          write_offset = write_batch_end;
          state = WAIT_WRITE_ACK;
          timeout_cnt = 0;
        }
        break;

      case WAIT_WRITE_ACK:
          DEBUG_PRINT("STATE: WAIT_WRITE_ACK");
        // 等待批量写确认（需要等待批次中所有元素都确认）
        {
          uint32_t batch_len = write_batch_end - write_batch_start;
          uint32_t acks_received = 0;
          
          // 等待所有确认（批量写可能只需要一个确认，取决于HBM_Write_Service的实现）
          // 为了简化，我们等待一个确认表示批次完成
          if (!wr_ack_in.empty()) {
            wr_ack_in.read();
            acks_received++;
            timeout_cnt = 0;
            
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ✅ WRITE_ACK received (batch len=" << batch_len 
                                  << ") seg_id=" << seg.seg_id << " write_offset=" << write_offset);

            if (write_offset >= win_len) {
              // 整个段写回完成
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 🧮 Segment done seg_id=" << seg.seg_id << " win_len=" << win_len);
              state = SEND_COL_DONE;
            } else {
              // 继续处理下一个批次
              state = WRITE_REQ;
            }
          } else {
            timeout_cnt++;
          }
        }
        break;

      case SEND_COL_DONE:
          DEBUG_PRINT("STATE: SEND_COL_DONE");
        // 每个段在全部 HBM 写 ACK 后发送自己的 seg_id。
        DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: >>> 进入 SEND_COL_DONE seg_id=" << seg.seg_id
                              << " j=" << seg.j
                              << " is_last=" << (uint32_t)seg.is_last
                              << " win_len=" << win_len);

        if (seg_done_out.try_write(seg.seg_id)) {
          seg_done_sent++;
          DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ✅ seg_done_sent seg_id=" << seg.seg_id << " j=" << seg.j << " total=" << seg_done_sent);
          state = DONE;
        } else {
          timeout_cnt++;
        }
        break;

      case DONE:
          DEBUG_PRINT("STATE: DONE");
        // 任务完成，返回IDLE
        segments_processed++;
        // 定期报告进度
        if (segments_processed % 100 == 0 || segments_processed <= 10) {
          DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: Completed " << segments_processed << " segments");
        }
        state = IDLE;
        timeout_cnt = 0;
        break;
    }
  }
  DEBUG_PRINT("EXITING SegmentExecutor");
}

//======================================================================
//  SparseLUKernel：顶层连接全部模块
//======================================================================
// PLU2 并行数值架构。列前端先产生 U(:,j)/pivot，收到 U 写 ACK
// 后调度器才释放同列 L 段；每个 L 段在写 ACK 后返回 seg_id。
// 只有列内全部 seg_id 到齐才生成一次 col_done，用于递减后继段入度。

struct ParMetaIndexKernel {
  uint32_t n, nSeg, seg_rows;
  uint32_t off_col, off_seg;
  uint32_t off_col_dep_ptr, off_col_dep_flat;
  uint32_t off_seg_dep_ptr, off_seg_dep_flat;
  uint32_t off_deg, off_post_ptr, off_post_flat, off_end;
  uint32_t off_col_post_ptr, off_col_post_flat;
};

// Keep the million-entry segment-degree table in a physically independent
// task.  In r16 the table lived inside ParallelScheduler; HLS consequently
// merged every scheduler branch and the dynamic nSeg initialization loop into
// one URAM address/write-enable cone.  The placed critical path was 9.38 ns
// from m.nSeg/control state to an URAM RDB_WR/BWE pin.  These compact commands
// make the Scheduler a client of a single, registered memory engine while
// preserving the full 2^20-entry capacity.
using ParDegreeState = ap_uint<18>;

struct ParDegreeRequest {
  uint32_t sid;
  ParDegreeState value;
  ap_uint<2> op;   // 0: read, 1: write, 2: stop
  ap_uint<2> tag;  // 1: release, 2: completion, 3: posting
};

struct ParDegreeResponse {
  ParDegreeState value;
  ap_uint<2> tag;
};

#ifndef __SYNTHESIS__
static ParDegreeState g_debug_degree_state[PAR_MAX_SEG];
#endif

void ParallelDegreeStore(
    tapa::istream<ParDegreeRequest>& request_in,
    tapa::ostream<ParDegreeResponse>& response_out) {
  // UltraRAM has a native 72-bit data word.  Storing one 18-bit degree per
  // address wastes three quarters of every physical word and maps the 2^20
  // entries to 256 URAMs.  Pack four independent degrees per word so the
  // complete capacity needs 64 URAMs; the serialized command protocol makes
  // the read/modify/write operation race-free.
  using ParDegreeWord = ap_uint<72>;
  constexpr uint32_t kDegreeStatesPerWord = 4;
  constexpr uint32_t kDegreeWords =
      (PAR_MAX_SEG + kDegreeStatesPerWord - 1) / kDegreeStatesPerWord;
  static ParDegreeWord deg_cur[kDegreeWords];
#pragma HLS bind_storage variable=deg_cur type=RAM_2P impl=URAM
#ifndef __SYNTHESIS__
  const char* trace_degree_env = std::getenv("SALUT_TRACE_DEGREE");
  const bool trace_degree =
      trace_degree_env != nullptr && trace_degree_env[0] == '1';
#endif

  // A two-state command register prevents FIFO payload/control signals from
  // feeding the deep URAM pins directly.  It also gives reads an explicit
  // register boundary before their value returns to ParallelScheduler.
  ParDegreeRequest pending{0, 0, 0, 0};
  bool pending_valid = false;
#pragma HLS pipeline off
  for (;;) {
    if (!pending_valid) {
      pending = request_in.read();
      pending_valid = true;
    } else if (pending.op == 2) {
      break;
    } else if (pending.op == 1) {
      const uint32_t word_addr = pending.sid >> 2;
      const uint32_t lane = pending.sid & 3u;
      ParDegreeWord word = deg_cur[word_addr];
      if (lane == 0) word.range(17, 0) = pending.value;
      else if (lane == 1) word.range(35, 18) = pending.value;
      else if (lane == 2) word.range(53, 36) = pending.value;
      else word.range(71, 54) = pending.value;
      deg_cur[word_addr] = word;
#ifndef __SYNTHESIS__
      g_debug_degree_state[pending.sid] = pending.value;
      if (trace_degree && pending.sid < 64) {
        std::cerr << "[DEG write] sid=" << pending.sid
                  << " value=" << uint32_t(pending.value) << std::endl;
      }
#endif
      pending_valid = false;
    } else {
      const uint32_t word_addr = pending.sid >> 2;
      const uint32_t lane = pending.sid & 3u;
      const ParDegreeWord word = deg_cur[word_addr];
      ParDegreeState value = 0;
      if (lane == 0) value = word.range(17, 0);
      else if (lane == 1) value = word.range(35, 18);
      else if (lane == 2) value = word.range(53, 36);
      else value = word.range(71, 54);
#ifndef __SYNTHESIS__
      if (trace_degree && pending.sid < 64) {
        std::cerr << "[DEG read] sid=" << pending.sid
                  << " tag=" << uint32_t(pending.tag)
                  << " value=" << uint32_t(value) << std::endl;
      }
#endif
      response_out.write(ParDegreeResponse{value, pending.tag});
      pending_valid = false;
    }
  }
}

static inline WORD64 ParPackElement(uint32_t row, REAL value) {
#pragma HLS inline
  WORD64 elem = 0;
  elem.range(31, 0) = tapa::bit_cast<ap_uint<32>>(value);
  elem.range(63, 32) = row;
  return elem;
}

static inline uint32_t ParElementRow(WORD64 elem) {
#pragma HLS inline
  return elem.range(63, 32);
}

static inline REAL ParElementValue(WORD64 elem) {
#pragma HLS inline
  return tapa::bit_cast<REAL>(ap_uint<32>(elem.range(31, 0)));
}

static inline ParDepEntry ParGetDep(const ParDepPacket& packet,
                                    const uint32_t lane) {
#pragma HLS inline
  switch (lane & (PAR_DEP_PACKET_WIDTH - 1u)) {
    case 0: return packet.d0;
    case 1: return packet.d1;
    case 2: return packet.d2;
    case 3: return packet.d3;
#ifndef __SYNTHESIS__
    case 4: return packet.d4; case 5: return packet.d5;
    case 6: return packet.d6; case 7: return packet.d7;
    case 8: return packet.d8; case 9: return packet.d9;
    case 10: return packet.d10; case 11: return packet.d11;
    case 12: return packet.d12; case 13: return packet.d13;
    case 14: return packet.d14; default: return packet.d15;
#else
    default: return packet.d3;
#endif
  }
}

static inline ParDepEntry ParReadDep(tapa::mmap<META_T> meta,
                                     const uint32_t base,
                                     const uint32_t index,
                                     const uint32_t count) {
#pragma HLS inline
  if (index >= count) return ParDepEntry{0, 0, 0, 0};
  const uint32_t p = base + index * 4u;
  return ParDepEntry{meta[p], meta[p + 1], meta[p + 2], meta[p + 3]};
}

static inline ParDepPacket ParReadDepPacket(
    tapa::mmap<META_T> meta, const uint32_t base,
    const uint32_t index, const uint32_t count) {
#pragma HLS inline
  ParDepPacket packet{};
  packet.d0 = ParReadDep(meta, base, index + 0, count);
  packet.d1 = ParReadDep(meta, base, index + 1, count);
  packet.d2 = ParReadDep(meta, base, index + 2, count);
  packet.d3 = ParReadDep(meta, base, index + 3, count);
#ifndef __SYNTHESIS__
  packet.d4 = ParReadDep(meta, base, index + 4, count);
  packet.d5 = ParReadDep(meta, base, index + 5, count);
  packet.d6 = ParReadDep(meta, base, index + 6, count);
  packet.d7 = ParReadDep(meta, base, index + 7, count);
  packet.d8 = ParReadDep(meta, base, index + 8, count);
  packet.d9 = ParReadDep(meta, base, index + 9, count);
  packet.d10 = ParReadDep(meta, base, index + 10, count);
  packet.d11 = ParReadDep(meta, base, index + 11, count);
  packet.d12 = ParReadDep(meta, base, index + 12, count);
  packet.d13 = ParReadDep(meta, base, index + 13, count);
  packet.d14 = ParReadDep(meta, base, index + 14, count);
  packet.d15 = ParReadDep(meta, base, index + 15, count);
#endif
  return packet;
}

static inline ParMetaIndexKernel ParReadMetaIndex(tapa::mmap<META_T> meta) {
#pragma HLS inline
  ParMetaIndexKernel m;
  m.n = meta[1]; m.nSeg = meta[2]; m.seg_rows = meta[3];
  m.off_col = meta[4]; m.off_seg = meta[5];
  m.off_col_dep_ptr = meta[6]; m.off_col_dep_flat = meta[7];
  m.off_seg_dep_ptr = meta[8]; m.off_seg_dep_flat = meta[9];
  m.off_deg = meta[10]; m.off_post_ptr = meta[11];
  m.off_post_flat = meta[12]; m.off_end = meta[13];
  m.off_col_post_ptr = meta[14]; m.off_col_post_flat = meta[15];
  return m;
}

static inline void ParEmitFront(
    tapa::mmap<META_T> meta, const ParMetaIndexKernel& m, uint32_t j,
    tapa::ostream<ParColumnTask>& task_out,
    tapa::ostream<ParDepPacket>& dep_out) {
#pragma HLS inline off
  const uint32_t c = m.off_col + j * PAR_COL_DESC_WORDS;
  const uint32_t dep_lo = meta[c + 7];
  const uint32_t dep_hi = meta[c + 8];
  ParColumnTask task{j, meta[c + 0], meta[c + 1], meta[c + 2],
                     meta[c + 3], dep_hi - dep_lo};
  task_out.write(task);
  const uint32_t dep_count = dep_hi - dep_lo;
#ifndef __SYNTHESIS__
  if (g_sim_direct_dependencies) return;
#endif
  for (uint32_t d = 0; d < dep_count; d += PAR_DEP_PACKET_WIDTH) {
#pragma HLS pipeline II=1
    const uint32_t base = m.off_col_dep_flat + dep_lo * 4u;
    dep_out.write(ParReadDepPacket(meta, base, d, dep_count));
  }
}

static inline void ParEmitSegment(
    tapa::mmap<META_T> meta, const ParMetaIndexKernel& m, uint32_t sid,
    const bool early,
    tapa::ostreams<ParSegmentTask, PAR_NUM_PEG>& ready_local_task_out,
    tapa::ostreams<ParDepPacket, PAR_NUM_PEG>& ready_local_dep_out,
    tapa::ostream<ParSegmentTask>& ready_global_task_out,
    tapa::ostream<ParDepPacket>& ready_global_dep_out) {
#pragma HLS inline off
  const uint32_t s = m.off_seg + sid * PAR_SEG_DESC_WORDS;
  const uint32_t dep_lo = meta[s + 10], dep_hi = meta[s + 11];
  const ParSegmentTask task{sid, meta[s + 0], meta[s + 2], meta[s + 3],
                            meta[s + 4], meta[s + 5], meta[s + 6], meta[s + 7],
                            meta[s + 8], meta[s + 9], dep_hi - dep_lo,
                            early ? 1u : 0u};
  const uint32_t affinity_peg = addr_bank(task.a_addr) & (PAR_NUM_PEG - 1);
  const bool use_local = !ready_local_task_out[affinity_peg].full();
  if (use_local) ready_local_task_out[affinity_peg].write(task);
  else ready_global_task_out.write(task);
  const uint32_t dep_count = dep_hi - dep_lo;
#ifndef __SYNTHESIS__
  if (g_sim_direct_dependencies) return;
#endif
  for (uint32_t d = 0; d < dep_count; d += PAR_DEP_PACKET_WIDTH) {
#pragma HLS pipeline II=1
    const uint32_t base = m.off_seg_dep_flat + dep_lo * 4u;
    const ParDepPacket packet = ParReadDepPacket(meta, base, d, dep_count);
    if (use_local) ready_local_dep_out[affinity_peg].write(packet);
    else ready_global_dep_out.write(packet);
  }
}

// The paper's GCU sends a compact Ready-seg stream and resolves the bulky
// descriptor/dependency metadata in an independent DepEntry pipeline.  Keep
// that separation explicit here.  In r13, calling ParEmitFront and
// ParEmitSegment from the degree scheduler made Vitis HLS merge both variable
// latency AXI readers and all nine output queues into one non-pipelined FSM
// (estimated control fanout 8100).  These two emitters own independent read-
// only metadata ports and decouple that work with bounded ID FIFOs.
void ParallelFrontEmitter(
    tapa::mmap<META_T> meta,
    tapa::istream<uint32_t>& ready_front_id_in,
    tapa::ostream<ParColumnTask>& task_out,
    tapa::ostream<ParDepPacket>& dep_out) {
#ifdef SALUT_HLS_PARALLEL_TASK_ONLY
#pragma HLS interface m_axi port=meta offset=direct bundle=meta
#endif
  const ParMetaIndexKernel m = ParReadMetaIndex(meta);
  for (;;) {
    const uint32_t j = ready_front_id_in.read();
    if (j == 0xFFFFFFFFu) break;
    ParEmitFront(meta, m, j, task_out, dep_out);
  }
  task_out.write(ParColumnTask{0xFFFFFFFFu, 0, 0, 0, 0, 0});
}

void ParallelSegmentEmitter(
    tapa::mmap<META_T> meta,
    tapa::istream<uint32_t>& ready_segment_id_in,
    tapa::ostreams<ParSegmentTask, PAR_NUM_PEG>& ready_local_task_out,
    tapa::ostreams<ParDepPacket, PAR_NUM_PEG>& ready_local_dep_out,
    tapa::ostream<ParSegmentTask>& ready_global_task_out,
    tapa::ostream<ParDepPacket>& ready_global_dep_out) {
#ifdef SALUT_HLS_PARALLEL_TASK_ONLY
#pragma HLS interface m_axi port=meta offset=direct bundle=meta
#endif
  const ParMetaIndexKernel m = ParReadMetaIndex(meta);
  for (;;) {
    const uint32_t sid = ready_segment_id_in.read();
    if (sid == 0xFFFFFFFFu) break;
    ParEmitSegment(meta, m, sid, false,
                   ready_local_task_out, ready_local_dep_out,
                   ready_global_task_out, ready_global_dep_out);
  }
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    ready_local_task_out[peg].write(
        ParSegmentTask{0xFFFFFFFFu, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  }
}

void ParallelScheduler(
    tapa::mmap<META_T> meta,
    tapa::ostream<uint32_t>& ready_front_id_out,
    tapa::istream<uint32_t>& front_done_in,
    tapa::ostream<uint32_t>& pivot_ready_out,
    tapa::ostream<uint32_t>& ready_segment_id_out,
    tapa::istream<uint32_t>& seg_done_in,
    tapa::ostream<ParDegreeRequest>& degree_request_out,
    tapa::istream<ParDegreeResponse>& degree_response_in,
    tapa::ostream<ap_uint<1>>& compute_done_out) {
#ifdef SALUT_HLS_PARALLEL_TASK_ONLY
#pragma HLS interface m_axi port=meta offset=direct bundle=meta
#endif
#ifndef __SYNTHESIS__
  const char* trace_degree_env = std::getenv("SALUT_TRACE_DEGREE");
  const bool trace_degree =
      trace_degree_env != nullptr && trace_degree_env[0] == '1';
#endif
  if (meta[0] != PAR_META_MAGIC) {
#ifndef __SYNTHESIS__
    std::cerr << "[SALUT error] invalid PLU2 metadata magic" << std::endl;
#endif
    ready_front_id_out.write(0xFFFFFFFFu);
    ready_segment_id_out.write(0xFFFFFFFFu);
    degree_request_out.write(ParDegreeRequest{0, 0, 2, 0});
    return;
  }
  const ParMetaIndexKernel m = ParReadMetaIndex(meta);
  if (m.n > PAR_RUNTIME_MAX_N || m.nSeg > PAR_MAX_SEG) {
#ifndef __SYNTHESIS__
    std::cerr << "[SALUT error] graph exceeds simulation capacity: n="
              << m.n << "/" << PAR_RUNTIME_MAX_N << " segments="
              << m.nSeg << "/" << PAR_MAX_SEG << std::endl;
#endif
    ready_front_id_out.write(0xFFFFFFFFu);
    ready_segment_id_out.write(0xFFFFFFFFu);
    degree_request_out.write(ParDegreeRequest{0, 0, 2, 0});
    return;
  }

  // Keep the segment dependency counter narrow and in dedicated on-chip RAM,
  // matching the
  // paper's Degscheduler.  Ready segments are activated directly by either
  // half of the segment-column trigger: a posting-list decrement that reaches
  // zero when the target front is already committed, or the target-front
  // completion scan when the degree reached zero earlier.  This remains
  // O(nSeg + postings).  Keep the full 2^20-entry hardware capacity, but put
  // this narrow, deep table in URAM: on U280 SLR1 has the URAM headroom while
  // its BRAM columns are already the placement bottleneck.  A BRAM binding
  // turns this table into roughly 512 BRAM36s and splits the scheduler across
  // SLR1/SLR2, creating the long cross-SLR routes seen in the r7 reports.
  // Column counters never exceed the number of columns.  Keep the hardware
  // representation narrow and store ready IDs in explicit append-only BRAM
  // queues.  The previous implementation embedded queue links and state tags
  // in the two 32-bit counter tables.  Besides wasting BRAM width, that made
  // one counter read drive a second counter write-enable through the shared
  // initialization/main-loop mux.  On U280 Vivado measured a 7.55 ns path
  // through seven cascaded BRAMs.  Each column enters each queue exactly once,
  // so monotonic head/tail indices need neither wraparound nor link fields.
#ifdef __SYNTHESIS__
  using ParColumnCount = ap_uint<17>;
  using ParColumnId = ap_uint<16>;
#else
  using ParColumnCount = uint32_t;
  using ParColumnId = uint32_t;
#endif
  static ParColumnCount col_remaining[PAR_RUNTIME_MAX_N];
  static ParColumnCount col_deg_cur[PAR_RUNTIME_MAX_N];
  static ap_uint<1> front_completed[PAR_RUNTIME_MAX_N];
  static ap_uint<1> col_completed[PAR_RUNTIME_MAX_N];
  static ParColumnId front_ready_fifo[PAR_RUNTIME_MAX_N];
  static ParColumnId completed_ready_fifo[PAR_RUNTIME_MAX_N];
#pragma HLS bind_storage variable=col_remaining type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=col_deg_cur type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=front_completed type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=col_completed type=RAM_2P impl=BRAM
// These append-only queues are 64K x 16 in hardware.  BRAM mapping fans each
// write enable across thirty RAMB36 primitives and r21 spent 5.74 ns in that
// route alone.  SLR1 has ample UltraRAM after the packed DegreeStore change;
// one queue uses sixteen URAMs and gives the control path a compact macro set.
#pragma HLS bind_storage variable=front_ready_fifo type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=completed_ready_fifo type=RAM_2P impl=URAM

  constexpr uint32_t kSegStarted = 0x3FFFEu;
  constexpr uint32_t kCompleted = 0x3FFFFu;
  uint32_t front_ready_head = 0;
  uint32_t front_ready_tail = 0;
  uint32_t front_ready_count = 0;
  // A front may make many segments ready at once.  Never emit that complete
  // range from inside the front-done handler: if the task FIFOs fill while PE
  // completion FIFOs are also full, the scheduler would stop draining
  // seg_done and create a cyclic backpressure deadlock.  Retain only the
  // contiguous range cursor and advance it cooperatively from the main loop.
  uint32_t release_sid = 0;
  uint32_t release_end = 0;
  bool release_active = false;
  bool release_degree_requested = false;
  bool release_degree_pending = false;
  uint32_t release_pending_sid = 0;
  ParDegreeState release_pending_degree = 0;
  bool release_emit_pending = false;
  uint32_t release_emit_sid = 0;
  // A front with no lower segments completes its column immediately.  Read
  // col_remaining/col_completed into explicit registers before deciding that
  // case; otherwise the deep BRAM output becomes part of the monolithic HLS
  // FSM enable cone (the r22 WNS path).
  ap_uint<2> front_done_col_stage = 0;
  uint32_t front_done_col_pending_j = 0;
  ParColumnCount front_done_col_pending_remaining = 0;
  bool front_done_col_pending_completed = false;

  // Completion validation also crosses the deep degree table.  Capture the
  // URAM read first and retire it on the next phase-3 service slot.
  bool seg_done_degree_requested = false;
  bool seg_done_degree_pending = false;
  uint32_t seg_done_pending_sid = 0;
  ParDegreeState seg_done_pending_degree = 0;
  // Retiring a segment also decrements its column's remaining-segment count.
  // Keep the BRAM read and write in different phase-3 service slots.  Without
  // this boundary the 64K-deep BRAM cascade fed its own EN pin through the
  // decrement/completion/queue-control cone; r19 placed that path at 7.12 ns.
  ap_uint<2> seg_done_col_stage = 0;
  uint32_t seg_done_col_pending_j = 0;
  ParColumnCount seg_done_col_pending_remaining = 0;
  bool seg_done_col_pending_completed = false;

  // Likewise, propagate one completed column's segment/column posting lists
  // incrementally.  Completed columns wait in their dedicated append-only
  // queue, keeping queue traffic out of the col_remaining BRAM write mux.
  uint32_t completed_ready_head = 0;
  uint32_t completed_ready_tail = 0;
  uint32_t completed_ready_count = 0;
  uint32_t posting_j = 0;
  uint32_t seg_post_cursor = 0;
  uint32_t seg_post_end = 0;
  // A segment posting used to read deg_cur (URAM), resolve the target-front
  // state, test output backpressure and update seg_post_cursor in one
  // scheduler iteration.  The integrated r12 timing report measured that
  // path at 7.96 ns.  Keep one posting in explicit registered stages so the
  // URAM result can never feed the cursor enable combinationally.  Advancing
  // the cursor when the posting is fetched is safe because stages 1..4 retain
  // the sid until its degree update (and any required task emission) commits.
  ap_uint<3> seg_post_stage = 0;
  uint32_t seg_post_pending_sid = 0;
  uint32_t seg_post_pending_target_j = 0;
  bool seg_post_pending_front_completed = false;
  ParDegreeState seg_post_pending_degree = 0;
  uint32_t col_post_cursor = 0;
  uint32_t col_post_end = 0;
  ap_uint<2> col_post_stage = 0;
  uint32_t col_post_pending_j = 0;
  ParColumnCount col_post_pending_degree = 0;
  bool posting_active = false;
  bool degree_transaction_active = false;
  bool degree_read_outstanding = false;

  for (uint32_t j = 0; j < m.n; ++j) {
#pragma HLS pipeline II=1
    const uint32_t c = m.off_col + j * PAR_COL_DESC_WORDS;
    const ParColumnCount initial_remaining = meta[c + 6] - meta[c + 5];
    const ParColumnCount initial_degree = meta[c + 8] - meta[c + 7];
    col_remaining[j] = initial_remaining;
    col_deg_cur[j] = initial_degree;
    front_completed[j] = 0;
    col_completed[j] = 0;
    if (initial_degree == 0) {
      front_ready_fifo[front_ready_tail++] = ParColumnId(j);
      ++front_ready_count;
    }
  }
  for (uint32_t sid = 0; sid < m.nSeg; ++sid) {
#pragma HLS pipeline II=1
    degree_request_out.write(
        ParDegreeRequest{sid, ParDegreeState(meta[m.off_deg + sid]), 1, 0});
  }

  uint32_t completed_columns = 0;
  // The scheduler owns several independent RAM-backed state machines.  When
  // all of them were evaluated in one non-pipelined loop iteration, HLS
  // built a single large enable/mux cone: the routed r15 critical path ran
  // from deg_cur URAM data through eleven LUT/MUX levels to an unrelated
  // completed-ready counter enable.  Execute one event class per cycle so a
  // RAM result can only control registers belonging to that event.  This is
  // still a cooperative scheduler (every class is serviced once per five
  // cycles), preserves the nonblocking drain behavior, and avoids the stale
  // exit state previously observed with automatic loop pipelining.
  ap_uint<3> scheduler_phase = 0;
#ifndef __SYNTHESIS__
  uint64_t no_completion_iters = 0;
  uint64_t dispatched_fronts = 0;
  uint64_t completed_fronts = 0;
  uint64_t dispatched_segments = 0;
  uint64_t completed_segments = 0;
#endif
#pragma HLS pipeline off
  while (completed_columns < m.n) {
    // This event loop must preserve sequential iteration/exit semantics.
    // Vitis HLS 2022.2 auto-pipelined it at II=50, but the generated RTL used
    // a stale loop-exit comparison: completed_columns reached m.n while the
    // pipeline continued forever and never emitted the termination tokens.
    // Disabling automatic pipelining here fixes that control hazard without
    // changing the independently concurrent 8-front/32-PE datapath or the
    // explicitly pipelined initialization loops above.
#ifndef __SYNTHESIS__
    ++no_completion_iters;
    if (no_completion_iters == (1ull << 20)) {
      std::cerr << "[SALUT watchdog] columns=" << completed_columns << "/"
                << m.n << " fronts=" << completed_fronts << "/"
                << dispatched_fronts << " segments=" << completed_segments
                << "/" << dispatched_segments
                << " hbm_req=" << g_hbm_req_issued
                << " bank_rsp=" << g_hbm_bank_rsp
                << " routed_rsp=" << g_hbm_rsp_routed
                << " lane_rsp=" << g_hbm_lane_rsp
                << " sm_fill=" << g_sm_fill_rsp
                << " lm_rsp=" << g_lm_rsp << std::endl;
      std::cerr << "  sched phase=" << uint32_t(scheduler_phase)
                << " front_ready=" << front_ready_count
                << " completed_ready=" << completed_ready_count
                << " release=" << release_active << ":" << release_sid
                << "/" << release_end
                << " rel_req/pending=" << release_degree_requested << "/"
                << release_degree_pending << " emit=" << release_emit_pending
                << " segdone_req/pending=" << seg_done_degree_requested << "/"
                << seg_done_degree_pending
                << " posting=" << posting_active << ":"
                << uint32_t(seg_post_stage) << " seg_cursor="
                << seg_post_cursor << "/" << seg_post_end
                << " col_cursor=" << col_post_cursor << "/" << col_post_end
                << " degree_tx/read=" << degree_transaction_active << "/"
                << degree_read_outstanding << std::endl;
      uint32_t debug_columns = 0;
      for (uint32_t j = 0; j < m.n && debug_columns < 12; ++j) {
        if (col_completed[j]) continue;
        const uint32_t c = m.off_col + j * PAR_COL_DESC_WORDS;
        const uint32_t lo = meta[c + 5];
        const uint32_t hi = meta[c + 6];
        std::cerr << "    blocked j=" << j
                  << " front=" << uint32_t(front_completed[j])
                  << " col_deg=" << col_deg_cur[j]
                  << " remaining=" << col_remaining[j]
                  << " segs=" << lo << ".." << hi;
        for (uint32_t sid = lo; sid < hi && sid < lo + 6; ++sid) {
          std::cerr << " " << sid << ":"
                    << uint32_t(g_debug_degree_state[sid]);
        }
        std::cerr << std::endl;
        ++debug_columns;
      }
      no_completion_iters = 0;
    }
#endif
    // At most one degree read is outstanding.  Writes may queue behind it,
    // but FIFO ordering guarantees that every read observes all preceding
    // initialization/state updates.
    ParDegreeResponse degree_response;
    if (degree_response_in.try_read(degree_response)) {
      degree_read_outstanding = false;
      if (degree_response.tag == 1) {
        release_pending_degree = degree_response.value;
        release_degree_requested = false;
        release_degree_pending = true;
      } else if (degree_response.tag == 2) {
        seg_done_pending_degree = degree_response.value;
        seg_done_degree_requested = false;
        seg_done_degree_pending = true;
      } else if (degree_response.tag == 3) {
        seg_post_pending_degree = degree_response.value;
        seg_post_stage = 5;
      }
    }
    // Degree-zero fronts are linked once and consumed directly.  Keeping the
    // head in place on backpressure preserves the bounded, non-blocking
    // startup behavior without rescanning all m.n columns.
    if (scheduler_phase == 0) {
      if (front_ready_count != 0 && !ready_front_id_out.full()) {
        const uint32_t front_candidate =
            uint32_t(front_ready_fifo[front_ready_head++]);
        --front_ready_count;
        ready_front_id_out.write(front_candidate);
#ifndef __SYNTHESIS__
        ++dispatched_fronts;
#endif
      }
    } else if (scheduler_phase == 1) {
      if (front_done_col_stage == 1) {
        const uint32_t j = front_done_col_pending_j;
        front_done_col_pending_remaining = col_remaining[j];
        front_done_col_pending_completed = bool(col_completed[j]);
        front_done_col_stage = 2;
      } else if (front_done_col_stage == 2) {
        const uint32_t j = front_done_col_pending_j;
        if (front_done_col_pending_remaining == 0 &&
            !front_done_col_pending_completed) {
          col_completed[j] = 1;
          ++completed_columns;
          completed_ready_fifo[completed_ready_tail++] = ParColumnId(j);
          ++completed_ready_count;
#ifndef __SYNTHESIS__
          no_completion_iters = 0;
#endif
        }
        front_done_col_stage = 0;
      } else {
        uint32_t j_front = 0;
        if (!release_active && front_done_in.try_read(j_front) &&
            j_front < m.n && !front_completed[j_front]) {
        front_completed[j_front] = 1;
#ifndef __SYNTHESIS__
        ++completed_fronts;
        no_completion_iters = 0;
#endif
        // Publish pivot readiness only after the front write received the full
        // LM/SM/HBM commit acknowledgement.
        pivot_ready_out.write(j_front);
        // Complete the second half of the segment-column trigger.  Segment
        // descriptors of a column are contiguous, so this is a bounded linear
        // release with no global nSeg scan or million-entry ready queue.
        const uint32_t front_desc =
            m.off_col + j_front * PAR_COL_DESC_WORDS;
        const uint32_t seg_lo = meta[front_desc + 5];
        const uint32_t seg_hi = meta[front_desc + 6];
        release_sid = seg_lo;
        release_end = seg_hi;
        release_active = seg_lo < seg_hi;
        // Columns with no lower segment still produce a committed U front and
        // must participate in both posting graphs.  Resolve that condition in
        // the two registered stages above.
        front_done_col_pending_j = j_front;
        front_done_col_stage = 1;
        }
      }
    } else if (scheduler_phase == 2) {
      // Release at most one ready segment per scheduler service slot.  A full
      // task output pauses only the final emission.  Commit kSegStarted and
      // release the degree transaction lock first, otherwise a full task FIFO
      // can prevent phase 3 from draining completions and form a cycle.
      if (release_emit_pending) {
        if (!ready_segment_id_out.full()) {
          ready_segment_id_out.write(release_emit_sid);
          release_emit_pending = false;
#ifndef __SYNTHESIS__
          ++dispatched_segments;
#endif
        }
      } else if (release_active) {
        if (release_sid >= release_end) {
          release_active = false;
          release_degree_requested = false;
          release_degree_pending = false;
        } else if (release_degree_pending) {
          if (release_pending_degree != 0) {
            release_sid = release_pending_sid + 1;
            release_degree_pending = false;
            degree_transaction_active = false;
          } else if (!degree_request_out.full()) {
            const uint32_t sid = release_pending_sid;
            release_sid = sid + 1;
            release_degree_pending = false;
            degree_request_out.write(
                ParDegreeRequest{sid, ParDegreeState(kSegStarted), 1, 0});
            degree_transaction_active = false;
            release_emit_sid = sid;
            release_emit_pending = true;
          }
        } else if (!release_degree_requested && !degree_transaction_active &&
                   !degree_request_out.full()) {
          release_pending_sid = release_sid;
          if (release_sid < m.nSeg) {
            degree_request_out.write(
                ParDegreeRequest{release_sid, 0, 0, 1});
            release_degree_requested = true;
            degree_transaction_active = true;
            degree_read_outstanding = true;
          } else {
            release_pending_degree = ParDegreeState(kCompleted);
            release_degree_pending = true;
          }
        }
      }
    } else if (scheduler_phase == 3) {
      if (seg_done_col_stage == 1) {
        const uint32_t j = seg_done_col_pending_j;
        seg_done_col_pending_completed = bool(col_completed[j]);
        seg_done_col_pending_remaining = col_remaining[j];
        seg_done_col_stage = 2;
      } else if (seg_done_col_stage == 2) {
        const uint32_t j = seg_done_col_pending_j;
        const ParColumnCount remaining_before =
            seg_done_col_pending_remaining;
        if (!seg_done_col_pending_completed && remaining_before != 0) {
          const ParColumnCount remaining = remaining_before - 1;
          col_remaining[j] = remaining;
          if (remaining == 0) {
            col_completed[j] = 1;
            ++completed_columns;
            completed_ready_fifo[completed_ready_tail++] = ParColumnId(j);
            ++completed_ready_count;
#ifndef __SYNTHESIS__
            const uint32_t progress_mask = 1023u;
            const bool dense_tail = m.n - completed_columns < 1024u;
            if ((completed_columns & progress_mask) == 0 ||
                (dense_tail && (completed_columns & 63u) == 0) ||
                completed_columns == m.n) {
              std::cerr << "[SALUT progress] completed_columns="
                        << completed_columns << "/" << m.n
                        << " front_ready=" << front_ready_count
                        << " seg_ready=direct"
                        << " fronts_inflight="
                        << (dispatched_fronts - completed_fronts)
                        << " segments_inflight="
                        << (dispatched_segments - completed_segments)
                        << " front_sm_hit=" << g_front_sm_hits
                        << " front_sm_miss=" << g_front_sm_misses
                        << std::endl;
            }
#endif
          }
        }
        seg_done_col_stage = 0;
      } else if (!seg_done_degree_requested && !seg_done_degree_pending &&
          !degree_transaction_active && !degree_request_out.full()) {
        uint32_t completed_sid = 0;
        if (seg_done_in.try_read(completed_sid) && completed_sid < m.nSeg) {
          seg_done_pending_sid = completed_sid;
          degree_request_out.write(
              ParDegreeRequest{completed_sid, 0, 0, 2});
          seg_done_degree_requested = true;
          degree_transaction_active = true;
          degree_read_outstanding = true;
        }
      } else if (seg_done_degree_pending) {
        const uint32_t completed_sid = seg_done_pending_sid;
        const bool accept_completion =
            seg_done_pending_degree == kSegStarted;
        if (!accept_completion) {
          seg_done_degree_pending = false;
          degree_transaction_active = false;
        } else if (!degree_request_out.full()) {
        seg_done_degree_pending = false;
        degree_request_out.write(
            ParDegreeRequest{completed_sid, ParDegreeState(kCompleted), 1, 0});
        degree_transaction_active = false;
#ifndef __SYNTHESIS__
        ++completed_segments;
        no_completion_iters = 0;
#endif
        const uint32_t sd = m.off_seg + completed_sid * PAR_SEG_DESC_WORDS;
        const uint32_t j = meta[sd + 0];
        if (j < m.n) {
          seg_done_col_pending_j = j;
          seg_done_col_stage = 1;
        }
        }
      }
    } else {
      if (!posting_active && completed_ready_count != 0) {
        posting_j = uint32_t(completed_ready_fifo[completed_ready_head++]);
        --completed_ready_count;
        seg_post_cursor = meta[m.off_post_ptr + posting_j];
        seg_post_end = meta[m.off_post_ptr + posting_j + 1];
        seg_post_stage = 0;
        col_post_cursor = meta[m.off_col_post_ptr + posting_j];
        col_post_end = meta[m.off_col_post_ptr + posting_j + 1];
        col_post_stage = 0;
        posting_active = true;
#ifndef __SYNTHESIS__
        if (trace_degree) {
          std::cerr << "[POST begin] j=" << posting_j
                    << " seg=" << seg_post_cursor << ".." << seg_post_end
                    << " col=" << col_post_cursor << ".." << col_post_end
                    << std::endl;
        }
#endif
      }

      if (posting_active && seg_post_stage == 1) {
        // Hold this posting until the single ordered degree transaction port
        // is available.  Folding availability into the outer condition made
        // a busy cycle fall through to the cursor-fetch branch below, which
        // overwrote seg_post_pending_sid and silently lost decrements.
        if (!degree_transaction_active && !degree_request_out.full()) {
          degree_request_out.write(
              ParDegreeRequest{seg_post_pending_sid, 0, 0, 3});
          degree_transaction_active = true;
          degree_read_outstanding = true;
          seg_post_stage = 6;
        }
      } else if (seg_post_stage == 5) {
        const ParDegreeState degree_state = seg_post_pending_degree;
        if (degree_state == 0 || degree_state >= kSegStarted) {
          seg_post_stage = 0;
          degree_transaction_active = false;
        } else {
          const ParDegreeState next_degree = degree_state - 1;
          if (next_degree != 0) {
            if (!degree_request_out.full()) {
              degree_request_out.write(ParDegreeRequest{
                  seg_post_pending_sid, next_degree, 1, 0});
              seg_post_stage = 0;
              degree_transaction_active = false;
            }
          } else {
            // Fetching the descriptor in a separate stage also keeps the
            // external metadata read off the URAM-to-control path.
            seg_post_stage = 2;
          }
        }
      } else if (seg_post_stage == 2) {
        const uint32_t successor_desc =
            m.off_seg + seg_post_pending_sid * PAR_SEG_DESC_WORDS;
        seg_post_pending_target_j = meta[successor_desc + 0];
        seg_post_stage = 3;
      } else if (seg_post_stage == 3) {
        const uint32_t target_j = seg_post_pending_target_j;
        // Register the BRAM result before it controls the deg_cur URAM write.
        // Without this stage Vivado routed front_completed/DOUT through the
        // write-decode cone directly to every URAM BWE/EN pin.
        seg_post_pending_front_completed =
            target_j < m.n && bool(front_completed[target_j]);
        seg_post_stage = 4;
      } else if (seg_post_stage == 4) {
        const uint32_t sid = seg_post_pending_sid;
        if (seg_post_pending_front_completed) {
          if (!degree_request_out.full()) {
            degree_request_out.write(
                ParDegreeRequest{sid, ParDegreeState(kSegStarted), 1, 0});
            // The state update is ordered before the task emission, but task
            // FIFO backpressure must not retain the global degree lock.
            seg_post_stage = 7;
            degree_transaction_active = false;
          }
        } else {
          // The target-front completion scan will activate it later.
          if (!degree_request_out.full()) {
            degree_request_out.write(
                ParDegreeRequest{sid, ParDegreeState(0), 1, 0});
            seg_post_stage = 0;
            degree_transaction_active = false;
          }
        }
      } else if (seg_post_stage == 6) {
        // Waiting for the tagged DegreeStore response.
      } else if (seg_post_stage == 7) {
        if (!ready_segment_id_out.full()) {
          ready_segment_id_out.write(seg_post_pending_sid);
          seg_post_stage = 0;
#ifndef __SYNTHESIS__
          ++dispatched_segments;
#endif
        }
      } else if (seg_post_cursor < seg_post_end) {
        const uint32_t sid = meta[m.off_post_flat + seg_post_cursor];
        ++seg_post_cursor;
        if (sid < m.nSeg) {
#ifndef __SYNTHESIS__
          if (trace_degree && sid < 64) {
            std::cerr << "[POST sid] j=" << posting_j
                      << " sid=" << sid << std::endl;
          }
#endif
          seg_post_pending_sid = sid;
          seg_post_stage = 1;
        }
      } else if (col_post_stage == 1) {
        // Capture the BRAM result in its own scheduler iteration.  This
        // register boundary prevents a deep counter output from directly
        // driving its own write enable and the ready-queue write enable.
        col_post_pending_degree = col_deg_cur[col_post_pending_j];
        col_post_stage = 2;
      } else if (col_post_stage == 2) {
        const uint32_t successor_j = col_post_pending_j;
        const ParColumnCount degree = col_post_pending_degree;
        if (degree != 0) {
          const ParColumnCount next_degree = degree - 1;
          col_deg_cur[successor_j] = next_degree;
          if (next_degree == 0) {
            front_ready_fifo[front_ready_tail++] = ParColumnId(successor_j);
            ++front_ready_count;
          }
        }
        col_post_stage = 0;
      } else if (col_post_cursor < col_post_end) {
        // The column posting list preserves every U dependency, including
        // those without an L intersection in a lower segment.  Fetch, BRAM
        // read, and update are three registered stages.
        const uint32_t successor_j =
            meta[m.off_col_post_flat + col_post_cursor++];
        if (successor_j < m.n) {
          col_post_pending_j = successor_j;
          col_post_stage = 1;
        }
      } else if (posting_active) {
        posting_active = false;
      }
    }
    if (scheduler_phase == 4) scheduler_phase = 0;
    else ++scheduler_phase;
  }

  // A final read may already be in flight when the last column completion is
  // retired.  Drain it before the ordered stop command so DegreeStore can
  // never block on an unconsumed response during kernel shutdown.
  if (degree_read_outstanding) {
    (void)degree_response_in.read();
  }
  degree_request_out.write(ParDegreeRequest{0, 0, 2, 0});
  ready_front_id_out.write(0xFFFFFFFFu);
  ready_segment_id_out.write(0xFFFFFFFFu);
  // Compute retirement is NOT physical memory completion. The drain join
  // owns top-level completion after every bank and response path closes.
  compute_done_out.write(ap_uint<1>(1));
}

// SALUT Task Allocation Unit (TAU).  Each PEG prioritizes its local FIFO and
// statically maps those tasks to its physical PEs.  Only when that PEG has no
// local work does the allocator consume the global FIFO, using a global RR
// pointer.  A task and all of its DepEntry packets are forwarded atomically.
void ParallelTaskAllocationUnit(
    tapa::istreams<ParSegmentTask, PAR_NUM_PEG>& ready_local_task_in,
    tapa::istreams<ParDepPacket, PAR_NUM_PEG>& ready_local_dep_in,
    tapa::istream<ParSegmentTask>& ready_global_task_in,
    tapa::istream<ParDepPacket>& ready_global_dep_in,
    tapa::ostreams<ParSegmentTask, PAR_NUM_PE>& task_out,
    tapa::ostreams<ParDepPacket, PAR_NUM_PE>& dep_out) {
  ap_uint<3> rr_peg = 0;
  ap_uint<3> rr_local[PAR_NUM_PEG];
  ap_uint<6> rr_global = 0;
#pragma HLS array_partition variable=rr_local complete
  // Vitis HLS 2022.2 does not reliably lower aggregate `{0}` initialization
  // for partitioned local arrays: only element zero may receive reset logic.
  // Explicit assignments are required to keep every PEG selector out of X.
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    rr_local[peg] = 0;
  }
  uint32_t local_terminators = 0;

  for (;;) {
    const uint32_t peg = rr_peg;
    rr_peg = (peg + 1 == PAR_NUM_PEG) ? 0 : peg + 1;

    ParSegmentTask task{};
    bool have_task = false;
    bool from_local = false;
    if (ready_local_task_in[peg].try_read(task)) {
      have_task = true;
      from_local = true;
    } else if (ready_global_task_in.try_read(task)) {
      have_task = true;
    }
    if (!have_task) continue;

    if (task.seg_id == 0xFFFFFFFFu) {
      // The scheduler emits one terminator per PEG only after every segment
      // has completed, so both local and global queues are already drained.
      ++local_terminators;
      if (local_terminators == PAR_NUM_PEG) {
        for (int pe = 0; pe < PAR_NUM_PE; ++pe) {
#pragma HLS unroll
          task_out[pe].write(task);
        }
        return;
      }
      continue;
    }

    uint32_t pe = 0;
    if (task.early) {
      // One dedicated context per PEG is allowed to wait for a future pivot.
      // Ready-pivot work never enters this PE's queue, eliminating HOL cycles.
      const uint32_t early_peg = addr_bank(task.a_addr) & (PAR_NUM_PEG - 1);
      pe = early_peg * PAR_PE_PER_PEG + (PAR_PE_PER_PEG - 1);
    } else if (from_local) {
      pe = peg * PAR_PE_PER_PEG + rr_local[peg];
      if (rr_local[peg] == PAR_PE_PER_PEG - 2) rr_local[peg] = 0;
      else rr_local[peg] = rr_local[peg] + 1;
    } else {
      pe = rr_global;
      if (rr_global == PAR_NUM_PE - 2) {
        rr_global = 0;
      } else {
        ap_uint<6> next = rr_global + 1;
        if ((next & (PAR_PE_PER_PEG - 1)) == PAR_PE_PER_PEG - 1) ++next;
        rr_global = next;
      }
    }

    task_out[pe].write(task);
#ifndef __SYNTHESIS__
    if (g_sim_direct_dependencies) continue;
#endif
    for (uint32_t d = 0; d < task.dep_count; d += PAR_DEP_PACKET_WIDTH) {
#pragma HLS pipeline II=1
      dep_out[pe].write(from_local ? ready_local_dep_in[peg].read()
                                   : ready_global_dep_in.read());
    }
  }
}

// Column-front and segment MAC pipelines run independently.  A PE posts its
// pivot request after completing all segment-local MACs; this gate releases
// the physical U(j,j) read only after the corresponding front write is fully
// committed.  Thus early segment activation cannot observe stale HBM data.
void ParallelPivotReadGate(
    tapa::istream<uint32_t>& ready_col_in,
    tapa::istreams<ParPivotReq, PAR_NUM_PE>& pivot_req_in,
    tapa::ostreams<ap_uint<1>, PAR_NUM_PE>& pivot_go_out) {
  static ap_uint<1> ready_col[PAR_RUNTIME_MAX_N];
#pragma HLS bind_storage variable=ready_col type=RAM_2P impl=BRAM
#pragma HLS array_partition variable=ready_col cyclic factor=16
  static_assert(PAR_RUNTIME_MAX_N % 16 == 0,
                "pivot ready capacity must contain complete clear banks");
  ParPivotReq pending[PAR_NUM_PEG];
  ap_uint<1> pending_valid[PAR_NUM_PEG];
#pragma HLS array_partition variable=pending complete
#pragma HLS array_partition variable=pending_valid complete

  // Clear all capacity, not just this launch's columns.  Independent narrow
  // banks bound the hardware startup to 4096 clear iterations (65536 / 16).
  for (uint32_t base = 0; base < PAR_RUNTIME_MAX_N; base += 16) {
#pragma HLS pipeline II=1
    for (int lane = 0; lane < 16; ++lane) {
#pragma HLS unroll
      ready_col[base + lane] = 0;
    }
  }
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    pending_valid[peg] = 0;
  }

  ap_uint<3> accept_rr = 0;
  ap_uint<3> issue_rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    uint32_t ready_j;
    if (ready_col_in.try_read(ready_j) && ready_j < PAR_RUNTIME_MAX_N)
      ready_col[ready_j] = 1;

    const uint32_t accept_peg = accept_rr;
    const uint32_t accept_pe =
        accept_peg * PAR_PE_PER_PEG + (PAR_PE_PER_PEG - 1);
    if (!pending_valid[accept_peg] &&
        pivot_req_in[accept_pe].try_read(pending[accept_peg]))
      pending_valid[accept_peg] = 1;
    accept_rr = accept_peg + 1 == PAR_NUM_PEG ? 0 : accept_peg + 1;

    const uint32_t issue_peg = issue_rr;
    const uint32_t issue_pe =
        issue_peg * PAR_PE_PER_PEG + (PAR_PE_PER_PEG - 1);
    if (pending_valid[issue_peg] && pending[issue_peg].j < PAR_RUNTIME_MAX_N &&
        ready_col[pending[issue_peg].j] && !pivot_go_out[issue_pe].full()) {
      pivot_go_out[issue_pe].write(1);
      pending_valid[issue_peg] = 0;
    }
    issue_rr = issue_peg + 1 == PAR_NUM_PEG ? 0 : issue_peg + 1;
  }
}

// SALUT private Local Memory (LM).  Each PEG has an independent tagged factor
// cache and arbitrates its physical PEs.  A and factor-cache misses continue to
// the shared-memory level.  Whole factor bursts are checked before any cached
// word is emitted, so a partial hit can safely fall back without duplication.
static inline ap_uint<33> ParallelCacheTag(const uint8_t kind,
                                           const uint32_t address) {
#pragma HLS inline
  ap_uint<33> tag = address & ~uint32_t(VEC_64_PER_512 - 1);
  tag[32] = kind == 2;
  return tag;
}

static inline uint32_t ParallelSmLIndex(const uint32_t address) {
#pragma HLS inline
  constexpr uint32_t kWordBits = PAR_GLOBAL_SM_INDEX_BITS - 6;
  const uint32_t bank = (addr_bank(address) - PAR_NUM_A_BANK) & 7u;
  const uint32_t word =
      (addr_off(address) >> 3) & ((1u << kWordBits) - 1u);
  return (bank << kWordBits) | word;
}

#ifdef SALUT_LEGACY_4PE_PEG_LANES  // legacy 4-port lane, not invoked by the top level
void ParallelLMLane(
    tapa::istream<RR>& pe_in0,
    tapa::istream<RR>& pe_in1,
    tapa::istream<RR>& pe_in2,
    tapa::istream<RR>& pe_in3,
    tapa::ostream<RP>& pe_out0,
    tapa::ostream<RP>& pe_out1,
    tapa::ostream<RP>& pe_out2,
    tapa::ostream<RP>& pe_out3,
    tapa::ostream<RR>& peg_req_out,
    tapa::istream<RP>& peg_rsp_in,
    tapa::istream<WR>& wr_req_in,
    tapa::istream<WP>& wr_elem_in,
    tapa::ostream<ap_uint<8>>& wr_commit_out) {
  static_assert(PAR_PE_PER_PEG == 4,
                "PEG-local LM has four physical PE ports");
  static WORD512 lm_data[PAR_SM_WORDS];
  static ap_uint<33> lm_tag[PAR_SM_WORDS];
  static ap_uint<1> lm_valid[PAR_SM_WORDS];
#pragma HLS bind_storage variable=lm_data type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=lm_tag type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=lm_valid type=RAM_2P impl=BRAM

  ap_uint<2> rr = 0;
  // 0 select, 1 validate-and-emit LM word, 2 forward miss suffix,
  // 3 SM/HBM response, 4 explicit cache-valid initialization.
  ap_uint<3> state = 4;
  ap_uint<2> active_local = 0;
  uint32_t remaining = 0;
  uint32_t offset = 0;
  uint32_t init_line = 0;
  ReadReqTagged held{};

  WriteReqTagged wr_held{};
  uint32_t wr_remaining = 0;
  uint32_t wr_offset = 0;
  ap_uint<1> wr_active = 0;

  for (;;) {
#pragma HLS pipeline II=1
    // Keep initialization in the same persistent FSM as normal service.
    // Vitis HLS 2022.2 left the service-state phi register undefined when a
    // separate finite pipelined clear loop preceded this infinite loop.  A
    // dedicated state makes the transition explicit in both C and RTL while
    // preserving one valid-bit clear per cycle and repeated-call correctness.
    if (state == 4) {
      lm_valid[init_line] = 0;
      if (init_line + 1 == PAR_SM_WORDS) {
        init_line = 0;
        state = 0;
      } else {
        init_line += 1;
      }
      continue;
    }

    // The upstream PEG-local write arbiter forks only this PEG's factor writes
    // into this snoop port.  Column-front writes bypass all private LMs and are
    // installed only in the globally shared SM.
    if (!wr_active && wr_req_in.try_read(wr_held)) {
      wr_remaining = wr_held.r.len;
      wr_offset = 0;
      if (wr_remaining == 0) {
        wr_commit_out.write(wr_held.pe);
      } else {
        wr_active = 1;
      }
    }
    if (wr_active && !wr_elem_in.empty()) {
      const WritePacketTagged packet = wr_elem_in.read();
      const uint32_t count = uint32_t(packet.count);
      if (wr_held.pe != 0 && wr_held.pe <= PAR_NUM_PE) {
        const uint32_t address = wr_held.r.addr + wr_offset;
        const uint32_t index = (address >> 3) & (PAR_SM_WORDS - 1);
        lm_data[index] = packet.v;
        lm_tag[index] = ParallelCacheTag(wr_held.r.kind, address);
        lm_valid[index] = 1;
      }
      if (wr_remaining == count) {
        wr_remaining = 0;
        wr_offset = 0;
        wr_active = 0;
        wr_commit_out.write(wr_held.pe);
      } else {
        wr_remaining -= count;
        wr_offset += count;
      }
    }

    if (state == 0) {
      ReadReqTagged request{};
      bool got = false;
      switch (uint32_t(rr)) {
        case 0: got = pe_in0.try_read(request); break;
        case 1: got = pe_in1.try_read(request); break;
        case 2: got = pe_in2.try_read(request); break;
        default: got = pe_in3.try_read(request); break;
      }
      if (got) {
        held = request;
        active_local = rr;
        remaining = held.r.len;
        offset = 0;
        state = held.r.len == 0 ? 0 : (held.r.kind == 0 ? 2 : 1);
      }
      if (rr == PAR_PE_PER_PEG - 1)
        rr = 0;
      else
        rr = rr + 1;
    } else if (state == 1) {
        const uint32_t address = held.r.addr + offset;
        const uint32_t index = (address >> 3) & (PAR_SM_WORDS - 1);
        const bool hit = lm_valid[index] &&
                         lm_tag[index] == ParallelCacheTag(held.r.kind, address);
        if (!hit) {
          // A direct-mapped line may be replaced by a concurrent factor
          // write after an earlier word was emitted.  Forward only the
          // not-yet-emitted suffix; never trust a burst-wide pre-scan.
          held.r.addr += offset;
          held.r.len = remaining;
          offset = 0;
          state = 2;
        } else {
          bool output_full = false;
          switch (uint32_t(active_local)) {
            case 0: output_full = pe_out0.full(); break;
            case 1: output_full = pe_out1.full(); break;
            case 2: output_full = pe_out2.full(); break;
            default: output_full = pe_out3.full(); break;
          }
          if (output_full) continue;
          const uint32_t first = address & 7u;
          const uint32_t available = VEC_64_PER_512 - first;
          const uint32_t count = remaining < available ? remaining : available;
          const ReadPacketTagged packet{
              lm_data[index], held.pe, ap_uint<5>(first), ap_uint<7>(count)};
          switch (uint32_t(active_local)) {
            case 0: pe_out0.write(packet); break;
            case 1: pe_out1.write(packet); break;
            case 2: pe_out2.write(packet); break;
            default: pe_out3.write(packet); break;
          }
          if (remaining == count) {
            remaining = 0;
            offset = 0;
            state = 0;
          } else {
            remaining -= count;
            offset += count;
          }
        }
      } else if (state == 2) {
        const ReadReqTagged req = held;
        if (!peg_req_out.full()) {
          peg_req_out.write(req);
          offset = 0;
          state = req.r.len == 0 ? 0 : 3;
        }
      } else {
        bool output_full = false;
        switch (uint32_t(active_local)) {
          case 0: output_full = pe_out0.full(); break;
          case 1: output_full = pe_out1.full(); break;
          case 2: output_full = pe_out2.full(); break;
          default: output_full = pe_out3.full(); break;
        }
        if (!peg_rsp_in.empty() && !output_full) {
          const ReadPacketTagged rsp = peg_rsp_in.read();
#ifndef __SYNTHESIS__
          ++g_lm_rsp;
#endif
          switch (uint32_t(active_local)) {
            case 0: pe_out0.write(rsp); break;
            case 1: pe_out1.write(rsp); break;
            case 2: pe_out2.write(rsp); break;
            default: pe_out3.write(rsp); break;
          }
          // Write snoops are accumulated outside the cache and installed as
          // one complete word, so concurrent lower-level allocations cannot
          // expose or destroy a partially assembled factor word.
          if (held.r.kind != 0) {
            const uint32_t address = held.r.addr + offset;
            const uint32_t index = (address >> 3) & (PAR_SM_WORDS - 1);
            lm_data[index] = rsp.v;
            lm_tag[index] = ParallelCacheTag(held.r.kind, address);
            lm_valid[index] = 1;
          }
          // `remaining` includes the response consumed in this iteration.
          // Test before decrementing so a one-word burst returns to the
          // request-selection state instead of waiting forever for word two.
          const uint32_t count = uint32_t(rsp.count);
          if (remaining == count) {
            remaining = 0;
            offset = 0;
            state = 0;
          } else {
            remaining -= count;
            offset += count;
          }
        }
      }
  }
}
#endif  // SALUT_LEGACY_4PE_PEG_LANES

// Classify requests before the shared cache.  A/U requests stay entirely on
// the private PEG lane; only L requests enter the shared-cache control loop.
void ParallelSMRequestSplit(
    const int peg_slot,
    tapa::istream<RR>& peg_in,
    tapa::istream<RR>& front_in,
    tapa::ostream<RR>& l_peg_out,
    tapa::ostream<RR>& l_front_out,
    tapa::ostream<RR>& direct_out) {
  bool prefer_front = false;
  for (;;) {
#pragma HLS pipeline II=1
    ReadReqTagged probe{};
    bool from_front = false;
    bool have = false;
    if (prefer_front) {
      if (front_in.try_peek(probe)) {
        from_front = true;
        have = true;
      } else if (peg_in.try_peek(probe)) {
        have = true;
      }
    } else {
      if (peg_in.try_peek(probe)) {
        have = true;
      } else if (front_in.try_peek(probe)) {
        from_front = true;
        have = true;
      }
    }
    if (!have) continue;
    const bool is_l = probe.r.kind == 1;
    const bool have_space = is_l
        ? (from_front ? !l_front_out.full() : !l_peg_out.full())
        : !direct_out.full();
    if (!have_space) continue;
    const ReadReqTagged req = from_front ? front_in.read() : peg_in.read();
    if (is_l) {
      if (from_front) l_front_out.write(req);
      else l_peg_out.write(req);
    } else {
      direct_out.write(req);
    }
    prefer_front = !from_front;
  }
}

// Each PEG has a real, independent HBM transaction lane.  Keeping the
// outstanding request state here (rather than in the shared cache task)
// prevents HLS from building a many-writer mux around the shared SM arrays.
void ParallelSMHBMLane(
    const int peg_slot,
    tapa::istream<RR>& direct_req_in,
    tapa::istream<RR>& l_miss_req_in,
    tapa::ostream<RR>& hbm_req_out,
    tapa::istream<HBMReadPacketTagged>& hbm_rsp_in,
    tapa::ostream<SMFillPacket>& fill_out,
    tapa::ostream<RP>& direct_peg_rsp_out,
    tapa::ostream<RP>& direct_front_rsp_out) {
  ReadReqTagged pending{};
  uint32_t remaining = 0;
  uint32_t offset = 0;
  bool valid = false;
  bool prefer_l = false;

  for (;;) {
#pragma HLS pipeline II=1
    if (valid) {
      HBMReadPacketTagged hbm_rsp{};
      if (hbm_rsp_in.try_peek(hbm_rsp)) {
        const ReadPacketTagged rsp{
            hbm_rsp.v, hbm_rsp.pe, hbm_rsp.first, hbm_rsp.count};
        const bool is_fill = pending.r.kind == 1;
        const bool to_front = hbm_rsp.pe > PAR_NUM_PE;
        const bool have_space = is_fill
            ? !fill_out.full()
            : (to_front ? !direct_front_rsp_out.full()
                        : !direct_peg_rsp_out.full());
        if (have_space) {
          (void)hbm_rsp_in.read();
#ifndef __SYNTHESIS__
          ++g_hbm_lane_rsp;
#endif
          if (is_fill) {
            fill_out.write(SMFillPacket{rsp, pending.r.addr + offset});
          } else if (to_front) {
            direct_front_rsp_out.write(rsp);
          } else {
            direct_peg_rsp_out.write(rsp);
          }
          const uint32_t count = uint32_t(rsp.count);
          if (remaining == count) {
            remaining = 0;
            offset = 0;
            valid = false;
          } else {
            remaining -= count;
            offset += count;
          }
        }
      }
    } else if (!hbm_req_out.full()) {
      bool have_req = false;
      if (prefer_l) {
        if (l_miss_req_in.try_read(pending)) {
          have_req = true;
          prefer_l = false;
        } else if (direct_req_in.try_read(pending)) {
          have_req = true;
          prefer_l = true;
        }
      } else {
        if (direct_req_in.try_read(pending)) {
          have_req = true;
          prefer_l = true;
        } else if (l_miss_req_in.try_read(pending)) {
          have_req = true;
          prefer_l = false;
        }
      }
      if (!have_req) continue;
      hbm_req_out.write(pending);
      remaining = pending.r.len;
      offset = 0;
      valid = pending.r.len != 0;
    }
  }
}

// Route the scalar shared-cache response after the cache pipeline.  The
// expensive cache task no longer contains a dynamic 16-way FIFO selector.
void ParallelSMCacheResponseRoute(
    tapa::istream<RP>& in,
    tapa::ostreams<RP, PAR_NUM_PEG>& peg_out,
    tapa::ostreams<RP, PAR_NUM_FRONT>& front_out) {
  for (;;) {
#pragma HLS pipeline II=1
    ReadPacketTagged rsp{};
    if (!in.try_peek(rsp)) continue;
    const bool to_front = rsp.pe > PAR_NUM_PE;
    const uint32_t lane = to_front
        ? uint32_t(rsp.pe) - PAR_NUM_PE - 1
        : (uint32_t(rsp.pe) - 1) / PAR_PE_PER_PEG;
    const bool have_space = to_front ? !front_out[lane].full()
                                     : !peg_out[lane].full();
    if (!have_space) continue;
    (void)in.read();
    if (to_front) front_out[lane].write(rsp);
    else peg_out[lane].write(rsp);
  }
}

// Independently buffer cache-hit/fill responses and direct A/U HBM responses
// for each PEG/front pair.  This is an actual eight-lane response fabric, not
// a scalar arbiter hidden behind an array interface.
void ParallelSMResponseMux(
    const int peg_slot,
    tapa::istream<RP>& cache_peg_in,
    tapa::istream<RP>& direct_peg_in,
    tapa::ostream<RP>& peg_out,
    tapa::istream<RP>& cache_front_in,
    tapa::istream<RP>& direct_front_in,
    tapa::ostream<RP>& front_out) {
  bool prefer_direct_peg = false;
  bool prefer_direct_front = false;
  for (;;) {
#pragma HLS pipeline II=1
    if (prefer_direct_peg) {
      if (!direct_peg_in.empty()) {
        peg_out.write(direct_peg_in.read());
        prefer_direct_peg = false;
      } else if (!cache_peg_in.empty()) {
        peg_out.write(cache_peg_in.read());
        prefer_direct_peg = true;
      }
    } else {
      if (!cache_peg_in.empty()) {
        peg_out.write(cache_peg_in.read());
        prefer_direct_peg = true;
      } else if (!direct_peg_in.empty()) {
        peg_out.write(direct_peg_in.read());
        prefer_direct_peg = false;
      }
    }
    if (prefer_direct_front) {
      if (!direct_front_in.empty()) {
        front_out.write(direct_front_in.read());
        prefer_direct_front = false;
      } else if (!cache_front_in.empty()) {
        front_out.write(cache_front_in.read());
        prefer_direct_front = true;
      }
    } else {
      if (!cache_front_in.empty()) {
        front_out.write(cache_front_in.read());
        prefer_direct_front = true;
      } else if (!direct_front_in.empty()) {
        front_out.write(direct_front_in.read());
        prefer_direct_front = false;
      }
    }
  }
}

// Shared Memory (SM) for all PEGs.  It is deliberately a separate level from
// the private LM above: LM misses are looked up here, SM misses retain the PEG
// tag and are allowed to remain outstanding concurrently in the HBM fabric.
void SM(
    tapa::istreams<RR, PAR_NUM_PEG>& peg_req_in,
    tapa::istreams<RR, PAR_NUM_FRONT>& front_req_in,
    tapa::ostream<RP>& cache_rsp_out,
    tapa::ostreams<RR, PAR_NUM_PEG>& lane_req_out,
    tapa::istreams<SMFillPacket, PAR_NUM_PEG>& fill_in,
    tapa::istream<WR>& wr_req_in,
    tapa::istream<WP>& wr_elem_in,
    tapa::ostream<ap_uint<8>>& wr_commit_out) {
  static WORD512 sm_data[8][PAR_GLOBAL_SM_WORDS / 8];
  static ap_uint<33> sm_tag[8][PAR_GLOBAL_SM_WORDS / 8];
  static ap_uint<1> sm_valid[8][PAR_GLOBAL_SM_WORDS / 8];
#pragma HLS array_partition variable=sm_data complete dim=1
#pragma HLS array_partition variable=sm_tag complete dim=1
#pragma HLS array_partition variable=sm_valid complete dim=1
#pragma HLS bind_storage variable=sm_data type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=sm_tag type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=sm_valid type=RAM_2P impl=BRAM

  ap_uint<1> prefer_front[PAR_NUM_PEG];
#pragma HLS array_partition variable=prefer_front complete

  for (uint32_t index = 0; index < PAR_GLOBAL_SM_WORDS / 8; ++index) {
#pragma HLS pipeline II=1
    for (uint32_t bank = 0; bank < 8; ++bank) {
#pragma HLS unroll
      sm_valid[bank][index] = 0;
    }
  }
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    prefer_front[peg] = 0;
  }

  ap_uint<3> rr = 0;
  ap_uint<3> factor_rsp_rr = 0;
  ap_uint<3> owner = 0;
  // 0 select, 1 issue bank reads, 2 forward miss suffix,
  // 3 validate/reorder/emit the registered bank reads.
  ap_uint<3> state = 0;
  ap_uint<4> front_chain = 0;
  ReadReqTagged held{};
  uint32_t offset = 0;
  WORD512 lookup_bank_words[8];
  ap_uint<1> lookup_bank_hits[8];
  ap_uint<3> lookup_first_bank = 0;
  ap_uint<5> lookup_first = 0;
  ap_uint<7> lookup_count = 0;
#pragma HLS array_partition variable=lookup_bank_words complete
#pragma HLS array_partition variable=lookup_bank_hits complete
  WriteReqTagged wr_held{};
  uint32_t wr_remaining = 0;
  uint32_t wr_offset = 0;
  ap_uint<1> wr_active = 0;

  for (;;) {
#pragma HLS pipeline II=4
    // SALUT write policy: factors are installed in shared memory before the
    // originating task may retire; HBM receives the same stream independently.
    bool installed_write = false;
    bool cache_write = false;
    uint32_t cache_write_base = 0;
    ap_uint<3> cache_write_beats = 0;
    WORD512 cache_write_words[8] = {};
#pragma HLS array_partition variable=cache_write_words complete
    if (!wr_active && wr_req_in.try_read(wr_held)) {
      wr_remaining = wr_held.r.len;
      wr_offset = 0;
      if (wr_remaining == 0) {
        wr_commit_out.write(wr_held.pe);
      } else {
        wr_active = 1;
      }
    }
    if (wr_active && !wr_elem_in.empty()) {
      const WritePacketTagged packet = wr_elem_in.read();
      const uint32_t count = uint32_t(packet.count);
      const uint32_t address = wr_held.r.addr + wr_offset;
      if (wr_held.r.kind == 1) {
        cache_write = true;
        cache_write_base = address;
        cache_write_beats = 1;
        cache_write_words[0] = packet.v;
        installed_write = true;
      }
      if (wr_remaining == count) {
        wr_remaining = 0;
        wr_offset = 0;
        wr_active = 0;
        wr_commit_out.write(wr_held.pe);
      } else {
        wr_remaining -= count;
        wr_offset += count;
      }
    }

    // The eight private HBM lanes return only L fills here.  Select exactly
    // one lane per cycle, making the shared cache's physical write-port use
    // explicit to HLS.  Direct A/U responses bypass this task entirely.
    const uint32_t fill_peg = factor_rsp_rr;
    SMFillPacket fill{};
    if (!installed_write && fill_in[fill_peg].try_peek(fill)) {
      const ReadPacketTagged& rsp = fill.rsp;
      const bool rsp_space = !cache_rsp_out.full();
      if (rsp_space) {
        (void)fill_in[fill_peg].read();
#ifndef __SYNTHESIS__
        ++g_sm_fill_rsp;
#endif
        cache_rsp_out.write(rsp);
        const uint32_t address = fill.address;
        const uint32_t first = uint32_t(rsp.first);
        const uint32_t base_address = address - first;
        cache_write = true;
        cache_write_base = base_address;
        cache_write_beats = 1;
        cache_write_words[0] = rsp.v;
      }
    }
    factor_rsp_rr = (factor_rsp_rr + 1 == PAR_NUM_PEG)
                        ? ap_uint<3>(0) : ap_uint<3>(factor_rsp_rr + 1);

    // Both factor installation and HBM fill use this one physical write site.
    // Consecutive 512-bit words map to distinct banks, so up to four beats can
    // be committed in parallel without creating a multi-writer cache mux.
    const uint32_t cache_write_line = ParallelSmLIndex(cache_write_base);
    const uint32_t cache_write_first_bank = cache_write_line & 7u;
    for (uint32_t bank = 0; bank < 8; ++bank) {
#pragma HLS unroll
      const uint32_t q = (bank + 8u - cache_write_first_bank) & 7u;
      if (cache_write && q < uint32_t(cache_write_beats)) {
        const uint32_t word_address = cache_write_base + q * 8u;
        const uint32_t line = ParallelSmLIndex(word_address);
        const uint32_t index = line >> 3;
        sm_data[bank][index] = cache_write_words[q];
        sm_tag[bank][index] = ParallelCacheTag(1, word_address);
        sm_valid[bank][index] = 1;
      }
    }

    if (cache_write) {
      // Cache lookup resumes next cycle, keeping read-after-write behaviour
      // independent of the selected BRAM/URAM primitive mode.
    } else if (state == 0) {
      const uint32_t peg = rr;
      rr = (peg + 1 == PAR_NUM_PEG) ? 0 : peg + 1;
      bool accepted = false;
      bool from_front = false;
      ReadReqTagged probe{};
      if (prefer_front[peg]) {
        if (front_req_in[peg].try_peek(probe)) {
          held = front_req_in[peg].read();
          accepted = true;
          from_front = true;
        } else if (peg_req_in[peg].try_peek(probe)) {
          held = peg_req_in[peg].read();
          accepted = true;
        }
      } else {
        if (peg_req_in[peg].try_peek(probe)) {
          held = peg_req_in[peg].read();
          accepted = true;
        } else if (front_req_in[peg].try_peek(probe)) {
          held = front_req_in[peg].read();
          accepted = true;
          from_front = true;
        }
      }
      if (accepted) {
        prefer_front[peg] = from_front ? ap_uint<1>(0) : ap_uint<1>(1);
        owner = peg;
        offset = 0;
        front_chain = 0;
        state = held.r.len == 0 ? 0 : (held.r.kind == 1 ? 1 : 2);
      }
    } else if (state == 1) {
      const uint32_t address = held.r.addr + offset;
      const uint32_t first = address & 7u;
      const uint32_t remaining = held.r.len - offset;
      const uint32_t capacity = 8u - first;
      const uint32_t count = remaining < capacity ? remaining : capacity;
      const uint32_t base_address = address - first;
      const uint32_t base_line = ParallelSmLIndex(base_address);
      const uint32_t first_bank = base_line & 7u;
      lookup_first_bank = first_bank;
      lookup_first = first;
      lookup_count = count;
      // One statically addressed read per physical cache bank.  The values
      // are registered and reordered only in state 3, breaking the URAM-to-
      // wide-response combinational path.
      for (uint32_t bank = 0; bank < 8; ++bank) {
#pragma HLS unroll
        const uint32_t q = (bank + 8u - first_bank) & 7u;
        const uint32_t word_address = base_address + q * 8u;
        const uint32_t line = base_line + q;
        const uint32_t index = line >> 3;
        lookup_bank_words[bank] = sm_data[bank][index];
        lookup_bank_hits[bank] = sm_valid[bank][index] &&
            sm_tag[bank][index] == ParallelCacheTag(1, word_address);
      }
      state = 3;
    } else if (state == 3) {
      const uint32_t bank = uint32_t(lookup_first_bank);
      const WORD512 word = lookup_bank_words[bank];
      const bool hit = bool(lookup_bank_hits[bank]);
#ifndef __SYNTHESIS__
      if (held.pe > PAR_NUM_PE) {
        if (hit) ++g_front_sm_hits;
        else ++g_front_sm_misses;
      }
#endif
      const uint32_t peg = owner;
      const bool to_front = held.pe > PAR_NUM_PE;
      const bool rsp_space = !cache_rsp_out.full();
      if (!hit) {
        held.r.addr += offset;
        held.r.len -= offset;
        offset = 0;
        state = 2;
      } else if (rsp_space) {
        const ReadPacketTagged packet{
            word, held.pe, lookup_first, lookup_count};
        cache_rsp_out.write(packet);
        if (offset + uint32_t(lookup_count) == held.r.len) {
          offset = 0;
          ReadReqTagged next{};
          if (to_front && front_chain < 7u &&
              front_req_in[owner].try_read(next)) {
            held = next;
            ++front_chain;
            state = held.r.len == 0 ? 0 : (held.r.kind == 1 ? 1 : 2);
          } else {
            front_chain = 0;
            state = 0;
          }
        } else {
          offset += uint32_t(lookup_count);
          state = 1;
        }
      }
    } else if (state == 2) {
      const uint32_t peg = owner;
      if (!lane_req_out[peg].full()) {
        lane_req_out[peg].write(held);
        state = 0;
      }
    }
  }
}

void ParallelReadReqMux(
    tapa::istream<ReadReqTagged>& front_in,
    tapa::istreams<ReadReqTagged, PAR_NUM_PEG>& peg_in,
    tapa::ostream<ReadReqTagged>& out) {
  ap_uint<4> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t client = rr;
    if (client == 0) {
      if (!front_in.empty()) out.write(front_in.read());
    } else if (!peg_in[client - 1].empty()) {
      out.write(peg_in[client - 1].read());
    }
    rr = (client == PAR_NUM_PEG) ? 0 : client + 1;
  }
}

void ParallelPegReadReqMux(
    tapa::istreams<ReadReqTagged, PAR_NUM_PEG>& in,
    tapa::ostream<ReadReqTagged>& out) {
  ap_uint<3> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t peg = rr;
    if (!in[peg].empty()) out.write(in[peg].read());
    rr = (peg + 1 == PAR_NUM_PEG) ? 0 : peg + 1;
  }
}

// Eight-lane PEG-to-HBM request crossbar.  One held request per PEG breaks
// input backpressure from bank arbitration; requests targeting distinct HBM
// banks can be accepted and issued in the same cycle.  Per-bank round robin
// prevents a busy PEG from starving another PEG that targets the same bank.
void ParallelHBMReadDispatch8(
    tapa::istreams<ReadReqTagged, PAR_NUM_PEG>& in,
    tapa::ostreams<ReadReqTagged, PAR_NUM_HBM_BANK>& bank_out) {
  ReadReqTagged pending[PAR_NUM_PEG];
  ap_uint<1> valid[PAR_NUM_PEG];
  ap_uint<3> rr[PAR_NUM_HBM_BANK];
#pragma HLS array_partition variable=pending complete
#pragma HLS array_partition variable=valid complete
#pragma HLS array_partition variable=rr complete
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    valid[peg] = 0;
  }
  for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
    rr[bank] = 0;
  }

  for (;;) {
#pragma HLS pipeline II=1
#ifndef __SYNTHESIS__
    uint32_t issued_this_cycle = 0;
#endif
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      if (!valid[peg] && !in[peg].empty()) {
        pending[peg] = in[peg].read();
        valid[peg] = 1;
      }
    }

    ap_uint<1> take[PAR_NUM_PEG];
#pragma HLS array_partition variable=take complete
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      take[peg] = 0;
    }
    for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
      ap_uint<4> winner = PAR_NUM_PEG;
      for (int step = 0; step < PAR_NUM_PEG; ++step) {
#pragma HLS unroll
        const ap_uint<3> candidate = rr[bank] + step;
        const uint32_t peg = uint32_t(candidate & (PAR_NUM_PEG - 1));
        if (winner == PAR_NUM_PEG && valid[peg] &&
            addr_bank(pending[peg].r.addr) == uint32_t(bank)) {
          winner = peg;
        }
      }
      if (winner != PAR_NUM_PEG && !bank_out[bank].full()) {
        const uint32_t peg = uint32_t(winner);
        bank_out[bank].write(pending[peg]);
        take[peg] = 1;
        rr[bank] = (peg + 1 == PAR_NUM_PEG) ? 0 : peg + 1;
#ifndef __SYNTHESIS__
        ++issued_this_cycle;
#endif
      }
    }
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      if (take[peg]) valid[peg] = 0;
    }
#ifndef __SYNTHESIS__
    g_hbm_req_issued += issued_this_cycle;
    if (issued_this_cycle > g_hbm_req_max_parallel)
      g_hbm_req_max_parallel = issued_this_cycle;
#endif
  }
}

// First request-routing stage: two PEG lanes share one shallow conflict
// arbiter.  Requests to different banks advance together; a same-bank clash
// is resolved alternately.  Four instances cover all eight PEGs.
void ParallelHBMReadPairRoute(
    tapa::istream<ReadReqTagged>& in0,
    tapa::istream<ReadReqTagged>& in1,
    tapa::ostreams<ReadReqTagged, PAR_NUM_HBM_BANK>& out) {
  ReadReqTagged pending0{}, pending1{};
  ap_uint<1> valid0 = 0, valid1 = 0, prefer1 = 0;
  for (;;) {
#pragma HLS pipeline II=1
    if (!valid0 && in0.try_read(pending0)) valid0 = 1;
    if (!valid1 && in1.try_read(pending1)) valid1 = 1;
    const uint32_t bank0 = ParallelPhysicalBank(pending0.r.kind, pending0.r.addr);
    const uint32_t bank1 = ParallelPhysicalBank(pending1.r.kind, pending1.r.addr);
    if (valid0 && valid1 && bank0 == bank1) {
      if (!out[bank0].full()) {
        if (prefer1) {
          out[bank0].write(pending1);
          valid1 = 0;
        } else {
          out[bank0].write(pending0);
          valid0 = 0;
        }
        prefer1 = !prefer1;
      }
    } else {
      if (valid0 && !out[bank0].full()) {
        out[bank0].write(pending0);
        valid0 = 0;
      }
      if (valid1 && !out[bank1].full()) {
        out[bank1].write(pending1);
        valid1 = 0;
      }
    }
  }
}

void ParallelHBMReadMerge2(
    tapa::istreams<ReadReqTagged, PAR_NUM_HBM_BANK>& in0,
    tapa::istreams<ReadReqTagged, PAR_NUM_HBM_BANK>& in1,
    tapa::ostreams<ReadReqTagged, PAR_NUM_HBM_BANK>& out) {
  ap_uint<1> prefer1[PAR_NUM_HBM_BANK];
#pragma HLS array_partition variable=prefer1 complete
  for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
    prefer1[bank] = 0;
  }
  for (;;) {
#pragma HLS pipeline II=1
    for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
      if (out[bank].full()) continue;
      if (prefer1[bank]) {
        if (!in1[bank].empty()) {
          out[bank].write(in1[bank].read());
          prefer1[bank] = 0;
        } else if (!in0[bank].empty()) {
          out[bank].write(in0[bank].read());
        }
      } else {
        if (!in0[bank].empty()) {
          out[bank].write(in0[bank].read());
          prefer1[bank] = 1;
        } else if (!in1[bank].empty()) {
          out[bank].write(in1[bank].read());
        }
      }
    }
  }
}

void ParallelHBMReadMerge2Final(
    tapa::istreams<ReadReqTagged, PAR_NUM_HBM_BANK>& in0,
    tapa::istreams<ReadReqTagged, PAR_NUM_HBM_BANK>& in1,
    tapa::ostreams<ReadReqTagged, PAR_NUM_HBM_BANK>& out) {
  ap_uint<1> prefer1[PAR_NUM_HBM_BANK];
#pragma HLS array_partition variable=prefer1 complete
  for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
    prefer1[bank] = 0;
  }
  for (;;) {
#pragma HLS pipeline II=1
#ifndef __SYNTHESIS__
    uint32_t issued_this_cycle = 0;
#endif
    for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
      if (out[bank].full()) continue;
      bool issued = false;
      if (prefer1[bank]) {
        if (!in1[bank].empty()) {
          out[bank].write(in1[bank].read());
          prefer1[bank] = 0;
          issued = true;
        } else if (!in0[bank].empty()) {
          out[bank].write(in0[bank].read());
          issued = true;
        }
      } else {
        if (!in0[bank].empty()) {
          out[bank].write(in0[bank].read());
          prefer1[bank] = 1;
          issued = true;
        } else if (!in1[bank].empty()) {
          out[bank].write(in1[bank].read());
          issued = true;
        }
      }
#ifndef __SYNTHESIS__
      if (issued) ++issued_this_cycle;
#endif
    }
#ifndef __SYNTHESIS__
    g_hbm_req_issued += issued_this_cycle;
    if (issued_this_cycle > g_hbm_req_max_parallel)
      g_hbm_req_max_parallel = issued_this_cycle;
#endif
  }
}

static inline uint32_t ParallelResponsePeg(const ap_uint<8> client) {
#pragma HLS inline
  return client <= PAR_NUM_PE
      ? (uint32_t(client) - 1u) / PAR_PE_PER_PEG
      : uint32_t(client) - PAR_NUM_PE - 1u;
}

// First response stage: each output lane owns three fixed physical banks
// (lane, lane+8, lane+16).  The selector is only 3:1 and therefore avoids the
// old 24:1 payload mux on every destination PEG.
void ParallelHBMReadResponseBankMux8(
    tapa::istreams<HBMReadPacketTagged, PAR_NUM_HBM_BANK>& bank_in,
    tapa::ostreams<HBMReadPacketTagged, PAR_NUM_PEG>& lane_out) {
  ap_uint<2> rr[PAR_NUM_PEG];
#pragma HLS array_partition variable=rr complete
  for (int lane = 0; lane < PAR_NUM_PEG; ++lane) {
#pragma HLS unroll
    rr[lane] = 0;
  }
  for (;;) {
#pragma HLS pipeline II=1
    for (int lane = 0; lane < PAR_NUM_PEG; ++lane) {
#pragma HLS unroll
      if (lane_out[lane].full()) continue;
      const uint32_t group = uint32_t(rr[lane]);
      const uint32_t bank = lane + group * PAR_NUM_PEG;
      if (!bank_in[bank].empty()) {
        lane_out[lane].write(bank_in[bank].read());
      }
      rr[lane] = group == 2 ? 0 : group + 1;
    }
  }
}

// Route four physical response lanes to eight destination PEG lanes.  The SM
// invariant of one outstanding miss per PEG guarantees that two live HBM
// responses cannot target the same PEG; the priority guard is retained for
// defensive backpressure handling.
void ParallelHBMReadResponseRoute4(
    tapa::istream<HBMReadPacketTagged>& in0,
    tapa::istream<HBMReadPacketTagged>& in1,
    tapa::istream<HBMReadPacketTagged>& in2,
    tapa::istream<HBMReadPacketTagged>& in3,
    tapa::ostreams<HBMReadPacketTagged, PAR_NUM_PEG>& out) {
  HBMReadPacketTagged pending[4];
  ap_uint<1> valid[4];
#pragma HLS array_partition variable=pending complete
#pragma HLS array_partition variable=valid complete
  for (int i = 0; i < 4; ++i) {
#pragma HLS unroll
    valid[i] = 0;
  }
  for (;;) {
#pragma HLS pipeline II=1
    if (!valid[0] && in0.try_read(pending[0])) valid[0] = 1;
    if (!valid[1] && in1.try_read(pending[1])) valid[1] = 1;
    if (!valid[2] && in2.try_read(pending[2])) valid[2] = 1;
    if (!valid[3] && in3.try_read(pending[3])) valid[3] = 1;
    ap_uint<1> take[4];
#pragma HLS array_partition variable=take complete
    for (int i = 0; i < 4; ++i) {
#pragma HLS unroll
      take[i] = 0;
    }
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      ap_uint<3> winner = 4;
      for (int i = 0; i < 4; ++i) {
#pragma HLS unroll
        if (winner == 4 && valid[i] && pending[i].pe != 0 &&
            pending[i].pe <= PAR_NUM_CLIENT &&
            ParallelResponsePeg(pending[i].pe) == uint32_t(peg)) {
          winner = i;
        }
      }
      if (winner != 4 && !out[peg].full()) {
        out[peg].write(pending[uint32_t(winner)]);
        take[uint32_t(winner)] = 1;
      }
    }
    for (int i = 0; i < 4; ++i) {
#pragma HLS unroll
      if (take[i]) valid[i] = 0;
    }
  }
}

void ParallelHBMReadResponseRoute2(
    tapa::istream<HBMReadPacketTagged>& in0,
    tapa::istream<HBMReadPacketTagged>& in1,
    tapa::ostreams<HBMReadPacketTagged, PAR_NUM_PEG>& out) {
  HBMReadPacketTagged pending0{}, pending1{};
  ap_uint<1> valid0 = 0, valid1 = 0, prefer1 = 0;
  for (;;) {
#pragma HLS pipeline II=1
    if (!valid0 && in0.try_read(pending0)) valid0 = 1;
    if (!valid1 && in1.try_read(pending1)) valid1 = 1;
    const uint32_t peg0 = valid0 ? ParallelResponsePeg(pending0.pe) : 0;
    const uint32_t peg1 = valid1 ? ParallelResponsePeg(pending1.pe) : 0;
    if (valid0 && valid1 && peg0 == peg1) {
      if (!out[peg0].full()) {
        if (prefer1) {
          out[peg0].write(pending1);
          valid1 = 0;
        } else {
          out[peg0].write(pending0);
          valid0 = 0;
        }
        prefer1 = !prefer1;
      }
    } else {
      if (valid0 && !out[peg0].full()) {
        out[peg0].write(pending0);
        valid0 = 0;
      }
      if (valid1 && !out[peg1].full()) {
        out[peg1].write(pending1);
        valid1 = 0;
      }
    }
  }
}

void ParallelHBMReadResponseMerge2Stage(
    tapa::istreams<HBMReadPacketTagged, PAR_NUM_PEG>& in0,
    tapa::istreams<HBMReadPacketTagged, PAR_NUM_PEG>& in1,
    tapa::ostreams<HBMReadPacketTagged, PAR_NUM_PEG>& out) {
  ap_uint<1> prefer1[PAR_NUM_PEG];
#pragma HLS array_partition variable=prefer1 complete
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    prefer1[peg] = 0;
  }
  for (;;) {
#pragma HLS pipeline II=1
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      if (out[peg].full()) continue;
      if (prefer1[peg]) {
        if (!in1[peg].empty()) {
          out[peg].write(in1[peg].read());
          prefer1[peg] = 0;
        } else if (!in0[peg].empty()) {
          out[peg].write(in0[peg].read());
        }
      } else {
        if (!in0[peg].empty()) {
          out[peg].write(in0[peg].read());
          prefer1[peg] = 1;
        } else if (!in1[peg].empty()) {
          out[peg].write(in1[peg].read());
        }
      }
    }
  }
}

void ParallelHBMReadResponseMerge2(
    tapa::istreams<HBMReadPacketTagged, PAR_NUM_PEG>& in0,
    tapa::istreams<HBMReadPacketTagged, PAR_NUM_PEG>& in1,
    tapa::ostreams<HBMReadPacketTagged, PAR_NUM_PEG>& out) {
  ap_uint<1> prefer1[PAR_NUM_PEG];
#pragma HLS array_partition variable=prefer1 complete
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    prefer1[peg] = 0;
  }
  for (;;) {
#pragma HLS pipeline II=1
#ifndef __SYNTHESIS__
    uint32_t routed_this_cycle = 0;
#endif
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      if (out[peg].full()) continue;
      bool routed = false;
      if (prefer1[peg]) {
        if (!in1[peg].empty()) {
          out[peg].write(in1[peg].read());
          prefer1[peg] = 0;
          routed = true;
        } else if (!in0[peg].empty()) {
          out[peg].write(in0[peg].read());
          routed = true;
        }
      } else {
        if (!in0[peg].empty()) {
          out[peg].write(in0[peg].read());
          prefer1[peg] = 1;
          routed = true;
        } else if (!in1[peg].empty()) {
          out[peg].write(in1[peg].read());
          routed = true;
        }
      }
#ifndef __SYNTHESIS__
      if (routed) ++routed_this_cycle;
#endif
    }
#ifndef __SYNTHESIS__
    g_hbm_rsp_routed += routed_this_cycle;
    if (routed_this_cycle > g_hbm_rsp_max_parallel)
      g_hbm_rsp_max_parallel = routed_this_cycle;
#endif
  }
}

// Twenty-four HBM return FIFOs feed eight independent PEG lanes.  Bank-local
// holding registers allow responses for different PEGs to advance together;
// collisions for one PEG are resolved fairly and remain ordered by that PEG's
// single-outstanding-request rule in SM.
void ParallelHBMReadResponseRoute8(
    tapa::istreams<ReadPacketTagged, PAR_NUM_HBM_BANK>& bank_in,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_PEG>& out) {
  ReadPacketTagged pending[PAR_NUM_HBM_BANK];
  ap_uint<1> valid[PAR_NUM_HBM_BANK];
  ap_uint<5> rr[PAR_NUM_PEG];
#pragma HLS array_partition variable=pending complete
#pragma HLS array_partition variable=valid complete
#pragma HLS array_partition variable=rr complete
  for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
    valid[bank] = 0;
  }
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    rr[peg] = 0;
  }

  for (;;) {
#pragma HLS pipeline II=1
#ifndef __SYNTHESIS__
    uint32_t routed_this_cycle = 0;
#endif
    for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
      if (!valid[bank] && !bank_in[bank].empty()) {
        pending[bank] = bank_in[bank].read();
        valid[bank] = 1;
      }
    }

    ap_uint<1> take[PAR_NUM_HBM_BANK];
#pragma HLS array_partition variable=take complete
    for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
      take[bank] = 0;
    }
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      ap_uint<5> winner = PAR_NUM_HBM_BANK;
      for (int step = 0; step < PAR_NUM_HBM_BANK; ++step) {
#pragma HLS unroll
        uint32_t bank = uint32_t(rr[peg]) + uint32_t(step);
        if (bank >= PAR_NUM_HBM_BANK) bank -= PAR_NUM_HBM_BANK;
        if (winner == PAR_NUM_HBM_BANK && valid[bank]) {
          const ap_uint<8> client = pending[bank].pe;
          if (client != 0 && client <= PAR_NUM_CLIENT &&
              ParallelResponsePeg(client) == uint32_t(peg)) {
            winner = bank;
          }
        }
      }
      if (winner != PAR_NUM_HBM_BANK && !out[peg].full()) {
        const uint32_t bank = uint32_t(winner);
        out[peg].write(pending[bank]);
        take[bank] = 1;
        rr[peg] = (bank + 1 == PAR_NUM_HBM_BANK) ? 0 : bank + 1;
#ifndef __SYNTHESIS__
        ++routed_this_cycle;
#endif
      }
    }
    for (int bank = 0; bank < PAR_NUM_HBM_BANK; ++bank) {
#pragma HLS unroll
      if (take[bank]) valid[bank] = 0;
    }
#ifndef __SYNTHESIS__
    g_hbm_rsp_routed += routed_this_cycle;
    if (routed_this_cycle > g_hbm_rsp_max_parallel)
      g_hbm_rsp_max_parallel = routed_this_cycle;
#endif
  }
}

// One private, four-way banked read cache per physical column front.  L
// factors are immutable after their producer retires, so fill-on-read is
// coherent without broadcasting every PE write to all eight caches.  Four
// interleaved URAM banks preserve the HBM path's four-beat packet width on
// hits; this is important both for hardware bandwidth and simulation events.
void ParallelFrontCache(
    const int front_slot,
    tapa::istream<ReadReqTagged>& front_req_in,
    tapa::ostream<ReadPacketTagged>& front_rsp_out,
    tapa::ostream<ReadReqTagged>& miss_req_out,
    tapa::istream<ReadPacketTagged>& miss_rsp_in) {
#ifndef __SYNTHESIS__
  static WORD512 data_all[PAR_NUM_FRONT][4][PAR_FRONT_CACHE_LINES / 4];
  static ap_uint<33> tag_all[PAR_NUM_FRONT][4][PAR_FRONT_CACHE_LINES / 4];
  static ap_uint<1> valid_all[PAR_NUM_FRONT][4][PAR_FRONT_CACHE_LINES / 4];
  WORD512 (*data)[PAR_FRONT_CACHE_LINES / 4] = data_all[front_slot];
  ap_uint<33> (*tag)[PAR_FRONT_CACHE_LINES / 4] = tag_all[front_slot];
  ap_uint<1> (*valid)[PAR_FRONT_CACHE_LINES / 4] = valid_all[front_slot];
#else
  WORD512 data[4][PAR_FRONT_CACHE_LINES / 4];
  ap_uint<33> tag[4][PAR_FRONT_CACHE_LINES / 4];
  ap_uint<1> valid[4][PAR_FRONT_CACHE_LINES / 4];
#endif
#pragma HLS array_partition variable=data complete dim=1
#pragma HLS array_partition variable=tag complete dim=1
#pragma HLS array_partition variable=valid complete dim=1
#pragma HLS bind_storage variable=data type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=tag type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=valid type=RAM_2P impl=BRAM

  for (uint32_t index = 0; index < PAR_FRONT_CACHE_LINES / 4; ++index) {
#pragma HLS pipeline II=1
    for (uint32_t bank = 0; bank < 4; ++bank) {
#pragma HLS unroll
      valid[bank][index] = 0;
    }
  }

  enum State : uint8_t { SELECT, LOOKUP, FORWARD_MISS, RECEIVE_MISS };
  State state = SELECT;
  ReadReqTagged held{};
  uint32_t remaining = 0;
  uint32_t offset = 0;

  for (;;) {
#pragma HLS pipeline II=1
    if (state == SELECT) {
      if (front_req_in.try_read(held)) {
        remaining = held.r.len;
        offset = 0;
        state = remaining == 0 ? SELECT
                               : (held.r.kind == 1 ? LOOKUP : FORWARD_MISS);
      }
    } else if (state == LOOKUP) {
      const uint32_t address = held.r.addr + offset;
      const uint32_t first = addr_off(address) & 7u;
      const uint32_t capacity = 8u - first;
      const uint32_t count = remaining < capacity ? remaining : capacity;
      const uint32_t base_address = address - first;
      const uint32_t line =
          (addr_off(base_address) >> 3) & (PAR_FRONT_CACHE_LINES - 1);
      const uint32_t bank = line & 3u;
      const uint32_t index = line >> 2;
      const WORD512 word = data[bank][index];
      const bool hit = valid[bank][index] &&
          tag[bank][index] == ParallelCacheTag(held.r.kind, base_address);
      if (!hit) {
        held.r.addr += offset;
        held.r.len = remaining;
        offset = 0;
        state = FORWARD_MISS;
      } else if (!front_rsp_out.full()) {
        front_rsp_out.write(ReadPacketTagged{
            word, held.pe,
            ap_uint<5>(first), ap_uint<7>(count)});
        remaining -= count;
        offset += count;
        state = remaining == 0 ? SELECT : LOOKUP;
      }
    } else if (state == FORWARD_MISS) {
      if (!miss_req_out.full()) {
        miss_req_out.write(held);
        state = held.r.len == 0 ? SELECT : RECEIVE_MISS;
      }
    } else if (state == RECEIVE_MISS) {
      if (!miss_rsp_in.empty() && !front_rsp_out.full()) {
        const ReadPacketTagged packet = miss_rsp_in.read();
        front_rsp_out.write(packet);
        const uint32_t address = held.r.addr + offset;
        if (held.r.kind == 1) {
          const uint32_t first = uint32_t(packet.first);
          const uint32_t base_address = address - first;
          const uint32_t line =
              (addr_off(base_address) >> 3) & (PAR_FRONT_CACHE_LINES - 1);
          const uint32_t bank = line & 3u;
          const uint32_t index = line >> 2;
          data[bank][index] = packet.v;
          tag[bank][index] = ParallelCacheTag(held.r.kind, base_address);
          valid[bank][index] = 1;
        }
        const uint32_t count = uint32_t(packet.count);
        remaining -= count;
        offset += count;
        state = remaining == 0 ? SELECT : RECEIVE_MISS;
      }
    }
  }
}

void ParallelReadRspDemux(
    tapa::istream<ReadPacketTagged>& in,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_FRONT>& front_out,
    tapa::ostream<ReadPacketTagged>& shared_out) {
  for (;;) {
#pragma HLS pipeline II=1
    ReadPacketTagged value;
    if (!in.try_read(value)) continue;
    if (value.pe > PAR_NUM_PE && value.pe <= PAR_NUM_CLIENT) {
      front_out[uint32_t(value.pe) - PAR_NUM_PE - 1].write(value);
    } else if (value.pe != 0 && value.pe <= PAR_NUM_PE) {
      shared_out.write(value);
    }
  }
}

// A column front is the ninth logical requester of its corresponding PEG.
// Routing it through SM (instead of bypassing directly to HBM) makes produced
// L/U factors reusable by both fronts and segment PEs.
void ParallelSmReadReqMux(
    tapa::istreams<ReadReqTagged, PAR_NUM_PEG>& lm_in,
    tapa::istreams<ReadReqTagged, PAR_NUM_FRONT>& front_in,
    tapa::ostreams<ReadReqTagged, PAR_NUM_PEG>& sm_out) {
  ap_uint<1> prefer_front[PAR_NUM_PEG];
#pragma HLS array_partition variable=prefer_front complete
  for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
    prefer_front[peg] = 0;
  }
  for (;;) {
#pragma HLS pipeline II=1
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      if (sm_out[peg].full()) continue;
      if (prefer_front[peg]) {
        if (!front_in[peg].empty()) {
          sm_out[peg].write(front_in[peg].read());
          prefer_front[peg] = 0;
        } else if (!lm_in[peg].empty()) {
          sm_out[peg].write(lm_in[peg].read());
        }
      } else {
        if (!lm_in[peg].empty()) {
          sm_out[peg].write(lm_in[peg].read());
          prefer_front[peg] = 1;
        } else if (!front_in[peg].empty()) {
          sm_out[peg].write(front_in[peg].read());
        }
      }
    }
  }
}

void ParallelSmReadRspDemux(
    tapa::istreams<ReadPacketTagged, PAR_NUM_PEG>& sm_in,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_PEG>& lm_out,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_FRONT>& front_out) {
  for (;;) {
#pragma HLS pipeline II=1
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      ReadPacketTagged value;
      if (!sm_in[peg].try_peek(value)) continue;
      if (value.pe > PAR_NUM_PE && value.pe <= PAR_NUM_CLIENT) {
        const uint32_t front = uint32_t(value.pe) - PAR_NUM_PE - 1;
        if (!front_out[front].full()) {
          (void)sm_in[peg].read();
          front_out[front].write(value);
        }
      } else if (!lm_out[peg].full()) {
        (void)sm_in[peg].read();
        lm_out[peg].write(value);
      }
    }
  }
}

void ParallelHBMReadDispatchMulti(
    tapa::istreams<ReadReqTagged, PAR_NUM_FRONT>& front_in,
    tapa::istreams<ReadReqTagged, PAR_NUM_PEG>& shared_in,
    tapa::ostreams<ReadReqTagged, PAR_NUM_HBM_BANK>& bank_out) {
  ap_uint<4> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t client = rr;
    ReadReqTagged req;
    const bool have = client < PAR_NUM_FRONT
        ? front_in[client].try_peek(req)
        : shared_in[client - PAR_NUM_FRONT].try_peek(req);
    if (have) {
      const uint32_t bank = addr_bank(req.r.addr);
      if (bank < PAR_NUM_HBM_BANK && !bank_out[bank].full()) {
        if (client < PAR_NUM_FRONT)
          (void)front_in[client].read();
        else
          (void)shared_in[client - PAR_NUM_FRONT].read();
        bank_out[bank].write(req);
      }
    }
    rr = (client + 1 == PAR_NUM_FRONT + PAR_NUM_PEG) ? 0 : client + 1;
  }
}

void ParallelHBMReadResponseRoute(
    tapa::istreams<ReadPacketTagged, PAR_NUM_HBM_BANK>& bank_in,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_FRONT>& front_out,
    tapa::ostream<ReadPacketTagged>& shared_out) {
  ap_uint<5> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t bank = rr;
    ReadPacketTagged rsp;
    if (bank_in[bank].try_peek(rsp)) {
      if (rsp.pe > PAR_NUM_PE && rsp.pe <= PAR_NUM_CLIENT) {
        const uint32_t front = uint32_t(rsp.pe) - PAR_NUM_PE - 1;
        if (!front_out[front].full()) {
          (void)bank_in[bank].read();
          front_out[front].write(rsp);
        }
      } else if (!shared_out.full()) {
        (void)bank_in[bank].read();
        shared_out.write(rsp);
      }
    }
    rr = (bank + 1 == PAR_NUM_HBM_BANK) ? 0 : bank + 1;
  }
}

// One physical task per PEG keeps the four local PE write streams beside their
// producers.  The previous implementation unrolled eight lanes inside one HLS
// task; Vivado still placed that 28k-primitive task wholly in SLR1, forcing the
// SLR2 PE streams through an already 92%-occupied SLL boundary.
#ifdef SALUT_LEGACY_4PE_PEG_LANES  // legacy 4-port lane, not invoked by the top level
void ParallelPegWriteArbiterLane(
    tapa::istream<WriteReqTagged>& pe_req0,
    tapa::istream<WriteReqTagged>& pe_req1,
    tapa::istream<WriteReqTagged>& pe_req2,
    tapa::istream<WriteReqTagged>& pe_req3,
    tapa::istream<WritePacketTagged>& pe_elem0,
    tapa::istream<WritePacketTagged>& pe_elem1,
    tapa::istream<WritePacketTagged>& pe_elem2,
    tapa::istream<WritePacketTagged>& pe_elem3,
    tapa::ostream<WriteReqTagged>& peg_req,
    tapa::ostream<WritePacketTagged>& peg_elem,
    tapa::ostream<WriteReqTagged>& lm_req,
    tapa::ostream<WritePacketTagged>& lm_elem) {
  static_assert(PAR_PE_PER_PEG == 4,
                "PEG-local write arbiter has four physical PE ports");
  ap_uint<2> rr = 0;
  ap_uint<2> locked_local = 0;
  uint32_t remaining = 0;
  bool forwarding = false;
  for (;;) {
#pragma HLS pipeline II=1
    if (!forwarding) {
      WriteReqTagged req;
      bool got = false;
      if (!peg_req.full() && !lm_req.full()) {
        switch (uint32_t(rr)) {
          case 0: got = pe_req0.try_read(req); break;
          case 1: got = pe_req1.try_read(req); break;
          case 2: got = pe_req2.try_read(req); break;
          default: got = pe_req3.try_read(req); break;
        }
      }
      if (got) {
        peg_req.write(req);
        lm_req.write(req);
        remaining = req.r.len;
        locked_local = rr;
        forwarding = true;
      }
      if (rr == PAR_PE_PER_PEG - 1)
        rr = 0;
      else
        rr += 1;
    } else if (remaining != 0) {
      WritePacketTagged packet;
      bool got = false;
      if (!peg_elem.full() && !lm_elem.full()) {
        switch (uint32_t(locked_local)) {
          case 0: got = pe_elem0.try_read(packet); break;
          case 1: got = pe_elem1.try_read(packet); break;
          case 2: got = pe_elem2.try_read(packet); break;
          default: got = pe_elem3.try_read(packet); break;
        }
      }
      if (got) {
        peg_elem.write(packet);
        lm_elem.write(packet);
        remaining -= uint32_t(packet.count);
      }
    } else {
      forwarding = false;
    }
  }
}
#endif  // SALUT_LEGACY_4PE_PEG_LANES

// Burst-locked two-input merge.  A balanced tree of these tasks replaces the
// former 9:1 global write arbiter, keeping the PEG-side wires local for the
// first two levels and presenting only one narrow trunk to HBM/SM.
void ParallelWriteMerge2(
    tapa::istream<WriteReqTagged>& req0,
    tapa::istream<WritePacketTagged>& elem0,
    tapa::istream<WriteReqTagged>& req1,
    tapa::istream<WritePacketTagged>& elem1,
    tapa::ostream<WriteReqTagged>& req_out,
    tapa::ostream<WritePacketTagged>& elem_out) {
  ap_uint<1> rr = 0;
  ap_uint<1> locked_input = 0;
  uint32_t remaining = 0;
  bool forwarding = false;
  for (;;) {
#pragma HLS pipeline II=1
    if (!forwarding) {
      WriteReqTagged req{};
      bool got = false;
      if (!req_out.full()) {
        got = rr == 0 ? req0.try_read(req) : req1.try_read(req);
      }
      if (got) {
        req_out.write(req);
        remaining = req.r.len;
        locked_input = rr;
        forwarding = true;
      }
      rr = !rr;
    } else if (remaining != 0) {
      WritePacketTagged elem{};
      const bool got = !elem_out.full() &&
                       (locked_input == 0 ? elem0.try_read(elem)
                                          : elem1.try_read(elem));
      if (got) {
        elem_out.write(elem);
        remaining -= uint32_t(elem.count);
      }
    } else {
      forwarding = false;
    }
  }
}

void ParallelWriteBroadcast(
    tapa::istream<WriteReqTagged>& req_in,
    tapa::istream<WritePacketTagged>& elem_in,
    tapa::ostream<WriteReqTagged>& req_out,
    tapa::ostream<WritePacketTagged>& elem_out,
    tapa::ostream<WriteReqTagged>& sm_req_out,
    tapa::ostream<WritePacketTagged>& sm_elem_out) {
  uint32_t remaining = 0;
  bool forwarding = false;
  for (;;) {
#pragma HLS pipeline II=1
    if (!forwarding) {
      if (!req_in.empty() && !req_out.full() && !sm_req_out.full()) {
        const WriteReqTagged req = req_in.read();
        req_out.write(req);
        sm_req_out.write(req);
        remaining = req.r.len;
        forwarding = true;
      }
    } else if (remaining != 0) {
      if (!elem_in.empty() && !elem_out.full() && !sm_elem_out.full()) {
        const WritePacketTagged elem = elem_in.read();
        elem_out.write(elem);
        sm_elem_out.write(elem);
        remaining -= uint32_t(elem.count);
      }
    } else {
      forwarding = false;
    }
  }
}

// A write becomes visible to dependency scheduling only after the private LM,
// shared SM and physical HBM have all accepted the complete burst.  Since each
// client waits for its acknowledgement before issuing another factor write,
// one three-bit scoreboard entry per client is sufficient even though HBM-bank
// acknowledgements may return out of order.
void ParallelWriteCommitJoin(
    tapa::istream<ap_uint<8>>& hbm_in,
    tapa::istream<ap_uint<8>>& sm_in,
    tapa::istreams<ap_uint<8>, PAR_NUM_PEG>& lm_in,
    tapa::ostream<ap_uint<8>>& out) {
  // Only 3*(PAR_NUM_CLIENT+1) bits are required.  The former 73x3 BRAM
  // scoreboard introduced a synchronous read/modify/write latency that HLS
  // did not preserve across the pipelined held-ACK state: directed RTL read
  // both HBM and SM acknowledgements but never emitted the completed client.
  // Three bitmaps are true registers, make the update atomic, and also remove
  // the separate finite initialization FSM.
  ap_uint<PAR_NUM_CLIENT + 1> seen_hbm = 0;
  ap_uint<PAR_NUM_CLIENT + 1> seen_sm = 0;
  ap_uint<PAR_NUM_CLIENT + 1> seen_lm = 0;
  ap_uint<4> rr = 0;
  ap_uint<8> pending_client = 0;
  bool pending_valid = false;
  for (;;) {
#pragma HLS pipeline II=1
    // A completed client may wait here under downstream backpressure.  Do not
    // consume another acknowledgement until it is visible to AckDemux.
    if (pending_valid) {
      if (!out.full()) {
        out.write(pending_client);
        pending_valid = false;
      }
      continue;
    }

    ap_uint<8> ack_client = 0;
    bool got = false;
    ap_uint<2> source = rr < 2 ? ap_uint<2>(rr) : ap_uint<2>(2);
    if (rr == 0) got = hbm_in.try_read(ack_client);
    else if (rr == 1) got = sm_in.try_read(ack_client);
    else got = lm_in[uint32_t(rr) - 2].try_read(ack_client);
    if (rr == PAR_NUM_PEG + 1) rr = 0;
    else rr = rr + 1;

    if (got && ack_client <= PAR_NUM_CLIENT) {
      const uint32_t client = uint32_t(ack_client);
      bool next_hbm = seen_hbm[client];
      bool next_sm = seen_sm[client];
      bool next_lm = seen_lm[client];
      if (source == 0) next_hbm = true;
      else if (source == 1) next_sm = true;
      else next_lm = true;
      const bool needs_lm = ack_client != 0 && ack_client <= PAR_NUM_PE;
      const bool done = next_hbm && next_sm && (!needs_lm || next_lm);
      if (done) {
        seen_hbm[client] = 0;
        seen_sm[client] = 0;
        seen_lm[client] = 0;
        if (!out.full()) {
          out.write(ack_client);
        } else {
          pending_client = ack_client;
          pending_valid = true;
        }
      } else {
        seen_hbm[client] = next_hbm;
        seen_sm[client] = next_sm;
        seen_lm[client] = next_lm;
      }
    }
  }
}

void ParallelAckDemux(
    tapa::istream<ap_uint<8>>& in,
    tapa::ostreams<ap_uint<1>, PAR_NUM_FRONT>& front_out,
    tapa::ostreams<ap_uint<8>, PAR_NUM_PEG>& peg_out) {
  for (;;) {
#pragma HLS pipeline II=1
    ap_uint<8> client;
    if (!in.try_read(client)) continue;
    if (client > PAR_NUM_PE && client <= PAR_NUM_CLIENT) {
      front_out[uint32_t(client) - PAR_NUM_PE - 1].write(1);
    } else if (client != 0 && client <= PAR_NUM_PE) {
      const uint32_t peg = (uint32_t(client) - 1) / PAR_PE_PER_PEG;
      peg_out[peg].write(client);
    }
  }
}

void ParallelPegAckDemux(
    tapa::istreams<ap_uint<8>, PAR_NUM_PEG>& peg_in,
    tapa::ostreams<ap_uint<1>, PAR_NUM_PE>& pe_out) {
  for (;;) {
#pragma HLS pipeline II=1
    for (int peg = 0; peg < PAR_NUM_PEG; ++peg) {
#pragma HLS unroll
      ap_uint<8> client;
      if (peg_in[peg].try_read(client)) {
        const uint32_t pe = uint32_t(client) - 1;
        if (pe < PAR_NUM_PE) pe_out[pe].write(1);
      }
    }
  }
}

void ParallelSegDoneMux(
    tapa::istreams<uint32_t, PAR_NUM_PE>& in,
    tapa::ostream<uint32_t>& out) {
  ap_uint<6> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t pe = rr;
    if (!in[pe].empty()) out.write(in[pe].read());
    rr = (pe + 1 == PAR_NUM_PE) ? 0 : pe + 1;
  }
}

// Eight SPU-style column fronts consume independent ready columns.  The
// allocator keeps every task and its packed dependency records atomic under
// bounded FIFO backpressure.
void ParallelFrontAllocationUnit(
    tapa::istream<ParColumnTask>& task_in,
    tapa::istream<ParDepPacket>& dep_in,
    tapa::ostreams<ParColumnTask, PAR_NUM_FRONT>& task_out,
    tapa::ostreams<ParDepPacket, PAR_NUM_FRONT>& dep_out) {
  ap_uint<3> rr = 0;
  for (;;) {
    const ParColumnTask task = task_in.read();
    if (task.j == 0xFFFFFFFFu) {
      for (int front = 0; front < PAR_NUM_FRONT; ++front) {
#pragma HLS unroll
        task_out[front].write(task);
      }
      return;
    }
    uint32_t selected = rr;
    bool selected_valid = false;
    while (!selected_valid) {
#pragma HLS pipeline II=1
      for (int q = 0; q < PAR_NUM_FRONT; ++q) {
#pragma HLS unroll
        const uint32_t candidate = (uint32_t(rr) + q) &
                                   (PAR_NUM_FRONT - 1);
        if (!selected_valid && !task_out[candidate].full()) {
          selected = candidate;
          selected_valid = true;
        }
      }
    }
    task_out[selected].write(task);
#ifndef __SYNTHESIS__
    if (g_sim_direct_dependencies) {
      rr = (selected + 1) & (PAR_NUM_FRONT - 1);
      continue;
    }
#endif
    for (uint32_t d = 0; d < task.dep_count; d += PAR_DEP_PACKET_WIDTH) {
#pragma HLS pipeline II=1
      dep_out[selected].write(dep_in.read());
    }
    rr = (selected + 1) & (PAR_NUM_FRONT - 1);
  }
}

void ParallelFrontReadReqMux(
    tapa::istreams<ReadReqTagged, PAR_NUM_FRONT>& in,
    tapa::ostream<ReadReqTagged>& out) {
  ap_uint<3> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t front = rr;
    if (!in[front].empty()) out.write(in[front].read());
    rr = (front + 1) & (PAR_NUM_FRONT - 1);
  }
}

void ParallelFrontWriteArbiter(
    tapa::istreams<WriteReqTagged, PAR_NUM_FRONT>& req_in,
    tapa::istreams<WritePacketTagged, PAR_NUM_FRONT>& elem_in,
    tapa::ostream<WriteReqTagged>& req_out,
    tapa::ostream<WritePacketTagged>& elem_out) {
  ap_uint<3> rr = 0, locked = 0;
  uint32_t remaining = 0;
  bool forwarding = false;
  for (;;) {
#pragma HLS pipeline II=1
    if (!forwarding) {
      const uint32_t front = rr;
      if (!req_in[front].empty() && !req_out.full()) {
        const WriteReqTagged req = req_in[front].read();
        req_out.write(req);
        remaining = req.r.len;
        locked = front;
        forwarding = true;
      }
      rr = (front + 1) & (PAR_NUM_FRONT - 1);
    } else if (remaining != 0) {
      const uint32_t front = locked;
      if (!elem_in[front].empty() && !elem_out.full()) {
        const WritePacketTagged packet = elem_in[front].read();
        elem_out.write(packet);
        remaining -= uint32_t(packet.count);
      }
    } else {
      forwarding = false;
    }
  }
}

void ParallelFrontDoneMux(
    tapa::istreams<uint32_t, PAR_NUM_FRONT>& in,
    tapa::ostream<uint32_t>& out) {
  ap_uint<3> rr = 0;
  for (;;) {
#pragma HLS pipeline II=1
    const uint32_t front = rr;
    if (!in[front].empty()) out.write(in[front].read());
    rr = (front + 1) & (PAR_NUM_FRONT - 1);
  }
}

void ParallelColumnFront(
    const int front_slot,
    tapa::istream<ParColumnTask>& task_in,
    tapa::istream<ParDepPacket>& dep_in,
    tapa::ostream<ReadReqTagged>& rd_req,
    tapa::istream<ReadPacketTagged>& rd_rsp,
    tapa::ostream<WriteReqTagged>& wr_req,
    tapa::ostream<WritePacketTagged>& wr_elem,
    tapa::istream<ap_uint<1>>& wr_ack,
    tapa::ostream<uint32_t>& front_done) {
  const ap_uint<8> client = PAR_NUM_PE + front_slot + 1;
#ifndef __SYNTHESIS__
  // TAPA software tasks use fiber stacks; keep the large private workspaces
  // in statically allocated, front-indexed slices to avoid stack overflow.
  static REAL work_linear_all[PAR_NUM_FRONT][PAR_RUNTIME_MAX_N];
  static uint32_t u_rows_all[PAR_NUM_FRONT][PAR_RUNTIME_MAX_N];
  static REAL u_values_all[PAR_NUM_FRONT][PAR_RUNTIME_MAX_N];
  REAL* work_linear = work_linear_all[front_slot];
  uint32_t* u_rows = u_rows_all[front_slot];
  REAL* u_values = u_values_all[front_slot];
#else
  REAL work0[PAR_RUNTIME_MAX_N / 8], work1[PAR_RUNTIME_MAX_N / 8];
  REAL work2[PAR_RUNTIME_MAX_N / 8], work3[PAR_RUNTIME_MAX_N / 8];
  REAL work4[PAR_RUNTIME_MAX_N / 8], work5[PAR_RUNTIME_MAX_N / 8];
  REAL work6[PAR_RUNTIME_MAX_N / 8], work7[PAR_RUNTIME_MAX_N / 8];
  ap_uint<16> u_rows[PAR_RUNTIME_MAX_N];
  REAL u_values[PAR_RUNTIME_MAX_N];
#endif
#pragma HLS bind_storage variable=work0 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work1 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work2 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work3 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work4 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work5 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work6 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work7 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=u_rows type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=u_values type=RAM_2P impl=URAM

  for (;;) {
    const ParColumnTask task = task_in.read();
    if (task.j == 0xFFFFFFFFu) return;
    const uint32_t a_len = task.a_words * 8u;
    if (a_len)
      rd_req.write(ReadReqTagged{ReadBurstReq{0, task.a_addr, a_len}, client});
    for (uint32_t received = 0; received < a_len;) {
      const ReadPacketTagged packet = rd_rsp.read();
      for (uint32_t lane = 0; lane < uint32_t(packet.count); ++lane) {
#pragma HLS pipeline II=1
        const WORD64 elem = get_read_packet_element(packet, lane);
        const uint32_t row = ParElementRow(elem);
        if (row >= PAR_RUNTIME_MAX_N) continue;
        const uint32_t index = row >> 3;
        const REAL value = ParElementValue(elem);
#ifndef __SYNTHESIS__
        work_linear[row] = value;
#else
        switch (row & 7u) {
          case 0: work0[index] = value; break; case 1: work1[index] = value; break;
          case 2: work2[index] = value; break; case 3: work3[index] = value; break;
          case 4: work4[index] = value; break; case 5: work5[index] = value; break;
          case 6: work6[index] = value; break; default: work7[index] = value; break;
        }
#endif
      }
      received += uint32_t(packet.count);
    }

    constexpr uint32_t kFrontDepBatch =
        PAR_DEP_PACKET_WIDTH < 8u ? 8u : PAR_DEP_PACKET_WIDTH;
    for (uint32_t base = 0; base < task.dep_count; base += kFrontDepBatch) {
      ParDepEntry deps[kFrontDepBatch];
#pragma HLS array_partition variable=deps complete
      const uint32_t valid = task.dep_count - base < kFrontDepBatch
          ? task.dep_count - base : kFrontDepBatch;
      ParDepPacket dep_packet{};
      for (uint32_t q = 0; q < valid; ++q) {
#pragma HLS pipeline II=1
        if ((q & (PAR_DEP_PACKET_WIDTH - 1u)) == 0)
          dep_packet = dep_in.read();
        deps[q] = ParGetDep(dep_packet, q);
      }
      for (uint32_t q = 0; q < valid; ++q) {
      const uint32_t d = base + q;
      const ParDepEntry dep = deps[q];
      const uint32_t k_index = dep.k >> 3;
      REAL ukj = 0.0f;
#ifndef __SYNTHESIS__
      ukj = work_linear[dep.k];
#else
      switch (dep.k & 7u) {
        case 0: ukj = work0[k_index]; break; case 1: ukj = work1[k_index]; break;
        case 2: ukj = work2[k_index]; break; case 3: ukj = work3[k_index]; break;
        case 4: ukj = work4[k_index]; break; case 5: ukj = work5[k_index]; break;
        case 6: ukj = work6[k_index]; break; default: ukj = work7[k_index]; break;
      }
#endif
      u_rows[d] = dep.k;
      u_values[d] = ukj;
      const uint32_t l_len = dep.l_words * 8u;
      // A single-beat response stream may return several packets for one L
      // request.  Consume that request completely before posting the next
      // dependency; otherwise a shallow response FIFO can fill while this
      // front is blocked trying to enqueue the rest of the batch.
      if (l_len)
        rd_req.write(ReadReqTagged{
            ReadBurstReq{1, dep.l_addr, l_len}, client});
      for (uint32_t received = 0; received < l_len;) {
        const ReadPacketTagged packet = rd_rsp.read();
        for (uint32_t lane = 0; lane < uint32_t(packet.count); ++lane) {
#pragma HLS pipeline II=1
          const WORD64 elem = get_read_packet_element(packet, lane);
          const uint32_t row = ParElementRow(elem);
          if (row >= PAR_RUNTIME_MAX_N || row > task.j) continue;
#ifndef __SYNTHESIS__
          work_linear[row] -= ParElementValue(elem) * ukj;
#else
          const uint32_t index = row >> 3;
          const REAL lval = ParElementValue(elem);
          switch (row & 7u) {
            case 0: work0[index] -= lval * ukj; break; case 1: work1[index] -= lval * ukj; break;
            case 2: work2[index] -= lval * ukj; break; case 3: work3[index] -= lval * ukj; break;
            case 4: work4[index] -= lval * ukj; break; case 5: work5[index] -= lval * ukj; break;
            case 6: work6[index] -= lval * ukj; break; default: work7[index] -= lval * ukj; break;
          }
#endif
        }
        received += uint32_t(packet.count);
      }
      }
    }

    const uint32_t j_index = task.j >> 3;
    REAL pivot = 0.0f;
#ifndef __SYNTHESIS__
    pivot = work_linear[task.j];
#else
    switch (task.j & 7u) {
      case 0: pivot = work0[j_index]; break; case 1: pivot = work1[j_index]; break;
      case 2: pivot = work2[j_index]; break; case 3: pivot = work3[j_index]; break;
      case 4: pivot = work4[j_index]; break; case 5: pivot = work5[j_index]; break;
      case 6: pivot = work6[j_index]; break; default: pivot = work7[j_index]; break;
    }
#endif
    pivot = StabilizeStaticPivot(pivot);
    u_rows[task.dep_count] = task.j;
    u_values[task.dep_count] = pivot;

    const uint32_t u_len = task.u_words * 8u;
    wr_req.write(
        WriteReqTagged{WriteBurstReq{2, task.u_addr, u_len}, client});
    for (uint32_t w = 0; w < task.u_words; ++w) {
#pragma HLS pipeline II=1
      WORD512 packet = 0;
      for (uint32_t lane = 0; lane < VEC_64_PER_512; ++lane) {
#pragma HLS unroll
        const uint32_t p = w * VEC_64_PER_512 + lane;
        const WORD64 elem = p <= task.dep_count
            ? ParPackElement(u_rows[p], u_values[p])
            : ParPackElement(0xFFFFFFFFu, 0.0f);
        set_word64(packet, lane, elem);
      }
      wr_elem.write(WritePacketTagged{packet, client, VEC_64_PER_512});
    }
    (void)wr_ack.read();
    front_done.write(task.j);
  }
}

void ParallelSegmentPE(
    const int pe_slot,
    tapa::istream<ParSegmentTask>& task_in,
    tapa::istream<ParDepPacket>& dep_in,
    tapa::ostream<ReadReqTagged>& rd_req,
    tapa::ostream<ParPivotReq>& pivot_req,
    tapa::istream<ap_uint<1>>& pivot_go,
    tapa::istream<ReadPacketTagged>& rd_rsp,
    tapa::ostream<WriteReqTagged>& wr_req,
    tapa::ostream<WritePacketTagged>& wr_elem,
    tapa::istream<ap_uint<1>>& wr_ack,
    tapa::ostream<uint32_t>& seg_done) {
  const ap_uint<8> client = pe_slot + 1;
  REAL work[PAR_SEG_ROWS_MAX];
  ap_uint<21> active_token[PAR_SEG_ROWS_MAX];
#pragma HLS bind_storage variable=work type=RAM_2P impl=BRAM
#pragma HLS bind_storage variable=active_token type=RAM_2P impl=BRAM
  // One parallel startup clear per physical PE replaces a full row-window
  // clear for every segment.  seg_id+1 is a unique nonzero token because the
  // metadata limit is PAR_MAX_SEG (2^20).
  for (uint32_t i = 0; i < PAR_SEG_ROWS_MAX; ++i) {
#pragma HLS pipeline II=1
    active_token[i] = 0;
  }
  for (;;) {
    const ParSegmentTask task = task_in.read();
    if (task.seg_id == 0xFFFFFFFFu) return;
    const uint32_t win_len = task.row_hi - task.row_lo;
    const ap_uint<21> task_token = ap_uint<21>(task.seg_id) + 1;
    const uint32_t a_len = task.a_words * 8u;
    if (a_len) rd_req.write(ReadReqTagged{ReadBurstReq{0, task.a_addr, a_len}, client});
    for (uint32_t received = 0; received < a_len;) {
      const ReadPacketTagged packet = rd_rsp.read();
      for (uint32_t lane = 0; lane < uint32_t(packet.count); ++lane) {
#pragma HLS pipeline II=1
        const WORD64 elem = get_read_packet_element(packet, lane);
        const uint32_t row = ParElementRow(elem);
        if (row < task.row_lo || row >= task.row_hi) continue;
        const uint32_t off = row - task.row_lo;
        work[off] = ParElementValue(elem);
        active_token[off] = task_token;
      }
      received += uint32_t(packet.count);
    }

    // U(k,j) and U(j,j) are produced by the target column front, not merely
    // by completion of predecessor column k.  An early task may prefetch A,
    // but must cross the front-commit gate before reading any target-column U
    // value.  The dedicated early PE retains this A_buffer without blocking
    // the other progress PEs in its PEG.
    if (task.early) {
      // A blocking write followed by the dependent GO read can be scheduled
      // atomically by HLS. Post the request in its own phase before waiting.
      bool pivot_request_posted = false;
      while (!pivot_request_posted) {
#pragma HLS pipeline II=1
        pivot_request_posted =
            pivot_req.try_write(ParPivotReq{task.j, task.u_diag_addr, client});
      }
      (void)pivot_go.read();
    }
    rd_req.write(
        ReadReqTagged{ReadBurstReq{2, task.u_diag_addr, 1}, client});
    ReadPacketTagged pivot_packet{};
    bool pivot_rsp_received = false;
    while (!pivot_rsp_received) {
#pragma HLS pipeline II=1
      pivot_rsp_received = rd_rsp.try_read(pivot_packet);
    }
    const WORD64 pivot_word = get_read_packet_element(pivot_packet, 0);
    const REAL pivot = ParElementValue(pivot_word);

    // Segment dependencies are ordered by k, hence their U(k,j) locations
    // are monotonic inside the target column.  Fetch four scalar U values as
    // one contiguous burst per dependency packet instead of sending four
    // independent request/response handshakes through LM/SM/HBM.
    for (uint32_t d = 0; d < task.dep_count; d += PAR_DEP_PACKET_WIDTH) {
      const ParDepPacket dep_packet = dep_in.read();
      ParDepEntry deps[PAR_DEP_PACKET_WIDTH];
      REAL ukj_values[PAR_DEP_PACKET_WIDTH];
#pragma HLS array_partition variable=deps complete
#pragma HLS array_partition variable=ukj_values complete
      const uint32_t valid =
          task.dep_count - d < PAR_DEP_PACKET_WIDTH
              ? task.dep_count - d : PAR_DEP_PACKET_WIDTH;
      for (uint32_t lane = 0; lane < PAR_DEP_PACKET_WIDTH; ++lane) {
#pragma HLS unroll
        deps[lane] = ParGetDep(dep_packet, lane);
        ukj_values[lane] = 0.0f;
      }
      const uint32_t first_u = deps[0].u_addr;
      const uint32_t first_off = addr_off(first_u);
      const uint32_t last_off = addr_off(deps[valid - 1].u_addr);
      const uint32_t u_span = last_off - first_off + 1u;
      rd_req.write(
          ReadReqTagged{ReadBurstReq{2, first_u, u_span}, client});
      for (uint32_t received = 0; received < u_span;) {
        const ReadPacketTagged packet = rd_rsp.read();
        for (uint32_t lane = 0; lane < uint32_t(packet.count); ++lane) {
#pragma HLS pipeline II=1
          const uint32_t ordinal = received + lane;
          const WORD64 elem = get_read_packet_element(packet, lane);
          for (uint32_t q = 0; q < PAR_DEP_PACKET_WIDTH; ++q) {
#pragma HLS unroll
            if (q < valid && addr_off(deps[q].u_addr) - first_off == ordinal)
              ukj_values[q] = ParElementValue(elem);
          }
        }
        received += uint32_t(packet.count);
      }

      for (uint32_t q = 0; q < valid; ++q) {
        const ParDepEntry dep = deps[q];
        const REAL ukj = ukj_values[q];
        const uint32_t l_len = dep.l_words * 8u;
        if (l_len)
          rd_req.write(ReadReqTagged{ReadBurstReq{1, dep.l_addr, l_len}, client});
        for (uint32_t received = 0; received < l_len;) {
          const ReadPacketTagged packet = rd_rsp.read();
          for (uint32_t lane = 0; lane < uint32_t(packet.count); ++lane) {
#pragma HLS pipeline II=1
            const WORD64 elem = get_read_packet_element(packet, lane);
            const uint32_t row = ParElementRow(elem);
            if (row < task.row_lo || row >= task.row_hi) continue;
            const uint32_t off = row - task.row_lo;
            if (active_token[off] != task_token) {
              work[off] = 0.0f;
              active_token[off] = task_token;
            }
            work[off] -= ParElementValue(elem) * ukj;
          }
          received += uint32_t(packet.count);
        }
      }
    }

    const uint32_t l_len = task.l_words * 8u;
    if (l_len) {
      wr_req.write(WriteReqTagged{WriteBurstReq{1, task.l_addr, l_len}, client});
      uint32_t emitted = 0;
      uint32_t emitted_packets = 0;
      ap_uint<4> packet_lane = 0;
      WORD512 packet = 0;
      for (uint32_t off = 0; off < win_len; ++off) {
#pragma HLS pipeline II=1
        const uint32_t row = task.row_lo + off;
        if (row > task.j && active_token[off] == task_token) {
          set_word64(packet, packet_lane,
                     ParPackElement(row, work[off] / pivot));
          if (packet_lane == VEC_64_PER_512 - 1) {
            wr_elem.write(WritePacketTagged{
                packet, client, VEC_64_PER_512});
            packet = 0;
            packet_lane = 0;
            ++emitted_packets;
          } else {
            ++packet_lane;
          }
          emitted++;
        }
      }
      while (emitted_packets < task.l_words) {
#pragma HLS pipeline II=1
        for (uint32_t lane = 0; lane < VEC_64_PER_512; ++lane) {
#pragma HLS unroll
          if (lane >= uint32_t(packet_lane)) {
            set_word64(packet, lane,
                       ParPackElement(0xFFFFFFFFu, 0.0f));
          }
        }
        wr_elem.write(WritePacketTagged{
            packet, client, VEC_64_PER_512});
        packet = 0;
        packet_lane = 0;
        ++emitted_packets;
      }
      (void)wr_ack.read();
#ifndef __SYNTHESIS__
      if (emitted != task.l_count) {
        std::cerr << "ParallelSegmentPE: L count mismatch seg=" << task.seg_id
                  << " expected=" << task.l_count << " emitted=" << emitted << std::endl;
      }
#endif
    }
    seg_done.write(task.seg_id);
  }
}

#ifndef __SYNTHESIS__
static WORD512* g_sim_bank_ptr[PAR_NUM_HBM_BANK] = {};
static std::unique_ptr<uint32_t[]> g_sim_bank_row[PAR_NUM_HBM_BANK];
static std::unique_ptr<REAL[]> g_sim_bank_value[PAR_NUM_HBM_BANK];
static uint64_t g_sim_bank_elements[PAR_NUM_HBM_BANK] = {};

static void ParSimInitNativeBank(const uint32_t bank, WORD512* packed,
                                 const uint64_t words,
                                 const bool decode_input) {
  const uint64_t elements = words * VEC_64_PER_512;
  g_sim_bank_ptr[bank] = packed;
  g_sim_bank_elements[bank] = elements;
  g_sim_bank_row[bank].reset(new uint32_t[elements]);
  g_sim_bank_value[bank].reset(new REAL[elements]);
  if (!decode_input) return;
  for (uint64_t w = 0; w < words; ++w) {
    const WORD512 word = packed[w];
    for (uint32_t lane = 0; lane < VEC_64_PER_512; ++lane) {
      const WORD64 elem = get_word64(word, lane);
      const uint64_t p = w * VEC_64_PER_512 + lane;
      g_sim_bank_row[bank][p] = ParElementRow(elem);
      g_sim_bank_value[bank][p] = ParElementValue(elem);
    }
  }
}

static inline uint64_t ParSimNativeIndex(const uint32_t address,
                                         const uint32_t ordinal) {
  return uint64_t(addr_off(address)) + ordinal;
}

static inline uint32_t ParSimReadRow(const uint32_t address,
                                     const uint32_t ordinal) {
  return g_sim_bank_row[addr_bank(address)][ParSimNativeIndex(address,
                                                               ordinal)];
}

static inline REAL ParSimReadValue(const uint32_t address,
                                   const uint32_t ordinal) {
  return g_sim_bank_value[addr_bank(address)][ParSimNativeIndex(address,
                                                                 ordinal)];
}

static inline ParDepEntry ParSimReadFrontDep(const uint32_t j,
                                             const uint32_t ordinal) {
  const uint32_t off_col = g_sim_meta_ptr[4];
  const uint32_t off_flat = g_sim_meta_ptr[7];
  const uint32_t desc = off_col + j * PAR_COL_DESC_WORDS;
  const uint32_t dep_lo = g_sim_meta_ptr[desc + 7];
  const uint32_t p = off_flat + (dep_lo + ordinal) * 4u;
  return ParDepEntry{g_sim_meta_ptr[p], g_sim_meta_ptr[p + 1],
                     g_sim_meta_ptr[p + 2], g_sim_meta_ptr[p + 3]};
}

static inline ParDepEntry ParSimReadSegmentDep(const uint32_t sid,
                                               const uint32_t ordinal) {
  const uint32_t off_seg = g_sim_meta_ptr[5];
  const uint32_t off_flat = g_sim_meta_ptr[9];
  const uint32_t desc = off_seg + sid * PAR_SEG_DESC_WORDS;
  const uint32_t dep_lo = g_sim_meta_ptr[desc + 10];
  const uint32_t p = off_flat + (dep_lo + ordinal) * 4u;
  return ParDepEntry{g_sim_meta_ptr[p], g_sim_meta_ptr[p + 1],
                     g_sim_meta_ptr[p + 2], g_sim_meta_ptr[p + 3]};
}

static inline WORD64 ParSimReadElementPacked(const uint32_t address,
                                             const uint32_t ordinal) {
  const uint32_t absolute = addr_off(address) + ordinal;
  return get_word64(g_sim_bank_ptr[addr_bank(address)][absolute >> 3],
                    absolute & 7u);
}

// Functional TAPA front: identical sparse FP32 operation order and dependency
// stream as ParallelColumnFront, with logical 24-bank memory accessed directly
// to avoid transport-only fiber events in host simulation.
void ParallelColumnFrontSimDirect(
    const int front_slot,
    tapa::istream<ParColumnTask>& task_in,
    tapa::ostream<uint32_t>& front_done) {
  static REAL work_all[PAR_NUM_FRONT][PAR_RUNTIME_MAX_N];
  static uint32_t u_rows_all[PAR_NUM_FRONT][PAR_RUNTIME_MAX_N];
  static REAL u_values_all[PAR_NUM_FRONT][PAR_RUNTIME_MAX_N];
  REAL* work = work_all[front_slot];
  uint32_t* u_rows = u_rows_all[front_slot];
  REAL* u_values = u_values_all[front_slot];

  for (;;) {
    const ParColumnTask task = task_in.read();
    if (task.j == 0xFFFFFFFFu) return;
    const uint32_t a_len = task.a_words * VEC_64_PER_512;
    for (uint32_t p = 0; p < a_len; ++p) {
      const uint32_t row = ParSimReadRow(task.a_addr, p);
      if (row < PAR_RUNTIME_MAX_N) work[row] = ParSimReadValue(task.a_addr, p);
    }

    for (uint32_t d = 0; d < task.dep_count; ++d) {
        const ParDepEntry dep = ParSimReadFrontDep(task.j, d);
        const REAL ukj = work[dep.k];
        u_rows[d] = dep.k;
        u_values[d] = ukj;
        const uint32_t l_len = dep.l_words * VEC_64_PER_512;
        const uint32_t l_bank = addr_bank(dep.l_addr);
        const uint64_t l_base = addr_off(dep.l_addr);
        const uint32_t* const rows = g_sim_bank_row[l_bank].get() + l_base;
        const REAL* const values = g_sim_bank_value[l_bank].get() + l_base;
#pragma omp simd
        for (uint32_t p = 0; p < l_len; ++p) {
          const uint32_t row = rows[p];
          if (row < PAR_RUNTIME_MAX_N && row <= task.j)
            work[row] -= values[p] * ukj;
        }
    }

    const REAL pivot = StabilizeStaticPivot(work[task.j]);
    u_rows[task.dep_count] = task.j;
    u_values[task.dep_count] = pivot;
    WORD512* const dst = g_sim_bank_ptr[addr_bank(task.u_addr)] +
                         (addr_off(task.u_addr) >> 3);
    const uint32_t u_bank = addr_bank(task.u_addr);
    const uint64_t u_base = addr_off(task.u_addr);
    for (uint32_t w = 0; w < task.u_words; ++w) {
      WORD512 word = 0;
      for (uint32_t lane = 0; lane < VEC_64_PER_512; ++lane) {
        const uint32_t p = w * VEC_64_PER_512 + lane;
        const uint32_t row = p <= task.dep_count ? u_rows[p] : 0xFFFFFFFFu;
        const REAL value = p <= task.dep_count ? u_values[p] : 0.0f;
        g_sim_bank_row[u_bank][u_base + p] = row;
        g_sim_bank_value[u_bank][u_base + p] = value;
        set_word64(word, lane, ParPackElement(row, value));
      }
      dst[w] = word;
    }
    front_done.write(task.j);
  }
}

// Functional TAPA PE: one instance is still created for every physical PE.
// Direct bank loads remove only the LM/SM/HBM message transport; the segment
// dependency order, pivot gate, sparse row filtering, division, and writeback
// are the same as ParallelSegmentPE.
void ParallelSegmentPESimDirect(
    const int pe_slot,
    tapa::istream<ParSegmentTask>& task_in,
    tapa::ostream<ParPivotReq>& pivot_req,
    tapa::istream<ap_uint<1>>& pivot_go,
    tapa::ostream<uint32_t>& seg_done) {
  static REAL work_all[PAR_NUM_PE][PAR_SEG_ROWS_MAX];
  static uint32_t token_all[PAR_NUM_PE][PAR_SEG_ROWS_MAX];
  REAL* work = work_all[pe_slot];
  uint32_t* active_token = token_all[pe_slot];

  for (;;) {
    const ParSegmentTask task = task_in.read();
    if (task.seg_id == 0xFFFFFFFFu) return;
    const uint32_t task_token = task.seg_id + 1u;
    const uint32_t a_len = task.a_words * VEC_64_PER_512;
    for (uint32_t p = 0; p < a_len; ++p) {
      const uint32_t row = ParSimReadRow(task.a_addr, p);
      if (row < task.row_lo || row >= task.row_hi) continue;
      const uint32_t off = row - task.row_lo;
      work[off] = ParSimReadValue(task.a_addr, p);
      active_token[off] = task_token;
    }

    if (task.early) {
      pivot_req.write(ParPivotReq{task.j, task.u_diag_addr,
                                  ap_uint<8>(pe_slot + 1)});
      (void)pivot_go.read();
    }
    const REAL pivot = ParSimReadValue(task.u_diag_addr, 0);

    for (uint32_t d = 0; d < task.dep_count; ++d) {
        const ParDepEntry dep = ParSimReadSegmentDep(task.seg_id, d);
        const REAL ukj = ParSimReadValue(dep.u_addr, 0);
        const uint32_t l_len = dep.l_words * VEC_64_PER_512;
        const uint32_t l_dep_bank = addr_bank(dep.l_addr);
        const uint64_t l_dep_base = addr_off(dep.l_addr);
        const uint32_t* const rows =
            g_sim_bank_row[l_dep_bank].get() + l_dep_base;
        const REAL* const values =
            g_sim_bank_value[l_dep_bank].get() + l_dep_base;
#pragma omp simd
        for (uint32_t p = 0; p < l_len; ++p) {
          const uint32_t row = rows[p];
          if (row < task.row_lo || row >= task.row_hi) continue;
          const uint32_t off = row - task.row_lo;
          if (active_token[off] != task_token) {
            work[off] = 0.0f;
            active_token[off] = task_token;
          }
          work[off] -= values[p] * ukj;
        }
    }

    WORD512* const dst = g_sim_bank_ptr[addr_bank(task.l_addr)] +
                         (addr_off(task.l_addr) >> 3);
    const uint32_t l_bank = addr_bank(task.l_addr);
    const uint64_t l_base = addr_off(task.l_addr);
    uint32_t emitted = 0;
    uint32_t word_index = 0;
    uint32_t lane = 0;
    WORD512 word = 0;
    const uint32_t win_len = task.row_hi - task.row_lo;
    for (uint32_t off = 0; off < win_len; ++off) {
      const uint32_t row = task.row_lo + off;
      if (row <= task.j || active_token[off] != task_token) continue;
      const REAL value = work[off] / pivot;
      g_sim_bank_row[l_bank][l_base + emitted] = row;
      g_sim_bank_value[l_bank][l_base + emitted] = value;
      set_word64(word, lane, ParPackElement(row, value));
      ++emitted;
      if (++lane == VEC_64_PER_512) {
        dst[word_index++] = word;
        word = 0;
        lane = 0;
      }
    }
    while (word_index < task.l_words) {
      for (uint32_t fill = lane; fill < VEC_64_PER_512; ++fill) {
        const uint64_t p = l_base + word_index * VEC_64_PER_512 + fill;
        g_sim_bank_row[l_bank][p] = 0xFFFFFFFFu;
        g_sim_bank_value[l_bank][p] = 0.0f;
        set_word64(word, fill, ParPackElement(0xFFFFFFFFu, 0.0f));
      }
      dst[word_index++] = word;
      word = 0;
      lane = 0;
    }
    if (emitted != task.l_count) {
      std::cerr << "ParallelSegmentPESimDirect: L count mismatch seg="
                << task.seg_id << " expected=" << task.l_count
                << " emitted=" << emitted << std::endl;
    }
    seg_done.write(task.seg_id);
  }
}

// Host-only batched execution of the same PLU2 static dependency graph.  A
// cooperative TAPA stream read/write costs far more than the sparse arithmetic
// for matrices with hundreds of thousands of nearly empty columns.  Once the
// graph has been topologically frozen by preprocessing, increasing column
// order is a valid ready order for every front and segment dependency.  This
// routine therefore coalesces only the control transport: it still consumes
// every column/segment descriptor, every dependency, all 24 logical banks and
// performs the identical FP32 operations and packed writeback.
//
// The synthesized dataflow graph is unaffected.  Set
// SALUT_EXACT_CONTROL_EVENTS=1 to run the stream-by-stream 8-front/32-PE host
// model above, or SALUT_EXACT_MEMORY_EVENTS=1 for LM/SM/HBM event simulation.
static void ParSimRunFrontBatched(const ParColumnTask& task) {
  static REAL work[PAR_RUNTIME_MAX_N];
  static uint32_t u_rows[PAR_RUNTIME_MAX_N];
  static REAL u_values[PAR_RUNTIME_MAX_N];

  const uint32_t a_len = task.a_words * VEC_64_PER_512;
  for (uint32_t p = 0; p < a_len; ++p) {
    const uint32_t row = ParSimReadRow(task.a_addr, p);
    if (row < PAR_RUNTIME_MAX_N)
      work[row] = ParSimReadValue(task.a_addr, p);
  }

  for (uint32_t d = 0; d < task.dep_count; ++d) {
    const ParDepEntry dep = ParSimReadFrontDep(task.j, d);
    const REAL ukj = work[dep.k];
    u_rows[d] = dep.k;
    u_values[d] = ukj;
    const uint32_t l_len = dep.l_words * VEC_64_PER_512;
    const uint32_t l_bank = addr_bank(dep.l_addr);
    const uint64_t l_base = addr_off(dep.l_addr);
    const uint32_t* const rows = g_sim_bank_row[l_bank].get() + l_base;
    const REAL* const values = g_sim_bank_value[l_bank].get() + l_base;
#pragma omp simd
    for (uint32_t p = 0; p < l_len; ++p) {
      const uint32_t row = rows[p];
      if (row < PAR_RUNTIME_MAX_N && row <= task.j)
        work[row] -= values[p] * ukj;
    }
  }

  const REAL pivot = StabilizeStaticPivot(work[task.j]);
  u_rows[task.dep_count] = task.j;
  u_values[task.dep_count] = pivot;
  WORD512* const dst = g_sim_bank_ptr[addr_bank(task.u_addr)] +
                       (addr_off(task.u_addr) >> 3);
  const uint32_t u_bank = addr_bank(task.u_addr);
  const uint64_t u_base = addr_off(task.u_addr);
  for (uint32_t w = 0; w < task.u_words; ++w) {
    WORD512 word = 0;
    for (uint32_t lane = 0; lane < VEC_64_PER_512; ++lane) {
      const uint32_t p = w * VEC_64_PER_512 + lane;
      const uint32_t row = p <= task.dep_count ? u_rows[p] : 0xFFFFFFFFu;
      const REAL value = p <= task.dep_count ? u_values[p] : 0.0f;
      g_sim_bank_row[u_bank][u_base + p] = row;
      g_sim_bank_value[u_bank][u_base + p] = value;
      set_word64(word, lane, ParPackElement(row, value));
    }
    dst[w] = word;
  }
}

static void ParSimRunSegmentBatched(const ParSegmentTask& task) {
  static REAL work[PAR_SEG_ROWS_MAX];
  static uint32_t active_token[PAR_SEG_ROWS_MAX];
  const uint32_t task_token = task.seg_id + 1u;
  const uint32_t a_len = task.a_words * VEC_64_PER_512;
  for (uint32_t p = 0; p < a_len; ++p) {
    const uint32_t row = ParSimReadRow(task.a_addr, p);
    if (row < task.row_lo || row >= task.row_hi) continue;
    const uint32_t off = row - task.row_lo;
    work[off] = ParSimReadValue(task.a_addr, p);
    active_token[off] = task_token;
  }

  const REAL pivot = ParSimReadValue(task.u_diag_addr, 0);
  for (uint32_t d = 0; d < task.dep_count; ++d) {
    const ParDepEntry dep = ParSimReadSegmentDep(task.seg_id, d);
    const REAL ukj = ParSimReadValue(dep.u_addr, 0);
    const uint32_t l_len = dep.l_words * VEC_64_PER_512;
    const uint32_t l_dep_bank = addr_bank(dep.l_addr);
    const uint64_t l_dep_base = addr_off(dep.l_addr);
    const uint32_t* const rows =
        g_sim_bank_row[l_dep_bank].get() + l_dep_base;
    const REAL* const values =
        g_sim_bank_value[l_dep_bank].get() + l_dep_base;
#pragma omp simd
    for (uint32_t p = 0; p < l_len; ++p) {
      const uint32_t row = rows[p];
      if (row < task.row_lo || row >= task.row_hi) continue;
      const uint32_t off = row - task.row_lo;
      if (active_token[off] != task_token) {
        work[off] = 0.0f;
        active_token[off] = task_token;
      }
      work[off] -= values[p] * ukj;
    }
  }

  WORD512* const dst = g_sim_bank_ptr[addr_bank(task.l_addr)] +
                       (addr_off(task.l_addr) >> 3);
  const uint32_t l_bank = addr_bank(task.l_addr);
  const uint64_t l_base = addr_off(task.l_addr);
  uint32_t emitted = 0;
  uint32_t word_index = 0;
  uint32_t lane = 0;
  WORD512 word = 0;
  const uint32_t win_len = task.row_hi - task.row_lo;
  for (uint32_t off = 0; off < win_len; ++off) {
    const uint32_t row = task.row_lo + off;
    if (row <= task.j || active_token[off] != task_token) continue;
    const REAL value = work[off] / pivot;
    g_sim_bank_row[l_bank][l_base + emitted] = row;
    g_sim_bank_value[l_bank][l_base + emitted] = value;
    set_word64(word, lane, ParPackElement(row, value));
    ++emitted;
    if (++lane == VEC_64_PER_512) {
      dst[word_index++] = word;
      word = 0;
      lane = 0;
    }
  }
  while (word_index < task.l_words) {
    for (uint32_t fill = lane; fill < VEC_64_PER_512; ++fill) {
      const uint64_t p = l_base + word_index * VEC_64_PER_512 + fill;
      g_sim_bank_row[l_bank][p] = 0xFFFFFFFFu;
      g_sim_bank_value[l_bank][p] = 0.0f;
      set_word64(word, fill, ParPackElement(0xFFFFFFFFu, 0.0f));
    }
    dst[word_index++] = word;
    word = 0;
    lane = 0;
  }
  if (emitted != task.l_count) {
    std::cerr << "ParSimRunSegmentBatched: L count mismatch seg="
              << task.seg_id << " expected=" << task.l_count
              << " emitted=" << emitted << std::endl;
  }
}

static void ParSimRunControlBatched(tapa::mmap<META_T> meta) {
  if (meta[0] != PAR_META_MAGIC) {
    std::cerr << "[SALUT error] invalid PLU2 metadata magic" << std::endl;
    return;
  }
  const ParMetaIndexKernel m = ParReadMetaIndex(meta);
  if (m.n > PAR_RUNTIME_MAX_N || m.nSeg > PAR_MAX_SEG) {
    std::cerr << "[SALUT error] graph exceeds simulation capacity: n="
              << m.n << "/" << PAR_RUNTIME_MAX_N << " segments="
              << m.nSeg << "/" << PAR_MAX_SEG << std::endl;
    return;
  }

  uint64_t completed_segments = 0;
  for (uint32_t j = 0; j < m.n; ++j) {
    const uint32_t c = m.off_col + j * PAR_COL_DESC_WORDS;
    const uint32_t dep_lo = meta[c + 7];
    const uint32_t dep_hi = meta[c + 8];
    const ParColumnTask front{j, meta[c + 0], meta[c + 1], meta[c + 2],
                              meta[c + 3], dep_hi - dep_lo};
    ParSimRunFrontBatched(front);

    // This is the segment-column trigger in its batched representation: the
    // front has committed U(j,j), and increasing-j execution guarantees that
    // all posted segment dependencies have already committed as well.
    const uint32_t seg_lo = meta[c + 5];
    const uint32_t seg_hi = meta[c + 6];
    for (uint32_t sid = seg_lo; sid < seg_hi; ++sid) {
      const uint32_t s = m.off_seg + sid * PAR_SEG_DESC_WORDS;
      const uint32_t sdep_lo = meta[s + 10];
      const uint32_t sdep_hi = meta[s + 11];
      const ParSegmentTask segment{
          sid, meta[s + 0], meta[s + 2], meta[s + 3], meta[s + 4],
          meta[s + 5], meta[s + 6], meta[s + 7], meta[s + 8], meta[s + 9],
          sdep_hi - sdep_lo, 0u};
      ParSimRunSegmentBatched(segment);
      ++completed_segments;
    }
    if (((j + 1u) & 16383u) == 0 || j + 1u == m.n) {
      std::cerr << "[SALUT batch progress] completed_columns=" << (j + 1u)
                << "/" << m.n << " completed_segments="
                << completed_segments << "/" << m.nSeg << std::endl;
    }
  }
  if (completed_segments != m.nSeg) {
    std::cerr << "[SALUT batch] segment coverage mismatch: completed="
              << completed_segments << " expected=" << m.nSeg << std::endl;
  }
}

// Fast functional memory backend for TAPA software simulation.  The complete
// scheduler, eight column fronts, 64 SegmentPEs, dependency FIFOs, and
// segment-column trigger remain separate TAPA tasks.  Only the LM/SM/HBM
// transport hierarchy is collapsed here: modelling every 512-bit hop through
// several cooperative fibers dominates host runtime but adds no information
// to the numerical/software validation.  The synthesized branch below still
// instantiates the full physical hierarchy unchanged.
void ParallelSimMemoryFabric(
    tapa::mmap<WORD512> A_0, tapa::mmap<WORD512> A_1,
    tapa::mmap<WORD512> A_2, tapa::mmap<WORD512> A_3,
    tapa::mmap<WORD512> A_4, tapa::mmap<WORD512> A_5,
    tapa::mmap<WORD512> A_6, tapa::mmap<WORD512> A_7,
    tapa::mmap<WORD512> L_0, tapa::mmap<WORD512> L_1,
    tapa::mmap<WORD512> L_2, tapa::mmap<WORD512> L_3,
    tapa::mmap<WORD512> L_4, tapa::mmap<WORD512> L_5,
    tapa::mmap<WORD512> L_6, tapa::mmap<WORD512> L_7,
    tapa::mmap<WORD512> U_0, tapa::mmap<WORD512> U_1,
    tapa::mmap<WORD512> U_2, tapa::mmap<WORD512> U_3,
    tapa::mmap<WORD512> U_4, tapa::mmap<WORD512> U_5,
    tapa::mmap<WORD512> U_6, tapa::mmap<WORD512> U_7,
    tapa::istreams<ReadReqTagged, PAR_NUM_FRONT>& front_rd_req,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_FRONT>& front_rd_rsp,
    tapa::istreams<WriteReqTagged, PAR_NUM_FRONT>& front_wr_req,
    tapa::istreams<WritePacketTagged, PAR_NUM_FRONT>& front_wr_elem,
    tapa::ostreams<ap_uint<1>, PAR_NUM_FRONT>& front_wr_ack,
    tapa::istreams<ReadReqTagged, PAR_NUM_PE>& pe_rd_req,
    tapa::ostreams<ReadPacketTagged, PAR_NUM_PE>& pe_rd_rsp,
    tapa::istreams<WriteReqTagged, PAR_NUM_PE>& pe_wr_req,
    tapa::istreams<WritePacketTagged, PAR_NUM_PE>& pe_wr_elem,
    tapa::ostreams<ap_uint<1>, PAR_NUM_PE>& pe_wr_ack) {
  auto read_word = [&](const uint32_t bank, const uint32_t index) -> WORD512 {
    switch (bank) {
      case 0: return A_0[index]; case 1: return A_1[index];
      case 2: return A_2[index]; case 3: return A_3[index];
      case 4: return A_4[index]; case 5: return A_5[index];
      case 6: return A_6[index]; case 7: return A_7[index];
      case 8: return L_0[index]; case 9: return L_1[index];
      case 10: return L_2[index]; case 11: return L_3[index];
      case 12: return L_4[index]; case 13: return L_5[index];
      case 14: return L_6[index]; case 15: return L_7[index];
      case 16: return U_0[index]; case 17: return U_1[index];
      case 18: return U_2[index]; case 19: return U_3[index];
      case 20: return U_4[index]; case 21: return U_5[index];
      case 22: return U_6[index]; default: return U_7[index];
    }
  };
  auto write_word = [&](const uint32_t bank, const uint32_t index,
                        const WORD512 value) {
    switch (bank) {
      case 8: L_0[index] = value; break; case 9: L_1[index] = value; break;
      case 10: L_2[index] = value; break; case 11: L_3[index] = value; break;
      case 12: L_4[index] = value; break; case 13: L_5[index] = value; break;
      case 14: L_6[index] = value; break; case 15: L_7[index] = value; break;
      case 16: U_0[index] = value; break; case 17: U_1[index] = value; break;
      case 18: U_2[index] = value; break; case 19: U_3[index] = value; break;
      case 20: U_4[index] = value; break; case 21: U_5[index] = value; break;
      case 22: U_6[index] = value; break; case 23: U_7[index] = value; break;
      default: break;
    }
  };

  auto serve_read = [&](ReadReqTagged req,
                        tapa::ostream<ReadPacketTagged> rsp) {
    uint32_t address = req.r.addr;
    uint32_t remaining = req.r.len;
    while (remaining != 0) {
      const uint32_t bank = addr_bank(address);
      const uint32_t off = addr_off(address);
      const uint32_t first = off & 7u;
      const uint32_t capacity = 8u - first;
      const uint32_t count = remaining < capacity ? remaining : capacity;
      const WORD512 word = read_word(bank, off >> 3);
      rsp.write(ReadPacketTagged{
          word, req.pe,
          ap_uint<5>(first), ap_uint<7>(count)});
      address += count;
      remaining -= count;
    }
  };

  auto serve_write = [&](WriteReqTagged req,
                         tapa::istream<WritePacketTagged> elem,
                         tapa::ostream<ap_uint<1>> ack) {
    uint32_t address = req.r.addr;
    uint32_t remaining = req.r.len;
    while (remaining != 0) {
      const WritePacketTagged packet = elem.read();
      const uint32_t count = uint32_t(packet.count);
      write_word(addr_bank(address), addr_off(address) >> 3, packet.v);
      address += count;
      remaining -= count;
    }
    ack.write(1);
  };

  uint32_t rr = 0;
  for (;;) {
    const uint32_t client = rr;
    rr = rr + 1 == PAR_NUM_CLIENT ? 0 : rr + 1;
    if (client < PAR_NUM_FRONT) {
      WriteReqTagged wr{};
      ReadReqTagged rd{};
      if (front_wr_req[client].try_read(wr)) {
        serve_write(wr, front_wr_elem[client], front_wr_ack[client]);
      } else if (front_rd_req[client].try_read(rd)) {
        serve_read(rd, front_rd_rsp[client]);
      }
    } else {
      const uint32_t pe = client - PAR_NUM_FRONT;
      WriteReqTagged wr{};
      ReadReqTagged rd{};
      if (pe_wr_req[pe].try_read(wr)) {
        serve_write(wr, pe_wr_elem[pe], pe_wr_ack[pe]);
      } else if (pe_rd_req[pe].try_read(rd)) {
        serve_read(rd, pe_rd_rsp[pe]);
      }
    }
  }
}

// One independent service per physical compute client.  TAPA host simulation
// can safely share the bank pointers because PLU2 allocates disjoint aligned
// output regions and the scheduler exposes a factor only after its write ACK.
// This retains the hardware's client concurrency instead of serialising all
// 72 requesters behind one polling loop.
void ParallelSimClientMemory(
    const int /*client_slot*/,
    tapa::istream<ReadReqTagged>& rd_req,
    tapa::ostream<ReadPacketTagged>& rd_rsp,
    tapa::istream<WriteReqTagged>& wr_req,
    tapa::istream<WritePacketTagged>& wr_elem,
    tapa::ostream<ap_uint<1>>& wr_ack) {
  for (;;) {
    WriteReqTagged wr{};
    if (wr_req.try_read(wr)) {
      uint32_t address = wr.r.addr;
      uint32_t remaining = wr.r.len;
      while (remaining != 0) {
        const WritePacketTagged packet = wr_elem.read();
        const uint32_t count = uint32_t(packet.count);
        g_sim_bank_ptr[addr_bank(address)][addr_off(address) >> 3] = packet.v;
        address += count;
        remaining -= count;
      }
      wr_ack.write(1);
      continue;
    }

    ReadReqTagged rd{};
    if (!rd_req.try_read(rd)) continue;
    uint32_t address = rd.r.addr;
    uint32_t remaining = rd.r.len;
    while (remaining != 0) {
      const uint32_t bank = addr_bank(address);
      const uint32_t off = addr_off(address);
      const uint32_t first = off & 7u;
      const uint32_t capacity = 8u - first;
      const uint32_t count = remaining < capacity ? remaining : capacity;
      const WORD512 word = g_sim_bank_ptr[bank][off >> 3];
      rd_rsp.write(ReadPacketTagged{
          word, rd.pe,
          ap_uint<5>(first), ap_uint<7>(count)});
      address += count;
      remaining -= count;
    }
  }
}

void SparseLUKernelFastTapaSim(
    tapa::mmap<WORD512> A_0, tapa::mmap<WORD512> A_1,
    tapa::mmap<WORD512> A_2, tapa::mmap<WORD512> A_3,
    tapa::mmap<WORD512> A_4, tapa::mmap<WORD512> A_5,
    tapa::mmap<WORD512> A_6, tapa::mmap<WORD512> A_7,
    tapa::mmap<WORD512> L_0, tapa::mmap<WORD512> L_1,
    tapa::mmap<WORD512> L_2, tapa::mmap<WORD512> L_3,
    tapa::mmap<WORD512> L_4, tapa::mmap<WORD512> L_5,
    tapa::mmap<WORD512> L_6, tapa::mmap<WORD512> L_7,
    tapa::mmap<WORD512> U_0, tapa::mmap<WORD512> U_1,
    tapa::mmap<WORD512> U_2, tapa::mmap<WORD512> U_3,
    tapa::mmap<WORD512> U_4, tapa::mmap<WORD512> U_5,
    tapa::mmap<WORD512> U_6, tapa::mmap<WORD512> U_7,
    tapa::mmap<META_T> meta_sched,
    tapa::mmap<META_T> meta_front,
    tapa::mmap<META_T> meta_segment) {
  g_sim_direct_dependencies = true;
  g_sim_meta_ptr = meta_sched.data();
  ParSimInitNativeBank(0, A_0.data(), A_0.size(), true);
  ParSimInitNativeBank(1, A_1.data(), A_1.size(), true);
  ParSimInitNativeBank(2, A_2.data(), A_2.size(), true);
  ParSimInitNativeBank(3, A_3.data(), A_3.size(), true);
  ParSimInitNativeBank(4, A_4.data(), A_4.size(), true);
  ParSimInitNativeBank(5, A_5.data(), A_5.size(), true);
  ParSimInitNativeBank(6, A_6.data(), A_6.size(), true);
  ParSimInitNativeBank(7, A_7.data(), A_7.size(), true);
  ParSimInitNativeBank(8, L_0.data(), L_0.size(), false);
  ParSimInitNativeBank(9, L_1.data(), L_1.size(), false);
  ParSimInitNativeBank(10, L_2.data(), L_2.size(), false);
  ParSimInitNativeBank(11, L_3.data(), L_3.size(), false);
  ParSimInitNativeBank(12, L_4.data(), L_4.size(), false);
  ParSimInitNativeBank(13, L_5.data(), L_5.size(), false);
  ParSimInitNativeBank(14, L_6.data(), L_6.size(), false);
  ParSimInitNativeBank(15, L_7.data(), L_7.size(), false);
  ParSimInitNativeBank(16, U_0.data(), U_0.size(), false);
  ParSimInitNativeBank(17, U_1.data(), U_1.size(), false);
  ParSimInitNativeBank(18, U_2.data(), U_2.size(), false);
  ParSimInitNativeBank(19, U_3.data(), U_3.size(), false);
  ParSimInitNativeBank(20, U_4.data(), U_4.size(), false);
  ParSimInitNativeBank(21, U_5.data(), U_5.size(), false);
  ParSimInitNativeBank(22, U_6.data(), U_6.size(), false);
  ParSimInitNativeBank(23, U_7.data(), U_7.size(), false);
  const char* exact_control_events =
      std::getenv("SALUT_EXACT_CONTROL_EVENTS");
  if (exact_control_events == nullptr || exact_control_events[0] != '1') {
    ParSimRunControlBatched(meta_sched);
    return;
  }
  tapa::stream<uint32_t, FIFO_COL_DONE> front_ready_id(
      "sim_front_ready_id");
  tapa::stream<uint32_t, FIFO_COL_DONE> segment_ready_id(
      "sim_segment_ready_id");
  tapa::stream<ParColumnTask, FIFO_CTRL> front_task("sim_front_task");
  tapa::stream<ParDepPacket, FIFO_CTRL> front_dep("sim_front_dep");
  tapa::streams<ParColumnTask, PAR_NUM_FRONT, FIFO_CTRL> front_task_spu("sim_front_task_spu");
  tapa::streams<ParDepPacket, PAR_NUM_FRONT, FIFO_CTRL> front_dep_spu("sim_front_dep_spu");
  tapa::streams<uint32_t, PAR_NUM_FRONT, FIFO_COL_DONE> front_done_spu("sim_front_done_spu");
  tapa::stream<uint32_t, FIFO_COL_DONE> front_done("sim_front_done");
  tapa::stream<uint32_t, FIFO_COL_DONE> pivot_ready("sim_pivot_ready");
  tapa::streams<ParSegmentTask, PAR_NUM_PEG, FIFO_CTRL> ready_local_task("sim_ready_local_task");
  tapa::streams<ParDepPacket, PAR_NUM_PEG, FIFO_CTRL> ready_local_dep("sim_ready_local_dep");
  tapa::stream<ParSegmentTask, FIFO_CTRL> ready_global_task("sim_ready_global_task");
  tapa::stream<ParDepPacket, FIFO_CTRL> ready_global_dep("sim_ready_global_dep");
  tapa::streams<ParSegmentTask, PAR_NUM_PE, FIFO_CTRL> seg_task("sim_seg_task");
  tapa::streams<ParDepPacket, PAR_NUM_PE, FIFO_CTRL> seg_dep("sim_seg_dep");
  tapa::streams<uint32_t, PAR_NUM_PE, FIFO_CTRL> seg_done_pe("sim_seg_done_pe");
  tapa::stream<uint32_t, FIFO_COL_DONE> seg_done("sim_seg_done");
  tapa::stream<ParDegreeRequest, FIFO_CTRL> degree_request(
      "sim_degree_request");
  tapa::stream<ParDegreeResponse, FIFO_CTRL> degree_response(
      "sim_degree_response");
  tapa::streams<ParPivotReq, PAR_NUM_PE, FIFO_CTRL> pe_pivot_req("sim_pe_pivot_req");
  tapa::streams<ap_uint<1>, PAR_NUM_PE, FIFO_CTRL> pe_pivot_go("sim_pe_pivot_go");

  tapa::stream<ap_uint<1>, 2> compute_done("sim_compute_done");
  tapa::task()
      .invoke<tapa::detach>(ParallelFrontEmitter,
                            meta_front, front_ready_id,
                            front_task, front_dep)
      .invoke<tapa::detach>(ParallelSegmentEmitter,
                            meta_segment, segment_ready_id,
                            ready_local_task, ready_local_dep,
                            ready_global_task, ready_global_dep)
      .invoke<tapa::detach>(ParallelFrontAllocationUnit, front_task, front_dep,
                            front_task_spu, front_dep_spu)
      .invoke<tapa::detach, PAR_NUM_FRONT>(
                            ParallelColumnFrontSimDirect, tapa::seq(),
                            front_task_spu, front_done_spu)
      .invoke<tapa::detach>(ParallelFrontDoneMux, front_done_spu, front_done)
      .invoke<tapa::detach, PAR_NUM_PE>(
                            ParallelSegmentPESimDirect, tapa::seq(),
                            seg_task, pe_pivot_req,
                            pe_pivot_go, seg_done_pe)
      .invoke<tapa::detach>(ParallelPivotReadGate,
                            pivot_ready, pe_pivot_req, pe_pivot_go)
      .invoke<tapa::detach>(ParallelSegDoneMux, seg_done_pe, seg_done)
      .invoke<tapa::detach>(ParallelTaskAllocationUnit,
                            ready_local_task, ready_local_dep,
                            ready_global_task, ready_global_dep,
                            seg_task, seg_dep)
      .invoke<tapa::detach>(ParallelDegreeStore,
                            degree_request, degree_response)
      .invoke<tapa::join>(ParallelScheduler,
                          meta_sched, front_ready_id,
                          front_done, pivot_ready, segment_ready_id, seg_done,
                          degree_request, degree_response, compute_done);
}
#endif  // __SYNTHESIS__

// Standalone task-level HLS compiles the scheduler/emitter functions directly.
// Vitis HLS 2022.2 still parses otherwise-unreferenced functions and rejects
// the TAPA top-level stream-array constructors below, so hide only the full
// sequential/kernel wrappers for those focused synthesis runs.  Normal host
// builds and TAPA kernel compilation do not define this macro.
#ifndef SALUT_HLS_PARALLEL_TASK_ONLY

constexpr uint32_t SEQ_META_MAGIC_KERNEL=0x534C5531u;
constexpr uint32_t SEQ_META_HEADER_KERNEL=8u;
constexpr uint32_t SEQ_META_DESC_KERNEL=8u;
constexpr uint32_t SEQ_MAX_N=65536u;
constexpr uint32_t SEQ_WORK_BANKS=8u;
constexpr uint32_t SEQ_WORK_BANK_DEPTH=SEQ_MAX_N/SEQ_WORK_BANKS;

static inline META_T SeqReadMeta(tapa::async_mmap<META_T>& mem,uint32_t addr) {
#pragma HLS inline
  bool request_sent=false;
  while(!request_sent) {
#pragma HLS pipeline II=1
    request_sent=mem.read_addr.try_write(addr);
  }
  META_T value=0;
  bool response_received=false;
  while(!response_received) {
#pragma HLS pipeline II=1
    response_received=mem.read_data.try_read(value);
  }
  return value;
}

static inline WORD512 SeqReadWord(tapa::async_mmap<WORD512>& mem,uint32_t addr) {
#pragma HLS inline
  bool request_sent=false;
  while(!request_sent) {
#pragma HLS pipeline II=1
    request_sent=mem.read_addr.try_write(addr);
  }
  WORD512 value=0;
  bool response_received=false;
  while(!response_received) {
#pragma HLS pipeline II=1
    response_received=mem.read_data.try_read(value);
  }
  return value;
}

static inline void SeqWriteWord(tapa::async_mmap<WORD512>& mem,uint32_t addr,
                                WORD512 value) {
#pragma HLS inline
  bool address_sent=false;
  while(!address_sent) {
#pragma HLS pipeline II=1
    address_sent=mem.write_addr.try_write(addr);
  }
  bool data_sent=false;
  while(!data_sent) {
#pragma HLS pipeline II=1
    data_sent=mem.write_data.try_write(value);
  }
  uint8_t response=0;
  bool response_received=false;
  while(!response_received) {
#pragma HLS pipeline II=1
    response_received=mem.write_resp.try_read(response);
  }
}

static inline WORD512 SeqReadA(uint32_t lane,uint32_t addr,
  tapa::async_mmap<WORD512>& A_0,tapa::async_mmap<WORD512>& A_1,
  tapa::async_mmap<WORD512>& A_2,tapa::async_mmap<WORD512>& A_3,
  tapa::async_mmap<WORD512>& A_4,tapa::async_mmap<WORD512>& A_5,
  tapa::async_mmap<WORD512>& A_6,tapa::async_mmap<WORD512>& A_7)
{
#pragma HLS inline
  WORD512 value=0;
  if(lane==0){A_0.read_addr.write(addr);value=A_0.read_data.read();}
  else if(lane==1){A_1.read_addr.write(addr);value=A_1.read_data.read();}
  else if(lane==2){A_2.read_addr.write(addr);value=A_2.read_data.read();}
  else if(lane==3){A_3.read_addr.write(addr);value=A_3.read_data.read();}
  else if(lane==4){A_4.read_addr.write(addr);value=A_4.read_data.read();}
  else if(lane==5){A_5.read_addr.write(addr);value=A_5.read_data.read();}
  else if(lane==6){A_6.read_addr.write(addr);value=A_6.read_data.read();}
  else {A_7.read_addr.write(addr);value=A_7.read_data.read();}
  return value;
}

static inline WORD512 SeqReadL(uint32_t lane,uint32_t addr,
  tapa::async_mmap<WORD512>& L_0,tapa::async_mmap<WORD512>& L_1,
  tapa::async_mmap<WORD512>& L_2,tapa::async_mmap<WORD512>& L_3,
  tapa::async_mmap<WORD512>& L_4,tapa::async_mmap<WORD512>& L_5,
  tapa::async_mmap<WORD512>& L_6,tapa::async_mmap<WORD512>& L_7)
{
#pragma HLS inline
  WORD512 value=0;
  if(lane==0){L_0.read_addr.write(addr);value=L_0.read_data.read();}
  else if(lane==1){L_1.read_addr.write(addr);value=L_1.read_data.read();}
  else if(lane==2){L_2.read_addr.write(addr);value=L_2.read_data.read();}
  else if(lane==3){L_3.read_addr.write(addr);value=L_3.read_data.read();}
  else if(lane==4){L_4.read_addr.write(addr);value=L_4.read_data.read();}
  else if(lane==5){L_5.read_addr.write(addr);value=L_5.read_data.read();}
  else if(lane==6){L_6.read_addr.write(addr);value=L_6.read_data.read();}
  else {L_7.read_addr.write(addr);value=L_7.read_data.read();}
  return value;
}

static inline void SeqWriteL(uint32_t lane,uint32_t addr,WORD512 value,
  tapa::async_mmap<WORD512>& L_0,tapa::async_mmap<WORD512>& L_1,
  tapa::async_mmap<WORD512>& L_2,tapa::async_mmap<WORD512>& L_3,
  tapa::async_mmap<WORD512>& L_4,tapa::async_mmap<WORD512>& L_5,
  tapa::async_mmap<WORD512>& L_6,tapa::async_mmap<WORD512>& L_7)
{
#pragma HLS inline
  if(lane==0){L_0.write_addr.write(addr);L_0.write_data.write(value);(void)L_0.write_resp.read();}
  else if(lane==1){L_1.write_addr.write(addr);L_1.write_data.write(value);(void)L_1.write_resp.read();}
  else if(lane==2){L_2.write_addr.write(addr);L_2.write_data.write(value);(void)L_2.write_resp.read();}
  else if(lane==3){L_3.write_addr.write(addr);L_3.write_data.write(value);(void)L_3.write_resp.read();}
  else if(lane==4){L_4.write_addr.write(addr);L_4.write_data.write(value);(void)L_4.write_resp.read();}
  else if(lane==5){L_5.write_addr.write(addr);L_5.write_data.write(value);(void)L_5.write_resp.read();}
  else if(lane==6){L_6.write_addr.write(addr);L_6.write_data.write(value);(void)L_6.write_resp.read();}
  else {L_7.write_addr.write(addr);L_7.write_data.write(value);(void)L_7.write_resp.read();}
}

static inline void SeqWriteU(uint32_t lane,uint32_t addr,WORD512 value,
  tapa::async_mmap<WORD512>& U_0,tapa::async_mmap<WORD512>& U_1,
  tapa::async_mmap<WORD512>& U_2,tapa::async_mmap<WORD512>& U_3)
{
#pragma HLS inline
  if(lane==0){U_0.write_addr.write(addr);U_0.write_data.write(value);(void)U_0.write_resp.read();}
  else if(lane==1){U_1.write_addr.write(addr);U_1.write_data.write(value);(void)U_1.write_resp.read();}
  else if(lane==2){U_2.write_addr.write(addr);U_2.write_data.write(value);(void)U_2.write_resp.read();}
  else {U_3.write_addr.write(addr);U_3.write_data.write(value);(void)U_3.write_resp.read();}
}

static inline WORD64 SeqPackElement(uint32_t row,REAL value) {
#pragma HLS inline
  WORD64 elem=0;
  elem.range(31,0)=tapa::bit_cast<ap_uint<32>>(value);
  elem.range(63,32)=row;
  return elem;
}

static inline REAL SeqElementValue(WORD64 elem) {
#pragma HLS inline
  return tapa::bit_cast<REAL>(ap_uint<32>(elem.range(31,0)));
}

void SequentialSparseLU(
  tapa::async_mmap<WORD512>& A_0,tapa::async_mmap<WORD512>& A_1,
  tapa::async_mmap<WORD512>& A_2,tapa::async_mmap<WORD512>& A_3,
  tapa::async_mmap<WORD512>& A_4,tapa::async_mmap<WORD512>& A_5,
  tapa::async_mmap<WORD512>& A_6,tapa::async_mmap<WORD512>& A_7,
  tapa::async_mmap<WORD512>& L_0,tapa::async_mmap<WORD512>& L_1,
  tapa::async_mmap<WORD512>& L_2,tapa::async_mmap<WORD512>& L_3,
  tapa::async_mmap<WORD512>& L_4,tapa::async_mmap<WORD512>& L_5,
  tapa::async_mmap<WORD512>& L_6,tapa::async_mmap<WORD512>& L_7,
  tapa::async_mmap<WORD512>& U_0,tapa::async_mmap<WORD512>& U_1,
  tapa::async_mmap<WORD512>& U_2,tapa::async_mmap<WORD512>& U_3,
  tapa::async_mmap<META_T>& meta)
{
  static REAL work0[SEQ_WORK_BANK_DEPTH];
  static REAL work1[SEQ_WORK_BANK_DEPTH];
  static REAL work2[SEQ_WORK_BANK_DEPTH];
  static REAL work3[SEQ_WORK_BANK_DEPTH];
  static REAL work4[SEQ_WORK_BANK_DEPTH];
  static REAL work5[SEQ_WORK_BANK_DEPTH];
  static REAL work6[SEQ_WORK_BANK_DEPTH];
  static REAL work7[SEQ_WORK_BANK_DEPTH];
  static WORD512 l_out_buf[SEQ_WORK_BANK_DEPTH];
  static REAL norm0[SEQ_WORK_BANK_DEPTH];
  static REAL norm1[SEQ_WORK_BANK_DEPTH];
  static REAL norm2[SEQ_WORK_BANK_DEPTH];
  static REAL norm3[SEQ_WORK_BANK_DEPTH];
  static REAL norm4[SEQ_WORK_BANK_DEPTH];
  static REAL norm5[SEQ_WORK_BANK_DEPTH];
  static REAL norm6[SEQ_WORK_BANK_DEPTH];
  static REAL norm7[SEQ_WORK_BANK_DEPTH];
  static uint32_t l_word_base[SEQ_MAX_N];
  static uint32_t l_word_count_cache[SEQ_MAX_N];
#pragma HLS bind_storage variable=work0 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work1 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work2 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work3 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work4 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work5 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work6 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=work7 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=l_out_buf type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm0 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm1 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm2 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm3 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm4 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm5 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm6 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=norm7 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=l_word_base type=RAM_1P impl=BRAM
#pragma HLS bind_storage variable=l_word_count_cache type=RAM_1P impl=BRAM

  const uint32_t magic=SeqReadMeta(meta,0);
  const uint32_t n=SeqReadMeta(meta,1);
  if(magic!=SEQ_META_MAGIC_KERNEL||n>SEQ_MAX_N) return;

  for(uint32_t j=0;j<n;j++) {
#pragma HLS loop_tripcount min=1 max=65536
    const uint32_t desc=SEQ_META_HEADER_KERNEL+j*SEQ_META_DESC_KERNEL;
    const uint32_t a_addr=SeqReadMeta(meta,desc+0);
    const uint32_t a_count=SeqReadMeta(meta,desc+1);
    const uint32_t l_addr=SeqReadMeta(meta,desc+2);
    const uint32_t l_count=SeqReadMeta(meta,desc+3);
    const uint32_t u_addr=SeqReadMeta(meta,desc+4);
    const uint32_t u_count=SeqReadMeta(meta,desc+5);
    const uint32_t l_words=SeqReadMeta(meta,desc+6);
    const uint32_t a_lane=addr_bank(a_addr),a_base=addr_off(a_addr);
    const uint32_t l_lane=addr_bank(l_addr),l_base=addr_off(l_addr);
    const uint32_t u_lane=addr_bank(u_addr),u_base=addr_off(u_addr);
    l_word_base[j]=l_base;
    l_word_count_cache[j]=l_words;

    // 初始化当前符号列（含 fill-in 的零位置）。
    const uint32_t a_words=(a_count+7u)/8u;
    for(uint32_t w=0;w<a_words;w++) {
#pragma HLS loop_tripcount min=1 max=8192
      WORD512 word=SeqReadWord(A_0,a_base+w);
      for(uint32_t slot=0;slot<8;slot++) {
#pragma HLS pipeline II=1
        const uint32_t pos=w*8u+slot;
        WORD64 elem=get_word64(word,slot);
        const uint32_t row=elem.range(63,32);
        if(pos<a_count&&row<n) {
          const uint32_t index=row>>3;
          const REAL value=SeqElementValue(elem);
          switch(row&7u) {
            case 0:work0[index]=value;break;
            case 1:work1[index]=value;break;
            case 2:work2[index]=value;break;
            case 3:work3[index]=value;break;
            case 4:work4[index]=value;break;
            case 5:work5[index]=value;break;
            case 6:work6[index]=value;break;
            default:work7[index]=value;break;
          }
        }
      }
    }

    WORD512 u_word=0;
    uint32_t u_slot=0,u_word_index=0,u_written=0;

    // A 的 row<j 项就是当前列有序的 U 依赖集合。
    for(uint32_t w=0;w<a_words;w++) {
#pragma HLS loop_tripcount min=1 max=8192
      WORD512 a_word=SeqReadWord(A_0,a_base+w);
      for(uint32_t slot=0;slot<8;slot++) {
        const uint32_t pos=w*8u+slot;
        WORD64 a_elem=get_word64(a_word,slot);
        const uint32_t k=a_elem.range(63,32);
        if(pos>=a_count||k>=j) continue;
        const uint32_t k_index=k>>3;
        REAL ukj=0.0f;
        switch(k&7u) {
          case 0:ukj=work0[k_index];break;
          case 1:ukj=work1[k_index];break;
          case 2:ukj=work2[k_index];break;
          case 3:ukj=work3[k_index];break;
          case 4:ukj=work4[k_index];break;
          case 5:ukj=work5[k_index];break;
          case 6:ukj=work6[k_index];break;
          default:ukj=work7[k_index];break;
        }
        set_word64(u_word,u_slot,SeqPackElement(k,ukj));
        u_slot++;u_written++;
        if(u_slot==8) {
          SeqWriteWord(U_0,u_base+u_word_index,u_word);
          u_word=0;u_slot=0;u_word_index++;
        }

        const uint32_t lk_words=l_word_count_cache[k];
        const uint32_t lk_base=l_word_base[k];
        // 连续发出 L word 请求，并在响应到达时并行更新 8 个 work bank。
        // 请求和响应计数分离后，async_mmap 可自动合并为 AXI burst。
        for(uint32_t req=0,resp=0;resp<lk_words;) {
#pragma HLS loop_tripcount min=0 max=8192
#pragma HLS pipeline II=1
#pragma HLS dependence variable=work0 inter false
#pragma HLS dependence variable=work1 inter false
#pragma HLS dependence variable=work2 inter false
#pragma HLS dependence variable=work3 inter false
#pragma HLS dependence variable=work4 inter false
#pragma HLS dependence variable=work5 inter false
#pragma HLS dependence variable=work6 inter false
#pragma HLS dependence variable=work7 inter false
          if(req<lk_words&&L_0.read_addr.try_write(lk_base+req)) req++;
          WORD512 l_word=0;
          if(L_0.read_data.try_read(l_word)) {
            for(uint32_t ls=0;ls<8;ls++) {
#pragma HLS unroll
              WORD64 l_elem=get_word64(l_word,ls);
              const uint32_t row=l_elem.range(63,32);
              if(row<n) {
                const uint32_t index=row>>3;
                const REAL lval=SeqElementValue(l_elem);
                switch(ls) {
                  case 0:work0[index]=work0[index]-lval*ukj;break;
                  case 1:work1[index]=work1[index]-lval*ukj;break;
                  case 2:work2[index]=work2[index]-lval*ukj;break;
                  case 3:work3[index]=work3[index]-lval*ukj;break;
                  case 4:work4[index]=work4[index]-lval*ukj;break;
                  case 5:work5[index]=work5[index]-lval*ukj;break;
                  case 6:work6[index]=work6[index]-lval*ukj;break;
                  default:work7[index]=work7[index]-lval*ukj;break;
                }
              }
            }
            resp++;
          }
        }
      }
    }

    const uint32_t j_index=j>>3;
    REAL pivot=0.0f;
    switch(j&7u) {
      case 0:pivot=work0[j_index];break;
      case 1:pivot=work1[j_index];break;
      case 2:pivot=work2[j_index];break;
      case 3:pivot=work3[j_index];break;
      case 4:pivot=work4[j_index];break;
      case 5:pivot=work5[j_index];break;
      case 6:pivot=work6[j_index];break;
      default:pivot=work7[j_index];break;
    }
    pivot=StabilizeStaticPivot(pivot);
    set_word64(u_word,u_slot,SeqPackElement(j,pivot));
    u_slot++;u_written++;
    if(u_slot!=0) SeqWriteWord(U_0,u_base+u_word_index,u_word);

    // L buffer 初始包含 host 写入的 bank-interleaved 行模板。读取模板、
    // 填入数值并原址写回，避免第三次扫描 A 列。模板读、归一化计算、
    // 结果写分成三个 burst/pipeline 阶段，避免每个 word 阻塞等待 HBM。
    for(uint32_t req=0,resp=0;resp<l_words;) {
#pragma HLS loop_tripcount min=1 max=8192
#pragma HLS pipeline II=1
      if(req<l_words&&L_0.read_addr.try_write(l_base+req)) req++;
      WORD512 l_word_in=0;
      if(L_0.read_data.try_read(l_word_in)) {
        l_out_buf[resp]=l_word_in;
        resp++;
      }
    }

    // Keep IEEE direct-division semantics while feeding one pipelined divider
    // every cycle.  Storing REAL results before ap_uint packing avoids the
    // Vitis 2022.2 false dependence caused by bit_cast/range proxies.
    for(uint32_t pos=0;pos<l_words*8u;pos++) {
#pragma HLS loop_tripcount min=1 max=65536
#pragma HLS pipeline II=1
#pragma HLS dependence variable=norm0 inter false
#pragma HLS dependence variable=norm1 inter false
#pragma HLS dependence variable=norm2 inter false
#pragma HLS dependence variable=norm3 inter false
#pragma HLS dependence variable=norm4 inter false
#pragma HLS dependence variable=norm5 inter false
#pragma HLS dependence variable=norm6 inter false
#pragma HLS dependence variable=norm7 inter false
      const uint32_t lw=pos>>3;
      const uint32_t slot=pos&7u;
      WORD64 elem=get_word64(l_out_buf[lw],slot);
      const uint32_t row=elem.range(63,32);
      REAL value=0.0f;
      if(row<n) {
        const uint32_t index=row>>3;
        REAL numerator=0.0f;
        switch(slot) {
          case 0:numerator=work0[index];break;
          case 1:numerator=work1[index];break;
          case 2:numerator=work2[index];break;
          case 3:numerator=work3[index];break;
          case 4:numerator=work4[index];break;
          case 5:numerator=work5[index];break;
          case 6:numerator=work6[index];break;
          default:numerator=work7[index];break;
        }
        value=numerator/pivot;
      }
      switch(slot) {
        case 0:norm0[lw]=value;break;
        case 1:norm1[lw]=value;break;
        case 2:norm2[lw]=value;break;
        case 3:norm3[lw]=value;break;
        case 4:norm4[lw]=value;break;
        case 5:norm5[lw]=value;break;
        case 6:norm6[lw]=value;break;
        default:norm7[lw]=value;break;
      }
    }

    for(uint32_t lw=0;lw<l_words;lw++) {
#pragma HLS loop_tripcount min=1 max=8192
#pragma HLS pipeline II=1
#pragma HLS dependence variable=l_out_buf inter false
      WORD512 l_word_out=l_out_buf[lw];
      for(uint32_t slot=0;slot<8;slot++) {
#pragma HLS unroll
        WORD64 elem=get_word64(l_word_out,slot);
        const uint32_t row=elem.range(63,32);
        if(row<n) {
          REAL value=0.0f;
          switch(slot) {
            case 0:value=norm0[lw];break;
            case 1:value=norm1[lw];break;
            case 2:value=norm2[lw];break;
            case 3:value=norm3[lw];break;
            case 4:value=norm4[lw];break;
            case 5:value=norm5[lw];break;
            case 6:value=norm6[lw];break;
            default:value=norm7[lw];break;
          }
          set_word64(l_word_out,slot,SeqPackElement(row,value));
        }
      }
      l_out_buf[lw]=l_word_out;
    }

    for(uint32_t req=0,acked=0;acked<l_words;) {
#pragma HLS loop_tripcount min=1 max=8192
#pragma HLS pipeline II=1
      if(req<l_words&&!L_0.write_addr.full()&&!L_0.write_data.full()) {
        L_0.write_addr.write(l_base+req);
        L_0.write_data.write(l_out_buf[req]);
        req++;
      }
      uint8_t response=0;
      if(L_0.write_resp.try_read(response)) acked+=(uint32_t)response+1u;
    }
    (void)l_count;(void)u_count;(void)u_written;
  }
}

void SequentialSparseLUKernelBaseline(
  // A矩阵: 8个HBM通道
  tapa::mmap<ap_uint<512>> A_0,
  tapa::mmap<ap_uint<512>> A_1,
  tapa::mmap<ap_uint<512>> A_2,
  tapa::mmap<ap_uint<512>> A_3,
  tapa::mmap<ap_uint<512>> A_4,
  tapa::mmap<ap_uint<512>> A_5,
  tapa::mmap<ap_uint<512>> A_6,
  tapa::mmap<ap_uint<512>> A_7,
  // L矩阵: 8个HBM通道（读写复用）
  tapa::mmap<ap_uint<512>> L_0,
  tapa::mmap<ap_uint<512>> L_1,
  tapa::mmap<ap_uint<512>> L_2,
  tapa::mmap<ap_uint<512>> L_3,
  tapa::mmap<ap_uint<512>> L_4,
  tapa::mmap<ap_uint<512>> L_5,
  tapa::mmap<ap_uint<512>> L_6,
  tapa::mmap<ap_uint<512>> L_7,
  // U矩阵: 4个HBM通道（读写复用）
  tapa::mmap<ap_uint<512>> U_0,
  tapa::mmap<ap_uint<512>> U_1,
  tapa::mmap<ap_uint<512>> U_2,
  tapa::mmap<ap_uint<512>> U_3,
  tapa::mmap<META_T> META_mmap                         
) {
  tapa::task()
    .invoke(SequentialSparseLU,
            A_0,A_1,A_2,A_3,A_4,A_5,A_6,A_7,
            L_0,L_1,L_2,L_3,L_4,L_5,L_6,L_7,
            U_0,U_1,U_2,U_3,META_mmap);
}

void SparseLUKernel(
  tapa::mmap<ap_uint<512>> A_0,
  tapa::mmap<ap_uint<512>> A_1,
  tapa::mmap<ap_uint<512>> A_2,
  tapa::mmap<ap_uint<512>> A_3,
  tapa::mmap<ap_uint<512>> A_4,
  tapa::mmap<ap_uint<512>> A_5,
  tapa::mmap<ap_uint<512>> A_6,
  tapa::mmap<ap_uint<512>> A_7,
  tapa::mmap<ap_uint<512>> L_0,
  tapa::mmap<ap_uint<512>> L_1,
  tapa::mmap<ap_uint<512>> L_2,
  tapa::mmap<ap_uint<512>> L_3,
  tapa::mmap<ap_uint<512>> L_4,
  tapa::mmap<ap_uint<512>> L_5,
  tapa::mmap<ap_uint<512>> L_6,
  tapa::mmap<ap_uint<512>> L_7,
  tapa::mmap<ap_uint<512>> U_0,
  tapa::mmap<ap_uint<512>> U_1,
  tapa::mmap<ap_uint<512>> U_2,
  tapa::mmap<ap_uint<512>> U_3,
  tapa::mmap<ap_uint<512>> U_4,
  tapa::mmap<ap_uint<512>> U_5,
  tapa::mmap<ap_uint<512>> U_6,
  tapa::mmap<ap_uint<512>> U_7,
  tapa::mmap<META_T> META_SCHED_mmap,
  tapa::mmap<META_T> META_FRONT_mmap,
  tapa::mmap<META_T> META_SEGMENT_mmap) {
#ifndef __SYNTHESIS__
  // Default to full-matrix batched functional execution for host simulation.
  // It coalesces control transport but consumes every PLU2 task/dependency and
  // preserves all bank accesses and FP32 operations.  The two event-accurate
  // levels remain selectable for focused protocol cross-checks.
  const char* exact_memory_events = std::getenv("SALUT_EXACT_MEMORY_EVENTS");
  if (exact_memory_events == nullptr || exact_memory_events[0] != '1') {
    const char* exact_control = std::getenv("SALUT_EXACT_CONTROL_EVENTS");
    std::cerr << ((exact_control != nullptr && exact_control[0] == '1')
        ? "[SALUT sim] stream-exact compute-control simulation "
          "with direct logical 24-bank accesses"
        : "[SALUT sim] full-matrix PLU2 functional simulation with batched "
          "control transport and direct 24-bank accesses")
              << "; logical " << PAR_NUM_FRONT << "-front/" << PAR_NUM_PE << "-PE ("
              << PAR_NUM_PEG << " PEG x " << PAR_PE_PER_PEG << ") allocation"
              << ((exact_control != nullptr && exact_control[0] == '1')
                  ? "."
                  : "; set SALUT_EXACT_CONTROL_EVENTS=1 for per-stream control "
                    "events or SALUT_EXACT_MEMORY_EVENTS=1 for full LM/SM/HBM events.")
              << std::endl;
    SparseLUKernelFastTapaSim(
        A_0, A_1, A_2, A_3, A_4, A_5, A_6, A_7,
        L_0, L_1, L_2, L_3, L_4, L_5, L_6, L_7,
        U_0, U_1, U_2, U_3, U_4, U_5, U_6, U_7,
        META_SCHED_mmap, META_FRONT_mmap, META_SEGMENT_mmap);
    return;
  }
  std::cerr << "[SALUT sim] exact LM/SM/24-HBM transport event model enabled."
            << std::endl;
  for (int owner = 0; owner < 8; ++owner) {
    salut_sm_stats[owner] = {}; salut_lm_stats[owner] = {};
  }
  g_sim_direct_dependencies = false;
  g_sim_meta_ptr = nullptr;
  g_front_sm_hits = 0;
  g_front_sm_misses = 0;
  g_hbm_req_issued = 0;
  g_hbm_rsp_routed = 0;
  g_hbm_bank_rsp = 0;
  g_hbm_lane_rsp = 0;
  g_sm_fill_rsp = 0;
  g_lm_rsp = 0;
  g_hbm_req_max_parallel = 0;
  g_hbm_rsp_max_parallel = 0;
#endif
  tapa::stream<uint32_t, FIFO_COL_DONE> front_ready_id("front_ready_id");
  tapa::stream<uint32_t, FIFO_COL_DONE> segment_ready_id("segment_ready_id");
  tapa::stream<uint32_t, FIFO_COL_DONE> front_done("front_done");
  tapa::stream<uint32_t, FIFO_COL_DONE> pivot_ready("pivot_ready");
  tapa::stream<uint32_t, FIFO_COL_DONE> seg_done("seg_done");
  tapa::stream<ParColumnTask, FIFO_CTRL> front_task("front_task");
  tapa::stream<ParDepPacket, FIFO_CTRL> front_dep("front_dep");
  tapa::stream<ParSegmentTask, FIFO_CTRL> ready_global_task("ready_global_task");
  tapa::stream<ParDepPacket, FIFO_CTRL> ready_global_dep("ready_global_dep");
  tapa::stream<ParDegreeRequest, FIFO_CTRL> degree_request("degree_request");
  tapa::stream<ParDegreeResponse, FIFO_CTRL> degree_response("degree_response");
  tapa::streams<ParColumnTask, PAR_NUM_FRONT, FIFO_CTRL> front_task_spu("front_task_spu");
  tapa::streams<ParDepPacket, PAR_NUM_FRONT, FIFO_CTRL> front_dep_spu("front_dep_spu");
  tapa::streams<uint32_t, PAR_NUM_FRONT, FIFO_COL_DONE> front_done_spu("front_done_spu");
  tapa::streams<ParSegmentTask, PAR_NUM_PEG, FIFO_CTRL> ready_local_task("ready_local_task");
  tapa::streams<ParDepPacket, PAR_NUM_PEG, FIFO_CTRL> ready_local_dep("ready_local_dep");
  tapa::streams<ParSegmentTask, PAR_NUM_PE, FIFO_CTRL> seg_task("seg_task");
  tapa::streams<ParDepPacket, PAR_NUM_PE, FIFO_CTRL> seg_dep("seg_dep");
  tapa::streams<uint32_t, PAR_NUM_PE, FIFO_CTRL> seg_done_pe("seg_done_pe");
  tapa::streams<RR, PAR_NUM_FRONT, FIFO_CTRL> front_rd_req_spu("front_rd_req_spu");
  tapa::streams<RP, PAR_NUM_FRONT, FIFO_DATA_IN> front_rd_rsp_spu("front_rd_rsp_spu");
  tapa::streams<WR, PAR_NUM_FRONT, FIFO_CTRL> front_wr_req_spu("front_wr_req_spu");
  tapa::streams<WP, PAR_NUM_FRONT, FIFO_DATA_OUT> front_wr_elem_spu("front_wr_elem_spu");
  tapa::streams<RR, PAR_NUM_PE, FIFO_CTRL> pe_rd_req("pe_rd_req");
  tapa::streams<RP, PAR_NUM_PE, FIFO_DATA_IN> pe_rd_rsp("pe_rd_rsp");
  tapa::streams<WR, PAR_NUM_PE, FIFO_CTRL> pe_wr_req("pe_wr_req");
  tapa::streams<WP, PAR_NUM_PE, FIFO_DATA_OUT> pe_wr_elem("pe_wr_elem");
  tapa::streams<ParPivotReq, PAR_NUM_PE, FIFO_CTRL> pe_pivot_req("pe_pivot_req");
  tapa::streams<ap_uint<1>, PAR_NUM_PE, FIFO_CTRL> pe_pivot_go("pe_pivot_go");
  tapa::streams<ap_uint<1>, PAR_NUM_FRONT, FIFO_CTRL> front_wr_ack_spu("front_wr_ack_spu");
  tapa::streams<ap_uint<8>, PAR_NUM_PEG, FIFO_CTRL> peg_wr_ack("peg_wr_ack");
  tapa::streams<ap_uint<1>, PAR_NUM_PE, FIFO_CTRL> pe_wr_ack("pe_wr_ack");
  tapa::streams<SalutReadEnvelope, 8, 4> peg_lower_req("peg_lower_req");
  tapa::streams<SalutReadEnvelope, 8, 4> front_lower_req("front_lower_req");
  tapa::streams<RP, 8, 4> peg_lower_rsp("peg_lower_rsp");
  tapa::streams<SalutCacheWriteReq, 8, 4> peg_global_wr("peg_global_wr");
  tapa::streams<WP, 8, 4> peg_global_data("peg_global_data");
  tapa::streams<SalutCacheWriteReq, 8, 4> lm_snoop_wr("lm_snoop_wr");
  tapa::streams<WP, 8, 4> lm_snoop_data("lm_snoop_data");
  tapa::streams<SalutCacheWriteReq, 8, 4> front_global_wr("front_global_wr");
  tapa::streams<WP, 8, 4> front_global_data("front_global_data");
  tapa::streams<SalutCacheWriteReq, 8, 4> lane_wr("lane_wr");
  tapa::streams<WP, 8, 4> lane_data("lane_data");
  tapa::streams<SalutReadEnvelope, 8, 4> factor_rd("factor_rd");
  tapa::streams<SalutReadEnvelope, 8, 4> a_rd("a_rd");
  tapa::streams<SalutResponseEnvelope, 8, 4> factor_rsp("factor_rsp");
  tapa::streams<SalutResponseEnvelope, 8, 4> a_rsp("a_rsp");
  tapa::streams<SalutWordReadReq, 8, 4> a_word_rd("a_word_rd");
  tapa::streams<SalutWordReadRsp, 8, 4> a_word_rsp("a_word_rsp");
  tapa::streams<SalutPublishAck, 8, 4> sm_published("sm_published");
  tapa::streams<SalutPublishAck, 8, 4> lm_installed("lm_installed");
  tapa::streams<ap_uint<3>, 8, 4> bank_drained("bank_drained");
  tapa::streams<ap_uint<3>, 8, 4> response_ended("response_ended");
  tapa::streams<SalutWordReadReq, 16, 4> factor_word_rd("factor_word_rd");
  tapa::streams<SalutWordReadRsp, 16, 4> factor_word_rsp("factor_word_rsp");
  tapa::streams<SalutWritebackWord, 16, 4> factor_writeback("factor_writeback");
  tapa::streams<SalutWritebackDone, 16, 4> factor_write_done("factor_write_done");
  tapa::streams<SalutReadEnvelope, 8, 4> rd_omega_0("rd_omega_0");
  tapa::streams<SalutReadEnvelope, 8, 4> rd_omega_1("rd_omega_1");
  tapa::streams<SalutReadEnvelope, 8, 4> rd_omega_2("rd_omega_2");
  tapa::streams<SalutReadEnvelope, 8, 4> rd_omega_3("rd_omega_3");
  tapa::streams<SalutWriteBeat, 8, 4> wr_omega_0("wr_omega_0");
  tapa::streams<SalutWriteBeat, 8, 4> wr_omega_1("wr_omega_1");
  tapa::streams<SalutWriteBeat, 8, 4> wr_omega_2("wr_omega_2");
  tapa::streams<SalutWriteBeat, 8, 4> wr_omega_3("wr_omega_3");
  tapa::streams<SalutResponseEnvelope, 8, 4> rsp_omega_0("rsp_omega_0");
  tapa::streams<SalutResponseEnvelope, 8, 4> rsp_omega_1("rsp_omega_1");
  tapa::streams<SalutResponseEnvelope, 8, 4> rsp_omega_2("rsp_omega_2");
  tapa::streams<SalutResponseEnvelope, 8, 4> rsp_omega_3("rsp_omega_3");
  tapa::streams<ap_uint<1>, 8, 2> peg_rd_close("peg_rd_close");
  tapa::streams<ap_uint<1>, 8, 2> peg_wr_close("peg_wr_close");
  tapa::streams<ap_uint<1>, 8, 2> front_rd_close("front_rd_close");
  tapa::streams<ap_uint<1>, 8, 2> front_wr_close("front_wr_close");
  tapa::stream<ap_uint<1>, 2> compute_done("compute_done");
  tapa::stream<ap_uint<1>, 2> publish_close("publish_close");
  tapa::stream<ap_uint<1>, 2> join_close("join_close");
  tapa::stream<ap_uint<1>, 2> publish_drained("publish_drained");
  tapa::stream<SalutPublishAck, 4> published_ack("published_ack");
  tapa::stream<ap_uint<8>, FIFO_CTRL> wr_ack("wr_ack");

  // Cross-SLR 4:1 gearboxes (generated by insert_cross_slr_gearboxes_4to1.py).
  tapa::stream<SalutResponseEnvelope, 4> a_rsp_tx6("a_rsp_tx6");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> a_rsp_gb6("a_rsp_gb6");
  tapa::stream<SalutResponseEnvelope, 4> a_rsp_tx7("a_rsp_tx7");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> a_rsp_gb7("a_rsp_gb7");
  tapa::stream<SalutWordReadRsp, 4> a_word_rsp_tx1("a_word_rsp_tx1");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> a_word_rsp_gb1("a_word_rsp_gb1");
  tapa::stream<SalutWordReadRsp, 4> a_word_rsp_tx2("a_word_rsp_tx2");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> a_word_rsp_gb2("a_word_rsp_gb2");
  tapa::stream<SalutWordReadRsp, 4> a_word_rsp_tx3("a_word_rsp_tx3");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> a_word_rsp_gb3("a_word_rsp_gb3");
  tapa::stream<SalutWordReadRsp, 4> a_word_rsp_tx4("a_word_rsp_tx4");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> a_word_rsp_gb4("a_word_rsp_gb4");
  tapa::stream<SalutResponseEnvelope, 4> factor_rsp_rx6("factor_rsp_rx6");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> factor_rsp_gb6("factor_rsp_gb6");
  tapa::stream<SalutResponseEnvelope, 4> factor_rsp_rx7("factor_rsp_rx7");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> factor_rsp_gb7("factor_rsp_gb7");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx2("factor_word_rsp_tx2");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb2("factor_word_rsp_gb2");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx3("factor_word_rsp_tx3");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb3("factor_word_rsp_gb3");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx4("factor_word_rsp_tx4");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb4("factor_word_rsp_gb4");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx5("factor_word_rsp_tx5");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb5("factor_word_rsp_gb5");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx6("factor_word_rsp_tx6");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb6("factor_word_rsp_gb6");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx7("factor_word_rsp_tx7");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb7("factor_word_rsp_gb7");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx8("factor_word_rsp_tx8");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb8("factor_word_rsp_gb8");
  tapa::stream<SalutWordReadRsp, 4> factor_word_rsp_tx9("factor_word_rsp_tx9");
  tapa::stream<SalutGearBeat<SalutWordReadRsp, 4>, 8> factor_word_rsp_gb9("factor_word_rsp_gb9");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx2("factor_writeback_rx2");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb2("factor_writeback_gb2");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx3("factor_writeback_rx3");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb3("factor_writeback_gb3");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx4("factor_writeback_rx4");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb4("factor_writeback_gb4");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx5("factor_writeback_rx5");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb5("factor_writeback_gb5");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx6("factor_writeback_rx6");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb6("factor_writeback_gb6");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx7("factor_writeback_rx7");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb7("factor_writeback_gb7");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx8("factor_writeback_rx8");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb8("factor_writeback_gb8");
  tapa::stream<SalutWritebackWord, 4> factor_writeback_rx9("factor_writeback_rx9");
  tapa::stream<SalutGearBeat<SalutWritebackWord, 4>, 8> factor_writeback_gb9("factor_writeback_gb9");
  tapa::stream<WP, FIFO_DATA_OUT> front_wr_elem_spu_rx1("front_wr_elem_spu_rx1");
  tapa::stream<SalutGearBeat<WP, 4>, 8> front_wr_elem_spu_gb1("front_wr_elem_spu_gb1");
  tapa::stream<RP, 4> peg_lower_rsp_tx1("peg_lower_rsp_tx1");
  tapa::stream<SalutGearBeat<RP, 4>, 8> peg_lower_rsp_gb1("peg_lower_rsp_gb1");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_0_rx0("rsp_omega_0_rx0");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_0_gb0("rsp_omega_0_gb0");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_0_rx5("rsp_omega_0_rx5");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_0_gb5("rsp_omega_0_gb5");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_2_tx2("rsp_omega_2_tx2");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_2_gb2("rsp_omega_2_gb2");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_2_tx3("rsp_omega_2_tx3");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_2_gb3("rsp_omega_2_gb3");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_2_tx6("rsp_omega_2_tx6");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_2_gb6("rsp_omega_2_gb6");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_2_tx7("rsp_omega_2_tx7");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_2_gb7("rsp_omega_2_gb7");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_3_tx0("rsp_omega_3_tx0");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_3_gb0("rsp_omega_3_gb0");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_3_tx4("rsp_omega_3_tx4");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_3_gb4("rsp_omega_3_gb4");
  tapa::stream<SalutResponseEnvelope, 4> rsp_omega_3_tx5("rsp_omega_3_tx5");
  tapa::stream<SalutGearBeat<SalutResponseEnvelope, 4>, 8> rsp_omega_3_gb5("rsp_omega_3_gb5");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx0("wr_omega_0_rx0");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb0("wr_omega_0_gb0");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx1("wr_omega_0_rx1");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb1("wr_omega_0_gb1");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx2("wr_omega_0_rx2");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb2("wr_omega_0_gb2");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx3("wr_omega_0_rx3");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb3("wr_omega_0_gb3");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx4("wr_omega_0_rx4");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb4("wr_omega_0_gb4");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx5("wr_omega_0_rx5");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb5("wr_omega_0_gb5");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx6("wr_omega_0_rx6");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb6("wr_omega_0_gb6");
  tapa::stream<SalutWriteBeat, 4> wr_omega_0_rx7("wr_omega_0_rx7");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_0_gb7("wr_omega_0_gb7");
  tapa::stream<SalutWriteBeat, 4> wr_omega_3_tx0("wr_omega_3_tx0");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_3_gb0("wr_omega_3_gb0");
  tapa::stream<SalutWriteBeat, 4> wr_omega_3_tx5("wr_omega_3_tx5");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_3_gb5("wr_omega_3_gb5");
  tapa::stream<SalutWriteBeat, 4> wr_omega_3_tx6("wr_omega_3_tx6");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_3_gb6("wr_omega_3_gb6");
  tapa::stream<SalutWriteBeat, 4> wr_omega_3_tx7("wr_omega_3_tx7");
  tapa::stream<SalutGearBeat<SalutWriteBeat, 4>, 8> wr_omega_3_gb7("wr_omega_3_gb7");
  tapa::task()
    .invoke<tapa::detach>(ParallelFrontEmitter,
                          META_FRONT_mmap, front_ready_id, front_task, front_dep)
    .invoke<tapa::detach>(ParallelSegmentEmitter,
                          META_SEGMENT_mmap, segment_ready_id, ready_local_task, ready_local_dep, ready_global_task, ready_global_dep)
    .invoke<tapa::detach>(ParallelFrontAllocationUnit,
                          front_task, front_dep, front_task_spu, front_dep_spu)
    .invoke<tapa::detach, PAR_NUM_FRONT>(ParallelColumnFront,
                          tapa::seq(), front_task_spu, front_dep_spu, front_rd_req_spu, front_rd_rsp_spu, front_wr_req_spu, front_wr_elem_spu, front_wr_ack_spu, front_done_spu)
    .invoke<tapa::detach>(ParallelFrontDoneMux,
                          front_done_spu, front_done)
    .invoke<tapa::detach, PAR_NUM_PE>(ParallelSegmentPE,
                          tapa::seq(), seg_task, seg_dep, pe_rd_req, pe_pivot_req, pe_pivot_go, pe_rd_rsp, pe_wr_req, pe_wr_elem, pe_wr_ack, seg_done_pe)
    .invoke<tapa::detach>(ParallelPivotReadGate,
                          pivot_ready, pe_pivot_req, pe_pivot_go)
    .invoke<tapa::detach, 8>(SalutLMLane,
                          pe_rd_req, pe_rd_rsp, peg_lower_req, peg_lower_rsp, lm_snoop_wr, lm_snoop_data, lm_installed, peg_rd_close)
    .invoke<tapa::detach, 8>(SalutPegWriteFork,
                          tapa::seq(), pe_wr_req, pe_wr_elem, peg_global_wr, peg_global_data, lm_snoop_wr, lm_snoop_data, peg_wr_close)
    .invoke<tapa::detach, 8>(SalutFrontReadSource,
                          tapa::seq(), front_rd_req_spu, front_lower_req, front_rd_close)
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          0, front_wr_req_spu[0], front_wr_elem_spu[0], front_global_wr[0], front_global_data[0], front_wr_close[0])
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          1, front_wr_req_spu[1], front_wr_elem_spu_rx1, front_global_wr[1], front_global_data[1], front_wr_close[1])
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          2, front_wr_req_spu[2], front_wr_elem_spu[2], front_global_wr[2], front_global_data[2], front_wr_close[2])
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          3, front_wr_req_spu[3], front_wr_elem_spu[3], front_global_wr[3], front_global_data[3], front_wr_close[3])
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          4, front_wr_req_spu[4], front_wr_elem_spu[4], front_global_wr[4], front_global_data[4], front_wr_close[4])
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          5, front_wr_req_spu[5], front_wr_elem_spu[5], front_global_wr[5], front_global_data[5], front_wr_close[5])
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          6, front_wr_req_spu[6], front_wr_elem_spu[6], front_global_wr[6], front_global_data[6], front_wr_close[6])
    .invoke<tapa::detach>(SalutFrontWriteSource,
                          7, front_wr_req_spu[7], front_wr_elem_spu[7], front_global_wr[7], front_global_data[7], front_wr_close[7])
    .invoke<tapa::detach, 8>(SalutReadLaneMerge,
                          peg_lower_req, front_lower_req, rd_omega_0)
    .invoke<tapa::detach, 8>(SalutWriteLaneMerge,
                          peg_global_wr, peg_global_data, front_global_wr, front_global_data, lane_wr, lane_data)
    .invoke<tapa::detach, 8>(SalutWriteBeatizer,
                          lane_wr, lane_data, wr_omega_0)
    // rd: three 4-switch stages, perfect shuffle between stages.
    .invoke<tapa::detach>(SalutReadSwitch2, 0,
                          rd_omega_0[0], rd_omega_0[1], rd_omega_1[0], rd_omega_1[2])
    .invoke<tapa::detach>(SalutReadSwitch2, 0,
                          rd_omega_0[2], rd_omega_0[3], rd_omega_1[4], rd_omega_1[6])
    .invoke<tapa::detach>(SalutReadSwitch2, 0,
                          rd_omega_0[4], rd_omega_0[5], rd_omega_1[1], rd_omega_1[3])
    .invoke<tapa::detach>(SalutReadSwitch2, 0,
                          rd_omega_0[6], rd_omega_0[7], rd_omega_1[5], rd_omega_1[7])
    .invoke<tapa::detach>(SalutReadSwitch2, 1,
                          rd_omega_1[0], rd_omega_1[1], rd_omega_2[0], rd_omega_2[2])
    .invoke<tapa::detach>(SalutReadSwitch2, 1,
                          rd_omega_1[2], rd_omega_1[3], rd_omega_2[4], rd_omega_2[6])
    .invoke<tapa::detach>(SalutReadSwitch2, 1,
                          rd_omega_1[4], rd_omega_1[5], rd_omega_2[1], rd_omega_2[3])
    .invoke<tapa::detach>(SalutReadSwitch2, 1,
                          rd_omega_1[6], rd_omega_1[7], rd_omega_2[5], rd_omega_2[7])
    .invoke<tapa::detach>(SalutReadSwitch2, 2,
                          rd_omega_2[0], rd_omega_2[1], rd_omega_3[0], rd_omega_3[1])
    .invoke<tapa::detach>(SalutReadSwitch2, 2,
                          rd_omega_2[2], rd_omega_2[3], rd_omega_3[2], rd_omega_3[3])
    .invoke<tapa::detach>(SalutReadSwitch2, 2,
                          rd_omega_2[4], rd_omega_2[5], rd_omega_3[4], rd_omega_3[5])
    .invoke<tapa::detach>(SalutReadSwitch2, 2,
                          rd_omega_2[6], rd_omega_2[7], rd_omega_3[6], rd_omega_3[7])
    // wr: three 4-switch stages, perfect shuffle between stages.
    .invoke<tapa::detach>(SalutWriteSwitch2, 0,
                          wr_omega_0_rx0, wr_omega_0_rx1, wr_omega_1[0], wr_omega_1[2])
    .invoke<tapa::detach>(SalutWriteSwitch2, 0,
                          wr_omega_0_rx2, wr_omega_0_rx3, wr_omega_1[4], wr_omega_1[6])
    .invoke<tapa::detach>(SalutWriteSwitch2, 0,
                          wr_omega_0_rx4, wr_omega_0_rx5, wr_omega_1[1], wr_omega_1[3])
    .invoke<tapa::detach>(SalutWriteSwitch2, 0,
                          wr_omega_0_rx6, wr_omega_0_rx7, wr_omega_1[5], wr_omega_1[7])
    .invoke<tapa::detach>(SalutWriteSwitch2, 1,
                          wr_omega_1[0], wr_omega_1[1], wr_omega_2[0], wr_omega_2[2])
    .invoke<tapa::detach>(SalutWriteSwitch2, 1,
                          wr_omega_1[2], wr_omega_1[3], wr_omega_2[4], wr_omega_2[6])
    .invoke<tapa::detach>(SalutWriteSwitch2, 1,
                          wr_omega_1[4], wr_omega_1[5], wr_omega_2[1], wr_omega_2[3])
    .invoke<tapa::detach>(SalutWriteSwitch2, 1,
                          wr_omega_1[6], wr_omega_1[7], wr_omega_2[5], wr_omega_2[7])
    .invoke<tapa::detach>(SalutWriteSwitch2, 2,
                          wr_omega_2[0], wr_omega_2[1], wr_omega_3_tx0, wr_omega_3[1])
    .invoke<tapa::detach>(SalutWriteSwitch2, 2,
                          wr_omega_2[2], wr_omega_2[3], wr_omega_3[2], wr_omega_3[3])
    .invoke<tapa::detach>(SalutWriteSwitch2, 2,
                          wr_omega_2[4], wr_omega_2[5], wr_omega_3[4], wr_omega_3_tx5)
    .invoke<tapa::detach>(SalutWriteSwitch2, 2,
                          wr_omega_2[6], wr_omega_2[7], wr_omega_3_tx6, wr_omega_3_tx7)
    // rsp: three 4-switch stages, perfect shuffle between stages.
    .invoke<tapa::detach>(SalutResponseSwitch2, 0,
                          rsp_omega_0_rx0, rsp_omega_0[1], rsp_omega_1[0], rsp_omega_1[2])
    .invoke<tapa::detach>(SalutResponseSwitch2, 0,
                          rsp_omega_0[2], rsp_omega_0[3], rsp_omega_1[4], rsp_omega_1[6])
    .invoke<tapa::detach>(SalutResponseSwitch2, 0,
                          rsp_omega_0[4], rsp_omega_0_rx5, rsp_omega_1[1], rsp_omega_1[3])
    .invoke<tapa::detach>(SalutResponseSwitch2, 0,
                          rsp_omega_0[6], rsp_omega_0[7], rsp_omega_1[5], rsp_omega_1[7])
    .invoke<tapa::detach>(SalutResponseSwitch2, 1,
                          rsp_omega_1[0], rsp_omega_1[1], rsp_omega_2[0], rsp_omega_2_tx2)
    .invoke<tapa::detach>(SalutResponseSwitch2, 1,
                          rsp_omega_1[2], rsp_omega_1[3], rsp_omega_2[4], rsp_omega_2_tx6)
    .invoke<tapa::detach>(SalutResponseSwitch2, 1,
                          rsp_omega_1[4], rsp_omega_1[5], rsp_omega_2[1], rsp_omega_2_tx3)
    .invoke<tapa::detach>(SalutResponseSwitch2, 1,
                          rsp_omega_1[6], rsp_omega_1[7], rsp_omega_2[5], rsp_omega_2_tx7)
    .invoke<tapa::detach>(SalutResponseSwitch2, 2,
                          rsp_omega_2[0], rsp_omega_2[1], rsp_omega_3_tx0, rsp_omega_3[1])
    .invoke<tapa::detach>(SalutResponseSwitch2, 2,
                          rsp_omega_2[2], rsp_omega_2[3], rsp_omega_3[2], rsp_omega_3[3])
    .invoke<tapa::detach>(SalutResponseSwitch2, 2,
                          rsp_omega_2[4], rsp_omega_2[5], rsp_omega_3_tx4, rsp_omega_3_tx5)
    .invoke<tapa::detach>(SalutResponseSwitch2, 2,
                          rsp_omega_2[6], rsp_omega_2[7], rsp_omega_3[6], rsp_omega_3[7])
    .invoke<tapa::detach, 8>(SalutRequestBankSplit,
                          tapa::seq(), rd_omega_3, factor_rd, a_rd)
    .invoke<tapa::detach, 8>(SalutSmBank,
                          tapa::seq(), factor_rd, wr_omega_3, factor_rsp, factor_word_rd, factor_word_rsp, factor_writeback, factor_write_done, sm_published, bank_drained)
    .invoke<tapa::detach>(SalutABurstAdapter,
                          0, a_rd[0], a_word_rd[0], a_word_rsp[0], a_rsp[0])
    .invoke<tapa::detach>(SalutABurstAdapter,
                          1, a_rd[1], a_word_rd[1], a_word_rsp[1], a_rsp[1])
    .invoke<tapa::detach>(SalutABurstAdapter,
                          2, a_rd[2], a_word_rd[2], a_word_rsp[2], a_rsp[2])
    .invoke<tapa::detach>(SalutABurstAdapter,
                          3, a_rd[3], a_word_rd[3], a_word_rsp[3], a_rsp[3])
    .invoke<tapa::detach>(SalutABurstAdapter,
                          4, a_rd[4], a_word_rd[4], a_word_rsp[4], a_rsp[4])
    .invoke<tapa::detach>(SalutABurstAdapter,
                          5, a_rd[5], a_word_rd[5], a_word_rsp[5], a_rsp[5])
    .invoke<tapa::detach>(SalutABurstAdapter,
                          6, a_rd[6], a_word_rd[6], a_word_rsp[6], a_rsp_tx6)
    .invoke<tapa::detach>(SalutABurstAdapter,
                          7, a_rd[7], a_word_rd[7], a_word_rsp[7], a_rsp_tx7)
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp[0], a_rsp[0], rsp_omega_0[0])
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp[1], a_rsp[1], rsp_omega_0[1])
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp[2], a_rsp[2], rsp_omega_0[2])
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp[3], a_rsp[3], rsp_omega_0[3])
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp[4], a_rsp[4], rsp_omega_0[4])
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp[5], a_rsp[5], rsp_omega_0[5])
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp_rx6, a_rsp[6], rsp_omega_0[6])
    .invoke<tapa::detach>(SalutResponseLaneMerge,
                          factor_rsp_rx7, a_rsp[7], rsp_omega_0[7])
    .invoke<tapa::detach>(SalutResponseEgress,
                          0, rsp_omega_3[0], peg_lower_rsp[0], front_rd_rsp_spu[0], response_ended[0])
    .invoke<tapa::detach>(SalutResponseEgress,
                          1, rsp_omega_3[1], peg_lower_rsp_tx1, front_rd_rsp_spu[1], response_ended[1])
    .invoke<tapa::detach>(SalutResponseEgress,
                          2, rsp_omega_3[2], peg_lower_rsp[2], front_rd_rsp_spu[2], response_ended[2])
    .invoke<tapa::detach>(SalutResponseEgress,
                          3, rsp_omega_3[3], peg_lower_rsp[3], front_rd_rsp_spu[3], response_ended[3])
    .invoke<tapa::detach>(SalutResponseEgress,
                          4, rsp_omega_3[4], peg_lower_rsp[4], front_rd_rsp_spu[4], response_ended[4])
    .invoke<tapa::detach>(SalutResponseEgress,
                          5, rsp_omega_3[5], peg_lower_rsp[5], front_rd_rsp_spu[5], response_ended[5])
    .invoke<tapa::detach>(SalutResponseEgress,
                          6, rsp_omega_3[6], peg_lower_rsp[6], front_rd_rsp_spu[6], response_ended[6])
    .invoke<tapa::detach>(SalutResponseEgress,
                          7, rsp_omega_3[7], peg_lower_rsp[7], front_rd_rsp_spu[7], response_ended[7])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_0, a_word_rd[0], a_word_rsp[0])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_0, factor_word_rd[0], factor_word_rsp[0], factor_writeback[0], factor_write_done[0])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_0, factor_word_rd[1], factor_word_rsp[1], factor_writeback[1], factor_write_done[1])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_1, a_word_rd[1], a_word_rsp_tx1)
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_1, factor_word_rd[2], factor_word_rsp_tx2, factor_writeback_rx2, factor_write_done[2])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_1, factor_word_rd[3], factor_word_rsp_tx3, factor_writeback_rx3, factor_write_done[3])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_2, a_word_rd[2], a_word_rsp_tx2)
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_2, factor_word_rd[4], factor_word_rsp_tx4, factor_writeback_rx4, factor_write_done[4])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_2, factor_word_rd[5], factor_word_rsp_tx5, factor_writeback_rx5, factor_write_done[5])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_3, a_word_rd[3], a_word_rsp_tx3)
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_3, factor_word_rd[6], factor_word_rsp_tx6, factor_writeback_rx6, factor_write_done[6])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_3, factor_word_rd[7], factor_word_rsp_tx7, factor_writeback_rx7, factor_write_done[7])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_4, a_word_rd[4], a_word_rsp_tx4)
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_4, factor_word_rd[8], factor_word_rsp_tx8, factor_writeback_rx8, factor_write_done[8])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_4, factor_word_rd[9], factor_word_rsp_tx9, factor_writeback_rx9, factor_write_done[9])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_5, a_word_rd[5], a_word_rsp[5])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_5, factor_word_rd[10], factor_word_rsp[10], factor_writeback[10], factor_write_done[10])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_5, factor_word_rd[11], factor_word_rsp[11], factor_writeback[11], factor_write_done[11])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_6, a_word_rd[6], a_word_rsp[6])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_6, factor_word_rd[12], factor_word_rsp[12], factor_writeback[12], factor_write_done[12])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_6, factor_word_rd[13], factor_word_rsp[13], factor_writeback[13], factor_write_done[13])
    .invoke<tapa::detach>(SalutHBMReadWordPort,
                          A_7, a_word_rd[7], a_word_rsp[7])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          L_7, factor_word_rd[14], factor_word_rsp[14], factor_writeback[14], factor_write_done[14])
    .invoke<tapa::detach>(SalutHBMWordPort,
                          U_7, factor_word_rd[15], factor_word_rsp[15], factor_writeback[15], factor_write_done[15])
    .invoke<tapa::detach>(SalutPublishJoin,
                          sm_published, lm_installed, published_ack, publish_close, publish_drained)
    .invoke<tapa::detach>(SalutAckClientAdapter,
                          published_ack, wr_ack)
    .invoke<tapa::detach>(ParallelAckDemux,
                          wr_ack, front_wr_ack_spu, peg_wr_ack)
    .invoke<tapa::detach>(ParallelPegAckDemux,
                          peg_wr_ack, pe_wr_ack)
    .invoke<tapa::detach>(ParallelSegDoneMux,
                          seg_done_pe, seg_done)
    .invoke<tapa::detach>(ParallelTaskAllocationUnit,
                          ready_local_task, ready_local_dep, ready_global_task, ready_global_dep, seg_task, seg_dep)
    .invoke<tapa::detach>(ParallelDegreeStore,
                          degree_request, degree_response)
    .invoke<tapa::detach>(ParallelScheduler,
                          META_SCHED_mmap, front_ready_id, front_done, pivot_ready, segment_ready_id, seg_done, degree_request, degree_response, compute_done)
    .invoke<tapa::detach>(SalutCompletionFanout,
                          compute_done, peg_rd_close, peg_wr_close, front_rd_close, front_wr_close, publish_close, join_close)
    .invoke<tapa::detach>(SalutGear4SplitResponse, a_rsp_tx6, a_rsp_gb6)
    .invoke<tapa::detach>(SalutGear4JoinResponse, a_rsp_gb6, a_rsp[6])
    .invoke<tapa::detach>(SalutGear4SplitResponse, a_rsp_tx7, a_rsp_gb7)
    .invoke<tapa::detach>(SalutGear4JoinResponse, a_rsp_gb7, a_rsp[7])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, a_word_rsp_tx1, a_word_rsp_gb1)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, a_word_rsp_gb1, a_word_rsp[1])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, a_word_rsp_tx2, a_word_rsp_gb2)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, a_word_rsp_gb2, a_word_rsp[2])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, a_word_rsp_tx3, a_word_rsp_gb3)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, a_word_rsp_gb3, a_word_rsp[3])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, a_word_rsp_tx4, a_word_rsp_gb4)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, a_word_rsp_gb4, a_word_rsp[4])
    .invoke<tapa::detach>(SalutGear4SplitResponse, factor_rsp[6], factor_rsp_gb6)
    .invoke<tapa::detach>(SalutGear4JoinResponse, factor_rsp_gb6, factor_rsp_rx6)
    .invoke<tapa::detach>(SalutGear4SplitResponse, factor_rsp[7], factor_rsp_gb7)
    .invoke<tapa::detach>(SalutGear4JoinResponse, factor_rsp_gb7, factor_rsp_rx7)
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx2, factor_word_rsp_gb2)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb2, factor_word_rsp[2])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx3, factor_word_rsp_gb3)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb3, factor_word_rsp[3])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx4, factor_word_rsp_gb4)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb4, factor_word_rsp[4])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx5, factor_word_rsp_gb5)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb5, factor_word_rsp[5])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx6, factor_word_rsp_gb6)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb6, factor_word_rsp[6])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx7, factor_word_rsp_gb7)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb7, factor_word_rsp[7])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx8, factor_word_rsp_gb8)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb8, factor_word_rsp[8])
    .invoke<tapa::detach>(SalutGear4SplitWordRead, factor_word_rsp_tx9, factor_word_rsp_gb9)
    .invoke<tapa::detach>(SalutGear4JoinWordRead, factor_word_rsp_gb9, factor_word_rsp[9])
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[2], factor_writeback_gb2)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb2, factor_writeback_rx2)
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[3], factor_writeback_gb3)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb3, factor_writeback_rx3)
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[4], factor_writeback_gb4)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb4, factor_writeback_rx4)
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[5], factor_writeback_gb5)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb5, factor_writeback_rx5)
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[6], factor_writeback_gb6)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb6, factor_writeback_rx6)
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[7], factor_writeback_gb7)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb7, factor_writeback_rx7)
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[8], factor_writeback_gb8)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb8, factor_writeback_rx8)
    .invoke<tapa::detach>(SalutGear4SplitWriteback, factor_writeback[9], factor_writeback_gb9)
    .invoke<tapa::detach>(SalutGear4JoinWriteback, factor_writeback_gb9, factor_writeback_rx9)
    .invoke<tapa::detach>(SalutGear4SplitWP, front_wr_elem_spu[1], front_wr_elem_spu_gb1)
    .invoke<tapa::detach>(SalutGear4JoinWP, front_wr_elem_spu_gb1, front_wr_elem_spu_rx1)
    .invoke<tapa::detach>(SalutGear4SplitRP, peg_lower_rsp_tx1, peg_lower_rsp_gb1)
    .invoke<tapa::detach>(SalutGear4JoinRP, peg_lower_rsp_gb1, peg_lower_rsp[1])
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_0[0], rsp_omega_0_gb0)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_0_gb0, rsp_omega_0_rx0)
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_0[5], rsp_omega_0_gb5)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_0_gb5, rsp_omega_0_rx5)
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_2_tx2, rsp_omega_2_gb2)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_2_gb2, rsp_omega_2[2])
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_2_tx3, rsp_omega_2_gb3)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_2_gb3, rsp_omega_2[3])
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_2_tx6, rsp_omega_2_gb6)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_2_gb6, rsp_omega_2[6])
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_2_tx7, rsp_omega_2_gb7)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_2_gb7, rsp_omega_2[7])
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_3_tx0, rsp_omega_3_gb0)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_3_gb0, rsp_omega_3[0])
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_3_tx4, rsp_omega_3_gb4)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_3_gb4, rsp_omega_3[4])
    .invoke<tapa::detach>(SalutGear4SplitResponse, rsp_omega_3_tx5, rsp_omega_3_gb5)
    .invoke<tapa::detach>(SalutGear4JoinResponse, rsp_omega_3_gb5, rsp_omega_3[5])
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[0], wr_omega_0_gb0)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb0, wr_omega_0_rx0)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[1], wr_omega_0_gb1)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb1, wr_omega_0_rx1)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[2], wr_omega_0_gb2)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb2, wr_omega_0_rx2)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[3], wr_omega_0_gb3)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb3, wr_omega_0_rx3)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[4], wr_omega_0_gb4)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb4, wr_omega_0_rx4)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[5], wr_omega_0_gb5)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb5, wr_omega_0_rx5)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[6], wr_omega_0_gb6)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb6, wr_omega_0_rx6)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_0[7], wr_omega_0_gb7)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_0_gb7, wr_omega_0_rx7)
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_3_tx0, wr_omega_3_gb0)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_3_gb0, wr_omega_3[0])
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_3_tx5, wr_omega_3_gb5)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_3_gb5, wr_omega_3[5])
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_3_tx6, wr_omega_3_gb6)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_3_gb6, wr_omega_3[6])
    .invoke<tapa::detach>(SalutGear4SplitWriteBeat, wr_omega_3_tx7, wr_omega_3_gb7)
    .invoke<tapa::detach>(SalutGear4JoinWriteBeat, wr_omega_3_gb7, wr_omega_3[7])
    .invoke<tapa::join>(SalutMemoryDrainJoin,
                          join_close, bank_drained, response_ended, publish_drained);
#ifndef __SYNTHESIS__
  uint64_t dirty_at_done = 0, queued_at_done = 0;
  for (int owner = 0; owner < 8; ++owner) {
    const auto& sm = salut_sm_stats[owner]; const auto& lm = salut_lm_stats[owner];
    dirty_at_done += sm.dirty_final;
    queued_at_done += sm.queue_final[0] + sm.queue_final[1];
    std::cerr << "[SALUT banked SM] owner=" << owner << " hits=" << sm.hits
              << " dirty_hits=" << sm.dirty_hits << " misses=" << sm.misses
              << " installs=" << sm.installs << " published=" << sm.publications
              << " wb_l_issued=" << sm.writeback_issued[0] << " wb_l_completed=" << sm.writeback_completed[0]
              << " wb_u_issued=" << sm.writeback_issued[1] << " wb_u_completed=" << sm.writeback_completed[1]
              << " peak_l=" << sm.queue_peak[0] << " peak_u=" << sm.queue_peak[1]
              << " dirty_peak=" << sm.dirty_peak << " capacity_stall_ticks=" << sm.capacity_stall_ticks
              << " queue_stall_ticks=" << sm.queue_stall_ticks << '\n';
    std::cerr << "[SALUT banked LM] peg=" << owner << " hits=" << lm.hits
              << " misses=" << lm.misses << " refill_words=" << lm.refill_words
              << " snoop_words=" << lm.installs << " installed=" << lm.publications
              << " a_bypass=" << lm.bypass_requests << '\n';
  }
  std::cerr << "[SALUT banked drain] banks=8 response_lanes=8 publisher=closed dirty="
            << dirty_at_done << " queued=" << queued_at_done << std::endl;
#endif
}
#endif  // SALUT_HLS_PARALLEL_TASK_ONLY
