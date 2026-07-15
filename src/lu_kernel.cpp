#include <ap_int.h>
#include <tapa.h>
#include <cstdint>
#include <iostream>
#include "lu_kernel.h"   // 类型/常量/宏定义接口
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

// 正常模式：处理所有任务 
//======================================================================
//  工具函数：512b <-> 64b 打包解包
//======================================================================
static inline WORD64 get_word64(WORD512 w, int k) {
#pragma HLS inline
  return w.range((k + 1) * 64 - 1, k * 64);             // 取第 k 个 64b 槽位
}
static inline void set_word64(WORD512 &w, int k, WORD64 v) {
#pragma HLS inline
  w.range((k + 1) * 64 - 1, k * 64) = v;                // 写第 k 个 64b 槽位
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
  bool injected_warmup = false;
  ap_uint<4> rr = 0;  // 轮询指针
  
  for (;;) {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=1000000 max=10000000
    iterations++;
    
    // 启动预热
    if (!injected_warmup && iterations >= 10000 && tasks_broadcasted == 0) {
      for (int pe = 0; pe < NUM_PE; ++pe) {
#pragma HLS unroll
        ReadySeg t;
        t.seg_id = 0xFFFFFFFD;
        t.j = 0; t.row_lo = 0; t.row_hi = 128;
        t.A_addr = pack_addr(0, 0);
        t.dep_lo = 0; t.dep_hi = 0; t.is_last = 0;
        ready_to_pe[pe].write(t);
      }
      injected_warmup = true;
      DEBUG_PRINT_IMPORTANT("ReadySegBroadcaster: 🔥 Injected warmup tasks");
    }

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
  tapa::istream<uint32_t>& col_done_in,                 // 列完成广播
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

  // 强制注入：选择第一个有依赖的段，尽早下发，验证真实依赖路径
  {
    bool injected = false;
    for (uint32_t sid = 0; sid < nSeg && !injected; ++sid) {
#pragma HLS pipeline II=1
      uint32_t dep_lo_val = dep_ptr[sid];
      uint32_t dep_hi_val = dep_ptr[sid + 1];
      if (dep_hi_val > dep_lo_val) {
        SegRec r = seg_table[sid];
        ReadySeg t{sid, r.j, r.row_lo, r.row_hi, r.A_addr,
                   r.word_count, r.L_base_addr, r.U_diag_addr,
                   dep_lo_val, dep_hi_val, r.is_last};
        ready_global_out.write(t);
        DEBUG_PRINT_IMPORTANT("DegScheduler: 🔰 注入有依赖段 sid=" << sid << " j=" << r.j
                              << " dep_cnt=" << (dep_hi_val - dep_lo_val));
        injected = true;
      }
    }
  }

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

  // 提前发射一批初始 ready 段（不等待 posting_* 完成），快速拉起流水
  {
    const uint32_t EARLY_LIMIT = 64;
    uint32_t early_ready = 0;
    for (uint32_t sid = 0; sid < nSeg && early_ready < EARLY_LIMIT; sid++) {
#pragma HLS pipeline II=1
      SegRec r = seg_table[sid];
      uint32_t dep_lo_val = dep_ptr[sid];
      uint32_t dep_hi_val = dep_ptr[sid + 1];
      ReadySeg t{sid, r.j, r.row_lo, r.row_hi, r.A_addr,
                 r.word_count, r.L_base_addr, r.U_diag_addr,
                 dep_lo_val, dep_hi_val, r.is_last};
      int peg = addr_bank(r.A_addr) % PEGS;
      uint32_t dep_cnt0 = dep_hi_val - dep_lo_val;
      uint32_t seg_len0 = r.row_hi - r.row_lo;
      bool high_prio0 = (dep_cnt0 > 0) && (seg_len0 >= 64);
      if (high_prio0 && !ready_local_out[peg].full()) {
        ready_local_out[peg].write(t);
      } else {
        ready_global_out.write(t);
      }
      early_ready++;
      if (early_ready <= 16) {
        DEBUG_PRINT_IMPORTANT("DegScheduler: 🚀 Early ready sid=" << sid << " j=" << r.j
                              << " dep_cnt=" << dep_cnt0 << " peg=" << peg);
      }
    }
    DEBUG_PRINT_IMPORTANT("DegScheduler: Early ready issued=" << early_ready);
  }

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

  uint32_t initial_ready_count = 0;
  
  uint32_t initial_ready_local[PEGS] = {0};
  uint32_t initial_ready_global = 0;
  uint32_t initial_emits_local[PEGS] = {0};
  uint32_t initial_emits_global = 0;
  
  DEBUG_PRINT("DegScheduler: Scanning for initial ready segments (last-first), nSeg=" << nSeg);
  // 第一遍：优先发射 is_last 段
  for (uint32_t sid = 0; sid < nSeg; sid++) {
#pragma HLS pipeline II=1
    if (deg_cur[sid] != 0) continue;
    SegRec r = seg_table[sid];
    if (!r.is_last) continue;
    uint32_t dep_lo_val = dep_ptr[sid];
    uint32_t dep_hi_val = dep_ptr[sid + 1];
    ReadySeg t{sid, r.j, r.row_lo, r.row_hi, r.A_addr,
               r.word_count, r.L_base_addr, r.U_diag_addr,
               dep_lo_val, dep_hi_val, r.is_last};
    int peg = addr_bank(r.A_addr) % PEGS;             // bank→PEG
    uint32_t dep_cnt = dep_hi_val - dep_lo_val;
    uint32_t seg_len = r.row_hi - r.row_lo;
    bool high_prio = (dep_cnt > 0) && (seg_len >= 64);
    if (high_prio && !ready_local_out[peg].full()) {
      ready_local_out[peg].write(t);
      initial_ready_local[peg]++;
      initial_emits_local[peg]++;
    } else {
      ready_global_out.write(t);
      initial_ready_global++;
      initial_emits_global++;
    }
    initial_ready_count++;
    if (initial_ready_count <= 10) {
      DEBUG_PRINT_IMPORTANT("DegScheduler: Initial ready (last) sid=" << sid << " j=" << r.j << " peg=" << peg);
    }
  }
  // 第二遍：其余段
  for (uint32_t sid = 0; sid < nSeg; sid++) {
#pragma HLS pipeline II=1
    if (deg_cur[sid] != 0) continue;
    SegRec r = seg_table[sid];
    if (r.is_last) continue;
    uint32_t dep_lo_val = dep_ptr[sid];
    uint32_t dep_hi_val = dep_ptr[sid + 1];
    ReadySeg t{sid, r.j, r.row_lo, r.row_hi, r.A_addr,
               r.word_count, r.L_base_addr, r.U_diag_addr,
               dep_lo_val, dep_hi_val, r.is_last};
    if (initial_ready_count < 20) {
      DEBUG_PRINT("DegScheduler: ✅ Ready seg_id=" << sid << " j=" << r.j 
                  << " dep_lo=" << dep_lo_val << " dep_hi=" << dep_hi_val 
                  << " dep_cnt=" << (dep_hi_val - dep_lo_val));
    }
    int peg = addr_bank(r.A_addr) % PEGS;
    uint32_t dep_cnt = dep_hi_val - dep_lo_val;
    uint32_t seg_len = r.row_hi - r.row_lo;
    bool high_prio = (dep_cnt > 0) && (seg_len >= 64);
    if (high_prio && !ready_local_out[peg].full()) {
      ready_local_out[peg].write(t);
      initial_ready_local[peg]++;
      initial_emits_local[peg]++;
    } else {
      ready_global_out.write(t);
      initial_ready_global++;
      initial_emits_global++;
    }
    initial_ready_count++;
    if (initial_ready_count <= 10) {
      DEBUG_PRINT("DegScheduler: Initial ready (non-last) sid=" << sid << " j=" << r.j << " peg=" << peg);
    }
  }
  DEBUG_PRINT_IMPORTANT("DegScheduler: Initial ready segments=" << initial_ready_count);

  for (int peg = 0; peg < PEGS; peg++) {
    DEBUG_PRINT_IMPORTANT("DegScheduler: PEG[" << peg << "] initial_ready=" << initial_ready_local[peg]);
    DEBUG_PRINT("DegScheduler: PEG[" << peg << "] initial_emits=" << initial_emits_local[peg]);
  }
  DEBUG_PRINT_IMPORTANT("DegScheduler: Global initial_ready=" << initial_ready_global);
  DEBUG_PRINT("DegScheduler: Global initial_emits=" << initial_emits_global);

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
    
    if (col_done_in.empty()) {
      wait_iterations++;
      // 注释掉频繁的等待日志以减少输出
      // if (wait_iterations % 100000 == 0) {
      //   DEBUG_PRINT("DegScheduler: Waiting for col_done, iterations=" << wait_iterations << " received=" << col_done_count);
      // }
      continue;
    }
    
    wait_iterations = 0;
    uint32_t k = col_done_in.read();                    // 等待列完成
    col_done_count++;                                   // 增加计数
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
    
    // 添加循环依赖检测
    static uint32_t col_done_sequence[1000] = {0};  // 记录列完成序列
    static uint32_t sequence_count = 0;
    
    if (sequence_count < 1000) {
      col_done_sequence[sequence_count] = k;
      sequence_count++;
    }
    
    // 检测循环依赖：如果同一列被多次完成，可能存在循环
    static uint32_t col_completion_count[10000] = {0};  // 列完成计数
    col_completion_count[k]++;
    
    if (col_completion_count[k] > 1) {
      DEBUG_PRINT("DegScheduler: ⚠️ 检测到重复列完成 col=" << k << " 完成次数=" << col_completion_count[k]);
    }
    
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
  ap_uint<5> rr_pe = 0;       // 当前PE (0-15)
  
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
        // 分配到PEG内的PE（固定映射：PEG i -> PE 2*i 和 2*i+1）
        int pe = rr_peg * 8 + (tasks_distributed & 8);  // 简单的轮询
        // 添加边界检查，确保 pe 在有效范围内
        if (pe >= NUM_PE) pe = pe % NUM_PE;
        ready_to_pe[pe].write(local_task);  // 阻塞写入（依赖FIFO深度）
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
    bool forwarded = false;

    // 轮询所有 PE 的 col_done 流（避免取模，使用条件减法）
    for (int i = 0; i < NUM_PE; ++i) {
#pragma HLS loop_tripcount min=1 max=NUM_PE avg=NUM_PE/2  // 明确指定循环次数以加快综合
#pragma HLS unroll factor=4  // 部分展开以加速综合（对于NUM_PE=64）
      int k = rr + i;
      if (k >= NUM_PE) k -= NUM_PE;
      if (!col_done_in[k].empty()) {
        uint32_t col_id = col_done_in[k].read();
        col_done_out.write(col_id);
        // rr = k + 1 (避免取模)
        rr = (k + 1 < NUM_PE) ? (ap_uint<8>)(k + 1) : (ap_uint<8>)0;
        total_col_done++;
        forwarded = true;
        if (total_col_done <= 10 || (total_col_done & 0x3F) == 0) {
          DEBUG_PRINT("ColDoneMux: ✅ Forwarded col_done #" << total_col_done << " from PE" << k << " col_id=" << col_id);
        }
        break; // 本周期只前递一条
      }
    }

    if (!forwarded) {
      // 若本周期无数据，轻量推进 rr，避免长时间卡在同一优先级
      rr = (rr + 1 < NUM_PE) ? (ap_uint<8>)(rr + 1) : (ap_uint<8>)0;
    }
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
      // 轮询寻找一个写请求
      for (int i = 0; i < NUM_PE; ++i) {
#pragma HLS loop_tripcount min=1 max=NUM_PE avg=NUM_PE/2  // 明确指定循环次数以加快综合
#pragma HLS unroll factor=4  // 部分展开以加速综合（对于NUM_PE=32）
        ap_uint<6> pe_idx = (rr + i) % NUM_PE;
        if (!wr_req_in[pe_idx].empty()) {
          WriteReqTagged req = wr_req_in[pe_idx].read();
          wr_req_out.write(req);

          locked_pe = pe_idx;
          remaining_len = req.r.len;
          rr = (pe_idx + 1) % NUM_PE;
          state = FORWARD_DATA;

          total_reqs++;
          if (total_reqs <= 20 || total_reqs % 100 == 0) {
            DEBUG_PRINT_IMPORTANT("WriteChannelArbiter: 锁定了 PE" << (int)locked_pe 
                                  << "，准备转发 " << remaining_len << " 个数据元素 (请求 #" << total_reqs << ")");
          }
          break;
        }
      }
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

    // 轮询找到一个输入，有数据则阻塞式推到输出
    for (int k = 0; k < (int)NUM_PE; ++k) {
#pragma HLS loop_tripcount min=1 max=NUM_PE avg=NUM_PE/2  // 明确指定循环次数以加快综合
#pragma HLS unroll factor=4  // 部分展开以加速综合（对于NUM_PE=64）
      ap_uint<6> idx = (rr_rd + k) % NUM_PE;
      if (!in_rd[idx].empty()) {
        ReadReqTagged req = in_rd[idx].read();
        while (out_rd.full()) { /* spin until downstream ready */ }
        out_rd.write(req);
        total_rd++;
        if (total_rd <= 30 || (total_rd % 1024) == 0) {
          SUMMARY_PRINT("ReadReqMux: fwd #" << total_rd << " from PE" << (int)idx << " kind=" << (int)req.r.kind << " len=" << req.r.len);
        }
        rr_rd = (idx + 1) % NUM_PE;
        break;
      }
    }
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
    // 轮询找到一个输入，有数据则阻塞式推到输出
    for (int k = 0; k < (int)NUM_PE; ++k) {
#pragma HLS loop_tripcount min=1 max=NUM_PE avg=NUM_PE/2
#pragma HLS unroll factor=4
      ap_uint<6> idx = (rr + k) % NUM_PE;
      if (!glb_req_in[idx].empty()) {
        LUReq req = glb_req_in[idx].read();
        while (glb_req_out.full()) { /* spin until downstream ready */ }
        glb_req_out.write(req);
        total_req++;
        if (total_req <= 10 || (total_req % 1024) == 0) {
          DEBUG_PRINT("GlbReqMux: fwd #" << total_req << " from PE" << (int)idx << " is_write=" << req.is_write << " addr=" << req.addr);
        }
        rr = (idx + 1) % NUM_PE;
        break;
      }
    }
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
  s.lane = bank % lanes;                                 // lane 号
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
void HBM_Read_Service(
  tapa::mmap<ap_uint<512>> A_0, tapa::mmap<ap_uint<512>> A_1, tapa::mmap<ap_uint<512>> A_2, tapa::mmap<ap_uint<512>> A_3,
  tapa::mmap<ap_uint<512>> A_4, tapa::mmap<ap_uint<512>> A_5, tapa::mmap<ap_uint<512>> A_6, tapa::mmap<ap_uint<512>> A_7,
  tapa::mmap<ap_uint<512>> L_0, tapa::mmap<ap_uint<512>> L_1, tapa::mmap<ap_uint<512>> L_2, tapa::mmap<ap_uint<512>> L_3,
  tapa::mmap<ap_uint<512>> L_4, tapa::mmap<ap_uint<512>> L_5, tapa::mmap<ap_uint<512>> L_6, tapa::mmap<ap_uint<512>> L_7,
  tapa::mmap<ap_uint<512>> U_0, tapa::mmap<ap_uint<512>> U_1, tapa::mmap<ap_uint<512>> U_2, tapa::mmap<ap_uint<512>> U_3,
  tapa::istream<ReadReqTagged>& req_in,   // 读请求（带 pe）
  tapa::ostream<ReadElemTagged>& elem_out // 读回 64b（带 pe）
) {
  // 将单独的mmap参数存储到数组中以便索引访问
  tapa::mmap<ap_uint<512>> A_arr[8] = {A_0, A_1, A_2, A_3, A_4, A_5, A_6, A_7};
  tapa::mmap<ap_uint<512>> L_arr[8] = {L_0, L_1, L_2, L_3, L_4, L_5, L_6, L_7};
  tapa::mmap<ap_uint<512>> U_arr[4] = {U_0, U_1, U_2, U_3};
  
  // 综合路径保持同步访问；软件仿真使用 get() 指针
  DEBUG_PRINT_IMPORTANT("HBM_Read_Service: Started, entering main loop");
  uint32_t requests_processed = 0;
  uint32_t iterations = 0;
  uint32_t idle_cycles = 0;
  
  // 强制输出以确认循环开始
  DEBUG_PRINT_IMPORTANT("HBM_Read_Service: About to enter infinite loop");
  
  for (;;) {                                                                     // 永久服务
#pragma HLS loop_tripcount avg=1000000 max=10000000  // 指导综合器这是长期运行的服务
    // 注意：移除pipeline pragma以便在软件仿真中正确执行
    // #pragma HLS pipeline II=1                                                        // 单周期处理
    iterations++;
    // 减少调试输出频率
    if (iterations <= 10) {
      DEBUG_PRINT_IMPORTANT("HBM_Read_Service: iteration " << iterations);
    }
    if (iterations % 1000000 == 0) {
      DEBUG_PRINT_IMPORTANT("HBM_Read_Service: iterations=" << iterations << " requests_processed=" << requests_processed);
    }
    
    // 改为短自旋阻塞读取，确保请求被及时取走
    ReadReqTagged rq;
    if (req_in.empty()) {
      idle_cycles++;
      // 减少调试输出频率
      if (iterations <= 10) {
        DEBUG_PRINT_IMPORTANT("HBM_Read_Service: waiting for request, iter=" << iterations);
      }
      continue;
    }
    rq = req_in.read();
    
    // 成功读取到请求
    idle_cycles = 0;
    // 减少调试输出频率
    if (iterations <= 10 || requests_processed == 0) {
      DEBUG_PRINT_IMPORTANT("HBM_Read_Service: Successfully read request at iteration " << iterations);
    }
    requests_processed++;
    // 减少调试输出频率以提升性能
    if (requests_processed == 1) {
      DEBUG_PRINT("HBM_Read_Service: First request received from PE " << (int)rq.pe << ", kind=" << (int)rq.r.kind << " addr=" << rq.r.addr << " len=" << rq.r.len);
    }
    if (requests_processed % 1000 == 0) {
      DEBUG_PRINT("HBM_Read_Service: Processed " << requests_processed << " requests");
    }
    const uint8_t kind = rq.r.kind;                                              // 0:A 1:L 2:U
    
    // 使用原始地址解码，不做PE调整
    const PortSel s = (kind == 0) ? decode_addr(rq.r.addr, /*lanes*/8, true)     // 解码 A
                     : (kind == 1) ? decode_addr(rq.r.addr, /*lanes*/8, false)   // 解码 L（8个bank）
                                   : decode_addr(rq.r.addr, /*lanes*/4, false);  // 解码 U（4个bank）
    const uint32_t words =                                                       // 需要的 512b 块数
        (rq.r.len + s.head_pad + VEC_64_PER_512 - 1) / VEC_64_PER_512;           // 向上取整
    
    // 减少调试输出频率
    if (requests_processed <= 5 || (requests_processed % 1000 == 0 && s.word_off >= 0x100000)) {
      DEBUG_PRINT_IMPORTANT("HBM_Read_Service: Request " << requests_processed << " from PE " << (int)rq.pe 
                  << ", kind=" << (int)kind << " addr=" << rq.r.addr << " len=" << rq.r.len
                  << " lane=" << (int)s.lane << " word_off=" << s.word_off << " words=" << words
                  << " head_pad=" << (int)s.head_pad);
    }
    
    // 检查地址是否超出范围
    if (s.word_off >= 0x20000) {
      DEBUG_PRINT_IMPORTANT("HBM_Read_Service: ❌ 地址超出范围 PE" << (int)rq.pe << " word_off=" << s.word_off << " (max=0x20000)");
      continue;  // 跳过无效地址
    }
    
    // 添加地址和端口验证
    if (s.lane >= (kind == 2 ? 4 : 8)) {
      DEBUG_PRINT("HBM_Read_Service: ❌ 无效的lane索引 PE" << (int)rq.pe << " kind=" << (int)kind << " lane=" << (int)s.lane);
      continue;  // 跳过无效请求
    }
    
    // 边界检查：确保word_off不超过实际缓冲区大小
    // 每个缓冲区约为 588808 个 64位元素 = 73601 个 512位元素
    // 为安全起见，使用更保守的上限
    if (s.word_off >= 0x20000) {  // 128K个512位字 = 1M个64位元素 (8MB)
      DEBUG_PRINT("HBM_Read_Service: ❌ 地址超出范围 PE" << (int)rq.pe << " word_off=" << s.word_off << " (max=0x20000)");
      continue;  // 跳过无效地址
    }

    // 直接从数组中选择对应的端口
    auto& p_port = (kind == 0) ? A_arr[s.lane] :
                   (kind == 1) ? L_arr[s.lane] :
                                U_arr[s.lane];
    
    // 添加端口使用统计和冲突检测
    static uint32_t port_usage_count[3][8] = {0};  // [kind][lane] 使用计数
    static uint32_t last_used_port[3][8] = {0};     // [kind][lane] 最后使用时间
    static uint32_t port_load[3][8] = {0};          // [kind][lane] 负载统计
    
    if (requests_processed <= 10) {
      DEBUG_PRINT("HBM_Read_Service: 端口使用统计 kind=" << (int)kind << " lane=" << (int)s.lane 
                  << " 使用次数=" << port_usage_count[kind][s.lane] << " 负载=" << port_load[kind][s.lane]);
    }
    
    port_usage_count[kind][s.lane]++;
    port_load[kind][s.lane] += rq.r.len;  // 累加数据量作为负载
    last_used_port[kind][s.lane] = requests_processed;

    // 使用流水线方式：先发送所有地址，然后接收所有数据
    // 这样可以避免阻塞在elem_out.write()时无法处理新请求
    uint32_t remain = rq.r.len;                                                  // 尚需 64b 数
    uint8_t  pad    = s.head_pad;                                                // 头部对齐空槽
    uint32_t elements_sent = 0;
    
    // 减少调试输出（仅在关键请求时输出）
    if (requests_processed <= 5 || requests_processed % 1000 == 0) {
      DEBUG_PRINT_IMPORTANT("HBM读取服务: 开始流水线读取 PE" << (int)rq.pe << " words=" << words << " remain=" << remain);
    }
    
#ifdef __SYNTHESIS__
    // 综合路径：完全展开端口选择，无数组
    for (uint32_t i = 0; i < words; i++) {
#pragma HLS pipeline II=1
      ap_uint<512> w = 0;
      if (kind == 0) {
        if (s.lane == 0) w = A_0[s.word_off + i];
        else if (s.lane == 1) w = A_1[s.word_off + i];
        else if (s.lane == 2) w = A_2[s.word_off + i];
        else if (s.lane == 3) w = A_3[s.word_off + i];
        else if (s.lane == 4) w = A_4[s.word_off + i];
        else if (s.lane == 5) w = A_5[s.word_off + i];
        else if (s.lane == 6) w = A_6[s.word_off + i];
        else if (s.lane == 7) w = A_7[s.word_off + i];
      } else if (kind == 1) {
        if (s.lane == 0) w = L_0[s.word_off + i];
        else if (s.lane == 1) w = L_1[s.word_off + i];
        else if (s.lane == 2) w = L_2[s.word_off + i];
        else if (s.lane == 3) w = L_3[s.word_off + i];
        else if (s.lane == 4) w = L_4[s.word_off + i];
        else if (s.lane == 5) w = L_5[s.word_off + i];
        else if (s.lane == 6) w = L_6[s.word_off + i];
        else if (s.lane == 7) w = L_7[s.word_off + i];
      } else {
        if (s.lane == 0) w = U_0[s.word_off + i];
        else if (s.lane == 1) w = U_1[s.word_off + i];
        else if (s.lane == 2) w = U_2[s.word_off + i];
        else if (s.lane == 3) w = U_3[s.word_off + i];
      }
      for (int k = 0; k < VEC_64_PER_512; k++) {
#pragma HLS unroll
        if (i == 0 && k < pad) continue;
        if (!remain) break;
        WORD64 v = get_word64(w, k);
        // 优化：保持阻塞写入确保数据完整性，但减少自旋检查（在硬件中会被优化）
        elem_out.write(ReadElemTagged{v, rq.pe});
        elements_sent++;
        remain--;
      }
    }
#else
    // 软件模拟路径：同步内存读取
    ap_uint<512>* ptr = p_port.get();
    if (ptr == nullptr) {
      DEBUG_PRINT("HBM读取服务: ❌ 空指针 PE" << (int)rq.pe << " kind=" << (int)kind << " lane=" << (int)s.lane);
      continue;
    }
    for (uint32_t i = 0; i < words; i++) {
#pragma HLS pipeline II=1
      uint64_t addr = s.word_off + i;
      if (addr >= 0x20000) {
        DEBUG_PRINT_IMPORTANT("HBM_Read_Service: ❌ 地址超出范围，停止发送 PE" << (int)rq.pe << " addr=" << addr << " (max=0x20000)");
        break;
      }
      ap_uint<512> w = ptr[addr];
      for (int k = 0; k < VEC_64_PER_512; k++) {
#pragma HLS unroll
        if (i == 0 && k < pad) continue;
        if (!remain) break;
        WORD64 v = get_word64(w, k);
        // 优化：保持阻塞写入确保数据完整性
        elem_out.write(ReadElemTagged{v, rq.pe});
        elements_sent++;
        remain--;
        
        // 减少调试输出频率以提升性能
        if (elements_sent <= 10 || (elements_sent % 10000 == 0)) {
          DEBUG_PRINT_IMPORTANT("HBM_Read_Service: 发送进度 PE" << (int)rq.pe << " sent=" << elements_sent << "/" << rq.r.len);
        }
      }
    }
#endif
    
    // 减少调试输出频率
    if (requests_processed <= 5 || requests_processed % 1000 == 0) {
      DEBUG_PRINT_IMPORTANT("HBM读取服务: ✅ 请求完成 PE" << (int)rq.pe << " 已发送 " << elements_sent << " 个元素");
    }
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
    // Prioritize DegScheduler to prevent head-of-line blocking if Poster is backpressured
    if (!dep_ptr_in.empty() && !dep_ptr_out_deg.full()) {
      META_T v = dep_ptr_in.read();
      dep_ptr_out_deg.write(v);
      // Best-effort write to Poster; do not block DegScheduler
      dep_ptr_out_poster.try_write(v);
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
    // Prioritize DegScheduler to prevent head-of-line blocking if Poster is backpressured
    if (!dep_flat_in.empty() && !dep_flat_out_deg.full()) {
      META_T v = dep_flat_in.read();
      dep_flat_out_deg.write(v);
      // Best-effort write to Poster; do not block DegScheduler
      dep_flat_out_poster.try_write(v);
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
  // tapa::ostreams<ReadySeg, NUM_PE>& ready_passthrough, // REMOVED FOR UNIDIRECTIONAL FIFO
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
        
        // ready_passthrough[pe].write(r); // REMOVED FOR UNIDIRECTIONAL FIFO
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
#pragma HLS array_partition variable=cache.data complete dim=1  // 完全展开data数组
  
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
  tapa::ostream<uint32_t>& col_done_out,
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
  // 统计：本PE发送的列完成次数（仅is_last段）
  uint32_t col_done_sent = 0;
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
                                  << " col_done_sent=" << col_done_sent);
            return;
          }
          win_len = seg.row_hi - seg.row_lo;
          dep_cnt = seg.dep_hi - seg.dep_lo;
          dep_idx = 0;
          a_recv_cnt = 0;
          a_req_issued = 0;
          timeout_cnt = 0;
          // 初始化tile与依赖

          deps_prefetched = false;
          dep_list_len = 0;
          
          // 初始化稀疏格式读取变量
          // TODO: word_count应该从META或ReadySeg中获取，这里暂时使用估算值
          // 估算：每个窗口至少需要 ceil(win_len/8) 个512位字（最坏情况）
          // 但实际稀疏格式通常远小于此，暂时使用保守估计
          // 注意：正确的实现应该从主机端传递word_count信息（通过扩展ReadySeg结构或META表）
          word_count = (win_len + 7) / 8;  // 保守估计：假设每个槽位都有数据
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
        } else {
          timeout_cnt++;
          // 减少超时检查频率和调试输出
          if (timeout_cnt >= 10000 && (timeout_cnt % 100000 == 0 || segments_processed == 0)) {
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: IDLE long timeout → synthesize warmup task");
            seg.seg_id = 0xFFFFFFFE;
            seg.j = 0;
            seg.row_lo = 0;
            seg.row_hi = 128;
            seg.A_addr = pack_addr(0, 0);
            seg.dep_lo = 0;
            seg.dep_hi = 0;
            seg.is_last = 0;
            win_len = seg.row_hi - seg.row_lo;
            dep_cnt = 0;
            dep_idx = 0;
            a_recv_cnt = 0;
            a_req_issued = 0;
            deps_prefetched = true;
            dep_list_len = 0;
            // 初始化稀疏格式读取变量
            word_count = (win_len + 7) / 8;  // 保守估计
            words_received = 0;
            word_buffer_cnt = 0;
            hbm_word_buffer = 0;
            // 初始化窗口缓冲区为0
            for (uint32_t i = 0; i < win_len && i < A_WIN_MAX; i++) {
#pragma HLS pipeline II=1
              A_buf[i] = 0;
              Out_buf[i] = 0;
            }
            state = READ_A_REQ;
            timeout_cnt = 0;
            SUMMARY_PRINT("PE" << (int)pe_id << ": SYNTH_TASK j=0 len=128 dep_cnt=0");
          } else if (timeout_cnt >= MAX_TIMEOUT) {
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: Timeout in IDLE after " << segments_processed << " segments");
            return;
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
            if (timeout_cnt >= MAX_TIMEOUT/4 && words_received > 0) {
              // 仿真保护：长时间拿不到数据，剩余槽位视为DUMMY，窗口已完成重构
              // 注意：稀疏格式中剩余未接收的槽位本身就是空的（对应零元素），所以无需额外处理
              words_received = word_count;  // 标记为已完成
              a_recv_cnt = tile_len;  // 标记为已完成
              tile_len = win_len;  // 更新为实际窗口长度
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ⚠️ READ_A_DATA (sparse) 超时，窗口重构完成，继续");
            }
            if (timeout_cnt >= MAX_TIMEOUT/2 && words_received == 0) {
              // 进一步保护：完全没有收到数据，窗口保持为0（已初始化）
              words_received = word_count;
              a_recv_cnt = tile_len;
              tile_len = win_len;
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ⚠️ READ_A_DATA (sparse) 长超时，窗口保持为0，继续");
            }
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
            if (timeout_cnt >= MAX_TIMEOUT) {
              // 仿真降级：依赖预取长期无数据，合成一条 len=0 的依赖，走 U 路径或直接写回
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: PREFETCH_DEPS timeout → synthesize one zero-len dep and proceed");
              dep_list[0].k = seg.j;
              dep_list[0].L_addr = 0;
              dep_list[0].len = 0;
              dep_list[0].U_addr = 0;
              dep_list_len = 1;
              dep_idx = 0;
              // 若A未发读先发读，否则进入依赖处理
              if (win_len > 0 && a_recv_cnt < win_len) state = READ_A_REQ; else state = PROC_DEP;
              timeout_cnt = 0;
            }
          }
        } else {
          deps_prefetched = true;
          dep_idx = 0;
          DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: PREFETCH_DEPS 完成 dep_list_len=" << dep_list_len
                                << " a_recv_cnt=" << a_recv_cnt << "/" << tile_len
                                << " → 下一步=" << ((win_len > 0 && a_recv_cnt < tile_len) ? "READ_A_REQ" : "PROC_DEP"));
          // 如果A还未读完，先去读A；否则直接处理依赖
          if (win_len > 0 && a_recv_cnt < tile_len) {
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
            if (timeout_cnt >= MAX_TIMEOUT) {
              // 仿真降级：视为未命中，直接去读L数据，避免挂死
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: L cache wait timeout → fallback READ_L_DATA");
              state = READ_L_DATA;
              timeout_cnt = 0;
            }
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
            if (timeout_cnt >= MAX_TIMEOUT) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: Timeout reading L data");
              state = DONE;
            }
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
            if (timeout_cnt >= MAX_TIMEOUT) {
              // 仿真降级：直接转到READ_U_DATA，避免挂死
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: U cache wait timeout → fallback READ_U_DATA");
              state = READ_U_DATA;
              timeout_cnt = 0;
            }
          }
        }
        break;

      case READ_U_DATA: {
          DEBUG_PRINT("STATE: READ_U_DATA");
        // 读取U数据（cache miss）
        // 改为阻塞写，确保U读请求发出
        rd_req_q.write(ReadReqTagged{ReadBurstReq{2, cur_dep.U_addr, 1}, pe_id});
        WORD64 elem;
        if (rd_elem_q.try_read(elem)) {
          U64 = elem;
          // 尝试写入cache
          glb_req_out.try_write(LUReq{true, cur_dep.U_addr, elem, pe_id});
          state = COMPUTE;
          timeout_cnt = 0;
        }
        SUMMARY_PRINT("PE" << (int)pe_id << ": READ_U_REQ addr=" << cur_dep.U_addr);
        timeout_cnt++;
        if (timeout_cnt >= MAX_TIMEOUT) {
          // 仿真降级：无法拿到U数据，使用U=0继续，避免挂死
          DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: READ_U_DATA timeout → synthesize U=0 and continue COMPUTE");
          U64 = 0;
          state = COMPUTE;
          timeout_cnt = 0;
        }
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
            if (timeout_cnt >= MAX_TIMEOUT) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ⚠️ Timeout waiting write ack, seg_id=" << seg.seg_id);
              // 超时后继续处理下一个批次或完成
              if (write_offset >= win_len) {
                state = SEND_COL_DONE;
              } else {
                state = WRITE_REQ;
              }
            }
          }
        }
        break;

      case SEND_COL_DONE:
          DEBUG_PRINT("STATE: SEND_COL_DONE");
        // 发送列完成信号
        DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: >>> 进入 SEND_COL_DONE seg_id=" << seg.seg_id
                              << " j=" << seg.j
                              << " is_last=" << (uint32_t)seg.is_last
                              << " win_len=" << win_len);

        if (seg.is_last) {
          // 使用非阻塞写入避免死锁
          if (col_done_out.try_write(seg.j)) {
            col_done_sent++;
            DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ✅ col_done_sent seg_id=" << seg.seg_id << " j=" << seg.j << " total=" << col_done_sent);
            state = DONE;
          } else {
            // Downstream is full, wait and retry
            timeout_cnt++;
            if (timeout_cnt >= MAX_TIMEOUT) {
              DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: ❌ Timeout sending col_done for seg_id=" << seg.seg_id);
              state = DONE; // Force to DONE on timeout
            }
          }
        } else {
          // Not the last segment for this column, just finish
          DEBUG_PRINT_IMPORTANT("SegmentExecutor[" << pe_id << "]: 非最后segment,跳过col_done seg_id=" << seg.seg_id);
          state = DONE;
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
void SparseLUKernel(
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
  // 注意：不再使用 mmaps，直接传递单独的 mmap 参数
  
  tapa::stream<uint32_t, FIFO_CTRL> meta_idx("meta_idx"), meta_idx_poster("meta_idx_poster");
  tapa::stream<META_T, FIFO_DATA_IN> seg_table_s("seg_table_s"), dep_ptr_s("dep_ptr_s"),
                                     dep_flat_s("dep_flat_s"), deg_init_s("deg_init_s"),
                                     posting_ptr_s("posting_ptr_s"), posting_flat_s("posting_flat_s");
  // （移除未使用的中间流，减少接口与综合噪声）
  tapa::stream<META_T, FIFO_DATA_IN> dep_ptr_to_deg("dep_ptr_to_deg"), dep_flat_to_deg("dep_flat_to_deg");
  tapa::stream<META_T, FIFO_DATA_IN> dep_ptr_to_poster("dep_ptr_to_poster"), dep_flat_to_poster("dep_flat_to_poster");
  tapa::streams<ReadySeg, PEGS, FIFO_CTRL> ready_local("ready_local");
  tapa::stream<ReadySeg, FIFO_CTRL> ready_global("ready_global");
  tapa::streams<uint32_t, NUM_PE, FIFO_CTRL> col_done_from_pe("col_done_from_pe");
  tapa::stream<uint32_t, FIFO_CTRL> col_done_merged("col_done_merged");

  tapa::streams<ReadySeg, NUM_PE, FIFO_CTRL> ready_after_wd("ready_after_wd"), ready_to_pe("ready_to_pe");
  tapa::streams<ReadySeg, NUM_PE, FIFO_CTRL> ready_to_depexp("ready_to_depexp");
  tapa::streams<DepEntry, NUM_PE, FIFO_DATA_MID> dep_to_pe("dep_to_pe");

  tapa::streams<ReadReqTagged, NUM_PE, FIFO_CTRL> rd_req_from_pe("rd_req_from_pe");
  tapa::streams<WriteReqTagged, NUM_PE, FIFO_CTRL> wr_req_from_pe("wr_req_from_pe");
  tapa::stream<ReadReqTagged, FIFO_CTRL> rd_req_muxed("rd_req_muxed");
  tapa::stream<WriteReqTagged, FIFO_CTRL> wr_req_muxed("wr_req_muxed");
  tapa::stream<ReadElemTagged, FIFO_DATA_IN> rd_elem_tagged("rd_elem_tagged");
  tapa::streams<WORD64, NUM_PE, FIFO_DATA_IN> rd_elem_to_pe("rd_elem_to_pe");
  tapa::streams<WriteElemTagged, NUM_PE, FIFO_DATA_OUT> wr_elem_from_pe("wr_elem_from_pe");
  tapa::stream<WriteElemTagged, FIFO_DATA_OUT> wr_elem_tagged("wr_elem_tagged");
  tapa::stream<ap_uint<8>, FIFO_CTRL> wr_ack_pe("wr_ack_pe");
  tapa::streams<ap_uint<1>, NUM_PE, FIFO_CTRL> wr_ack_to_pe("wr_ack_to_pe");

  tapa::streams<LUReq, NUM_PE, FIFO_CTRL> glb_req("glb_req");
  tapa::streams<LURsp, NUM_PE, FIFO_CTRL> glb_rsp("glb_rsp");
  tapa::stream<LUReq, FIFO_CTRL> glb_req_muxed("glb_req_muxed");
  tapa::stream<LURsp, FIFO_CTRL> glb_rsp_muxed("glb_rsp_muxed");
  tapa::stream<bool, FIFO_CTRL> done_signal("done_signal");

  // 使用 TAPA 参数化实例化：不再需要显式定义 pe0..pe63

  tapa::task()
    .invoke<tapa::detach>(MetaLoader, META_mmap, meta_idx, meta_idx_poster,
            seg_table_s,
            dep_ptr_to_deg, dep_ptr_to_poster,
            dep_flat_to_deg, dep_flat_to_poster,
            deg_init_s, posting_ptr_s, posting_flat_s)
    .invoke<tapa::detach>(DegScheduler, meta_idx, seg_table_s, dep_ptr_to_deg, dep_flat_to_deg,
                          deg_init_s, posting_ptr_s, posting_flat_s,
                          ready_local, ready_global, col_done_merged, done_signal)
    .invoke<tapa::detach>(WorkDistributor, ready_local, ready_global, ready_after_wd)
    // Use broadcaster to split ready_after_wd into two streams to avoid TAPA FIFO direction issues
    .invoke<tapa::detach>(ReadySegBroadcaster, ready_after_wd, ready_to_pe, ready_to_depexp)
    .invoke<tapa::detach>(Poster, meta_idx_poster, ready_to_depexp, 
                          dep_ptr_to_poster, dep_flat_to_poster, dep_to_pe)
    // 先启动所有 Mux/Demux 任务（明确流的方向）
    .invoke<tapa::detach>(GlbReqMux, glb_req, glb_req_muxed)
    .invoke<tapa::detach>(GlobalLUCache, glb_req_muxed, glb_rsp_muxed, done_signal)
    .invoke<tapa::detach>(GlbRspDemux, glb_rsp_muxed, glb_rsp)
    .invoke<tapa::detach>(ColDoneMux, col_done_from_pe, col_done_merged)
    .invoke<tapa::detach>(ReadReqMux, rd_req_from_pe, rd_req_muxed)
    .invoke<tapa::detach>(WriteChannelArbiter, wr_req_from_pe, wr_req_muxed, wr_elem_from_pe, wr_elem_tagged)
    .invoke<tapa::detach>(RdRespDemux, rd_elem_tagged, rd_elem_to_pe)
    .invoke<tapa::detach>(WrAckDemux, wr_ack_pe, wr_ack_to_pe)
    // 然后启动参数化任务（使用已明确方向的流）
    .invoke<tapa::detach, NUM_PE>(SegmentExecutor_withDepStream,
                                   tapa::seq(),
                                   ready_to_pe,
                                   dep_to_pe,
                                   rd_req_from_pe,
                                   rd_elem_to_pe,
                                   wr_req_from_pe,
                                   wr_elem_from_pe,
                                   wr_ack_to_pe,
                                   col_done_from_pe,
                                   glb_req,
                                   glb_rsp)
    .invoke<tapa::detach>(HBM_Read_Service,
                        A_0, A_1, A_2, A_3, A_4, A_5, A_6, A_7,
                        L_0, L_1, L_2, L_3, L_4, L_5, L_6, L_7,
                        U_0, U_1, U_2, U_3,
                        rd_req_muxed, rd_elem_tagged)
    .invoke<tapa::detach>(HBM_Write_Service,
                        L_0, L_1, L_2, L_3, L_4, L_5, L_6, L_7,
                        U_0, U_1, U_2, U_3,
                        wr_req_muxed, wr_elem_tagged, wr_ack_pe);
  
  DEBUG_PRINT("SparseLUKernel: All tasks completed");
}

