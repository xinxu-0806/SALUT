//======================================================================
//  File: lu_kernel.h
//  Description: FPGA端 稀疏LU分解核 头文件（完整版）
//  定义：类型结构、常量配置、辅助宏、模块接口。
//  支持图驱动调度 (col_done → DegScheduler)，配合纯表版 META。
//======================================================================
#pragma once

#include <ap_int.h>
#include <tapa.h>
#include <cstdint>
#include <hls_stream.h>

//======================================================================
//  顶层参数配置
//======================================================================

// ---------- FPGA 资源配置 ----------
constexpr int NUM_PE        = 64;    // PE 总数（快速综合模式保持较小值）
constexpr int PEGS          = 8;    // PEG 数
constexpr int PE_PER_PEG    = 8;    // 每 PEG 内 PE 数
constexpr int LOCAL_CACHE   = 8;    // 每 PEG 共享 cache 槽数（简化）

// ---------- HBM 配置 ----------
constexpr int A_BANKS       = 8;    // A 空间 HBM bank 数
constexpr int L_BANKS       = 8;    // L 空间 bank
constexpr int U_BANKS       = 4;    // U 空间 bank

// ---------- FIFO 深度（优化配置）----------
// 解决方案：通过修复Poster的依赖发送逻辑，避免死锁
// 1. Poster必须先发送完当前任务的所有依赖，才能接收新任务
// 2. 使用阻塞写入确保依赖必须被发送
// 3. 这样可以避免"任务已转发但依赖未发送"的死锁情况
// 为软件仿真场景放大 FIFO，降低回压与阻塞风险
constexpr int FIFO_CTRL      = 1024;    // 控制流
constexpr int FIFO_DATA_IN   = 65536;   // 输入数据（HBM读）
constexpr int FIFO_DATA_MID  = 65536;   // 中间数据（依赖/分发）
constexpr int FIFO_DATA_OUT  = 65536;   // 输出数据（写回）
constexpr int FIFO_CACHE_REQ = 1024;    // 缓存请求
constexpr int FIFO_CACHE_RSP = 1024;    // 缓存响应
constexpr int FIFO_COL_DONE  = 1024;    // 列完成缓冲

// ---------- 快速综合开关（缩小缓冲/改用BRAM以加快 HLS/RTL） ----------
#ifndef FAST_SYNTH
#define FAST_SYNTH 1
#endif

// ---------- Buffer 配置 ----------
#if FAST_SYNTH
constexpr int A_WIN_MAX     = 256;    // 快综合：缩小窗口
constexpr int L_SLICE_MAX   = 256;    // 快综合：缩小L切片
constexpr int OUT_BUF_MAX   = 256;    // 快综合：缩小输出缓冲
#else
constexpr int A_WIN_MAX     = 1024;   // A 段窗口长度
constexpr int L_SLICE_MAX   = 1024;   // L 片段最大长度
constexpr int OUT_BUF_MAX   = 1024;   // 输出缓存长度
#endif
constexpr int WRITE_BATCH_SIZE = 256; // 写回数据的批处理大小（可按需要调大）

// ---------- 特殊信号定义 ----------
constexpr uint32_t SEG_ID_TERMINATE = 0xFFFFFFFF; // 用于终止模块的特殊段ID

// ---------- 基础类型 ----------
using WORD64  = ap_uint<64>;     // 对应 double/float 宽度
using WORD512 = ap_uint<512>;    // HBM 突发宽度
using META_T  = uint32_t;        // META 区控制类型
using REAL    = float;           // 浮点数类型（用于数值计算）

constexpr int VEC_64_PER_512 = 8;  // 每 512b 含 8 个 64b 元素

//======================================================================
// 地址打包函数（32-bit 元素地址：高8位是 bank，低24位是偏移）
//======================================================================

// 每个 bank 的大小（2^24 个 64b 元素）
#define BANK_SIZE (1 << 24)

static inline uint32_t addr_bank(uint32_t addr) { return (addr >> 24) & 0xFF; } // bank 编号 (高8位)
static inline uint32_t addr_off (uint32_t addr) { return addr & 0x00FFFFFF; }     // 偏移 (低24位)
static inline uint32_t pack_addr(uint32_t bank, uint32_t off) {
  return ((bank & 0xFF) << 24) | (off & 0xFFFFFF);  // 拼接 bank 和 offset
}

//======================================================================
//  HBM 请求 / 响应结构定义
//======================================================================

// ---------- 读请求 ----------
struct ReadBurstReq {
  uint8_t  kind;   // 0:A  1:L  2:U
  uint32_t addr;   // packed 地址
  uint32_t len;    // 读取长度（以 64b 为单位）
};

// ---------- 写请求 ----------
struct WriteBurstReq {
  uint8_t  kind;   // 1:L  2:U
  uint32_t addr;   // packed 地址
  uint32_t len;    // 写回长度（以 64b 为单位）
};

// ---------- 带 PE 标签请求 ----------
struct ReadReqTagged  { ReadBurstReq  r; ap_uint<8> pe; };
struct WriteReqTagged { WriteBurstReq r; ap_uint<8> pe; };
struct ReadElemTagged { WORD64 v; ap_uint<8> pe; };
struct WriteElemTagged{ WORD64 v; ap_uint<8> pe; };

//======================================================================
//  控制与调度结构
//======================================================================

// ---------- MetaIndex ----------
struct MetaIndex {
  uint32_t n;                     // 矩阵维度
  uint32_t seg_rows;              // 每段行数
  uint32_t nSeg;                  // 段总数
  uint32_t off_seg_table;
  uint32_t off_seg_of_col_ptr;
  uint32_t off_seg_of_col_flat;
  uint32_t off_dep_ptr;
  uint32_t off_dep_flat;
  uint32_t off_deg_init;
  uint32_t off_post_ptr;
  uint32_t off_post_flat;
  uint32_t off_dummy;
};

// ---------- 段表 ----------
struct SegRec {
  uint32_t j;          // 列号
  uint32_t s;          // 段号
  uint32_t row_lo;     // 段起始行
  uint32_t row_hi;     // 段结束行
  uint32_t A_addr;     // 段起始 packed 地址（512位字偏移）
  uint32_t word_count; // 该段占用的512位字数（稀疏格式）
  uint32_t L_base_addr;// L(:,j) 的基地址（用于写回L元素，kind=1）
  uint32_t U_diag_addr;// U(j,j) 的地址（用于写回U对角线元素，kind=2）
  uint32_t is_last;    // 是否为该列最后一段（改为uint32_t以匹配主机端打包）
};

// ---------- 依赖项 ----------
struct DepEntry {
  uint32_t k;        // 依赖列号
  uint32_t L_addr;   // L(k,:) 段地址
  uint32_t len;      // 段长度
  uint32_t U_addr;   // U(k,:) 段地址
};

// ---------- 准备好执行的段 ----------
struct ReadySeg {
  uint32_t seg_id;     // 段编号
  uint32_t j;          // 所属列
  uint32_t row_lo;     // 起始行
  uint32_t row_hi;     // 结束行
  uint32_t A_addr;      // 段 A 地址（512位字偏移）
  uint32_t word_count; // 该段占用的512位字数（稀疏格式）
  uint32_t L_base_addr;// L(:,j) 的基地址（用于写回L元素，kind=1）
  uint32_t U_diag_addr;// U(j,j) 的地址（用于写回U对角线元素，kind=2）
  uint32_t dep_lo;     // 依赖区间起
  uint32_t dep_hi;     // 依赖区间止
  uint32_t is_last;    // 是否列末段（改为uint32_t以保持对齐）
};

//======================================================================
//  LU Cache 请求结构
//======================================================================
struct LUReq {
  bool     is_write;  // 写/读
  ap_uint<32> addr;   // LU 地址
  WORD64   data;      // 写入数据
  ap_uint<8> pe;      // PE 标识（用于响应路由）
};

struct LURsp {
  ap_uint<32> addr;   // 地址
  WORD64      data;   // 返回数据
  bool        hit;    // 缓存命中标志
  ap_uint<8> pe;      // PE 标识（用于响应路由）
};

//======================================================================
//  块缓存配置（V1: Block Cache）
//======================================================================
#define USE_BLOCK_CACHE 1               // 启用块缓存（与 Mux/Demux 架构匹配的单流实现）
#define DISABLE_GLOBAL_CACHE 0          // 启用GlobalLUCache以获得性能提升

#if USE_BLOCK_CACHE
  // 块缓存参数
  constexpr int CACHE_BLOCK_SIZE = 8;              // 每块8个元素（64B × 8 = 512b）
  constexpr int CACHE_BLOCKS = 1024;               // 1024个块（总容量8192元素）
  constexpr int CACHE_BLOCK_BITS = 10;             // log2(1024)
  constexpr int CACHE_OFFSET_BITS = 3;             // log2(8)
  constexpr int CACHE_TAG_BITS = 19;               // 32 - 10 - 3
  
  // 块缓存数据结构
  struct CacheBlock {
    bool valid;                                    // 块有效位
    ap_uint<CACHE_TAG_BITS> tag;                   // 标签（19位）
    WORD64 data[CACHE_BLOCK_SIZE];                 // 8个连续元素
    ap_uint<CACHE_BLOCK_SIZE> valid_mask;          // 每个元素的有效位（8位）
  };
  
  // 块缓存统计
  struct CacheStats {
    uint64_t total_requests;                       // 总请求数
    uint64_t total_hits;                           // 总命中数
    uint64_t total_misses;                         // 总未命中数
    uint64_t total_writes;                         // 总写入数
    uint64_t partial_hits;                         // 部分命中（块存在但元素无效）
    uint64_t block_fills;                          // 块填充次数
  };
  
  // 地址分解辅助函数
  static inline uint32_t get_block_addr(uint32_t addr) {
    return addr >> CACHE_OFFSET_BITS;              // 去掉块内偏移
  }
  
  static inline uint32_t get_block_index(uint32_t addr) {
    return (addr >> CACHE_OFFSET_BITS) & ((1 << CACHE_BLOCK_BITS) - 1);
  }
  
  static inline uint32_t get_block_offset(uint32_t addr) {
    return addr & ((1 << CACHE_OFFSET_BITS) - 1);
  }
  
  static inline ap_uint<CACHE_TAG_BITS> get_block_tag(uint32_t addr) {
    return (addr >> CACHE_OFFSET_BITS) >> CACHE_BLOCK_BITS;
  }
  
#else
  // 原有直接映射缓存配置
  struct CacheEntry {
    bool        valid;                             // 有效位
    ap_uint<20> tag;                               // 地址标签（高20位）
    WORD64      data;                              // 缓存数据
  };
  
  constexpr int GLOBAL_LU_CACHE_SIZE = 8192;       // Cache 容量（按元素数，13位索引）
  constexpr int CACHE_INDEX_BITS = 13;             // 索引位数 (2^13 = 8192)
  constexpr int CACHE_TAG_BITS = 20;               // 标签位数 (32 - 13 = 19，向上取整到20)
  
  // Cache 统计结构
  struct CacheStats {
    uint64_t total_requests;                       // 总请求数
    uint64_t total_hits;                           // 总命中数
    uint64_t total_misses;                         // 总未命中数
    uint64_t total_writes;                         // 总写入数
  };
#endif

//======================================================================
//  FPGA 模块间流类型别名
//======================================================================

// 控制流
// 注意：MetaIndex 不再作为整体结构体传输，而是逐字段传输 uint32_t
using stream_metaidx_t  = tapa::stream<uint32_t, FIFO_CTRL>;
using stream_ready_t    = tapa::stream<ReadySeg, FIFO_CTRL>;
using stream_dep_t      = tapa::stream<DepEntry, FIFO_CTRL>;

// 数据请求流
using stream_rdreq_t    = tapa::stream<ReadReqTagged, FIFO_CTRL>;
using stream_wrreq_t    = tapa::stream<WriteReqTagged, FIFO_CTRL>;
using stream_rdele_t    = tapa::stream<ReadElemTagged, FIFO_DATA_MID>;
using stream_wrele_t    = tapa::stream<WriteElemTagged, FIFO_DATA_MID>;

// 完成广播
using stream_done_t     = tapa::stream<uint32_t, FIFO_COL_DONE>;  // 使用专用大缓冲

//======================================================================
//  顶层 Kernel 声明（按 lu_kernel.cpp 中的函数顺序）
//======================================================================

void ReadySegBroadcaster(
  tapa::istreams<ReadySeg, NUM_PE>& ready_in,
  tapa::ostreams<ReadySeg, NUM_PE>& ready_to_pe,
  tapa::ostreams<ReadySeg, NUM_PE>& ready_to_depexp
);

void MetaLoader(
  tapa::mmap<META_T> meta,
  tapa::ostream<uint32_t>& meta_idx_out,
  tapa::ostream<uint32_t>& meta_idx_poster_out,
  tapa::ostream<META_T>& seg_table_out,
  tapa::ostream<META_T>& dep_ptr_out_deg,
  tapa::ostream<META_T>& dep_ptr_out_poster,
  tapa::ostream<META_T>& dep_flat_out_deg,
  tapa::ostream<META_T>& dep_flat_out_poster,
  tapa::ostream<META_T>& deg_init_out,
  tapa::ostream<META_T>& posting_ptr_out,
  tapa::ostream<META_T>& posting_flat_out
);

void DegScheduler(
  tapa::istream<uint32_t>& meta_idx_in,
  tapa::istream<META_T>& seg_table_in,
  tapa::istream<META_T>& dep_ptr_in,
  tapa::istream<META_T>& dep_flat_in,
  tapa::istream<META_T>& deg_init_in,
  tapa::istream<META_T>& posting_ptr_in,
  tapa::istream<META_T>& posting_flat_in,
  tapa::ostreams<ReadySeg, PEGS>& ready_local_out,
  tapa::ostream<ReadySeg>& ready_global_out,
  tapa::istream<uint32_t>& col_done_in,
  tapa::ostream<bool>& done_out
);

void WorkDistributor(
  tapa::istreams<ReadySeg, PEGS>& ready_local_in,
  tapa::istream<ReadySeg>& ready_global_in,
  tapa::ostreams<ReadySeg, NUM_PE>& ready_to_pe
);

void ColDoneMux(
  tapa::istreams<uint32_t, NUM_PE>& col_done_in,
  tapa::ostream<uint32_t>& col_done_out
);

void WriteChannelArbiter(
  tapa::istreams<WriteReqTagged, NUM_PE>& wr_req_in,
  tapa::ostream<WriteReqTagged>& wr_req_out,
  tapa::istreams<WriteElemTagged, NUM_PE>& wr_elem_in,
  tapa::ostream<WriteElemTagged>& wr_elem_out
);

void ReadReqMux(
  tapa::istreams<ReadReqTagged, NUM_PE>& in_rd,
  tapa::ostream<ReadReqTagged>& out_rd
);

void RdRespDemux(
  tapa::istream<ReadElemTagged>& in_rd_elem,
  tapa::ostreams<WORD64, NUM_PE>& out_rd_elem
);

void WrAckDemux(
  tapa::istream<ap_uint<8>>& in_wr_ack,
  tapa::ostreams<ap_uint<1>, NUM_PE>& out_wr_ack
);

void WriteElemMux(
  tapa::istreams<WriteElemTagged, NUM_PE>& wr_elem_in,
  tapa::ostream<WriteElemTagged>& wr_elem_out
);

void HBM_Read_Service(
  tapa::mmap<ap_uint<512>> A_0, tapa::mmap<ap_uint<512>> A_1, tapa::mmap<ap_uint<512>> A_2, tapa::mmap<ap_uint<512>> A_3,
  tapa::mmap<ap_uint<512>> A_4, tapa::mmap<ap_uint<512>> A_5, tapa::mmap<ap_uint<512>> A_6, tapa::mmap<ap_uint<512>> A_7,
  tapa::mmap<ap_uint<512>> L_0, tapa::mmap<ap_uint<512>> L_1, tapa::mmap<ap_uint<512>> L_2, tapa::mmap<ap_uint<512>> L_3,
  tapa::mmap<ap_uint<512>> L_4, tapa::mmap<ap_uint<512>> L_5, tapa::mmap<ap_uint<512>> L_6, tapa::mmap<ap_uint<512>> L_7,
  tapa::mmap<ap_uint<512>> U_0, tapa::mmap<ap_uint<512>> U_1, tapa::mmap<ap_uint<512>> U_2, tapa::mmap<ap_uint<512>> U_3,
  tapa::istream<ReadReqTagged>& req_in,
  tapa::ostream<ReadElemTagged>& elem_out
);

void DepPtrBroadcaster(
  tapa::istream<META_T>& dep_ptr_in,
  tapa::ostream<META_T>& dep_ptr_out_deg,
  tapa::ostream<META_T>& dep_ptr_out_poster
);

void DepFlatBroadcaster(
  tapa::istream<META_T>& dep_flat_in,
  tapa::ostream<META_T>& dep_flat_out_deg,
  tapa::ostream<META_T>& dep_flat_out_poster
);

void Poster(
  tapa::istream<uint32_t>& meta_idx_in,
  tapa::istreams<ReadySeg, NUM_PE>& ready_in_per_pe,
  tapa::istream<META_T>& dep_ptr_in,
  tapa::istream<META_T>& dep_flat_in,
  tapa::ostreams<DepEntry, NUM_PE>& dep_stream_out
);

void HBM_Write_Service(
  tapa::mmap<ap_uint<512>> L_0, tapa::mmap<ap_uint<512>> L_1, tapa::mmap<ap_uint<512>> L_2, tapa::mmap<ap_uint<512>> L_3,
  tapa::mmap<ap_uint<512>> L_4, tapa::mmap<ap_uint<512>> L_5, tapa::mmap<ap_uint<512>> L_6, tapa::mmap<ap_uint<512>> L_7,
  tapa::mmap<ap_uint<512>> U_0, tapa::mmap<ap_uint<512>> U_1, tapa::mmap<ap_uint<512>> U_2, tapa::mmap<ap_uint<512>> U_3,
  tapa::istream<WriteReqTagged>& req_in,
  tapa::istream<WriteElemTagged>& elem_in,
  tapa::ostream<ap_uint<8>>& wr_ack_pe
);

void GlbReqMux(
  tapa::istreams<LUReq, NUM_PE>& glb_req_in,
  tapa::ostream<LUReq>& glb_req_out
);

void GlbRspDemux(
  tapa::istream<LURsp>& glb_rsp_in,
  tapa::ostreams<LURsp, NUM_PE>& glb_rsp_out
);

void GlobalLUCache(
  tapa::istream<LUReq>& req_in,
  tapa::ostream<LURsp>& rsp_out,
  tapa::istream<bool>& done_in
);

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
);

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
  // L矩阵: 8个HBM通道
  tapa::mmap<ap_uint<512>> L_0,
  tapa::mmap<ap_uint<512>> L_1,
  tapa::mmap<ap_uint<512>> L_2,
  tapa::mmap<ap_uint<512>> L_3,
  tapa::mmap<ap_uint<512>> L_4,
  tapa::mmap<ap_uint<512>> L_5,
  tapa::mmap<ap_uint<512>> L_6,
  tapa::mmap<ap_uint<512>> L_7,
  // U矩阵: 4个HBM通道
  tapa::mmap<ap_uint<512>> U_0,
  tapa::mmap<ap_uint<512>> U_1,
  tapa::mmap<ap_uint<512>> U_2,
  tapa::mmap<ap_uint<512>> U_3,
  tapa::mmap<META_T> META_mmap
);


