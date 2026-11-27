// ============================================================================
//  host_lu.cpp
//  真正的图驱动稀疏 LU 分解主机端
//  功能：
//   1. 从矩阵结构生成完整依赖图（deg/dep/posting）
//   2. 构造 segment（分段）表
//   3. 打包为 FPGA META 数据格式
//   4. 调用 SparseLUKernel 运行 LU 分解
// ============================================================================

#include <iostream>       // 控制台输出
#include <vector>         // 动态数组容器
#include <string>         // 字符串操作
#include <cstring>        // C 风格字符串函数
#include <chrono>         // 性能计时
#include <gflags/gflags.h>// 命令行参数解析
#include <algorithm>      // 排序、搜索
#include <iomanip>        // 格式化输出
#include <fstream>        // cosim 输出 bank 回放
#include <unordered_map>  // 哈希表映射
#include <set>            // 去重有序集合
#include <cassert>        // 调试断言
#include <cmath>          // 数值误差计算
#include <limits>         // 数值边界
#include <array>          // 固定大小 bank 容器
#include <cstdlib>        // getenv/setenv
#include <unistd.h>       // execv: make TAPA_CONCURRENCY visible at library startup

#include <ap_int.h>       // FPGA 端类型 (ap_uint)
#include <tapa.h>         // TAPA 框架

#ifdef __TAPA_SIM__
template <typename T>
using aligned_vector = std::vector<T>;
#else
template <typename T>
using aligned_vector = std::vector<T, tapa::aligned_allocator<T>>;
#endif

// 工程内部头文件
#include "type.h"                     // 包含 REAL、常量 A_BANKS/L_BANKS/U_BANKS
#include "../src/lu_kernel.h"         // FPGA 端接口声明
#include "salut_layout_validation.h"
#include "symbolic.h"                 // 符号分析（生成依赖图）
#include "preprocess.h"               // 读取矩阵并标准化
#include "nicslu.h"                   // NICSLU 稀疏求解库接口

// ============================================================================
// 命令行参数定义
// ============================================================================
DEFINE_string(bitstream, "", "Path to FPGA bitstream (.xclbin)"); // FPGA 比特流文件路径
DEFINE_string(matrix_file, "data2/bcsstm37/bcsstm37.mtx", "Path to the input matrix file (.mtx)");     // 输入矩阵文件路径
DEFINE_int32(seg_rows, 2048, "Segment window height");             // 每个 segment 的行数
DEFINE_int32(dense_seg_rows, 256, "Segment height for dense/high-dependency columns");
DEFINE_bool(adaptive_segments, false,
            "Enable mixed segment heights (experimental; uniform segmentation is default)");
DEFINE_uint32(adaptive_dep_threshold, 64,
              "Use dense_seg_rows when a column has at least this many dependencies");
DEFINE_int32(A_bank_base, 0, "HBM bank base for A (0..31)");       // A 矩阵起始 bank ID
DEFINE_int32(L_bank_base, 8, "HBM bank base for L (0..31)");       // L 矩阵起始 bank ID
DEFINE_int32(U_bank_base, 16,"HBM bank base for U (0..31)");       // U 矩阵起始 bank ID
DEFINE_bool(check_result, true, "Compare FPGA L/U values with a CPU static-pivot reference");
DEFINE_bool(fast_model, false,
            "Run the packed-metadata SALUT transaction model instead of TAPA event simulation");
DEFINE_string(cosim_replay_dir, "",
              "Load completed TAPA cosim *_out.bin banks from this directory instead of invoking the kernel");
DEFINE_uint32(sim_cols, 0,
              "Debug a leading column prefix while retaining the matrix row space (0=all)");
DEFINE_int32(tapa_workers, 1,
             "TAPA host-simulation worker count (environment TAPA_CONCURRENCY overrides)");
DEFINE_double(check_abs_tol, 1e-3, "Absolute tolerance for CPU/FPGA comparison");
DEFINE_double(check_rel_tol, 1e-2, "Relative tolerance for CPU/FPGA comparison");
DEFINE_uint32(check_max_errors, 20, "Maximum number of numerical mismatches to print");


// ============================================================================
// float -> 64b（低 32 位保存 float，方便与 FPGA 对齐）
// ============================================================================
inline ap_uint<64> f32_to_u64(float x){
  union { float f; uint32_t u; } t; t.f = x;            // 取 IEEE754 位模式
  ap_uint<64> w; w.range(31,0)=t.u; w.range(63,32)=0;   // 存入低位
  return w;
}

static void load_cosim_u64_bank(const std::string& dir, int file_index,
                                aligned_vector<ap_uint<64>>& bank) {
  const std::string path = dir + "/" + std::to_string(file_index) + "_out.bin";
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if(!input) throw std::runtime_error("cannot open cosim output bank: " + path);
  const std::streamoff actual_bytes = input.tellg();
  const std::streamoff expected_bytes =
      static_cast<std::streamoff>(bank.size() * sizeof(uint64_t));
  if(actual_bytes != expected_bytes) {
    throw std::runtime_error("cosim output bank size mismatch: " + path +
                             " expected=" + std::to_string(expected_bytes) +
                             " actual=" + std::to_string(actual_bytes));
  }
  input.seekg(0);
  for(auto& word : bank) {
    uint64_t raw = 0;
    input.read(reinterpret_cast<char*>(&raw), sizeof(raw));
    if(!input) throw std::runtime_error("short read from cosim output bank: " + path);
    word = raw;
  }
}

// ============================================================================
// 段描述结构（与 FPGA 端 SegRec 完全一致）
// ============================================================================
struct SegRecHost {
  uint32_t j, s;          // 列号 j，段号 s
  uint32_t row_lo, row_hi;// 段对应的行范围
  uint32_t A_addr;        // 段起始地址（packed）- 现在表示512位字偏移
  uint32_t word_count;    // 该段占用的512位字数（稀疏格式）
  uint32_t L_base_addr;   // L(:,j) 的基地址 (kind=1) - 用于写回 L 元素
  uint32_t U_diag_addr;   // U(j,j) 的地址 (kind=2) - 用于写回 U 对角线元素
  uint32_t is_last;       // 是否是列 j 的最后一个段
};

// ============================================================================
// L/U 矩阵的布局结构（bank 分配 + 偏移）
// ============================================================================
struct LLayout {
  std::vector<uint32_t> bank; // 每列 L 的 bank
  std::vector<uint32_t> base; // 每列 L 在 bank 内偏移
  std::vector<uint32_t> used; // 每个 bank 已使用元素数
};
struct ULayout {
  std::vector<uint32_t> bank_ids;    // 可用 U bank 列表
  std::vector<uint32_t> used;        // 每个 U bank 使用量
  std::unordered_map<uint64_t,uint32_t> addr; // (k,j)->packed_addr 映射
};
inline uint64_t key_kj(uint32_t k,uint32_t j){return (uint64_t(k)<<32)|j;} // 组合成唯一键
// pack_addr 已在 lu_kernel.h 中定义，无需重复定义

// ============================================================================
// L 矩阵列主序布局（每列按 i>k 的严格下三角存储）
// ============================================================================
static LLayout layout_L(uint32_t n,
  const std::vector<uint32_t>& L_ptr,const std::vector<uint32_t>& L_idx,
  const std::vector<uint32_t>& banks)
{
  LLayout l; l.bank.resize(n); l.base.resize(n); l.used.assign(banks.size(),0); // 初始化
  const uint32_t nb=banks.size();                                              // bank 数
  for(uint32_t k=0;k<n;k++){                                                   // 遍历每列
    const uint32_t lo=L_ptr[k],hi=L_ptr[k+1];                                  // 取列范围
    uint32_t cnt=0;                                                            // 严格下三角元素数
    for(uint32_t p=lo;p<hi;p++) if(L_idx[p]>k) cnt++;                          // 统计 i>k
    uint32_t lane=k%nb;                                                        // 轮转分配 bank
    l.bank[k]=banks[lane];                                                     // bank ID
    l.base[k]=l.used[lane];                                                    // 列 k 起始偏移
    l.used[lane]+=cnt;                                                         // 更新用量
  }
  return l;
}
static ULayout layout_U(const std::vector<uint32_t>& banks){
  ULayout u; u.bank_ids=banks; u.used.assign(banks.size(),0); return u;        // 初始化 U 布局
}
static uint32_t alloc_U_addr(ULayout& u,uint32_t k,uint32_t j){
  uint64_t key=key_kj(k,j);                                                    // 组合键 (k,j)
  auto it=u.addr.find(key);
  if(it!=u.addr.end()) return it->second;                                      // 已存在则直接返回
  uint32_t lane=j%u.bank_ids.size();                                           // 按列号轮转分配
  uint32_t bank=u.bank_ids[lane];                                              // 目标 bank
  uint32_t off=u.used[lane]++;                                                 // 当前偏移
  uint32_t addr=pack_addr(bank,off);                                           // 打包地址
  u.addr.emplace(key,addr);                                                    // 注册
  return addr;
}

// ============================================================================
// 构建依赖图 + 打包 META 数据（真正的图驱动）
// ============================================================================


struct MetaOut {
  aligned_vector<uint32_t> meta;  // 打包好的 META buffer
  std::vector<uint32_t> A_used,L_used,U_used; // 各 bank 用量（A_used现在表示512位字数）
  aligned_vector<ap_uint<64>> L_init; // lane 0 的 bank-interleaved L 行模板
  std::vector<aligned_vector<ap_uint<64>>> L_init_banks; // PLU2: 8 个 L bank 的行标签模板
  std::vector<SegRecHost> segs;   // 段表（用于写 A）
  std::vector<uint32_t> L_bank,L_base; // 每列 L 的物理布局（用于结果还原）
  std::unordered_map<uint64_t,uint32_t> U_addr; // (k,j)->物理地址
  std::unordered_map<uint64_t,uint32_t> L_addr; // PLU2: (i,j)->物理地址
  // Compact PLU2 result maps.  One 32-bit entry per symbolic factor avoids
  // the multi-gigabyte overhead of unordered_map on large paper matrices.
  std::vector<uint32_t> L_addr_by_pred;
  std::vector<uint32_t> U_col_addr;
};

constexpr uint32_t SEQ_META_MAGIC=0x534C5531u; // "SLU1"
constexpr uint32_t SEQ_META_HEADER_WORDS=8;
constexpr uint32_t SEQ_META_DESC_WORDS=8;

static ap_uint<64> pack_seq_element(uint32_t row,float value) {
  uint32_t bits;
  std::memcpy(&bits,&value,sizeof(bits));
  ap_uint<64> elem=0;
  elem.range(31,0)=bits;
  elem.range(63,32)=row;
  return elem;
}

// 数值优先的顺序稀疏 LU 布局：
// META[8+j*8] = {A_word_addr,A_nnz,L_word_addr,L_nnz,U_word_addr,U_nnz,L_words,0}
// 地址高 8 位是端口 lane，低 24 位是 512 位 word offset。
static MetaOut build_sequential_layout(
  const Symbolic_Matrix& sym,const std::vector<uint32_t>& Lbanks,
  const std::vector<uint32_t>& Ubanks,
  std::vector<aligned_vector<ap_uint<512>>>& A_host)
{
  MetaOut out;
  out.meta.assign(SEQ_META_HEADER_WORDS+sym.n*SEQ_META_DESC_WORDS,0);
  out.meta[0]=SEQ_META_MAGIC;
  out.meta[1]=sym.n;
  out.meta[2]=SEQ_META_HEADER_WORDS;
  out.meta[3]=SEQ_META_DESC_WORDS;
  out.L_bank.resize(sym.n);
  out.L_base.resize(sym.n);
  out.L_used.assign(L_BANKS,0);
  out.U_used.assign(U_BANKS,0);
  out.A_used.assign(A_BANKS,0);

  auto align8=[](uint32_t value){return (value+7u)&~7u;};
  for(uint32_t j=0;j<sym.n;j++) {
    // 正确性基线固定使用每类第 0 个端口，避免动态 bank mux 放大 RTL。
    const uint32_t a_lane=0;
    const uint32_t l_lane=0;
    const uint32_t u_lane=0;
    const uint32_t a_base_word=A_host[a_lane].size();
    const uint32_t a_count=sym.sym_c_ptr[j+1]-sym.sym_c_ptr[j];
    const uint32_t l_count=sym.L_ptr_pred[j+1]-sym.L_ptr_pred[j]-1u;
    const uint32_t u_count=sym.dep_lists_per_col[j].size()+1u;
    const uint32_t l_base_slot=out.L_init.size();
    const uint32_t u_base_slot=out.U_used[u_lane];

    // L 的每个 512-bit word 固定为 slot==row%8。这样 FPGA 可把 work
    // 分成 8 个独立 URAM bank，并行更新一个 word 中的 8 个元素。
    std::array<std::vector<uint32_t>,8> l_rows_by_bank;
    for(uint32_t p=sym.L_ptr_pred[j];p<sym.L_ptr_pred[j+1];p++) {
      const uint32_t row=sym.L_idx_pred[p];
      if(row>j) l_rows_by_bank[row&7u].push_back(row);
    }
    uint32_t l_words=0;
    for(const auto& rows:l_rows_by_bank) {
      l_words=std::max<uint32_t>(l_words,rows.size());
    }
    for(uint32_t round=0;round<l_words;round++) {
      for(uint32_t bank=0;bank<8;bank++) {
        const uint32_t row=round<l_rows_by_bank[bank].size()
          ? l_rows_by_bank[bank][round] : 0xFFFFFFFFu;
        out.L_init.push_back(pack_seq_element(row,0.0f));
      }
    }

    out.L_bank[j]=Lbanks[l_lane];
    out.L_base[j]=l_base_slot;
    out.L_used[l_lane]+=l_words*8u;
    out.U_used[u_lane]+=align8(u_count);

    uint32_t u_ordinal=0;
    for(uint32_t k:sym.dep_lists_per_col[j]) {
      out.U_addr[key_kj(k,j)]=pack_addr(Ubanks[u_lane],u_base_slot+u_ordinal++);
    }
    out.U_addr[key_kj(j,j)]=pack_addr(Ubanks[u_lane],u_base_slot+u_ordinal);

    for(uint32_t p=0;p<a_count;p+=8) {
      ap_uint<512> word=0;
      for(uint32_t slot=0;slot<8;slot++) {
        const uint32_t pos=p+slot;
        const ap_uint<64> elem=pos<a_count
          ? pack_seq_element(sym.sym_r_idx[sym.sym_c_ptr[j]+pos],
                             sym.val[sym.sym_c_ptr[j]+pos])
          : pack_seq_element(0xFFFFFFFFu,0.0f);
        word.range(slot*64+63,slot*64)=elem;
      }
      A_host[a_lane].push_back(word);
    }

    const uint32_t d=SEQ_META_HEADER_WORDS+j*SEQ_META_DESC_WORDS;
    out.meta[d+0]=pack_addr(a_lane,a_base_word);
    out.meta[d+1]=a_count;
    out.meta[d+2]=pack_addr(l_lane,l_base_slot/8u);
    out.meta[d+3]=l_count;
    out.meta[d+4]=pack_addr(u_lane,u_base_slot/8u);
    out.meta[d+5]=u_count;
    out.meta[d+6]=l_words;
  }
  for(uint32_t lane=0;lane<A_BANKS;lane++) out.A_used[lane]=A_host[lane].size();
  std::cout<<"[SEQ] Packed symbolic CSC for sequential FPGA LU: n="<<sym.n
           <<" A_words=";
  uint64_t a_words=0;
  for(uint32_t value:out.A_used) a_words+=value;
  std::cout<<a_words<<" L_slots=";
  uint64_t l_slots=0,u_slots=0;
  for(uint32_t value:out.L_used) l_slots+=value;
  for(uint32_t value:out.U_used) u_slots+=value;
  std::cout<<l_slots<<" U_slots="<<u_slots<<std::endl;
  return out;
}

struct ParColHost {
  uint32_t a_addr=0,a_words=0,u_addr=0,u_words=0,u_count=0;
  uint32_t seg_lo=0,seg_hi=0,dep_lo=0,dep_hi=0;
  uint32_t l_addr=0,l_words=0;
};

struct ParSegHost {
  uint32_t j=0,s=0,row_lo=0,row_hi=0;
  uint32_t a_addr=0,a_words=0,l_addr=0,l_words=0,l_count=0;
  uint32_t u_diag_addr=0,dep_lo=0,dep_hi=0,is_last=0;
  std::vector<ap_uint<64>> a_elements;
  std::vector<uint32_t> l_rows;
};

static void append_tagged_words(
    aligned_vector<ap_uint<512>>& dst,
    const std::vector<ap_uint<64>>& elements) {
  const uint32_t words=(elements.size()+7u)/8u;
  for(uint32_t w=0;w<words;w++) {
    ap_uint<512> packed=0;
    for(uint32_t slot=0;slot<8;slot++) {
      const uint32_t p=w*8u+slot;
      const ap_uint<64> elem=p<elements.size()
        ? elements[p] : pack_seq_element(0xFFFFFFFFu,0.0f);
      packed.range(slot*64+63,slot*64)=elem;
    }
    dst.push_back(packed);
  }
}

// PLU2 数值布局：
//   col_done(k) -> 后继段入度-- -> 列 U/pivot 前端 -> U write ACK
//   -> 同列 L 段并行 -> 每段 L write ACK -> 列内聚合为唯一 col_done(j)。
static MetaOut build_parallel_layout(
    const Symbolic_Matrix& sym,int requested_seg_rows,
    const std::vector<uint32_t>& Abanks,
    const std::vector<uint32_t>& Lbanks,
    const std::vector<uint32_t>& Ubanks,
    std::vector<aligned_vector<ap_uint<512>>>& A_host,
    uint32_t requested_cols=0, bool hardware_limits=false) {
  MetaOut out;
  const uint32_t matrix_n=sym.n;
  const uint32_t n=requested_cols==0
      ? matrix_n : std::min<uint32_t>(requested_cols,matrix_n);
  if (!SalutValidateMetadataLimits(n, 0, hardware_limits))
    throw std::invalid_argument("PLU2 column count exceeds selected hardware/software limit");
  const uint32_t seg_rows=std::max<uint32_t>(1,std::min<uint32_t>(
      requested_seg_rows,PAR_SEG_ROWS_MAX));
  if(static_cast<uint32_t>(requested_seg_rows)!=seg_rows) {
    std::cout<<"[PLU2] seg_rows clamped to "<<seg_rows
             <<" (PE local-buffer limit)"<<std::endl;
  }
  out.A_used.assign(A_BANKS,0);
  out.L_used.assign(L_BANKS,0);
  out.U_used.assign(U_BANKS,0);
  out.L_bank.resize(n);
  out.L_base.resize(n);
  out.L_init_banks.resize(L_BANKS);
  out.L_addr_by_pred.assign(sym.L_idx_pred.size(),UINT32_MAX);
  out.U_col_addr.resize(n);

  std::vector<ParColHost> cols(n);
  std::vector<ParSegHost> segs;
  std::vector<std::vector<uint32_t>> segs_of_col(n);
  std::vector<SalutFactorRange> output_ranges;

  // 先分配每列的 U 连续块，使 U(k,j) 地址在构造段依赖时已确定。
  for(uint32_t j=0;j<n;j++) {
    const uint32_t lane=j%U_BANKS;
    const uint32_t count=sym.dep_lists_per_col[j].size()+1u;
    const uint32_t words=(count+7u)/8u;
    const uint32_t base=out.U_used[lane];
    SalutCheckBankAllocation(Ubanks[lane], base, uint64_t(words)*8u);
    cols[j].u_addr=SalutCheckedPackAddress(Ubanks[lane],base);
    output_ranges.push_back({2,cols[j].u_addr,words*8u,j});
    out.U_col_addr[j]=cols[j].u_addr;
    cols[j].u_words=words;
    cols[j].u_count=count;
    out.U_used[lane]+=words*8u;
  }

  // 列前端 A 在 j%8 上；段 A 用 (j+s)%8 分布。
  for(uint32_t j=0;j<n;j++) {
    std::vector<ap_uint<64>> full_a;
    for(uint32_t p=sym.sym_c_ptr[j];p<sym.sym_c_ptr[j+1];p++)
      full_a.push_back(pack_seq_element(sym.sym_r_idx[p],sym.val[p]));
    const uint32_t front_lane=j%A_BANKS;
    const uint64_t front_base=A_host[front_lane].size();
    SalutCheckBankAllocation(Abanks[front_lane], front_base*8u,
                            ((uint64_t(full_a.size())+7u)/8u)*8u);
    append_tagged_words(A_host[front_lane],full_a);
    cols[j].a_addr=SalutCheckedPackAddress(Abanks[front_lane],front_base*8u);
    cols[j].a_words=(full_a.size()+7u)/8u;
    cols[j].seg_lo=segs.size();

    std::unordered_map<uint32_t,float> a_value;
    std::set<uint32_t> active_rows;
    std::set<uint32_t> l_target_rows;
    for(uint32_t p=sym.sym_c_ptr[j];p<sym.sym_c_ptr[j+1];p++) {
      const uint32_t row=sym.sym_r_idx[p];
      active_rows.insert(row);
      a_value[row]=sym.val[p];
    }
    for(uint32_t p=sym.L_ptr_pred[j];p<sym.L_ptr_pred[j+1];p++) {
      const uint32_t row=sym.L_idx_pred[p];
      if(row>j) {
        active_rows.insert(row);
        l_target_rows.insert(row);
      }
    }

    const uint32_t dense_seg_rows=std::max<uint32_t>(1,std::min<uint32_t>(
        FLAGS_dense_seg_rows,seg_rows));
    const bool dense_column=FLAGS_adaptive_segments&&(
        sym.dep_lists_per_col[j].size()>=FLAGS_adaptive_dep_threshold||
        l_target_rows.size()>=seg_rows);
    const uint32_t column_seg_rows=dense_column?dense_seg_rows:seg_rows;
    uint32_t current_bin=UINT32_MAX;
    for(uint32_t row:active_rows) {
      const uint32_t bin=row/column_seg_rows;
      if(bin!=current_bin) {
        if (segs.size() >= uint64_t(hardware_limits ? 1u<<20 : 1u<<21))
          throw std::invalid_argument("PLU2 segment count exceeds selected hardware/software limit");
        ParSegHost sg;
        sg.j=j; sg.s=bin; sg.row_lo=bin*column_seg_rows;
        sg.row_hi=std::min<uint32_t>(sg.row_lo+column_seg_rows,matrix_n);
        segs.push_back(std::move(sg));
        segs_of_col[j].push_back(segs.size()-1);
        current_bin=bin;
      }
      ParSegHost& sg=segs.back();
      const auto value_it=a_value.find(row);
      const float value=value_it==a_value.end()?0.0f:value_it->second;
      sg.a_elements.push_back(pack_seq_element(row,value));
      if(l_target_rows.count(row)) sg.l_rows.push_back(row);
    }
    // 非空符号列理论上总有 pivot；保留一个控制段处理异常空列。
    if(segs_of_col[j].empty()) {
      if (segs.size() >= uint64_t(hardware_limits ? 1u<<20 : 1u<<21))
        throw std::invalid_argument("PLU2 segment count exceeds selected hardware/software limit");
      ParSegHost sg;
      sg.j=j; sg.s=j/column_seg_rows; sg.row_lo=sg.s*column_seg_rows;
      sg.row_hi=std::min<uint32_t>(sg.row_lo+column_seg_rows,matrix_n);
      sg.a_elements.push_back(pack_seq_element(j,0.0f));
      segs.push_back(std::move(sg));
      segs_of_col[j].push_back(segs.size()-1);
    }
    cols[j].seg_hi=segs.size();

    for(uint32_t sid:segs_of_col[j]) {
      ParSegHost& sg=segs[sid];
      const uint32_t lane=(j+sg.s)%A_BANKS;
      const uint64_t base=A_host[lane].size();
      SalutCheckBankAllocation(Abanks[lane], base*8u,
                              ((uint64_t(sg.a_elements.size())+7u)/8u)*8u);
      append_tagged_words(A_host[lane],sg.a_elements);
      sg.a_addr=SalutCheckedPackAddress(Abanks[lane],base*8u);
      sg.a_words=(sg.a_elements.size()+7u)/8u;
      sg.u_diag_addr=pack_addr(addr_bank(cols[j].u_addr),
          addr_off(cols[j].u_addr)+cols[j].u_count-1u);
    }
    segs[segs_of_col[j].back()].is_last=1;
  }

  // L 每列固定一个 bank，每段独立 512-bit 对齐，杜绝并发段写回相互覆盖。
  for(uint32_t j=0;j<n;j++) {
    const uint32_t lane=j%L_BANKS;
    out.L_bank[j]=Lbanks[lane];
    const uint32_t col_base=out.L_used[lane];
    out.L_base[j]=col_base;
    cols[j].l_addr=SalutCheckedPackAddress(Lbanks[lane],col_base);
    uint32_t pred_pos=sym.L_ptr_pred[j];
    while(pred_pos<sym.L_ptr_pred[j+1]&&sym.L_idx_pred[pred_pos]<=j)
      ++pred_pos;
    for(uint32_t sid:segs_of_col[j]) {
      ParSegHost& sg=segs[sid];
      sg.l_count=sg.l_rows.size();
      sg.l_words=(sg.l_count+7u)/8u;
      SalutCheckBankAllocation(Lbanks[lane],out.L_used[lane],uint64_t(sg.l_words)*8u);
      sg.l_addr=SalutCheckedPackAddress(Lbanks[lane],out.L_used[lane]);
      output_ranges.push_back({1,sg.l_addr,sg.l_words*8u,sid});
      for(uint32_t row:sg.l_rows) {
        const uint32_t off=out.L_init_banks[lane].size();
        if(pred_pos>=sym.L_ptr_pred[j+1]||sym.L_idx_pred[pred_pos]!=row)
          throw std::runtime_error("PLU2 L compact address order mismatch");
        out.L_addr_by_pred[pred_pos++]=SalutCheckedPackAddress(Lbanks[lane],off);
        out.L_init_banks[lane].push_back(pack_seq_element(row,0.0f));
      }
      while((out.L_init_banks[lane].size()&7u)!=0)
        out.L_init_banks[lane].push_back(pack_seq_element(0xFFFFFFFFu,0.0f));
      out.L_used[lane]=out.L_init_banks[lane].size();
    }
    cols[j].l_words=(out.L_used[lane]-col_base)/8u;
  }

  // 列前端依赖读 L(:,k) 整列；段 PE 只读 k 在相同行 bin 的 L 块。
  std::vector<uint32_t> col_dep_ptr(n+1,0),col_dep_flat;
  std::vector<std::vector<uint32_t>> col_post(n);
  std::vector<uint32_t> seg_dep_ptr(segs.size()+1,0),seg_dep_flat;
  std::vector<uint32_t> deg(segs.size(),0);
  std::vector<std::vector<uint32_t>> post(n);
  for(uint32_t j=0;j<n;j++) {
    cols[j].dep_lo=col_dep_ptr[j];
    uint32_t dep_ordinal=0;
    for(uint32_t k:sym.dep_lists_per_col[j]) {
      // U(k,j) only needs L rows up to the target row j.  L blocks of a
      // source column are laid out contiguously in ascending row windows, so
      // trim the burst at the last window that can contain row <= j instead
      // of reading the full lower tail and discarding it in ColumnFront.
      uint32_t l_words_upto_j=0;
      for(uint32_t source_sid:segs_of_col[k]) {
        const ParSegHost& source=segs[source_sid];
        if(source.row_lo>j) break;
        l_words_upto_j+=source.l_words;
      }
      col_dep_flat.push_back(k);
      col_dep_flat.push_back(cols[k].l_addr);
      col_dep_flat.push_back(l_words_upto_j);
      col_dep_flat.push_back(pack_addr(addr_bank(cols[j].u_addr),
          addr_off(cols[j].u_addr)+dep_ordinal));
      col_post[k].push_back(j);
      ++dep_ordinal;
    }
    col_dep_ptr[j+1]=col_dep_ptr[j]+sym.dep_lists_per_col[j].size();
    cols[j].dep_hi=col_dep_ptr[j+1];
  }
  for(uint32_t sid=0;sid<segs.size();sid++) {
    ParSegHost& sg=segs[sid];
    sg.dep_lo=seg_dep_ptr[sid];
    uint32_t dep_ordinal=0;
    for(uint32_t k:sym.dep_lists_per_col[sg.j]) {
      // Adaptive segments need not share bin boundaries.  Generate one
      // DepEntry for every physical source block whose row interval overlaps
      // this target window.  Duplicate posting entries are intentional: the
      // predecessor's single col_done event retires all overlapping blocks.
      for(uint32_t source_sid:segs_of_col[k]) {
        const ParSegHost& source=segs[source_sid];
        const bool overlaps=source.row_lo<sg.row_hi&&sg.row_lo<source.row_hi;
        if(!overlaps||source.l_words==0) continue;
        seg_dep_flat.push_back(k);
        seg_dep_flat.push_back(source.l_addr);
        seg_dep_flat.push_back(source.l_words);
        seg_dep_flat.push_back(pack_addr(addr_bank(cols[sg.j].u_addr),
            addr_off(cols[sg.j].u_addr)+dep_ordinal));
        deg[sid]++;
        post[k].push_back(sid);
      }
      ++dep_ordinal;
    }
    seg_dep_ptr[sid+1]=seg_dep_ptr[sid]+deg[sid];
    sg.dep_hi=seg_dep_ptr[sid+1];
  }
  std::vector<uint32_t> post_ptr(n+1,0),post_flat;
  for(uint32_t k=0;k<n;k++) {
    post_ptr[k+1]=post_ptr[k]+post[k].size();
    post_flat.insert(post_flat.end(),post[k].begin(),post[k].end());
  }
  std::vector<uint32_t> col_post_ptr(n+1,0),col_post_flat;
  for(uint32_t k=0;k<n;k++) {
    col_post_ptr[k+1]=col_post_ptr[k]+col_post[k].size();
    col_post_flat.insert(col_post_flat.end(),col_post[k].begin(),col_post[k].end());
  }

  uint32_t max_col_deps=0,max_col_j=0;
  uint64_t tail_1024_deps=0,tail_397_deps=0;
  uint64_t tail_397_l_words=0,tail_397_one_word=0,tail_397_zero_word=0;
  for(uint32_t j=0;j<n;j++) {
    const uint32_t count=col_dep_ptr[j+1]-col_dep_ptr[j];
    if(count>max_col_deps) { max_col_deps=count; max_col_j=j; }
    if(j+1024u>=n) tail_1024_deps+=count;
    if(j+397u>=n) {
      tail_397_deps+=count;
      for(uint32_t d=col_dep_ptr[j];d<col_dep_ptr[j+1];d++) {
        const uint32_t words=col_dep_flat[d*4u+2u];
        tail_397_l_words+=words;
        tail_397_one_word+=words==1u;
        tail_397_zero_word+=words==0u;
      }
    }
  }
  std::cout<<"[PLU2 U profile] max_col_deps="<<max_col_deps
           <<" at j="<<max_col_j
           <<" last1024="<<tail_1024_deps
           <<" last397="<<tail_397_deps
           <<" tail397_l_words="<<tail_397_l_words
           <<" one_word="<<tail_397_one_word
           <<" zero_word="<<tail_397_zero_word<<std::endl;

  SalutValidateOutputOwnership(output_ranges);
  if (!SalutValidateMetadataLimits(n,uint32_t(segs.size()),hardware_limits))
    throw std::invalid_argument("PLU2 graph exceeds selected hardware/software capacity");
  // All additions are wide BEFORE conversion to the packed 32-bit metadata.
  const uint64_t off_col=PAR_META_HEADER_WORDS;
  const uint64_t off_seg=off_col+uint64_t(n)*PAR_COL_DESC_WORDS;
  const uint64_t off_col_dep_ptr=off_seg+segs.size()*PAR_SEG_DESC_WORDS;
  const uint64_t off_col_dep_flat=off_col_dep_ptr+col_dep_ptr.size();
  const uint64_t off_seg_dep_ptr=off_col_dep_flat+col_dep_flat.size();
  const uint64_t off_seg_dep_flat=off_seg_dep_ptr+seg_dep_ptr.size();
  const uint64_t off_deg=off_seg_dep_flat+seg_dep_flat.size();
  const uint64_t off_post_ptr=off_deg+deg.size();
  const uint64_t off_post_flat=off_post_ptr+post_ptr.size();
  const uint64_t off_col_post_ptr=off_post_flat+post_flat.size();
  const uint64_t off_col_post_flat=off_col_post_ptr+col_post_ptr.size();
  const uint64_t off_end=off_col_post_flat+col_post_flat.size();
  if (off_end > UINT32_MAX)
    throw std::invalid_argument("PLU2 metadata offsets exceed 32-bit capacity before packing");
  out.meta.assign(PAR_META_HEADER_WORDS,0);
  out.meta[0]=PAR_META_MAGIC; out.meta[1]=n; out.meta[2]=segs.size();
  out.meta[3]=seg_rows; out.meta[4]=off_col; out.meta[5]=off_seg;
  out.meta[6]=off_col_dep_ptr; out.meta[7]=off_col_dep_flat;
  out.meta[8]=off_seg_dep_ptr; out.meta[9]=off_seg_dep_flat;
  out.meta[10]=off_deg; out.meta[11]=off_post_ptr;
  out.meta[12]=off_post_flat; out.meta[13]=off_end;
  out.meta[14]=off_col_post_ptr; out.meta[15]=off_col_post_flat;
  for(const ParColHost& c:cols) {
    out.meta.push_back(c.a_addr); out.meta.push_back(c.a_words);
    out.meta.push_back(c.u_addr); out.meta.push_back(c.u_words);
    out.meta.push_back(c.u_count); out.meta.push_back(c.seg_lo);
    out.meta.push_back(c.seg_hi); out.meta.push_back(c.dep_lo);
    out.meta.push_back(c.dep_hi); out.meta.push_back(c.l_words);
  }
  for(const ParSegHost& s:segs) {
    out.meta.push_back(s.j); out.meta.push_back(s.s);
    out.meta.push_back(s.row_lo); out.meta.push_back(s.row_hi);
    out.meta.push_back(s.a_addr); out.meta.push_back(s.a_words);
    out.meta.push_back(s.l_addr); out.meta.push_back(s.l_words);
    out.meta.push_back(s.l_count); out.meta.push_back(s.u_diag_addr);
    out.meta.push_back(s.dep_lo); out.meta.push_back(s.dep_hi);
    out.meta.push_back(s.is_last);
  }
  out.meta.insert(out.meta.end(),col_dep_ptr.begin(),col_dep_ptr.end());
  out.meta.insert(out.meta.end(),col_dep_flat.begin(),col_dep_flat.end());
  out.meta.insert(out.meta.end(),seg_dep_ptr.begin(),seg_dep_ptr.end());
  out.meta.insert(out.meta.end(),seg_dep_flat.begin(),seg_dep_flat.end());
  out.meta.insert(out.meta.end(),deg.begin(),deg.end());
  out.meta.insert(out.meta.end(),post_ptr.begin(),post_ptr.end());
  out.meta.insert(out.meta.end(),post_flat.begin(),post_flat.end());
  out.meta.insert(out.meta.end(),col_post_ptr.begin(),col_post_ptr.end());
  out.meta.insert(out.meta.end(),col_post_flat.begin(),col_post_flat.end());
  if(out.meta.size()>std::numeric_limits<uint32_t>::max())
    throw std::runtime_error("PLU2 metadata exceeds 32-bit offset space");
  for(uint32_t lane=0;lane<A_BANKS;lane++) {
    const uint64_t elements=uint64_t(A_host[lane].size())*8u;
    if(elements>BANK_SIZE)
      throw std::runtime_error("PLU2 A bank exceeds packed address capacity");
    out.A_used[lane]=A_host[lane].size();
  }
  for(uint32_t lane=0;lane<L_BANKS;lane++)
    if(uint64_t(out.L_used[lane])>BANK_SIZE)
      throw std::runtime_error("PLU2 L bank exceeds packed address capacity");
  for(uint32_t lane=0;lane<U_BANKS;lane++)
    if(uint64_t(out.U_used[lane])>BANK_SIZE)
      throw std::runtime_error("PLU2 U bank exceeds packed address capacity");
  std::cout<<"[PLU2] n="<<n<<" segments="<<segs.size()
           <<" column_deps="<<col_dep_flat.size()/4
           <<" segment_deps="<<seg_dep_flat.size()/4
           <<" meta_words="<<out.meta.size()<<std::endl;
  return out;
}

static MetaOut build_meta(
  const Symbolic_Matrix& sym,int seg_rows,
  const std::vector<uint32_t>& Abanks,
  const std::vector<uint32_t>& Lbanks,
  const std::vector<uint32_t>& Ubanks)
{
  const uint32_t n=sym.n;                                             // 矩阵维度
  std::cout << "[DEBUG] build_meta: n=" << n << std::endl;
  
  LLayout L=layout_L(n,sym.L_ptr_pred,sym.L_idx_pred,Lbanks);         // 生成 L 布局
  std::cout << "[DEBUG] L layout done" << std::endl;
  
  ULayout U=layout_U(Ubanks);                                         // 初始化 U 布局
  std::cout << "[DEBUG] U layout done" << std::endl;
  
  // 注意：稀疏格式下不再使用Aused（窗口压缩），bank使用量在fill_A时计算
  std::vector<uint32_t> Aused(Abanks.size(),0);  // 保留用于兼容，但不再更新

  std::vector<SegRecHost> segs;                                       // 段表
  std::vector<std::vector<uint32_t>> segs_of_col(n);                  // 每列的段 ID 集

  std::cout << "[DEBUG] Starting column segmentation..." << std::endl;
  // ---- 按列切段 ----
  for(uint32_t j=0;j<n;j++){                                          // 遍历列
    if(j % 500 == 0) std::cout << "[DEBUG] Processing column " << j << "/" << n << std::endl;
    std::set<uint32_t> rows;                                          // 活跃行集合
    for(uint32_t p=sym.sym_c_ptr[j];p<(uint32_t)sym.sym_c_ptr[j+1];p++)
      rows.insert(sym.sym_r_idx[p]);                                  // A(:,j)
    for(uint32_t k: sym.dep_lists_per_col[j]){                        // 依赖列 k 的 L(:,k)
      for(uint32_t p=sym.L_ptr_pred[k];p<(uint32_t)sym.L_ptr_pred[k+1];p++){
        uint32_t i=sym.L_idx_pred[p]; if(i>k) rows.insert(i);         // 仅 i>k
      }
    }
    if(rows.empty()) continue;                                        // 空列跳过
    
    // 计算该列的 L 和 U 地址（每列只需要计算一次）
    uint32_t L_base_addr = pack_addr(L.bank[j], L.base[j]);          // L(:,j) 的基地址
    uint32_t U_diag_addr = alloc_U_addr(U, j, j);                    // U(j,j) 的地址
    
    uint32_t prev_seg=UINT32_MAX;
    for(uint32_t r:rows){                                             // 遍历行
      uint32_t s=r/seg_rows;                                          // 段号
      if(s!=prev_seg){                                                // 新段
        // 注意：稀疏格式下，A的bank分配和地址在fill_A时确定
        // L和U地址在列级别确定，这里直接填入
        uint32_t lo=s*seg_rows, hi=std::min<uint32_t>(lo+seg_rows,n);
        segs.push_back({j,s,lo,hi,0,0,L_base_addr,U_diag_addr,0});  // A_addr和word_count在fill_A时填充
        segs_of_col[j].push_back(segs.size()-1);
        prev_seg=s;
      }
    }
    // 标记当前列的最后一个段
    if (!segs_of_col[j].empty()) {
      segs[segs_of_col[j].back()].is_last = 1;
    }
  }
  const uint32_t nSeg=segs.size();                                   // 段总数
  std::cout << "[DEBUG] Column segmentation done. Total segments: " << nSeg << std::endl;
  
  // 验证 is_last 标记
  uint32_t is_last_count = 0;
  for (uint32_t sid = 0; sid < nSeg; sid++) {
    if (segs[sid].is_last) is_last_count++;
  }
  std::cout << "[DEBUG] Segments marked as is_last: " << is_last_count << " (expected: " << n << ")" << std::endl;

  // ---- 构造 dep/deg/posting ----
  std::cout << "[DEBUG] Building dep/deg/posting..." << std::endl;
  std::vector<uint32_t> dep_ptr(nSeg+1,0),dep_flat,deg(nSeg,0);      // 初始化表
  std::vector<std::vector<uint32_t>> post(n);                        // posting[k] = 段集合

  for(uint32_t sid=0;sid<nSeg;sid++){                                // 遍历段
    if(sid % 500 == 0) std::cout << "[DEBUG] Processing segment " << sid << "/" << nSeg << std::endl;
    const auto& sg=segs[sid];
    const uint32_t j=sg.j;
    uint32_t deps_added = 0;  // 记录为这个段添加的依赖数
    for(uint32_t k: sym.dep_lists_per_col[j]){                       // 遍历依赖列
      uint32_t lo=sym.L_ptr_pred[k],hi=sym.L_ptr_pred[k+1];
      std::vector<uint32_t> rows;
      for(uint32_t p=lo;p<hi;p++){                                   // 找窗口内的 L 元素
        uint32_t i=sym.L_idx_pred[p];
        if(i>k && i>=sg.row_lo && i<sg.row_hi) rows.push_back(i);
      }
      uint32_t len=rows.size();
      // ✅ 细粒度依赖分析：
      // - 如果 len>0：段内需要 L(:,k) 的数据，需要读取L和U
      // - 如果 len==0：段内不需要 L(:,k) 的数据，但仍需要 U(k,j)，因此仍需要等待列k完成
      // 所以无论 len 是否为0，只要 U(k,j)≠0（已由dep_lists_per_col保证），就应该添加依赖
      uint32_t offL = 0;
      uint32_t Laddr = 0;
      if (len > 0) {
        offL = rows.front()>k? rows.front()-k-1:0;                   // 片内偏移
        Laddr = pack_addr(L.bank[k],L.base[k]+offL);                 // L 片地址
      } else {
        // len==0 时，不需要读取L（PE会跳过L读取），但需要U(k,j)
        Laddr = 0;  // 标记为不需要L
      }
      uint32_t Uaddr=alloc_U_addr(U,k,j);                            // U(k,j) 地址
      dep_flat.push_back(k); dep_flat.push_back(Laddr);              // 依赖项写入
      dep_flat.push_back(len); dep_flat.push_back(Uaddr);
      deg[sid]++; post[k].push_back(sid);                            // 入度+1，登记触发关系
      deps_added++;
    }
    dep_ptr[sid+1]=dep_ptr[sid]+deg[sid];                            // 前缀和累加
    // 打印前20个段的依赖信息
    if (sid < 20) {
      std::cout << "[DEBUG] seg_id=" << sid << " j=" << j << " deps_added=" << deps_added 
                << " deg[sid]=" << deg[sid] << " dep_ptr[sid]=" << dep_ptr[sid] 
                << " dep_ptr[sid+1]=" << dep_ptr[sid+1] << std::endl;
    }
  }
  std::vector<uint32_t> post_ptr(n+1,0),post_flat;
  for(uint32_t k=0;k<n;k++) post_ptr[k+1]=post_ptr[k]+post[k].size();// posting 前缀和
  for(uint32_t k=0;k<n;k++) for(uint32_t sid:post[k]) post_flat.push_back(sid);
  
  std::cout << "[DEBUG] dep/deg/posting done. Packing META..." << std::endl;
  std::cout << "[DEBUG] dep_flat.size()=" << dep_flat.size() << " post_flat.size()=" << post_flat.size() << std::endl;
  std::cout.flush();

  // ---- 打包 META ----
  aligned_vector<uint32_t> meta(12,0);                               // 头部占位
  uint32_t off_seg=12, off_dep_ptr=off_seg+nSeg*9;  // 注意：每个段现在9个uint32（增加了word_count, L_base_addr, U_diag_addr）
  uint32_t off_dep_flat=off_dep_ptr+dep_ptr.size();
  uint32_t off_deg=off_dep_flat+dep_flat.size();
  uint32_t off_post_ptr=off_deg+deg.size();
  uint32_t off_post_flat=off_post_ptr+post_ptr.size();
  uint32_t off_dummy=off_post_flat+post_flat.size();                // META 区域结束位置

  meta[0]=n; meta[1]=seg_rows; meta[2]=nSeg;                         // 核心参数
  meta[3]=off_seg;                                                   // seg_table offset
  meta[4]=off_dep_ptr; meta[5]=off_dep_ptr;                          // unused fields, set to off_dep_ptr
  meta[6]=off_dep_ptr; meta[7]=off_dep_flat;                         // 表偏移
  meta[8]=off_deg; meta[9]=off_post_ptr; meta[10]=off_post_flat;
  meta[11]=off_dummy;                                                // META 区域结束标记

  std::cout << "[DEBUG] Writing seg_table..." << std::endl;
  for(auto&s:segs){                                                  // 写 seg_table
    meta.push_back(s.j);meta.push_back(s.s);
    meta.push_back(s.row_lo);meta.push_back(s.row_hi);
    meta.push_back(s.A_addr);  // 512位字偏移
    meta.push_back(s.word_count);  // 该段占用的字数
    meta.push_back(s.L_base_addr);  // L(:,j) 的基地址
    meta.push_back(s.U_diag_addr);  // U(j,j) 的地址
    meta.push_back(s.is_last);
  }
  std::cout << "[DEBUG] Inserting dep_ptr..." << std::endl;
  meta.insert(meta.end(),dep_ptr.begin(),dep_ptr.end());             // dep_ptr
  std::cout << "[DEBUG] Inserting dep_flat..." << std::endl;
  meta.insert(meta.end(),dep_flat.begin(),dep_flat.end());           // dep_flat
  std::cout << "[DEBUG] Inserting deg..." << std::endl;
  meta.insert(meta.end(),deg.begin(),deg.end());                     // deg_init
  std::cout << "[DEBUG] Inserting post_ptr..." << std::endl;
  meta.insert(meta.end(),post_ptr.begin(),post_ptr.end());           // posting_ptr
  std::cout << "[DEBUG] Inserting post_flat..." << std::endl;
  meta.insert(meta.end(),post_flat.begin(),post_flat.end());         // posting_flat
  std::cout << "[DEBUG] META packing complete" << std::endl;

  MetaOut o; 
  o.meta.assign(meta.begin(), meta.end()); 
  // 注意：稀疏格式下，A_used不再表示64位元素数，而是512位字数（在fill_A后更新）
  // 为了兼容，这里先保留原值
  o.A_used=Aused; 
  o.L_used=L.used; 
  o.U_used=U.used; 
  o.segs=segs;
  o.L_bank=L.bank;
  o.L_base=L.base;
  o.U_addr=U.addr;
  std::cout<<"[META] n="<<n<<" segs="<<nSeg<<" deps="<<dep_flat.size()/4<<"\n";
  std::cout<<"[META] Note: A is now sparse format (512-bit words), A_used will be updated after fill_A\n";
  return o;
}

// ============================================================================
// 打包元素：存储row_offset和value（64位）
// ============================================================================
inline ap_uint<64> pack_element_with_offset(uint32_t row_offset, float val) {
  ap_uint<64> packed = 0;
  packed(15, 0) = ap_uint<16>(row_offset);  // 低16位：行偏移（相对于row_lo，0-511）
  packed(63, 32) = tapa::bit_cast<ap_uint<32>>(val);  // 高32位：浮点值位模式
  return packed;
}

// DUMMY_ELEMENT定义（用于填充空槽位）
const ap_uint<64> DUMMY_ELEMENT_SPARSE = (ap_uint<64>(0xFFFF) << 16);

// ============================================================================
// 填充 A：使用稀疏槽位映射格式（只存储非零元素）
// ============================================================================
static void fill_A(const double* ax, const unsigned* ai, const unsigned* ap,
  std::vector<SegRecHost>& segs,  // 注意：需要修改segs中的地址和字数信息
  std::vector<aligned_vector<ap_uint<512>>>& A_host,  // 改为512位字数组
  uint32_t n)  // 添加矩阵维度参数用于边界检查
{
  constexpr int PACKED_ELEMENTS_PER_HBM_WORD = 8;  // 每个512位字包含8个64位元素
  std::vector<uint32_t> bank_word_count(A_BANKS, 0);  // 每个bank当前使用的512位字数
  
  for(auto& seg : segs){  // 遍历每段
    uint32_t j = seg.j, lo = seg.row_lo, hi = seg.row_hi;
    
    // 边界检查
    if(j >= n) {
      std::cerr << "[ERROR] fill_A: Invalid column index j=" << j << ", n=" << n << std::endl;
      continue;
    }
    if(ap[j] > ap[j+1]) {
      std::cerr << "[ERROR] fill_A: Invalid column pointer for j=" << j << std::endl;
      continue;
    }
    
    // 1. 收集该段窗口内的非零元素
    std::vector<std::pair<uint32_t, double>> nnz_elements;
    // 确保 ap[j+1] 不会越界
    if(j+1 > n) {
      std::cerr << "[ERROR] fill_A: Column index j+1=" << (j+1) << " exceeds n=" << n << std::endl;
      continue;
    }
    for(uint32_t p = ap[j]; p < ap[j+1]; p++) {
      uint32_t r = ai[p];
      if(r >= lo && r < hi) {
        uint32_t row_offset = r - lo;  // 相对于row_lo的偏移（0到win-1）
        nnz_elements.push_back({row_offset, ax[p]});
      }
    }
    
    if(nnz_elements.empty()) {
      // 空段：word_count = 0，A_addr设为0（无效地址标记）
      seg.word_count = 0;
      seg.A_addr = 0;  // 明确设置为0，表示无效地址
      continue;
    }
    
    // 2. 选择bank（负载均衡）
    uint32_t bank_sel = 0;
    uint32_t min_words = bank_word_count[0];
    for(uint32_t b = 1; b < A_BANKS; b++) {
      if(bank_word_count[b] < min_words) {
        min_words = bank_word_count[b];
        bank_sel = b;
      }
    }
    
    uint32_t bank = bank_sel;
    // 确保 bank 索引有效
    if(bank >= A_BANKS) {
      std::cerr << "[ERROR] fill_A: Invalid bank index " << bank << " (max=" << A_BANKS << ")" << std::endl;
      continue;
    }
    if(bank >= A_host.size()) {
      std::cerr << "[ERROR] fill_A: Bank index " << bank << " exceeds A_host size " << A_host.size() << std::endl;
      continue;
    }
    uint32_t word_offset = bank_word_count[bank];  // 该段在bank中的起始字偏移
    
    // 3. 使用槽位映射打包非零元素
    ap_uint<512> hbm_word_buffer = 0;
    std::vector<bool> slot_occupied(PACKED_ELEMENTS_PER_HBM_WORD, false);
    uint32_t words_count = 0;
    
    for(const auto& elem : nnz_elements) {
      uint32_t row_offset = elem.first;
      int target_slot = row_offset % PACKED_ELEMENTS_PER_HBM_WORD;  // 槽位映射：row_offset % 8
      
      // 如果槽位冲突，先写入当前字
      if(slot_occupied[target_slot]) {
        // 填充空槽位为DUMMY
        for(int s = 0; s < PACKED_ELEMENTS_PER_HBM_WORD; s++) {
          if(!slot_occupied[s]) {
            hbm_word_buffer(s*64+63, s*64) = DUMMY_ELEMENT_SPARSE;
          }
        }
        // 写入HBM
        A_host[bank].push_back(hbm_word_buffer);
        words_count++;
        
        // 重置缓冲区
        std::fill(slot_occupied.begin(), slot_occupied.end(), false);
        hbm_word_buffer = 0;
      }
      
      // 打包元素：存储row_offset和value
      ap_uint<64> packed_elem = pack_element_with_offset(row_offset, static_cast<float>(elem.second));
      
      // 放入槽位
      hbm_word_buffer(target_slot*64+63, target_slot*64) = packed_elem;
      slot_occupied[target_slot] = true;
    }
    
    // 4. 写入最后一个字（如果有）
    bool has_occupied = false;
    for(int k = 0; k < PACKED_ELEMENTS_PER_HBM_WORD; k++) {
      if(slot_occupied[k]) has_occupied = true;
    }
    if(has_occupied) {
      for(int s = 0; s < PACKED_ELEMENTS_PER_HBM_WORD; s++) {
        if(!slot_occupied[s]) {
          hbm_word_buffer(s*64+63, s*64) = DUMMY_ELEMENT_SPARSE;
        }
      }
      A_host[bank].push_back(hbm_word_buffer);
      words_count++;
    }
    
    // 5. 更新段的地址和字数信息
    seg.A_addr = pack_addr(bank, word_offset);  // 存储512位字偏移
    seg.word_count = words_count;
    
    // 6. 更新bank使用量
    bank_word_count[bank] += words_count;
  }
  
  // 打印统计信息
  uint64_t total_words = 0;
  for(uint32_t b = 0; b < A_BANKS; b++) {
    total_words += bank_word_count[b];
    std::cout << "[DEBUG] fill_A: Bank " << b << " packed " << bank_word_count[b] 
              << " words (" << A_host[b].size() << " total)" << std::endl;
  }
  std::cout << "[DEBUG] fill_A: Total packed words: " << total_words << std::endl;
  
  // 返回bank使用量（用于统计）
  // 注意：这需要在函数外部访问，可以通过MetaOut或全局变量
  // 这里暂时不返回，如果需要可以在MetaOut中添加
}

// ============================================================================
// 计算 FPGA LU 分解的 FLOPs
// ============================================================================
double calculate_fpga_lu_flops(const Symbolic_Matrix& A_sym) {
    double total_flops = 0.0;
    unsigned int n = A_sym.n;

    std::vector<unsigned int> llen(n, 0);
    std::vector<unsigned int> ulen(n, 0);

    // 计算每列的 L 和 U 长度
    for (unsigned int j = 0; j < n; ++j) {
        for (unsigned int p = A_sym.sym_c_ptr[j]; p < A_sym.sym_c_ptr[j+1]; ++p) {
            unsigned int r_idx = A_sym.sym_r_idx[p];
            if (r_idx > j) {
                llen[j]++;
            } else {
                ulen[j]++;
            }
        }
    }

    // 计算总 FLOPs：对于每列 j，遍历其依赖列 k，累加 L(:,k) 的长度 * 2（乘法和减法）
    // 然后加上该列的 L 长度（除法操作）
    for (unsigned int j = 0; j < n; ++j) {
        for (unsigned int k_left_col : A_sym.dep_lists_per_col[j]) {
            if (k_left_col < n) {
                total_flops += (double)llen[k_left_col] * 2.0;  // 每个 L 元素需要一次乘法和一次减法
            }
        }
        total_flops += (double)llen[j] * 1.0;  // L 列的除法操作
    }

    return total_flops;
}

// ============================================================================
// CPU/FPGA 数值结果对比
// CPU 参考使用与 FPGA 一致的单精度、静态主元、单位下三角约定。
// L_host/U_host 的每个 64 位槽位中，低 32 位是 float 的 IEEE754 位模式。
// ============================================================================
struct CpuLUReference {
  std::vector<float> L_values; // 与 sym.L_idx_pred 一一对应；只使用 row>col 项
  std::vector<float> U_values; // 与 sym.sym_r_idx 对齐；只使用 row<=col 项
  uint32_t replaced_pivots=0;
};

static CpuLUReference build_cpu_reference(const Symbolic_Matrix& sym,
                                           uint32_t active_cols) {
  CpuLUReference ref;
  ref.L_values.assign(sym.L_idx_pred.size(),0.0f);

  ref.U_values.assign(sym.sym_r_idx.size(),0.0f);

  // sym 的每一列已经包含符号填充结构，因此可以复用一个稠密工作向量，
  // 当前列涉及的所有位置都会在消去前被重新初始化。
  std::vector<float> work(sym.n,0.0f);
  for(uint32_t j=0;j<active_cols;j++) {
    for(uint32_t p=sym.sym_c_ptr[j];p<sym.sym_c_ptr[j+1];p++) {
      work[sym.sym_r_idx[p]]=sym.val[p];
    }

    // 依赖按行号递增，等价于无动态选主元的左看稀疏 LU。
    uint32_t u_pos=sym.sym_c_ptr[j];
    for(uint32_t k:sym.dep_lists_per_col[j]) {
      const float ukj=work[k];
      while(u_pos<sym.sym_c_ptr[j+1]&&sym.sym_r_idx[u_pos]<k) ++u_pos;
      if(u_pos>=sym.sym_c_ptr[j+1]||sym.sym_r_idx[u_pos]!=k)
        throw std::runtime_error("CPU U compact value order mismatch");
      ref.U_values[u_pos++]=ukj;
      for(uint32_t p=sym.L_ptr_pred[k];p<sym.L_ptr_pred[k+1];p++) {
        const uint32_t row=sym.L_idx_pred[p];
        if(row>k) work[row]=static_cast<float>(work[row]-ref.L_values[p]*ukj);
      }
    }

    const float raw_pivot=work[j];
    const float pivot=StabilizeStaticPivot(raw_pivot);
    while(u_pos<sym.sym_c_ptr[j+1]&&sym.sym_r_idx[u_pos]<j) ++u_pos;
    if(u_pos>=sym.sym_c_ptr[j+1]||sym.sym_r_idx[u_pos]!=j)
      throw std::runtime_error("CPU compact pivot order mismatch");
    ref.U_values[u_pos]=pivot;
    if(pivot!=raw_pivot) ref.replaced_pivots++;

    for(uint32_t p=sym.L_ptr_pred[j];p<sym.L_ptr_pred[j+1];p++) {
      const uint32_t row=sym.L_idx_pred[p];
      if(row<=j) continue;
      ref.L_values[p]=static_cast<float>(work[row]/pivot);
    }
  }
  return ref;
}

static int find_bank_lane(const std::vector<uint32_t>& banks,uint32_t bank) {
  for(size_t i=0;i<banks.size();i++) if(banks[i]==bank) return static_cast<int>(i);
  return -1;
}

static float unpack_result_float(ap_uint<64> word) {
  uint32_t bits=static_cast<uint32_t>(word.range(31,0));
  float value;
  std::memcpy(&value,&bits,sizeof(value));
  return value;
}

struct FastPackedElement {
  uint32_t row;
  float value;
};

static FastPackedElement unpack_fast_element(ap_uint<64> word) {
  FastPackedElement elem;
  elem.row=static_cast<uint32_t>(word.range(63,32));
  elem.value=unpack_result_float(word);
  return elem;
}

// Fast functional model of the synthesized SALUT transaction graph.  Unlike
// build_cpu_reference(), this path consumes the packed PLU2 descriptors and
// physical-bank addresses, reads the same padded A/L/U bursts as the RTL, and
// writes results back through the generated HBM layout.  It intentionally
// omits FIFO cycle timing; protocol timing remains covered by TAPA/RTL tests.
static void run_fast_salut_model(
  const MetaOut& layout,const std::vector<uint32_t>& Abanks,
  const std::vector<uint32_t>& Lbanks,const std::vector<uint32_t>& Ubanks,
  const std::vector<aligned_vector<ap_uint<512>>>& A_host,
  std::vector<aligned_vector<ap_uint<64>>>& L_host,
  std::vector<aligned_vector<ap_uint<64>>>& U_host)
{
  const auto& meta=layout.meta;
  if(meta.size()<PAR_META_HEADER_WORDS||meta[0]!=PAR_META_MAGIC)
    throw std::runtime_error("fast SALUT model received invalid PLU2 metadata");
  const uint32_t n=meta[1],nseg=meta[2],seg_rows=meta[3];
  const uint32_t off_col=meta[4],off_seg=meta[5];
  const uint32_t off_col_dep_flat=meta[7],off_seg_dep_flat=meta[9];

  std::array<int,256> a_lane_of_bank,l_lane_of_bank,u_lane_of_bank;
  a_lane_of_bank.fill(-1); l_lane_of_bank.fill(-1); u_lane_of_bank.fill(-1);
  for(size_t i=0;i<Abanks.size();++i) a_lane_of_bank[Abanks[i]&255u]=i;
  for(size_t i=0;i<Lbanks.size();++i) l_lane_of_bank[Lbanks[i]&255u]=i;
  for(size_t i=0;i<Ubanks.size();++i) u_lane_of_bank[Ubanks[i]&255u]=i;

  // Decode each physical factor slot once.  Keeping packed ap_uint bit-range
  // operations out of the tens-of-billions-of-MAC hot loop makes full-matrix
  // software validation practical without changing arithmetic order.
  std::vector<std::vector<uint32_t>> l_rows(L_host.size()),u_rows_mem(U_host.size());
  std::vector<std::vector<float>> l_values(L_host.size()),u_values_mem(U_host.size());
  for(size_t lane=0;lane<L_host.size();++lane) {
    l_rows[lane].resize(L_host[lane].size());
    l_values[lane].resize(L_host[lane].size());
    for(size_t off=0;off<L_host[lane].size();++off) {
      l_rows[lane][off]=static_cast<uint32_t>(L_host[lane][off].range(63,32));
      l_values[lane][off]=unpack_result_float(L_host[lane][off]);
    }
  }
  for(size_t lane=0;lane<U_host.size();++lane) {
    u_rows_mem[lane].resize(U_host[lane].size());
    u_values_mem[lane].resize(U_host[lane].size());
    for(size_t off=0;off<U_host[lane].size();++off) {
      u_rows_mem[lane][off]=static_cast<uint32_t>(U_host[lane][off].range(63,32));
      u_values_mem[lane][off]=unpack_result_float(U_host[lane][off]);
    }
  }

  auto read_a=[&](uint32_t packed_addr,uint32_t index)->ap_uint<64> {
    const int lane=a_lane_of_bank[addr_bank(packed_addr)];
    const uint32_t off=addr_off(packed_addr)+index;
    if(lane<0||static_cast<size_t>(lane)>=A_host.size()||
       off/8u>=A_host[lane].size())
      throw std::runtime_error("fast SALUT A address out of range");
    return A_host[lane][off/8u].range((off%8u)*64u+63u,(off%8u)*64u);
  };
  std::vector<float> front_work(n,0.0f);
  std::vector<uint32_t> front_stamp(n,0);
  std::vector<float> seg_work(seg_rows,0.0f);
  std::vector<uint8_t> seg_active(seg_rows,0);
  uint64_t modeled_segments=0;

  for(uint32_t j=0;j<n;j++) {
    const uint32_t c=off_col+j*PAR_COL_DESC_WORDS;
    const uint32_t epoch=j+1;
    const uint32_t front_a_addr=meta[c+0],front_a_len=meta[c+1]*8u;
    const uint32_t u_addr=meta[c+2],u_len=meta[c+3]*8u;
    const uint32_t dep_lo=meta[c+7],dep_hi=meta[c+8];
    for(uint32_t p=0;p<front_a_len;p++) {
      const FastPackedElement elem=unpack_fast_element(read_a(front_a_addr,p));
      if(elem.row>=n) continue;
      front_work[elem.row]=elem.value;
      front_stamp[elem.row]=epoch;
    }
    std::vector<uint32_t> u_rows;
    std::vector<float> u_values;
    u_rows.reserve(dep_hi-dep_lo+1u);
    u_values.reserve(dep_hi-dep_lo+1u);
    for(uint32_t d=dep_lo;d<dep_hi;d++) {
      const uint32_t p=off_col_dep_flat+d*4u;
      const uint32_t k=meta[p+0],l_addr=meta[p+1],l_len=meta[p+2]*8u;
      const float ukj=front_stamp[k]==epoch?front_work[k]:0.0f;
      u_rows.push_back(k); u_values.push_back(ukj);
      const int l_lane=l_lane_of_bank[addr_bank(l_addr)];
      const uint32_t l_base=addr_off(l_addr);
      for(uint32_t q=0;q<l_len;q++) {
        const uint32_t row=l_rows[l_lane][l_base+q];
        if(row>=n||row>j) continue;
        if(front_stamp[row]!=epoch) {
          front_work[row]=0.0f;
          front_stamp[row]=epoch;
        }
        front_work[row]=static_cast<float>(
            front_work[row]-l_values[l_lane][l_base+q]*ukj);
      }
    }
    const float pivot=StabilizeStaticPivot(
        front_stamp[j]==epoch?front_work[j]:0.0f);
    u_rows.push_back(j); u_values.push_back(pivot);
    const int u_lane=u_lane_of_bank[addr_bank(u_addr)];
    const uint32_t u_base=addr_off(u_addr);
    for(uint32_t q=0;q<u_len;q++) {
      u_rows_mem[u_lane][u_base+q]=q<u_rows.size()?u_rows[q]:0xFFFFFFFFu;
      u_values_mem[u_lane][u_base+q]=q<u_values.size()?u_values[q]:0.0f;
    }

    for(uint32_t sid=meta[c+5];sid<meta[c+6];sid++) {
      if(sid>=nseg) throw std::runtime_error("fast SALUT segment id out of range");
      const uint32_t s=off_seg+sid*PAR_SEG_DESC_WORDS;
      const uint32_t row_lo=meta[s+2],row_hi=meta[s+3];
      const uint32_t win_len=row_hi-row_lo;
      std::fill(seg_work.begin(),seg_work.begin()+win_len,0.0f);
      std::fill(seg_active.begin(),seg_active.begin()+win_len,0);
      const uint32_t a_addr=meta[s+4],a_len=meta[s+5]*8u;
      for(uint32_t q=0;q<a_len;q++) {
        const FastPackedElement elem=unpack_fast_element(read_a(a_addr,q));
        if(elem.row<row_lo||elem.row>=row_hi) continue;
        seg_work[elem.row-row_lo]=elem.value;
        seg_active[elem.row-row_lo]=1;
      }
      const uint32_t sdep_lo=meta[s+10],sdep_hi=meta[s+11];
      for(uint32_t d=sdep_lo;d<sdep_hi;d++) {
        const uint32_t p=off_seg_dep_flat+d*4u;
        const uint32_t l_addr=meta[p+1],l_len=meta[p+2]*8u;
        const uint32_t uj_addr=meta[p+3];
        const int uj_lane=u_lane_of_bank[addr_bank(uj_addr)];
        const float ukj=u_values_mem[uj_lane][addr_off(uj_addr)];
        const int l_lane=l_lane_of_bank[addr_bank(l_addr)];
        const uint32_t l_base=addr_off(l_addr);
        for(uint32_t q=0;q<l_len;q++) {
          const uint32_t row=l_rows[l_lane][l_base+q];
          if(row<row_lo||row>=row_hi) continue;
          const uint32_t off=row-row_lo;
          seg_work[off]=static_cast<float>(
              seg_work[off]-l_values[l_lane][l_base+q]*ukj);
          seg_active[off]=1;
        }
      }
      const uint32_t l_addr=meta[s+6],l_len=meta[s+7]*8u;
      const int out_lane=l_lane_of_bank[addr_bank(l_addr)];
      const uint32_t out_base=addr_off(l_addr);
      uint32_t emitted=0;
      for(uint32_t off=0;off<win_len;off++) {
        const uint32_t row=row_lo+off;
        if(row>j&&seg_active[off]) {
          l_rows[out_lane][out_base+emitted]=row;
          l_values[out_lane][out_base+emitted]=
              static_cast<float>(seg_work[off]/pivot);
          ++emitted;
        }
      }
      for(uint32_t q=emitted;q<l_len;q++) {
        l_rows[out_lane][out_base+q]=0xFFFFFFFFu;
        l_values[out_lane][out_base+q]=0.0f;
      }
      if(emitted!=meta[s+8])
        throw std::runtime_error("fast SALUT model emitted unexpected L count");
      ++modeled_segments;
    }
    if(((j+1u)&4095u)==0||j+1u==n)
      std::cout<<"[FastModel] completed_columns="<<(j+1u)<<"/"<<n<<std::endl;
  }
  for(size_t lane=0;lane<L_host.size();++lane)
    for(size_t off=0;off<L_host[lane].size();++off)
      L_host[lane][off]=pack_seq_element(l_rows[lane][off],l_values[lane][off]);
  for(size_t lane=0;lane<U_host.size();++lane)
    for(size_t off=0;off<U_host[lane].size();++off)
      U_host[lane][off]=pack_seq_element(u_rows_mem[lane][off],u_values_mem[lane][off]);
  std::cout<<"[FastModel] modeled_segments="<<modeled_segments<<"/"<<nseg<<"\n";
}

struct CompareStats {
  uint64_t expected=0,matching=0,mismatching=0,nonzero_output=0,invalid_addr=0;
  uint64_t significant_expected=0,significant_matching=0;
  double max_abs_error=0.0,max_rel_error=0.0,sum_abs_error=0.0;
};

static void compare_one_value(const char* kind,uint32_t row,uint32_t col,
  uint32_t bank,uint32_t off,float cpu,float fpga,CompareStats& stats,
  uint32_t& printed_errors)
{
  stats.expected++;
  if(fpga!=0.0f) stats.nonzero_output++;
  const double abs_error=std::fabs(static_cast<double>(fpga)-static_cast<double>(cpu));
  const double rel_error=abs_error/std::max(1.0,std::fabs(static_cast<double>(cpu)));
  stats.max_abs_error=std::max(stats.max_abs_error,abs_error);
  stats.max_rel_error=std::max(stats.max_rel_error,rel_error);
  stats.sum_abs_error+=abs_error;
  const bool significant=std::fabs(static_cast<double>(cpu))>FLAGS_check_abs_tol;
  if(significant) stats.significant_expected++;
  const bool ok=std::isfinite(cpu)&&std::isfinite(fpga)&&
    abs_error<=FLAGS_check_abs_tol+FLAGS_check_rel_tol*std::fabs(static_cast<double>(cpu));
  if(ok) {
    stats.matching++;
    if(significant) stats.significant_matching++;
  } else {
    stats.mismatching++;
    if(printed_errors<FLAGS_check_max_errors) {
      std::cerr<<std::scientific<<std::setprecision(6)
               <<"[CHECK][MISMATCH] "<<kind<<"("<<row<<","<<col<<")"
               <<" cpu="<<cpu<<" fpga="<<fpga
               <<" abs_err="<<abs_error<<" rel_err="<<rel_error
               <<" bank="<<bank<<" off="<<off<<"\n";
      printed_errors++;
    }
  }
}

static bool compare_cpu_fpga(const Symbolic_Matrix& sym,const MetaOut& layout,
  const std::vector<uint32_t>& Lbanks,const std::vector<uint32_t>& Ubanks,
  const std::vector<aligned_vector<ap_uint<64>>>& L_host,
  const std::vector<aligned_vector<ap_uint<64>>>& U_host)
{
  std::cout<<"[CHECK] Building CPU float static-pivot LU reference..."<<std::endl;
  const auto start=std::chrono::high_resolution_clock::now();
  CpuLUReference cpu=build_cpu_reference(sym,layout.L_bank.size());
  const auto end=std::chrono::high_resolution_clock::now();
  const double cpu_ms=std::chrono::duration<double,std::milli>(end-start).count();
  std::cout<<std::fixed<<std::setprecision(3)
           <<"[CHECK] CPU reference completed in "<<cpu_ms<<" ms"
           <<"; replaced static pivots="<<cpu.replaced_pivots<<std::endl;

  CompareStats l_stats,u_stats;
  uint32_t printed_errors=0;

  for(uint32_t j=0;j<layout.L_bank.size();j++) {
    const bool parallel_layout=!layout.meta.empty()&&layout.meta[0]==PAR_META_MAGIC;
    const uint32_t legacy_bank=layout.L_bank[j];
    const int legacy_lane=find_bank_lane(Lbanks,legacy_bank);
    std::array<uint32_t,8> bank_ordinal{};
    for(uint32_t p=sym.L_ptr_pred[j];p<sym.L_ptr_pred[j+1];p++) {
      const uint32_t row=sym.L_idx_pred[p];
      if(row<=j) continue;
      uint32_t bank=legacy_bank,off=0;
      int lane=legacy_lane;
      if(parallel_layout) {
        if(p>=layout.L_addr_by_pred.size()||layout.L_addr_by_pred[p]==UINT32_MAX) {
          l_stats.expected++; l_stats.invalid_addr++; l_stats.mismatching++;
          continue;
        }
        bank=addr_bank(layout.L_addr_by_pred[p]);
        off=addr_off(layout.L_addr_by_pred[p]);
        lane=find_bank_lane(Lbanks,bank);
      } else {
        const uint32_t row_bank=row&7u;
        off=layout.L_base[j]+bank_ordinal[row_bank]++*8u+row_bank;
      }
      if(lane<0||static_cast<size_t>(lane)>=L_host.size()||off>=L_host[lane].size()) {
        l_stats.expected++; l_stats.invalid_addr++; l_stats.mismatching++;
        continue;
      }
      compare_one_value("L",row,j,bank,off,cpu.L_values[p],
                        unpack_result_float(L_host[lane][off]),l_stats,printed_errors);
    }
  }

  if(!layout.U_col_addr.empty()) {
    for(uint32_t col=0;col<layout.U_col_addr.size();++col) {
      const uint32_t base_addr=layout.U_col_addr[col];
      const uint32_t bank=addr_bank(base_addr);
      const uint32_t base=addr_off(base_addr);
      const int lane=find_bank_lane(Ubanks,bank);
      uint32_t ordinal=0;
      for(uint32_t p=sym.sym_c_ptr[col];p<sym.sym_c_ptr[col+1];++p) {
        const uint32_t row=sym.sym_r_idx[p];
        if(row>col) break;
        const uint32_t off=base+ordinal++;
        if(lane<0||static_cast<size_t>(lane)>=U_host.size()||
           off>=U_host[lane].size()||p>=cpu.U_values.size()) {
          u_stats.expected++; u_stats.invalid_addr++; u_stats.mismatching++;
          continue;
        }
        compare_one_value("U",row,col,bank,off,cpu.U_values[p],
                          unpack_result_float(U_host[lane][off]),u_stats,
                          printed_errors);
      }
    }
  } else {
    for(const auto& entry:layout.U_addr) {
      const uint64_t key=entry.first;
      const uint32_t row=static_cast<uint32_t>(key>>32);
      const uint32_t col=static_cast<uint32_t>(key);
      const uint32_t bank=addr_bank(entry.second);
      const uint32_t off=addr_off(entry.second);
      const int lane=find_bank_lane(Ubanks,bank);
      if(lane<0||static_cast<size_t>(lane)>=U_host.size()||off>=U_host[lane].size()) {
        u_stats.expected++; u_stats.invalid_addr++; u_stats.mismatching++;
        continue;
      }
      const auto it=std::lower_bound(sym.sym_r_idx.begin()+sym.sym_c_ptr[col],
          sym.sym_r_idx.begin()+sym.sym_c_ptr[col+1],row);
      if(it==sym.sym_r_idx.begin()+sym.sym_c_ptr[col+1]) {
        u_stats.expected++; u_stats.invalid_addr++; u_stats.mismatching++;
        continue;
      }
      const uint32_t p=it-sym.sym_r_idx.begin();
      compare_one_value("U",row,col,bank,off,cpu.U_values[p],
                        unpack_result_float(U_host[lane][off]),u_stats,
                        printed_errors);
    }
  }

  auto print_stats=[](const char* name,const CompareStats& s) {
    const double accuracy=s.expected?100.0*static_cast<double>(s.matching)/s.expected:0.0;
    const double significant_accuracy=s.significant_expected?
      100.0*static_cast<double>(s.significant_matching)/s.significant_expected:0.0;
    const double mean_abs=s.expected?s.sum_abs_error/static_cast<double>(s.expected):0.0;
    std::cout<<name<<": expected="<<s.expected
             <<", nonzero_fpga="<<s.nonzero_output
             <<", matching="<<s.matching
             <<", mismatching="<<s.mismatching
             <<", invalid_addr="<<s.invalid_addr
             <<", accuracy="<<std::fixed<<std::setprecision(2)<<accuracy<<"%\n"
             <<"  significant_cpu="<<s.significant_expected
             <<", significant_matching="<<s.significant_matching
             <<", significant_accuracy="<<significant_accuracy<<"%\n"
             <<"  max_abs="<<std::scientific<<s.max_abs_error
             <<", mean_abs="<<mean_abs
             <<", max_rel="<<s.max_rel_error<<"\n";
  };

  std::cout<<"\n=== CPU/"<<(FLAGS_fast_model?"SALUT Transaction Model":"FPGA")
           <<" LU Numerical Comparison ===\n"
           <<"Reference: float static-pivot LU (unit lower L)\n"
           <<"Tolerance: abs="<<FLAGS_check_abs_tol<<", rel="<<FLAGS_check_rel_tol<<"\n";
  print_stats("L",l_stats);
  print_stats("U",u_stats);
  const bool pass=l_stats.mismatching==0&&u_stats.mismatching==0;
  std::cout<<"Overall numerical check: "<<(pass?"PASS":"FAIL")<<std::endl;
  if(!pass) {
    std::cerr<<"[CHECK] "<<(FLAGS_fast_model?"SALUT model":"FPGA")
             <<" L/U does not match the CPU reference."<<std::endl;
  }
  std::cout<<std::defaultfloat;
  return pass;
}

// ============================================================================
// 主函数入口
// ============================================================================
int main(int argc,char**argv){
  // libtapa creates its worker pool before the first kernel call, so setting
  // TAPA_CONCURRENCY immediately before tapa::invoke is too late on this
  // runtime.  Re-exec once with the environment already present, preserving
  // every command-line option.  Exact transport-event simulation needs many
  // workers; the compute-control backend is fastest and deterministic with 1.
  if(std::getenv("TAPA_CONCURRENCY")==nullptr) {
    int requested_workers=1;
    for(int i=1;i<argc;++i) {
      constexpr const char* prefix="--tapa_workers=";
      if(std::strncmp(argv[i],prefix,std::strlen(prefix))==0)
        requested_workers=std::max(1,std::atoi(argv[i]+std::strlen(prefix)));
    }
    const char* exact_events=std::getenv("SALUT_EXACT_MEMORY_EVENTS");
    const bool exact=exact_events!=nullptr&&exact_events[0]=='1';
    const std::string workers=std::to_string(exact?128:requested_workers);
    setenv("TAPA_CONCURRENCY",workers.c_str(),1);
    execv(argv[0],argv);
    std::cerr << "failed to re-exec with TAPA_CONCURRENCY=" << workers << "\n";
    return 2;
  }
  gflags::ParseCommandLineFlags(&argc,&argv,true);                   // 解析命令行参数
  if (FLAGS_A_bank_base != 0 || FLAGS_L_bank_base != 8 || FLAGS_U_bank_base != 16) {
    std::cerr << "[Error] full 27-port ABI requires A[0..7], L[8..15], U[16..23] bank bases\n";
    return 2;
  }
  // if(FLAGS_bitstream.empty()||FLAGS_matrix_file.empty()){            // 参数检查
  //   std::cerr<<"Usage: --bitstream=xxx.xclbin --matrix_file=xxx.mtx\n";return -1;
  // }

  // --- 读取稀疏矩阵 ---
  SNicsLU nicslu;
  std::cout << "--- CPU Preprocessing Started ---" << std::endl;
  
  // 开始计时整个预处理过程
  auto cpu_preprocess_start = std::chrono::high_resolution_clock::now();

  double *ax=nullptr; 
  unsigned *ai=nullptr,*ap=nullptr;
  
  if(preprocess(const_cast<char*>(FLAGS_matrix_file.c_str()),&nicslu,&ax,&ai,&ap)){
    std::cerr<<"[Error] preprocess failed\n";return -1;
  }
  uint32_t n=nicslu.n; 
  std::cout<<"[Matrix] n="<<n<<" original nnz="<<nicslu.nnz<<"\n";

  // --- 执行符号分析（生成预测 L/U 结构与依赖）---
  Symbolic_Matrix sym(n,std::cout,std::cerr);
  sym.import_nicslu_factor_pattern(&nicslu,ai,ap,ax);

  // The MT pivot-selection workers use an active spin loop while idle.  The
  // factor pattern has now been copied into `sym`, so leaving those workers
  // alive during TAPA software simulation wastes 31 cores and severely
  // distorts the kernel timing of large matrices.  Destroy only the worker
  // pool here; the NicsLU factor storage remains valid until final cleanup.
  if(nicslu.flag[3]) {
    const int destroy_threads_ret=NicsLU_DestroyThreads(&nicslu);
    if(destroy_threads_ret!=NICS_OK) {
      std::cerr << "[Warning] failed to stop idle NicsLU workers: "
                << destroy_threads_ret << "\n";
    } else {
      std::cout << "[Host] Idle NicsLU worker threads stopped before kernel "
                   "simulation.\n";
    }
  }
  
  // 计算符号分解及之前的时间
  auto symbolic_end = std::chrono::high_resolution_clock::now();
  double symbolic_time = std::chrono::duration<double, std::milli>(symbolic_end - cpu_preprocess_start).count();
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "[Symbolic] Time (from matrix read to symbolic factorization): " << symbolic_time << " ms" << std::endl;

  // --- 定义 bank 列表 ---
  std::vector<uint32_t> Ab(A_BANKS),Lb(L_BANKS),Ub(U_BANKS);
  for(int i=0;i<A_BANKS;i++) Ab[i]=FLAGS_A_bank_base+i;
  for(int i=0;i<L_BANKS;i++) Lb[i]=FLAGS_L_bank_base+i;
  for(int i=0;i<U_BANKS;i++) Ub[i]=FLAGS_U_bank_base+i;

  // --- 构建 PLU2：U/pivot 列前端 + 多 PE L 段 + 段→列触发 ---
  std::vector<aligned_vector<ap_uint<512>>> A_host(A_BANKS);
  MetaOut meta_out;
  try {
    const char* hardware_env=std::getenv("SALUT_VALIDATE_HARDWARE_LIMITS");
    const bool hardware_limits=!FLAGS_bitstream.empty() ||
        (hardware_env!=nullptr && hardware_env[0]=='1');
    meta_out=build_parallel_layout(
        sym,FLAGS_seg_rows,Ab,Lb,Ub,A_host,FLAGS_sim_cols,hardware_limits);
    std::cout << "[Host preflight] output ownership and raw address ranges verified; limits="
              << (hardware_limits ? "hardware/RTL" : "offline software") << '\n';
  } catch (const std::exception& error) {
    std::cerr << "[Host preflight ERROR] " << error.what() << '\n';
    return 2;
  }
  
  // 计算整个CPU预处理的总时间（包括元数据提取）
  auto cpu_preprocess_end = std::chrono::high_resolution_clock::now();
  double cpu_preprocess_time = std::chrono::duration<double, std::milli>(cpu_preprocess_end - cpu_preprocess_start).count();
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "[Total] CPU Preprocessing time (including metadata extraction): " << cpu_preprocess_time << " ms" << std::endl;

  // --- 分配主机 HBM 缓冲 ---
  // 注意：稀疏格式下，A_host使用512位字（不再使用64位元素）
  std::cout << "[DEBUG] Allocating HBM buffers..." << std::endl;
  
  // 计算每个 bank 需要的512位字数（稀疏格式下需要统计实际使用的字数）
  const uint32_t MIN_SIZE_512 = 1; // 未使用端口只保留一个 512 位字
  
  std::vector<aligned_vector<ap_uint<64>>> L_host(L_BANKS),U_host(U_BANKS);
  for(int i=0;i<L_BANKS;i++) {
    // L和U仍然使用64位元素（因为它们已经是稀疏存储）
    auto align_to_8 = [](uint32_t x) { return ((x + 7) / 8) * 8; };
    const uint32_t MIN_SIZE_64 = MIN_SIZE_512 * 8;
    uint32_t size = std::max(MIN_SIZE_64, align_to_8(meta_out.L_used[i]));
    L_host[i].assign(size, 0);
    if(!meta_out.L_init_banks.empty()) {
      std::copy(meta_out.L_init_banks[i].begin(),meta_out.L_init_banks[i].end(),
                L_host[i].begin());
    } else if(i==0) {
      std::copy(meta_out.L_init.begin(),meta_out.L_init.end(),L_host[i].begin());
    }
    std::cout << "[DEBUG] L_host[" << i << "] size: " << size << " (used: " << meta_out.L_used[i] << ")" << std::endl;
  }
  for(int i=0;i<U_BANKS;i++) {
    auto align_to_8 = [](uint32_t x) { return ((x + 7) / 8) * 8; };
    const uint32_t MIN_SIZE_64 = MIN_SIZE_512 * 8;
    uint32_t size = std::max(MIN_SIZE_64, align_to_8(meta_out.U_used[i]));
    U_host[i].assign(size, 0);
    std::cout << "[DEBUG] U_host[" << i << "] size: " << size << " (used: " << meta_out.U_used[i] << ")" << std::endl;
  }
  std::cout << "[DEBUG] HBM buffers allocated" << std::endl;

  // 确保所有 A_host 向量至少有一个元素（避免空向量传递给 TAPA）
  std::cout << "[DEBUG] Checking A_host vectors..." << std::endl;
  for(int i = 0; i < A_BANKS; i++) {
    if(A_host[i].empty()) {
      std::cout << "[WARNING] A_host[" << i << "] is empty, adding dummy element" << std::endl;
      A_host[i].push_back(ap_uint<512>(0));  // 添加一个全0的512位字
    }
    std::cout << "[DEBUG] A_host[" << i << "] size: " << A_host[i].size() << std::endl;
  }
    // --- 计算 FLOPS ---
    double total_flops_for_lu = calculate_fpga_lu_flops(sym);
    std::cout << "Calculated FLOPs for LU Factorization: " << std::fixed << std::setprecision(0) 
              << total_flops_for_lu << std::endl;
    std::cout << std::fixed << std::setprecision(6); // 恢复默认精度
  // 启动 FPGA 核 (TAPA)
  std::cout << "[DEBUG] Preparing to invoke FPGA kernel..." << std::endl;
  std::cout.flush();
  int64_t kernel_time_ns = 0; // 保存 FPGA 执行时间（纳秒）
  try {
    if(!FLAGS_cosim_replay_dir.empty()) {
      std::cout << "[Host] Replaying completed RTL cosim banks from "
                << FLAGS_cosim_replay_dir << "...\n";
      for(int lane=0; lane<L_BANKS; ++lane)
        load_cosim_u64_bank(FLAGS_cosim_replay_dir, 8+lane, L_host[lane]);
      for(int lane=0; lane<U_BANKS; ++lane)
        load_cosim_u64_bank(FLAGS_cosim_replay_dir, 16+lane, U_host[lane]);
      std::cout << "[Host] Loaded all 16 L/U RTL cosim output banks.\n";
    } else if(FLAGS_fast_model) {
      std::cout << "[Host] Running packed SALUT transaction model...\n";
      const auto model_start=std::chrono::high_resolution_clock::now();
      run_fast_salut_model(meta_out,Ab,Lb,Ub,A_host,L_host,U_host);
      const auto model_end=std::chrono::high_resolution_clock::now();
      kernel_time_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
          model_end-model_start).count();
      std::cout << "[Host] SALUT transaction model done. Time = "
                << (double)kernel_time_ns * 1e-6 << " ms\n";
    } else {
      if(FLAGS_bitstream.empty() && std::getenv("TAPA_CONCURRENCY")==nullptr) {
        const char* exact_events=std::getenv("SALUT_EXACT_MEMORY_EVENTS");
        const bool exact=exact_events!=nullptr&&exact_events[0]=='1';
        const std::string workers=std::to_string(
            exact?128:std::max(1,FLAGS_tapa_workers));
        setenv("TAPA_CONCURRENCY",workers.c_str(),0);
        std::cout << "[Host] TAPA_CONCURRENCY=" << workers
                  << (exact?" (exact memory-event backend)\n"
                           :" (compute-control simulation optimum)\n");
      }
      std::cout << "[Host] Invoking FPGA kernel...\n";
      std::cout.flush();
      kernel_time_ns = tapa::invoke(
      SparseLUKernel, FLAGS_bitstream,
      // A: 只读 512b 端口，现在直接使用512b组织（稀疏格式）
      tapa::read_only_mmap<ap_uint<512>>(A_host[0]),
      tapa::read_only_mmap<ap_uint<512>>(A_host[1]),
      tapa::read_only_mmap<ap_uint<512>>(A_host[2]),
      tapa::read_only_mmap<ap_uint<512>>(A_host[3]),
      tapa::read_only_mmap<ap_uint<512>>(A_host[4]),
      tapa::read_only_mmap<ap_uint<512>>(A_host[5]),
      tapa::read_only_mmap<ap_uint<512>>(A_host[6]),
      tapa::read_only_mmap<ap_uint<512>>(A_host[7]),
      // L: 读写 512b 端口
      tapa::read_write_mmap<ap_uint<64>>(L_host[0]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(L_host[1]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(L_host[2]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(L_host[3]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(L_host[4]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(L_host[5]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(L_host[6]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(L_host[7]).reinterpret<ap_uint<512>>(),
      // U: 读写 512b 端口
      tapa::read_write_mmap<ap_uint<64>>(U_host[0]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(U_host[1]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(U_host[2]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(U_host[3]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(U_host[4]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(U_host[5]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(U_host[6]).reinterpret<ap_uint<512>>(),
      tapa::read_write_mmap<ap_uint<64>>(U_host[7]).reinterpret<ap_uint<512>>(),
      // META: three independent read-only ports for the degree scheduler,
      // front descriptor emitter and segment/DepEntry emitter.  Hardware
      // maps these copies to HBM[24..26] so metadata resolution runs in
      // parallel instead of serializing inside the central scheduler.
      tapa::read_only_mmap<uint32_t>(meta_out.meta),
      tapa::read_only_mmap<uint32_t>(meta_out.meta),
      tapa::read_only_mmap<uint32_t>(meta_out.meta)
      );

      std::cout << "[Host] FPGA kernel done. Time = "
                << (double)kernel_time_ns * 1e-6 << " ms\n"; // 转毫秒显示
    }

    // --- 计算吞吐量 (GFLOPS) ---
    if (total_flops_for_lu > 0 && kernel_time_ns > 0) {
        double kernel_time_s = (double)kernel_time_ns * 1e-9; // 将纳秒转换为秒
        double fpga_gflops = total_flops_for_lu / kernel_time_s / 1e9; // 计算 GFLOPS

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Total FLOPs for LU Factorization: " << total_flops_for_lu << std::endl;
        std::cout << (FLAGS_fast_model?"Model GFLOPS: ":"FPGA GFLOPS: ")
                  << fpga_gflops << std::endl;
    } else {
        std::cout << "Cannot calculate GFLOPS: Total FLOPs or Kernel Time is zero/negative." << std::endl;
    }
    // --- 吞吐量计算结束 ---

  } catch (const std::exception& e) {
    std::cerr << "FPGA invoke failed: " << e.what() << "\n";
    // --- 释放内存与销毁对象 ---
    if (ax) free(ax);
    if (ai) free(ai);
    if (ap) free(ap);
    NicsLU_Destroy(&nicslu);
    return -1;
  }

  bool numerical_check_passed=true;
  if(FLAGS_check_result) {
    numerical_check_passed=compare_cpu_fpga(sym,meta_out,Lb,Ub,L_host,U_host);
  } else {
    std::cout<<"[CHECK] CPU/FPGA numerical comparison disabled."<<std::endl;
  }

  // ========================================================================
  // 输出统计信息
  // ========================================================================
  auto sum = [](const std::vector<uint32_t>& v){ uint64_t s=0; for(auto x:v) s+=x; return s; };
  uint64_t A_tot = sum(meta_out.A_used); // A 总元素
  uint64_t L_tot = sum(meta_out.L_used); // L 总元素
  uint64_t U_tot = sum(meta_out.U_used); // U 总元素

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[INFO] A elems total: " << A_tot << " (64b elements)\n";
  std::cout << "[INFO] L elems total: " << L_tot << " (64b elements)\n";
  std::cout << "[INFO] U elems total: " << U_tot << " (64b elements)\n";
  std::cout << "[INFO] META words    : " << meta_out.meta.size() << " (uint32)\n";

  // ========================================================================
  // 清理与退出
  // ========================================================================
  if (ax) free(ax); 
  if (ai) free(ai); 
  if (ap) free(ap);
  NicsLU_Destroy(&nicslu); // 释放 NICSLU 结构体资源

  return numerical_check_passed?0:2;
}
