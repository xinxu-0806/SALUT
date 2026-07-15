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
#include <unordered_map>  // 哈希表映射
#include <set>            // 去重有序集合
#include <cassert>        // 调试断言

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
#include "symbolic.h"                 // 符号分析（生成依赖图）
#include "preprocess.h"               // 读取矩阵并标准化
#include "nicslu.h"                   // NICSLU 稀疏求解库接口

// ============================================================================
// 命令行参数定义
// ============================================================================
DEFINE_string(bitstream, "", "Path to FPGA bitstream (.xclbin)"); // FPGA 比特流文件路径
DEFINE_string(matrix_file, "/data/xxu/code/Cpp/SALUT/data2/bcsstm37/bcsstm37.mtx", "Path to the input matrix file (.mtx)");     // 输入矩阵文件路径
DEFINE_int32(seg_rows, 2048, "Segment window height");             // 每个 segment 的行数
DEFINE_int32(A_bank_base, 0, "HBM bank base for A (0..31)");       // A 矩阵起始 bank ID
DEFINE_int32(L_bank_base, 8, "HBM bank base for L (0..31)");       // L 矩阵起始 bank ID
DEFINE_int32(U_bank_base, 12,"HBM bank base for U (0..31)");       // U 矩阵起始 bank ID


// ============================================================================
// float -> 64b（低 32 位保存 float，方便与 FPGA 对齐）
// ============================================================================
inline ap_uint<64> f32_to_u64(float x){
  union { float f; uint32_t u; } t; t.f = x;            // 取 IEEE754 位模式
  ap_uint<64> w; w.range(31,0)=t.u; w.range(63,32)=0;   // 存入低位
  return w;
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
  std::vector<SegRecHost> segs;   // 段表（用于写 A）
};

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
// 主函数入口
// ============================================================================
int main(int argc,char**argv){
  gflags::ParseCommandLineFlags(&argc,&argv,true);                   // 解析命令行参数
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
  sym.fill_in(ai,ap); 
  sym.csr(); 
  sym.predictLU(ai,ap,ax);
  
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

  // --- 构建 META（元数据提取）---
  auto meta_out=build_meta(sym,FLAGS_seg_rows,Ab,Lb,Ub);
  
  // 计算整个CPU预处理的总时间（包括元数据提取）
  auto cpu_preprocess_end = std::chrono::high_resolution_clock::now();
  double cpu_preprocess_time = std::chrono::duration<double, std::milli>(cpu_preprocess_end - cpu_preprocess_start).count();
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "[Total] CPU Preprocessing time (including metadata extraction): " << cpu_preprocess_time << " ms" << std::endl;

  // --- 分配主机 HBM 缓冲 ---
  // 注意：稀疏格式下，A_host使用512位字（不再使用64位元素）
  std::cout << "[DEBUG] Allocating HBM buffers..." << std::endl;
  
  // 计算每个 bank 需要的512位字数（稀疏格式下需要统计实际使用的字数）
  const uint32_t MIN_SIZE_512 = 1024; // 最小 512 位字数
  
  std::vector<aligned_vector<ap_uint<512>>> A_host(A_BANKS);  // 改为512位字数组
  std::vector<aligned_vector<ap_uint<64>>> L_host(L_BANKS),U_host(U_BANKS);
  
  // 注意：A_host的实际大小在fill_A后确定，这里先预留空间
  for(int i=0;i<A_BANKS;i++) {
    A_host[i].reserve(MIN_SIZE_512 * 4);  // 预留空间，实际大小在fill_A时确定
    std::cout << "[DEBUG] A_host[" << i << "] reserved space" << std::endl;
  }
  for(int i=0;i<L_BANKS;i++) {
    // L和U仍然使用64位元素（因为它们已经是稀疏存储）
    auto align_to_8 = [](uint32_t x) { return ((x + 7) / 8) * 8; };
    const uint32_t MIN_SIZE_64 = MIN_SIZE_512 * 8;
    uint32_t size = std::max(MIN_SIZE_64, align_to_8(meta_out.L_used[i]));
    L_host[i].assign(size, 0);
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

  // --- 填充 A ---
  std::cout << "[DEBUG] Filling A matrix..." << std::endl;
  fill_A(ax,ai,ap,meta_out.segs,A_host,n);
  std::cout << "[DEBUG] A matrix filled" << std::endl;
  
  // 更新 META 中的段表（因为 fill_A 更新了 A_addr 和 word_count）
  std::cout << "[DEBUG] Updating seg_table in META..." << std::endl;
  uint32_t off_seg = meta_out.meta[3];  // 从 META 头部获取段表偏移
  uint32_t nSeg = meta_out.meta[2];  // 段总数
  if(meta_out.segs.size() != nSeg) {
    std::cerr << "[ERROR] Mismatch: segs.size()=" << meta_out.segs.size() 
              << " but nSeg=" << nSeg << std::endl;
  }
  for(uint32_t sid = 0; sid < meta_out.segs.size() && sid < nSeg; sid++) {
    uint32_t idx = off_seg + sid * 9;  // 每个段9个uint32
    if(idx + 8 >= meta_out.meta.size()) {
      std::cerr << "[ERROR] Index out of bounds: idx=" << idx << " meta.size()=" << meta_out.meta.size() << std::endl;
      break;
    }
    meta_out.meta[idx + 0] = meta_out.segs[sid].j;
    meta_out.meta[idx + 1] = meta_out.segs[sid].s;
    meta_out.meta[idx + 2] = meta_out.segs[sid].row_lo;
    meta_out.meta[idx + 3] = meta_out.segs[sid].row_hi;
    meta_out.meta[idx + 4] = meta_out.segs[sid].A_addr;  // 更新后的地址
    meta_out.meta[idx + 5] = meta_out.segs[sid].word_count;  // 更新后的字数
    meta_out.meta[idx + 6] = meta_out.segs[sid].L_base_addr;  // L 基地址（已填充）
    meta_out.meta[idx + 7] = meta_out.segs[sid].U_diag_addr;  // U 对角线地址（已填充）
    meta_out.meta[idx + 8] = meta_out.segs[sid].is_last;
  }
  std::cout << "[DEBUG] Seg_table updated in META (updated " << meta_out.segs.size() << " segments)" << std::endl;
  
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
      // META: 只读 uint32
      tapa::read_only_mmap<uint32_t>(meta_out.meta)
    );

    std::cout << "[Host] FPGA kernel done. Time = "
              << (double)kernel_time_ns * 1e-6 << " ms\n"; // 转毫秒显示

    // --- 计算吞吐量 (GFLOPS) ---
    if (total_flops_for_lu > 0 && kernel_time_ns > 0) {
        double kernel_time_s = (double)kernel_time_ns * 1e-9; // 将纳秒转换为秒
        double fpga_gflops = total_flops_for_lu / kernel_time_s / 1e9; // 计算 GFLOPS

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Total FLOPs for LU Factorization: " << total_flops_for_lu << std::endl;
        std::cout << "FPGA GFLOPS: " << fpga_gflops << std::endl;
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

  return 0;
}

