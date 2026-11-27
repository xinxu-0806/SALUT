#ifndef _SYMBOLIC_H_
#define _SYMBOLIC_H_
#include <vector>
#include <unordered_set>
#include <iostream>
#include "type.h"
#include "nicslu.h"

class Symbolic_Matrix
{
public:
    unsigned int n;
    unsigned int nnz;
    int num_lev;

    // 原始矩阵的符号结构（CSC）
    std::vector<unsigned> sym_c_ptr;
    std::vector<unsigned> sym_r_idx;

    // CSR 结构
    std::vector<unsigned> csr_r_ptr;
    std::vector<unsigned> csr_c_idx;
    std::vector<unsigned> csr_diag_ptr;

    std::vector<REAL> val;
    std::vector<unsigned> l_col_ptr; // 对角元素索引
    std::vector<int> level_idx;
    std::vector<int> level_ptr;

    // --- 新增：符号分析生成的 L 的预测结构 ---
    // 每列的起始指针 (L_ptr_pred[j]) 与 行索引数组 (L_idx_pred)
    std::vector<unsigned> L_ptr_pred;
    std::vector<unsigned> L_idx_pred;

    // --- 已有：列依赖关系表 ---
    // dep_lists_per_col[j] = 所有 U(k,j)≠0 的 k（即列 j 的前驱列集合）
    std::vector<std::vector<unsigned int>> dep_lists_per_col;

#if GLU_DEBUG
    void PrintLevel();
    std::vector<REAL> CCA;
    void ABFTCalculateCCA();
    void ABFTCheckResult();
#endif

    // 构造函数
    Symbolic_Matrix(unsigned int dim, std::ostream& out_s, std::ostream& err_s)
        : n(dim), nnz(0), num_lev(0), m_out(out_s), m_err(err_s) {
        sym_c_ptr.clear();
        sym_r_idx.clear();
        l_col_ptr.clear();
        csr_r_ptr.clear();
        csr_c_idx.clear();
        csr_diag_ptr.clear();
        val.clear();
        level_ptr.clear();
        level_idx.clear();
        L_ptr_pred.clear();
        L_idx_pred.clear();
        dep_lists_per_col.clear();
#if GLU_DEBUG
        CCA.clear();
#endif
    }

    // 成员函数声明
    void fill_in(unsigned int *ai, unsigned int *ap);
    void csr();
    void predictLU(unsigned int *ai, unsigned int *ap, double *ax);
    void import_nicslu_factor_pattern(const SNicsLU *nicslu,
                                      unsigned int *ai,
                                      unsigned int *ap,
                                      double *ax);
    void build_predicted_L();   // <-- 新增：构建预测L结构
    void leveling();
    std::vector<REAL> solve(SNicsLU *nicslu, const std::vector<REAL> &rhs);

#if GLU_DEBUG
    void PrintLevel();
    void ABFTCalculateCCA();
    void ABFTCheckResult();
#endif

private:
    std::ostream& m_out;
    std::ostream& m_err;
};

#endif // _SYMBOLIC_H_
