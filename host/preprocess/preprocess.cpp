#include "preprocess.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include <cmath>
#include "nicslu.h"
#include "nicslu_util.h"
#include <chrono>
#include <cstring>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>

// ============================================================================
// 检查MTX文件头是否标记为对称矩阵
// ============================================================================
int is_mtx_file_symmetric(const char *matrixName)
{
    std::ifstream file(matrixName);
    if (!file.is_open())
    {
        return 0;  // 无法打开文件，返回0
    }
    
    std::string line;
    // 读取第一行（文件头）
    if (std::getline(file, line))
    {
        // 转换为小写以便比较
        std::string lower_line = line;
        std::transform(lower_line.begin(), lower_line.end(), lower_line.begin(), ::tolower);
        
        // 检查是否包含"symmetric"关键字
        if (lower_line.find("symmetric") != std::string::npos)
        {
            file.close();
            return 1;  // 文件头标记为对称
        }
    }
    
    file.close();
    return 0;  // 文件头未标记为对称
}

// ============================================================================
// 检测矩阵是否为对称的（检查元素值是否对称）
// 注意：这个函数用于检查已经展开的矩阵是否对称
// 对于只存储了一半的矩阵，这个函数会返回false（因为找不到转置元素）
// ============================================================================
int is_matrix_symmetric(uint__t n, const double *ax, const unsigned int *ai, const unsigned int *ap, double tol)
{
    uint__t i, j, p, q;
    
    // 对于每个非对角线元素A(i,j)，检查A(j,i)是否存在且值相等
    // 一旦发现不对称，立即返回false
    for (i = 0; i < n; ++i)
    {
        for (p = ap[i]; p < ap[i+1]; ++p)
        {
            j = ai[p];
            if (i != j)  // 只检查非对角线元素
            {
                // 在列j中查找行i
                int found = 0;
                double val_ji = 0.0;
                for (q = ap[j]; q < ap[j+1]; ++q)
                {
                    if (ai[q] == i)
                    {
                        found = 1;
                        val_ji = ax[q];
                        break;
                    }
                }
                
                // 如果A(j,i)不存在，或者值不相等，则矩阵不对称
                if (!found || fabs(ax[p] - val_ji) > tol)
                {
                    return 0;  // 不对称，立即返回
                }
            }
        }
    }
    
    // 如果检查了所有元素都没有发现不对称，则认为矩阵是对称的
    return 1;  // 矩阵是对称的
}

// ============================================================================
// 将对称矩阵（仅存储上或下三角）扩展为完整矩阵
// ============================================================================
int expand_symmetric_matrix(uint__t n, double **ax, unsigned int **ai, unsigned int **ap)
{
    uint__t nnz_old, nnz_new, i, j, p, k;
    double *ax_new;
    unsigned int *ai_new, *ap_new;
    
    nnz_old = (*ap)[n];
    
    // 计算扩展后的非零元素数量
    // 对于对称矩阵，需要加上对称的上三角部分（但要排除对角线）
    nnz_new = 0;
    for (i = 0; i < n; ++i)
    {
        for (p = (*ap)[i]; p < (*ap)[i+1]; ++p)
        {
            j = (*ai)[p];
            if (i != j)  // 不是对角线元素，需要添加转置
            {
                nnz_new++;
            }
        }
    }
    nnz_new += nnz_old;  // 原来的元素 + 转置后的元素
    
    printf("[Symmetric Matrix] Original nnz: %u, Expanded nnz: %u\n", (uint32_t)nnz_old, (uint32_t)nnz_new);
    
    // 分配新的存储空间
    ax_new = (double *)malloc(sizeof(double) * nnz_new);
    ai_new = (unsigned int *)malloc(sizeof(unsigned int) * nnz_new);
    ap_new = (unsigned int *)malloc(sizeof(unsigned int) * (n + 1));
    
    if (NULL == ax_new || NULL == ai_new || NULL == ap_new)
    {
        printf("[Error] Failed to allocate memory for symmetric matrix expansion\n");
        if (ax_new) free(ax_new);
        if (ai_new) free(ai_new);
        if (ap_new) free(ap_new);
        return -1;
    }
    
    // 构建扩展矩阵（采用插入排序方式）
    std::vector<double> temp_x;
    std::vector<unsigned int> temp_i;
    
    ap_new[0] = 0;
    
    for (i = 0; i < n; ++i)
    {
        temp_x.clear();
        temp_i.clear();
        
        // 1. 先收集原矩阵第i列的元素
        for (p = (*ap)[i]; p < (*ap)[i+1]; ++p)
        {
            temp_i.push_back((*ai)[p]);
            temp_x.push_back((*ax)[p]);
        }
        
        // 2. 再收集转置矩阵的第i列（即原矩阵第i行）
        for (j = 0; j < n; ++j)
        {
            for (p = (*ap)[j]; p < (*ap)[j+1]; ++p)
            {
                if ((*ai)[p] == i && i != j)  // A(i,j) 的转置是 A(j,i)
                {
                    // 检查是否已经在列表中
                    int found = 0;
                    for (k = 0; k < temp_i.size(); ++k)
                    {
                        if (temp_i[k] == j)
                        {
                            found = 1;
                            break;
                        }
                    }
                    if (!found)
                    {
                        temp_i.push_back(j);
                        temp_x.push_back((*ax)[p]);  // 对称矩阵，值相同
                    }
                }
            }
        }
        
        // 3. 对该列进行排序
        for (k = 0; k < temp_i.size(); ++k)
        {
            for (p = k + 1; p < temp_i.size(); ++p)
            {
                if (temp_i[k] > temp_i[p])
                {
                    std::swap(temp_i[k], temp_i[p]);
                    std::swap(temp_x[k], temp_x[p]);
                }
            }
        }
        
        // 4. 复制到新数组
        uint__t col_size = temp_i.size();
        // 安全检查：确保列大小合理（不超过矩阵大小n）
        if (col_size > n)
        {
            printf("[Error] Column %u has invalid size %u (max allowed: %u)\n", (uint32_t)i, (uint32_t)col_size, (uint32_t)n);
            free(ax_new);
            free(ai_new);
            free(ap_new);
            return -1;
        }
        ap_new[i+1] = ap_new[i] + col_size;
        if (ap_new[i+1] < ap_new[i])  // 检查溢出
        {
            printf("[Error] Overflow in column pointer calculation at column %u\n", (uint32_t)i);
            free(ax_new);
            free(ai_new);
            free(ap_new);
            return -1;
        }
        for (k = 0; k < col_size; ++k)
        {
            ai_new[ap_new[i] + k] = temp_i[k];
            ax_new[ap_new[i] + k] = temp_x[k];
        }
    }
    
    // 验证最终的非零元素数
    uint__t final_nnz = ap_new[n];
    if (final_nnz != nnz_new)
    {
        printf("[Warning] Mismatch: expected nnz=%u, got nnz=%u\n", (uint32_t)nnz_new, (uint32_t)final_nnz);
    }
    
    // 释放旧的数据
    free(*ax);
    free(*ai);
    free(*ap);
    
    // 指向新的数据
    *ax = ax_new;
    *ai = ai_new;
    *ap = ap_new;
    
    return 0;
}

int my_DumpA(SNicsLU *nicslu, double **ax, unsigned int **ai, unsigned int **ap)
{
    uint__t n, nnz;
    double *ax0;
    unsigned int *ai0, *ap0;
    uint__t *rowperm, *pinv, *piv, oldrow, start, end;
    uint__t i, j, p;

    if (NULL == nicslu || NULL == ax || NULL == ai || NULL == ap)
    {
        return -1;
    }

    n = nicslu->n;
    nnz = nicslu->nnz;
    ax0 = nicslu->ax;
    ai0 = nicslu->ai;
    ap0 = nicslu->ap;
    
    // 检查传入的指针是否与nicslu内部的指针相同
    // 如果相同，不要释放，因为nicslu会管理这些内存
    int ax_is_nicslu = (*ax == ax0);
    int ai_is_nicslu = (*ai == ai0);
    int ap_is_nicslu = (*ap == ap0);
    
    if (!ax_is_nicslu && *ax != NULL)
    {
        free(*ax);
        *ax = NULL;
    }
    if (!ai_is_nicslu && *ai != NULL)
    {
        free(*ai);
        *ai = NULL;
    }
    if (!ap_is_nicslu && *ap != NULL)
    {
        free(*ap);
        *ap = NULL;
    }
    rowperm = nicslu->row_perm;/*row_perm[i]=j-->row i in the permuted matrix is row j in the original matrix*/
    pinv = (uint__t *)nicslu->pivot_inv;/*pivot_inv[i]=j-->column i is the jth pivot column*/
    piv = (uint__t *)nicslu->pivot;

    // 检查rowperm是否有效
    if (rowperm == NULL)
    {
        printf("[ERROR] my_DumpA: row_perm is NULL\n");
        goto FAIL;
    }

    //generate pivot and pivot_inv for function NicsLU_DumpA
    for (i = 0; i < n; ++i)
    {
        pinv[i] = i;
        piv[i] = i;
        // 如果rowperm未初始化，初始化为单位排列
        if (rowperm[i] >= n)
        {
            printf("[WARNING] my_DumpA: row_perm[%u]=%u >= n=%u, setting to %u\n", 
                   (uint32_t)i, (uint32_t)rowperm[i], (uint32_t)n, (uint32_t)i);
            rowperm[i] = i;
        }
    }

    *ax = (double *)malloc(sizeof(double)*nnz);
    *ai = (unsigned int *)malloc(sizeof(unsigned int)*nnz);
    *ap = (unsigned int *)malloc(sizeof(unsigned int)*(n+1));
    // *ax = (real__t *)malloc(sizeof(real__t)*nnz);
    // *ai = (uint__t *)malloc(sizeof(uint__t)*nnz);
    // *ap = (uint__t *)malloc(sizeof(uint__t)*(n+1));

    if (NULL == *ax || NULL == *ai || NULL == *ap)
    {
        goto FAIL;
    }
    (*ap)[0] = 0;

    p = 0;
    for (i=0; i<n; ++i)
    {
        oldrow = rowperm[i];
        if (oldrow >= n)
        {
            printf("[ERROR] my_DumpA: rowperm[%u]=%u >= n=%u\n", (uint32_t)i, (uint32_t)oldrow, (uint32_t)n);
            goto FAIL;
        }
        start = ap0[oldrow];
        end = ap0[oldrow+1];
        if (end < start)
        {
            printf("[ERROR] my_DumpA: ap0[%u+1]=%u < ap0[%u]=%u\n", (uint32_t)oldrow, (uint32_t)end, (uint32_t)oldrow, (uint32_t)start);
            goto FAIL;
        }
        (*ap)[i+1] = (*ap)[i] + end - start;

        for (j=start; j<end; ++j)
        {
            (*ax)[p] = ax0[j];
            (*ai)[p++] = pinv[ai0[j]];
        }
    }

    return 0;

FAIL:
    if (*ax != NULL)
    {
        free(*ax);
        *ax = NULL;
    }
    if (*ai != NULL)
    {
        free(*ai);
        *ai = NULL;
    }
    if (*ap != NULL)
    {
        free(*ap);
        *ap = NULL;
    }
    return -2;
}

int preprocess(char *matrixName, SNicsLU *nicslu, double **ax, unsigned int **ai, unsigned int **ap)
{
    int ret;
    uint__t *n, *nnz;

    // nicslu = (SNicsLU *)malloc(sizeof(SNicsLU));
    NicsLU_Initialize(nicslu);

    n = (uint__t *)malloc(sizeof(uint__t));
    nnz = (uint__t *)malloc(sizeof(uint__t));

    printf("Reading matrix...\n");
    fflush(stdout);

    ret = NicsLU_ReadTripletColumnToSparse(matrixName, n, nnz, ax, ai, ap);
    if (ret == NICSLU_MATRIX_INVALID)
    {    
        printf("Read invalid matrix\n");
        goto EXIT;
    }
    else if (ret == NICSLU_FILE_CANNOT_OPEN) 
    {    
        printf("File cannot open\n");
        goto EXIT;
    }
    else if (ret != NICS_OK) 
    {    
        printf("Open file error\n");
        goto EXIT;
    }

    // 检测并扩展对称矩阵
    {
        printf("Detecting matrix symmetry...\n");
        fflush(stdout);
        
        // 首先检查MTX文件头是否标记为对称矩阵
        int file_marked_symmetric = is_mtx_file_symmetric(matrixName);
        
        // 检查矩阵是否只存储了一半（上三角或下三角）
        uint__t lower_count = 0, upper_count = 0;
        for (uint__t i = 0; i < *n; ++i)
        {
            for (uint__t p = (*ap)[i]; p < (*ap)[i+1]; ++p)
            {
                uint__t j = (*ai)[p];
                if (i > j) lower_count++;
                else if (i < j) upper_count++;
            }
        }
        int only_half_stored = (lower_count == 0 && upper_count > 0) || (upper_count == 0 && lower_count > 0);
        
        // 如果文件头标记为对称矩阵，且矩阵只存储了一半，才进行扩展
        // 如果矩阵已经完整（上下三角都有），则不需要扩展
        // 临时禁用扩展以调试段错误问题
        if (0 && file_marked_symmetric && only_half_stored)
        {
            printf("[Info] Matrix file marked as symmetric and only half stored. Expanding to full matrix...\n");
            fflush(stdout);
            if (expand_symmetric_matrix(*n, ax, ai, ap) != 0)
            {
                printf("[Error] Failed to expand symmetric matrix\n");
                if (n) { free(n); n = NULL; }
                if (nnz) { free(nnz); nnz = NULL; }
                return -1;
            }
            *nnz = (*ap)[*n];  // 更新非零元素数
            printf("[Info] Expanded nnz: %u\n", (uint32_t)(*nnz));
            fflush(stdout);
        }
        else if (file_marked_symmetric && !only_half_stored)
        {
            printf("[Info] Matrix file marked as symmetric but already fully populated.\n");
            fflush(stdout);
        }
        else if (only_half_stored)
        {
            // 矩阵只存储了一半，即使文件头未标记，也可能是对称矩阵
            // 为了安全起见，我们也扩展它
            printf("[Info] Matrix appears to store only half (upper or lower triangle). Expanding to full matrix...\n");
            fflush(stdout);
            if (expand_symmetric_matrix(*n, ax, ai, ap) != 0)
            {
                printf("[Error] Failed to expand symmetric matrix\n");
                if (n) { free(n); n = NULL; }
                if (nnz) { free(nnz); nnz = NULL; }
                return -1;
            }
            *nnz = (*ap)[*n];  // 更新非零元素数
            printf("[Info] Expanded nnz: %u\n", (uint32_t)(*nnz));
            fflush(stdout);
        }
        else
        {
            // 检查矩阵内容是否对称（用于已经展开的矩阵）
            int matrix_content_symmetric = is_matrix_symmetric(*n, *ax, *ai, *ap, 1e-14);
            if (matrix_content_symmetric)
            {
                printf("[Info] Matrix is symmetric and fully populated.\n");
            }
            else
            {
                printf("[Info] Matrix is not symmetric or fully populated.\n");
            }
            fflush(stdout);
        }
    }

    printf("[DEBUG] Before NicsLU_CreateMatrix: n=%u, nnz=%u\n", (uint32_t)(*n), (uint32_t)(*nnz));
    fflush(stdout);
    
    NicsLU_CreateMatrix(nicslu, *n, *nnz, *ax, *ai, *ap);
    
    printf("[DEBUG] After NicsLU_CreateMatrix: nicslu->n=%u, nicslu->nnz=%u\n", 
           (uint32_t)nicslu->n, (uint32_t)nicslu->nnz);
    fflush(stdout);
    
    nicslu->cfgi[0] = 1;
    nicslu->cfgf[1] = 0;

    printf("Preprocessing matrix...\n");
    fflush(stdout);

    // 使用chrono精确计时预处理
    {
        auto preprocess_start = std::chrono::high_resolution_clock::now();
        NicsLU_Analyze(nicslu);
        auto preprocess_end = std::chrono::high_resolution_clock::now();
        
        double chrono_time = std::chrono::duration<double, std::milli>(preprocess_end - preprocess_start).count();
        
        printf("Preprocessing time (NicsLU_Analyze only): %f ms (chrono)\n", chrono_time);
        fflush(stdout);
    }

    printf("[DEBUG] Before my_DumpA: nicslu->n=%u, nicslu->nnz=%u\n", 
           (uint32_t)nicslu->n, (uint32_t)nicslu->nnz);
    if (nicslu->ap != NULL && nicslu->n > 0)
    {
        printf("[DEBUG] nicslu->ap[0]=%u, nicslu->ap[n]=%u\n", 
               (uint32_t)nicslu->ap[0], (uint32_t)nicslu->ap[nicslu->n]);
    }
    if (ap != NULL && *ap != NULL && *n > 0)
    {
        printf("[DEBUG] *ap[0]=%u, *ap[n]=%u\n", 
               (uint32_t)(*ap)[0], (uint32_t)(*ap)[*n]);
    }
    fflush(stdout);
    
    my_DumpA(nicslu, ax, ai, ap);
    
    printf("[DEBUG] After my_DumpA\n");
    if (ap != NULL && *ap != NULL && *n > 0)
    {
        printf("[DEBUG] After my_DumpA: *ap[0]=%u, *ap[n]=%u, expected nnz=%u\n", 
               (uint32_t)(*ap)[0], (uint32_t)(*ap)[*n], (uint32_t)(*nnz));
        // 检查ap数组是否单调递增，并检查是否有异常大的差值
        for (uint__t i = 0; i < *n && i < 100; ++i)
        {
            if ((*ap)[i+1] < (*ap)[i])
            {
                printf("[ERROR] ap[%u]=%u > ap[%u]=%u (not monotonic!)\n", 
                       (uint32_t)i, (uint32_t)(*ap)[i], (uint32_t)(i+1), (uint32_t)(*ap)[i+1]);
            }
            uint__t diff = (*ap)[i+1] - (*ap)[i];
            if (diff > 10000)  // 如果某列的非零元素数异常大
            {
                printf("[WARNING] ap[%u+1]-ap[%u]=%u (very large!)\n", 
                       (uint32_t)i, (uint32_t)i, (uint32_t)diff);
            }
        }
    }
    fflush(stdout);
    //rp = nicslu->col_perm;
    //cp = nicslu->row_perm_inv;
    //piv = nicslu->pivot;
    //rows = nicslu->col_scale_perm;
    //cols = nicslu->row_scale;
    //cscale = nicslu->cscale;

    // 释放临时分配的 n 与 nnz
    if (n) { free(n); n = NULL; }
    if (nnz) { free(nnz); nnz = NULL; }
    return 0;
EXIT:
    // 释放临时分配的 n 与 nnz
    if (n) { free(n); n = NULL; }
    if (nnz) { free(nnz); nnz = NULL; }
    // 不在此处销毁 nicslu 或释放 ax/ai/ap，由调用方负责资源管理
    return -1;

}
