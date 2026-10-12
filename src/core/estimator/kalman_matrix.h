/**
 * @title  Kalman 用矩阵工具
 * @author: vaie
 * @date: 2026-10-12
 *
 * 只实现 kalman_core 用到的两个运算：9×9 矩阵乘法、转置。
 * 替代 Crazyflie 的 xtensa_math.h（CMSIS-DSP）——不引一整个库。
 *
 * 无平台依赖，SIL 与实机共用。
 */
#pragma once

namespace kmat
{

    // C = A · B（9×9，行主序二维数组）
    inline void mult9(const float A[9][9], const float B[9][9], float C[9][9])
    {
        for (int i = 0; i < 9; ++i)
        {
            for (int j = 0; j < 9; ++j)
            {
                float s = 0.0f;
                for (int k = 0; k < 9; ++k)
                    s += A[i][k] * B[k][j];
                C[i][j] = s;
            }
        }
    }

    // B = A^T
    inline void trans9(const float A[9][9], float B[9][9])
    {
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 9; ++j)
                B[j][i] = A[i][j];
    }

} // namespace kmat