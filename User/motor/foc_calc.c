/**
 * @file    foc_calc.c
 * @brief   FOC 纯数学算法: 坐标变换 + 空间矢量调制
 */
#include "common.h"

/* ================================================================
 *  sin / cos 计算
 * ================================================================ */
_RAM_FUNC void sin_cos_val(pmsm_foc_t *foc)
{
    foc->sin_val = sinf(foc->theta);
    foc->cos_val = cosf(foc->theta);
}

/* ================================================================
 *  Clarke 变换  (abc → αβ)
 *  i_alph = i_a
 *  i_beta = (i_b - i_c) / √3
 * ================================================================ */
_RAM_FUNC void clarke_transform(pmsm_foc_t *foc)
{
    foc->i_alph = TWO_BY_THREE * (foc->i_a - 0.5f * foc->i_b - 0.5f * foc->i_c);
    foc->i_beta = ONE_BY_SQRT3 * (foc->i_b - foc->i_c);
}

/* ================================================================
 *  Park 变换  (αβ → dq)
 * ================================================================ */
_RAM_FUNC void park_transform(pmsm_foc_t *foc)
{
    foc->i_d =  foc->i_alph * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q =  foc->i_beta * foc->cos_val - foc->i_alph * foc->sin_val;
}

/* ================================================================
 *  逆 Park 变换  (dq → αβ)
 * ================================================================ */
_RAM_FUNC void inverse_park(pmsm_foc_t *foc)
{
    foc->v_alph = foc->v_d * foc->cos_val - foc->v_q * foc->sin_val;
    foc->v_beta = foc->v_d * foc->sin_val + foc->v_q * foc->cos_val;
}


