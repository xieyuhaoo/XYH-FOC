/**
 * @file    foc_ctrl.c
 * @brief   FOC 控制层: HAL中断回调 + 状态机 + 运行模式
 * @details
 *   HAL_ADCEx_InjectedConvCpltCallback  → 单电阻双采样存储
 *   HAL_TIM_PeriodElapsedCallback       → TIM1(10kHz电流环) / TIM3(1kHz速度环)
 *   pmsm_state_ctrl                     → 状态机 (start/opera/reset)
 *   force_curr_mode                     → 强拖启动 + SMO切换
 */
#include "common.h"
#include "modlue.h"

/* ======================== 诊断计数器 (调试时置1) ======================== */
#define FOC_DIAG_ENABLE  0
#if FOC_DIAG_ENABLE
volatile uint32_t diag_adc_inj_cnt   = 0;
volatile uint32_t diag_shunt_ready   = 0;
volatile uint32_t diag_tim1_of_cnt   = 0;
volatile uint32_t diag_tim1_uf_cnt   = 0;
volatile uint16_t diag_last_jdr1     = 0;
volatile uint16_t diag_last_jdr2     = 0;
volatile uint16_t diag_ccr4_val      = 0;
volatile uint16_t diag_ccr6_val      = 0;
volatile uint16_t diag_trig_gap      = 0;
#endif

/* ================================================================
 *  ADC1 注入序列完成回调 (JEOS)
 *  配置: 2-rank + JDISCEN
 *    第1次 TRGO2 → Rank1 → JDR1 (仅 JEOC, 不触发ISR)
 *    第2次 TRGO2 → Rank2 → JDR2 → JEOS → 本回调
 *  一次回调同时拿到两个采样值
 * ================================================================ */
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* 直接读寄存器, 比 HAL_ADCEx_InjectedGetValue 更快 */
        uint16_t raw1 = (uint16_t)(ADC1->JDR1);  /* Rank1: 第一个触发采样 */
        uint16_t raw2 = (uint16_t)(ADC1->JDR2);  /* Rank2: 第二个触发采样 */

#if FOC_DIAG_ENABLE
        diag_adc_inj_cnt++;
        diag_last_jdr1 = raw1;
        diag_last_jdr2 = raw2;
#endif

         pm.foc.i_shunt_raw_1 = (float)raw1;
         pm.foc.i_shunt_raw_2 = (float)raw2;
//         /* 仅接受下计数半周期的好数据, 忽略上计数半周期的第二组触发.
//          * shunt_pair_ready==0 说明 ISR 已消费上次数据, 当前是新的好数据. */
//         if (pm.foc.shunt_pair_ready == 0U)
//         {
//             pm.foc.i_shunt_raw_1 = (float)raw1;
//             pm.foc.i_shunt_raw_2 = (float)raw2;
//             pm.foc.shunt_pair_ready = 1U;
// #if FOC_DIAG_ENABLE
//             diag_shunt_ready++;
// #endif
//         }
    }
}

/* ================================================================
 *  TIM 更新中断回调 (TIM1 + TIM3 共用)
 * ================================================================ */
_RAM_FUNC void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
#if FOC_DIAG_ENABLE
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
        {
            diag_tim1_of_cnt++;
        }
        else
        {
            diag_tim1_uf_cnt++;
            diag_ccr4_val = (uint16_t)htim1.Instance->CCR4;
            diag_ccr6_val = (uint16_t)htim1.Instance->CCR6;
            diag_trig_gap = (diag_ccr6_val > diag_ccr4_val) ?
                            (diag_ccr6_val - diag_ccr4_val) : 0;
        }
#endif

        /* 10kHz FOC 调度 (上溢+下溢各一次) */
        foc_tim1_update_isr(&pm);
         vofa_start();     /* USB CDC -> VOFA+ JustFloat, ~100Hz */
    }
    else if (htim->Instance == TIM3)
    {
        /* 1kHz 速度环 */
        foc_spd_pi_calc(&pm);
       
    }
}


