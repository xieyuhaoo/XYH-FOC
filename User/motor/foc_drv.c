/**
 * @file    foc_drv.c
 * @brief   FOC 驱动层: 初始化 / 采样 / 单电阻重构 / SMO / PI / 控制环 / ISR调度
 * @details 单电阻采样 + 不对称PWM + SMO无感FOC
 *          TIM1 中心对齐, CH1-3 PWM, CH4+CH6 触发ADC1注入
 *          电流环  20 kHz (TIM1 上溢/下溢各一次)
 *          速度环   1 kHz (TIM3 独立中断)
 *
 * ════════════════════════════════════════════════════════════════
 *  FOC 控制流程中电压变量正确使用说明与数学推导
 * ════════════════════════════════════════════════════════════════
 *
 * 【问题背景】
 * 原代码中不同环节对电压归一化的定义存在冲突，主要体现在 Clarke 变换系数
 * (2/3 与 √2/3)、SVM 归一化系数与实际占空比计算的不匹配。
 * 下面是修正后统一且自洽的电压流转流程。
 *
 * 【基本准则与约定】
 * 1. 物理量单位：v_d, v_q, v_alph, v_beta 全部使用【真实物理电压】(单位: V)
 * 2. 坐标变换：采用【等幅值变换】(非等功率变换)。
 *    等幅值 Clarke 变换系数为 2/3 (这也是代码中 `TWO_BY_THREE` 所用的)。
 *    在此约定下：
 *       V_α_peak = V_phase_peak
 * 3. SVPWM 限制：
 *    全桥逆变器最大不失真(不过调制)相电压峰值为 Vbus / √3= SVPWM 六边形内切圆半径=。
 *    因此控制器的电压矢量的最大模长为 vs = (Vbus / √3)
 *
 * ─────────────────────────────────────────────────────────────────
 * 【关键常数的正确逻辑】
 * ─────────────────────────────────────────────────────────────────
 *  一、 电压圆限幅 vs (SVPWM 线性区极限)
 * 
 *  在等幅值变换下，空间矢量长度 vmag = √(v_α² + v_β²) 代表了三相相电压的峰值。
 *  SVPWM 最大不失真相电压峰值为 Vbus / √3。
 *  => `vs = Vbus / √3 * 0.96` (0.96 为安全余量)
 *  用途：PI 积分限幅、(vd,vq) 圆形限幅。
 *
 *
 *  二、 SVM 函数归一化电压 (inv_vbus)
 *  
 *  SVM 函数期望输入的 alpha, beta 是相对于【合成全压矢量】的归一化占空比偏置。
 *  如果归一化定义是令 α, β ∈ [-√3/2, +√3/2]：
 *  合成的六个基本有效非零电压矢量长度（线电压）为 Vbus。
 *  在此定义下，相电压(从三相中性点出发)的六边形顶点矢量长度为 (2/3)*Vbus。
 *  
 *  因为我们在程序中的 v_alph 是等幅值变换的结果，所以它就是实际的相电压。
 *  SVPWM 基准矢量圆的理论最大相电压输出为 Vbus / √3，而此时我们希望传递给 `svm()` 
 *  的是刚好触碰内切圆的值。
 *  如果 SVM 的核心实现认为是：
 *    Ta = (1 - T1 - T2)/2 ; Tb = Ta + t1 ; Tc = Tb + t2
 *  并且内部是 `t1 = (α - 1/√3 β) * ARR` 等公式。
 *  为了让电压 V_α 产生正确的占空比 α_SVM，对应的关系是：
 *    α_SVM = V_α * (√3 / Vbus)
 *  
 *  证明：
 *  考虑 α轴最大电压(内接圆) V_α = Vbus / √3, β=0.
 *  此时 α_SVM = (Vbus / √3) * (√3 / Vbus) = 1.0
 *  但是在标准 SVM 里，对应第一扇区的占空比：
 *  t1 = α_SVM - β/√3 = 1.0 (这是最大线电压占空比基准)。但这其实是过调制点！
 *  
 *  【修正后的正确归一化】
 *  传统 FOC 控制库(如 TI / ST) 使用相电压进行空间矢量调制时：
 *  Duty_x = (V_x / Vbus) + 0.5  (此处 V_x 范围是 [-Vbus/2, Vbus/2])
 *  由于代码里的 `svm` 函数使用的输入公式类似重构马鞍波：
 *  其实 `svm` 内的 `alpha, beta` 就是直接对应相电压除以 `Vbus * (1/√3)` 还是别的？
 *  根据代码里的：
 *    t1 = alpha - ONE_BY_SQRT3 * beta; t2 = TWO_BY_SQRT3 * beta;
 *  这个 `svm` 的输入 alpha 和 beta 是：
 *    alpha = sqrt(3) * V_α / Vbus
 *    beta = sqrt(3) * V_β / Vbus
 *  所以 `inv_vbus = sqrt(3) / vbus` 才对！
 *  
 *  我们来验证：
 *  假设输出最大相电压 V_α = Vbus / √3，β = 0。
 *  则 alpha = (Vbus / √3) * (√3 / Vbus) = 1.0
 *  进 SVM:
 *  Sextant 1: t1 = 1.0, t2 = 0 ; 
 *  Ta = 0, Tb = 1.0, Tc = 1.0 (马鞍波顶点满占空比)
 *  这完全吻合！
 *  
 *  结论：原代码的 inv_vbus = 1.5 / vbus 是有偏差的，正确的应该是：
 *  inv_vbus = √3 / vbus 
 *
 * ─────────────────────────────────────────────────────────────────
 * 流程总结：
 *     vbus = 采样并转换为伏特值
 *     vs   = vbus / √3 * 0.96   
 *     inv_vbus = √3 / vbus       // 修正的关键点
 *     
 *     vd, vq (PI输出，范围 [-vs, vs]) => v_alph, v_beta (单位 V)
 *     alpha = v_alph * inv_vbus;
 *     beta = v_beta * inv_vbus;
 *     进 svm 生成 ta, tb, tc [0, 1]
 * ════════════════════════════════════════════════════════════════
 */
#include "common.h"
#include "modlue.h"

/* ======================== 全局实例 ======================== */
_RAM_DATA pmsm_t pm;

/* ======================== 内部前向声明 ======================== */

/* ======================== 内部工具函数 ======================== */
static float foc_clamp_f32(float in, float low, float high)
{
    if (in > high) return high;
    if (in < low)  return low;
    return in;
}

static uint16_t foc_clamp_u16(int32_t val, uint16_t low, uint16_t high)
{
    if (val < (int32_t)low)  return low;
    if (val > (int32_t)high) return high;
    return (uint16_t)val;
}

static int32_t foc_clamp_i32(int32_t val, int32_t low, int32_t high)
{
    if (val < low)  return low;
    if (val > high) return high;
    return val;
}

/* ================================================================
 *  A. PI 控制器
 * ================================================================ */
static float foc_pi_run(pid_para_t *pi, float ref, float fbk)
{
    float err = ref - fbk;
    pi->p_term  = pi->kp * err;
    pi->i_term += pi->ki * pi->ts * err;
    MIN_MAX_LIMT(pi->i_term, pi->i_term_min, pi->i_term_max);
    pi->out_value = pi->p_term + pi->i_term;
    return pi->out_value;
}

/* ================================================================
 *  B. 板级参数初始化
 * ================================================================ */
void pmsm_board_init(void)
{
    pm.board.v_ref = 3.3f;
    pm.board.v_adc = 4096.0f;

    pm.board.i_res = 0.005f;      /* 采样电阻 5 mΩ */
    pm.board.i_op  = 20.0f;       /* 运放增益 20x */

    pm.board.v1_res = 20000.0f;   /* 母线分压上桥 20kΩ */
    pm.board.v2_res = 1000.0f;    /* 母线分压下桥 1kΩ */

    pm.board.v_op    = (pm.board.v1_res + pm.board.v2_res) / pm.board.v2_res;
    pm.board.i_ratio = pm.board.v_ref / pm.board.v_adc / pm.board.i_res / pm.board.i_op;
    pm.board.v_ratio = pm.board.v_ref / pm.board.v_adc * pm.board.v_op;

    pm.board.i_max = pm.board.v_adc * pm.board.i_ratio * 0.5f;
    pm.board.v_max = pm.board.v_adc * pm.board.v_ratio;

    pm.board.dead_time = 0.5f;    /* µs */
}

/* ================================================================
 *  C. 电机参数初始化  (2312S PMSM)
 * ================================================================ */
static void pmsm_motor_init(void)
{
    pm.para.pn   = 6;
    pm.para.Rs   = 0.202977806f;
    pm.para.Ld   = 0.000108778855f;
    pm.para.Lq   = 0.000112416135f;
    pm.para.Ls   = 0.000110597495f;
    pm.para.Ldif = 3.63728032e-06f;
    pm.para.flux = 0.006488f;
    pm.para.B    = 0.000188353f;
    pm.para.Js   = 9.08865259e-05f;

    pm.para.Gr    = 1.0f;
    pm.para.ibw   = 300.0f;    /* 单电阻重构噪声大, 降低带宽 1000→300 */
    pm.para.delta = 4.0f;

    pm.para.div_pn  = 1.0f / pm.para.pn;
    pm.para.pnd_2pi = pm.para.pn / M_2PI;
    pm.para.div_Gr  = 1.0f / pm.para.Gr;
    pm.para.Gref    = 1.0f;

    pm.para.Kt     = 1.5f * pm.para.pn * pm.para.flux;
    pm.para.div_Kt = 1.0f / pm.para.Kt;

    pm.para.e_off = 2.34354496f;
    pm.para.r_off = 2.12998796f;
    pm.para.m_off = 0.0f;
}

/* ================================================================
 *  D. 时基参数初始化
 * ================================================================ */
static void pmsm_period_init(void)
{
    pm.period.foc_fs     = 20000.0f;
    pm.period.foc_ts     = 1.0f / pm.period.foc_fs;

    pm.period.cur_pid_fs = 20000.0f;
    pm.period.cur_pid_ts = 1.0f / pm.period.cur_pid_fs;

    /* 速度环由 TIM3 1kHz 中断驱动，不再在 FOC ISR 中计数分频 */
    pm.period.spd_pid_fs = 1000.0f;
    pm.period.spd_pid_ts = 1.0f / pm.period.spd_pid_fs;
}

/* ================================================================
 *  E. 总初始化入口
 * ================================================================ */
void pmsm_init(void)
{
    memset(&pm, 0, sizeof(pm));

    pmsm_motor_init();
    pmsm_board_init();
    pmsm_period_init();

    pm.ctrl_bit = opera;
    pm.foc.pwm_mode = pwm_svpwm;
    pm.foc.shunt_window_min = 0.12f;
    pm.foc.shunt_lpf_alpha  = 0.2f;

    /* 默认母线电压 (必须在 PI 限幅之前设置!)
     *   vbus:     直流母线电压, 后续所有电压计算的基准
     *   inv_vbus: = √3/vbus, SVM 归一化因子
     *             乘以实际相电压(V)转化为 SVM 输入 alpha/beta
     *   vs:       = vbus/√3 × 0.96, SVPWM 线性区最大电压矢量幅值
     *             vbus/√3 = SVPWM 六边形内切圆半径
     *             ×0.96   = 留 4% 安全余量防过调制 */
    pm.foc.vbus     = 12.0f;
    pm.foc.inv_vbus = SQRT3 / 12.0f;                 /* SVM 归一化修复: alpha = v_alph * (√3/Vbus) */
    pm.foc.vs       = 12.0f * ONE_BY_SQRT3 * 0.96f;  /* ≈6.65V, PI输出和电压圆的限幅参考 */
    pm.foc.svm_sector = 1;  /* 有效初始扇区 */

    /* 电流 PI (注意: i_term_max 使用 pm.foc.vs, 必须先初始化 vs)
     *   积分限幅 = ±vs 的原因:
     *   PI 积分项代表稳态所需电压, 限幅为 vs 实现抗积分饱和(anti-windup)
     *   当输出电压饱和时停止积分增长, 避免脱饱和后出现大幅超调 */
    pm.id_pi.kp = pm.para.Ld * pm.para.ibw;
    pm.id_pi.ki = pm.para.Rs * pm.para.ibw;
    pm.id_pi.ts = pm.period.cur_pid_ts;
    pm.id_pi.i_term_max =  pm.foc.vs;   /* ≈6.65V, d轴积分项上限 = SVPWM线性区极限 */
    pm.id_pi.i_term_min = -pm.foc.vs;

    pm.iq_pi.kp = pm.para.Lq * pm.para.ibw;
    pm.iq_pi.ki = pm.para.Rs * pm.para.ibw;
    pm.iq_pi.ts = pm.period.cur_pid_ts;
    pm.iq_pi.i_term_max =  pm.foc.vs;   /* q轴积分限幅同理 */
    pm.iq_pi.i_term_min = -pm.foc.vs;

    /* 速度 PI */
    pm.spd_pi.kp = (pm.para.Js * 100.0f) * pm.para.div_Kt;
    pm.spd_pi.ki = (pm.para.B  * 100.0f) * pm.para.div_Kt;
    pm.spd_pi.ts = pm.period.spd_pid_ts;
    pm.spd_pi.i_term_max =  8.0f;
    pm.spd_pi.i_term_min = -8.0f;

    /* 控制初始值 */
    pm.ctrl.id_set = 0.0f;
    pm.ctrl.iq_set = 0.0f;
    pm.ctrl.iq_lim = 8.0f;
    pm.ctrl.id_if  = 1.5f;    /* IF启动时d轴励磁电流 A */
    pm.ctrl.iq_if  = 2.0f;    /* IF爬速时q轴拖动电流 A */
    pm.ctrl.wr_set = 30.0f;
    pm.ctrl.vd_set = 0.0f;
    pm.ctrl.vq_set = 1.0f;   /* 仅V/f模式备用, IF阶段已改为电流PI */
    pm.ctrl.wm_acc = 200.0f;
    pm.ctrl.wm_dec = 200.0f;

    /* 采集电流零偏 */
    foc_get_curr_off();

    /* 单电阻重构初始化 */
    foc_single_shunt_init(&pm);

    /* SMO 初始化 */
    foc_smo_init(&pm);
}

/* ================================================================
 *  F. 上电自启动
 * ================================================================ */
void pmsm_power_on_autostart(pmsm_t *pm)
{
    pm->ctrl.id_set = 0.3f;
    pm->ctrl.id_if  = 0.0f;   /* IF对齐/爬速 d轴电流 A */
    pm->ctrl.iq_if  = 2.0f;   /* IF爬速 q轴拖动电流 A */
    pm->ctrl.iq_lim = 4.0f;
    pm->ctrl.wr_set = 30.0f;

    pm->period.start_cnt = 0U;

    pm->spd_pi.i_term = 0.0f;
    pm->id_pi.i_term  = 0.0f;
    pm->iq_pi.i_term  = 0.0f;

    pm->ctrl_bit = opera;
}

/* ================================================================
 *  G. 电流零偏采集 (自包含: 自行使能ADC + 软件触发 + 采集 + 关闭)
 *     此函数不依赖 TIM1 是否已启动, 也不依赖 HAL_ADCEx_InjectedStart_IT
 *     必须在 HAL_ADCEx_InjectedStart_IT 之前调用
 * ================================================================ */
void foc_get_curr_off(void)
{
    /* 1. 如果 ADC1 注入正在运行(JADSTART=1), 先停止 */
    if (ADC1->CR & ADC_CR_JADSTART)
    {
        ADC1->CR |= ADC_CR_JADSTP;
        while (ADC1->CR & ADC_CR_JADSTP) {}
    }

    /* 2. 如果 ADC1 尚未使能, 手动使能 */
    uint8_t was_enabled = (ADC1->CR & ADC_CR_ADEN) ? 1U : 0U;
    if (!was_enabled)
    {
        /* 确保 DEEPPWD=0, ADVREGEN 已完成 (MX_ADC1_Init 应已处理) */
        ADC1->ISR |= ADC_ISR_ADRDY;          /* 清 ADRDY 标志 */
        ADC1->CR  |= ADC_CR_ADEN;
        while (!(ADC1->ISR & ADC_ISR_ADRDY)) {} /* 等待 ADC 就绪 */
    }

    /* 3. 保存 JSQR, 临时改为软件触发 (JEXTEN=00) + 单次转换 (JL=0) */
    uint32_t jsqr_saved = ADC1->JSQR;
    ADC1->JSQR = jsqr_saved & ~(ADC_JSQR_JEXTEN_Msk | ADC_JSQR_JL_Msk);

    /* 4. 采集 1000 次零偏 */
    float sum = 0.0f;
    for (int i = 0; i < 1000; i++)
    {
        HAL_Delay(1);
        ADC1->CR |= ADC_CR_JADSTART;               /* 软件触发 */
        while (ADC1->CR & ADC_CR_JADSTART) {}       /* 等JADSTART自动清零 */
        sum += (float)(ADC1->JDR1);
    }
    pm.adc.ia_off = sum * 0.001f;
    pm.adc.ib_off = pm.adc.ia_off;  /* 单电阻: 两次采样共用同一零偏 */
    pm.adc.ic_off = pm.adc.ia_off;

    /* 5. 恢复 JSQR (含硬件触发配置) */
    ADC1->JSQR = jsqr_saved;

    /* 6. 如果之前未使能, 关掉 ADC (让后续 HAL_ADCEx_InjectedStart_IT 正常启动) */
    if (!was_enabled)
    {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN) {}
    }
}

/* ================================================================
 *  I. FOC 参数计算 (仅母线电压, 速度由 SMO 提供)
 * ================================================================ */

_RAM_FUNC void foc_para_calc(pmsm_t *pm)
{
    /* [vbus] ADC 原始值 → 实际母线电压 (V)
     *   v_ratio = (Vref/4096) × ((R1+R2)/R2), 将 ADC counts 转为实际电压 */
    pm->foc.vbus     = (float)pm->adc.vbus * pm->board.v_ratio;

    /* [inv_vbus] = √3 / vbus
     *   作用: 将 αβ 实际电压 (V) 归一化为 SVM 输入 [0,1] 的基准
     *   推导: alpha_norm = V_α * √3 / Vbus */
    pm->foc.inv_vbus = SQRT3 / pm->foc.vbus;

    /* [vs] = vbus/√3 × 0.96  (SVPWM 线性区最大电压矢量)
     *   vbus/√3: SVPWM 六边形内切圆半径, 即任意角度下不过调制的最大电压
     *   ×0.96:   留 4% 余量防止死区/纹波引起过调制
     *   用途:    PI 积分限幅 & 电压圆限幅的参考值 */
    pm->foc.vs       = pm->foc.vbus * ONE_BY_SQRT3 * 0.96f;

    /* 速度已由 foc_smo_run 写入 pm->foc.we / wr, 此处不再重复计算 */
}

/* ================================================================
 *  J. (已合并至 ISR)
 * ================================================================ */


/* ================================================================
 *  L. 单电阻重构: 初始化
 * ================================================================ */
void foc_single_shunt_init(pmsm_t *pm)
{
    pm->foc.i_a = 0.0f;
    pm->foc.i_b = 0.0f;
    pm->foc.i_c = 0.0f;
    pm->foc.shunt_window_min = 0.12f;
    pm->foc.shunt_lpf_alpha  = 0.2f;
    pm->foc.shunt_pair_ready = 0U;
    pm->foc.t1_eff = 0.0f;
    pm->foc.t2_eff = 0.0f;

    uint16_t arr  = (uint16_t)PWM_ARR();
    uint16_t half = (uint16_t)(arr / 2U);

    pm->foc.arr1_up = half;  pm->foc.arr2_up = half;  pm->foc.arr3_up = half;
    pm->foc.arr1_down = half; pm->foc.arr2_down = half; pm->foc.arr3_down = half;

    pm->foc.sample_trig1 = (uint16_t)(arr / 4U);
    pm->foc.sample_trig2 = (uint16_t)(arr / 3U);
}

/* ================================================================
 *  M. 单电阻三相电流重构 (基于SVM电压扇区)
 *
 *  下计数半周期采样顺序:
 *    窗口1(CCR_max→CCR_mid): 仅 Max相 上管导通 → ibus = I_{max相}
 *    窗口2(CCR_mid→CCR_min): Max+Mid相 上管导通 → ibus = -(I_{min相})
 *
 *  SVM扇区 | i1 (第一采样) | i2 (第二采样) | 重构
 *     3    | Ia            | -Ic           | ib = -(ia+ic)
 *     1    | Ib            | -Ic           | ia = -(ib+ic)
 *     5    | Ib            | -Ia           | ic = -(ia+ib)
 *     4    | Ic            | -Ia           | ib = -(ia+ic)
 *     6    | Ic            | -Ib           | ia = -(ic+ib)
 *     2    | Ia            | -Ib           | ic = -(ia+ib)
 * ================================================================ */
_RAM_FUNC static void foc_reconstruct_current(pmsm_t *pm, float i1, float i2)
{
    float ia, ib, ic;
    const uint8_t sector = pm->foc.svm_sector;

    /* 有效窗口判定 (T1+T2 是否足够) 无需判定窗口处于足够状态 */

    
        switch (sector)
        {
        case 3:  ia =  i1; ic = -i2; ib = (-i1 + i2); break;
        case 1:  ib =  i1; ic = -i2; ia = (-i1 + i2); break;
        case 5:  ib =  i1; ia = -i2; ic = (-i1 + i2); break;
        case 4:  ic =  i1; ia = -i2; ib = (-i1 + i2); break;
        case 6:  ic =  i1; ib = -i2; ia = (-i1 + i2); break;
        case 2:  ia =  i1; ib = -i2; ic = (-i1 + i2); break;
        }
    


    /* 一阶低通滤波 (单电阻重构必须启用, 否则PI震荡) */
    float lp = pm->foc.shunt_lpf_alpha;
    pm->foc.i_a += lp * (ia - pm->foc.i_a);
    pm->foc.i_b += lp * (ib - pm->foc.i_b);
    pm->foc.i_c += lp * (ic - pm->foc.i_c);
}

/* ================================================================
 *  N. 单电阻 PWM 移相调度 (TI F2803x 三级平移策略)
 *
 *  原理: 基于扇区查找表确定 H(Max)/M(Mid)/L(Min) 三相CCR,
 *        根据 T1(=H-M) 和 T2(=M-L) 与 Tmin 的关系分四种情况:
 *
 *  Case A: T1>=Tmin && T2>=Tmin  → 无需平移
 *  Case B: T1< Tmin && T2>=Tmin  → 窗口1不够, 推 H_dn 上升
 *  Case C: T1>=Tmin && T2< Tmin  → 窗口2不够, 推 L_dn 下降
 *  Case D: T1< Tmin && T2< Tmin  → 两窗口都不够, 同时推 H↑ L↓
 *
 *  每种情况最多三级(stage):
 *    Stage1: 仅调整一相, 伏秒补偿 xA = 2*Ton - xB
 *    Stage2: 一相饱和, 调整邻相
 *    Stage3: 两相饱和, 三相联动(共模偏移, 保持相间差分不变)
 *
 *  参考: TI SSPS_MACRO (f2803xSingleshunt.h)
 * ================================================================ */
#define ADC_CONV_TICKS  80   /* ADC 转换 + 安全余量 (ticks @ TIM1 clock) */

_RAM_FUNC void foc_single_shunt_schedule_update(pmsm_t *pm)
{
    const int16_t arr      = (int16_t)PWM_ARR();
    const int16_t Tmin     = (int16_t)600;
    const int16_t Tmindiv2    = (int16_t)(0.5f * Tmin);
    const int32_t Tuplimit   = arr - 1;   /* CCR 上限 (对应TI Tuplimit) */
    const int32_t Tdownlimit = 1;         /* CCR 下限 (对应TI Tdownlimit) */
    int16_t Ton1U ; 
    int16_t Ton2U ;
    int16_t Ton3U ;

    int16_t Ton1D ;
    int16_t Ton2D ;
    int16_t Ton3D ;

    uint16_t trig1;//第一次触发点
    uint16_t trig2;//第二次触发点

    pm->foc.shunt_case = 0;  /* 默认: 无移相 (Case A: T1和T2都足) */

    /* 归一化: 实际 αβ 电压 (V) × inv_vbus (√3/vbus) → SVM 归一化输入
     *   alpha, beta 归一化后使得 SVM 中的基本矢量运算能够正确匹配 Vbus
     *   因为 SVPWM 内切圆的最大有效相电压是 Vbus / √3, 
     *   如果输入相电压刚好也是 Vbus / √3，那么 (Vbus/√3) * (√3/Vbus) = 1.0
     *   正好对应占空比达到马鞍波的 1.0 上限 */
    float alpha = pm->foc.v_alph * pm->foc.inv_vbus;
    float beta  = pm->foc.v_beta * pm->foc.inv_vbus;

    int Sextant;
    float tmp1 = beta;
    float tmp2 = -0.5f * beta + SQRT3_BY_2 * alpha;
    float tmp3 = -tmp2 - tmp1;

    if (tmp3 > 0.0f)
    {
        if (tmp2 > 0.0f) Sextant = (tmp1 > 0.0f) ? 0 : 6;
        else             Sextant = (tmp1 > 0.0f) ? 5 : 4;
    }
    else
    {
        if (tmp2 > 0.0f) Sextant = (tmp1 > 0.0f) ? 3 : 2;
        else             Sextant = (tmp1 > 0.0f) ? 1 : 0;
    }
    int16_t X = (int16_t)(  beta * arr);
    int16_t Y = (int16_t)(( SQRT3_BY_2 * alpha + 0.5f * beta) * arr);
    int16_t Z = (int16_t)((0.5f * beta - SQRT3_BY_2 * alpha ) * arr);
    int16_t tA, tB, tC, t1, t2,Tdelta;
    switch (Sextant)
    {
    case 3:
    {
        t1 = (int16_t)( -Z );
        t2 = (int16_t)( X );
        tA = (int16_t)(( arr - t1 - t2) * 0.5f);
        tB = (int16_t)(tA + t1);
        tC = (int16_t)(tB + t2);
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
        if(t2 < Tmin && t1 >= Tmin)//0-30度
        {
            Ton3U=tB+Tmin;//调整第一阶段：仅调整C相，
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 1;
            if(Ton3U>Tuplimit)//C相调整后仍不足，进入第二阶段：调整B相
            {
                Ton3U=Tuplimit;
                Ton3D=2*tC-Ton3U;
                Ton2U=Tuplimit-Tmin;//调整B相
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 2;
                if(Ton2D>Tuplimit)//B相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton2D=Tuplimit;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton1U=Ton1U-0.5*Tdelta;
                    Ton1D=Ton1D-0.5*Tdelta;
                    Ton2U=Ton2U-0.5*Tdelta;
                    Ton3U=Ton3U-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }


        }
        else if (t2 >= Tmin && t1 < Tmin)//30-60°: 窗口2(t1)不足,推A相下移
        {
            Ton1U=tB-Tmin;//Stage1: 仅调整A相下移
            Ton1D=2*tA-Ton1U;  /* FIX: A相中心=tA,原2*tB错误 */
            pm->foc.shunt_case = 1;
            if(Ton1U<Tdownlimit)//A相调整后仍不足，进入第二阶段：调整C相
            {
                Ton1U=Tdownlimit;
                Ton1D=2*tA-Ton1U;
                Ton2U=Tmin;//调整B相
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 2;
                if(Ton2D<Tdownlimit)//C相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton2D=Tdownlimit;
                    Tdelta=Tmin-t2;//计算剩余不足量
                    Ton3U=Ton3U+0.5*Tdelta;
                    Ton3D=Ton3D+0.5*Tdelta;
                    Ton2U=Ton2U+0.5*Tdelta;
                    Ton1D=Ton1D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }

            }

        }
        else if (t1 < Tmin && t2 < Tmin)//
        {
            Ton1U=tB-Tmin;//调整第一阶段：调整A相，
            Ton1D=2*tA-Ton1U;
            Ton3U=tB+Tmin;//调整第三阶段：调整C相
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 4;
            if(Ton1U<Tdownlimit)
            { 
                Ton1U=Tdownlimit;
                Ton1D=2*tA-Ton1U;
                // Ton2U=Tmin;//调整B相
                // Ton2D=2*tB-Ton2U;
                // Ton3U=2*Tmin;//调整C相
                // Ton3D=2*tC-Ton3U;
                // if(Ton2D<Tdownlimit)
                // {
                //     Ton2D=Tdownlimit;
                // }
                // if(Ton3D<Tdownlimit)
                // {
                //     Ton3D=Tdownlimit;
                //     Ton3D=2*tC-Ton3U;
                // }
            }
            if(Ton3U>Tuplimit)
            {
                Ton3U=Tuplimit;
                Ton3D=2*tC-Ton3U;
                // Ton2U=Tuplimit-Tmin;//调整B相
                // Ton2D=2*tB-Ton2U;
                // Ton1U=Tuplimit-2*Tmin;//调整A相
                // Ton1D=2*tA-Ton1U;
                // if(Ton2D>Tuplimit)
                // {
                //     Ton2D=Tuplimit;
                // }
                // if (Ton1D > Tuplimit)
                // {
                //     Ton1D=Tuplimit;
                // }
                

            }
        }
        trig1=(Ton1U+Ton2U)/2;
        trig2=(Ton3U+Ton2U)/2;
    } break;
    case 1:
    {
        t1 = (int16_t)( Z );
        t2 = (int16_t)( Y );
        tB = ( arr - t1 - t2) * 0.5f;
        tA = tB + t1;
        tC = tA + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
         if(t2 < Tmin && t1 >= Tmin)//90-120度
         {
            Ton3U=tA+Tmin;//调整第一阶段：仅调整C相，
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 1;
            if(Ton3U>Tuplimit)//C相调整后仍不足，进入第二阶段：调整A相
            {
                Ton3U=Tuplimit;
                Ton3D=2*tC-Ton3U;
                Ton1U=Tuplimit-Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                pm->foc.shunt_case = 2;
                if(Ton1D>Tuplimit)
                {
                    Ton1D=Tuplimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton2U=Ton2U-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    Ton1U=Ton1U-0.5*Tdelta;
                    Ton3D=Ton3D-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t2 >= Tmin && t1 < Tmin)//60-90度
         {
            Ton2U=tA-Tmin;//调整第一阶段：仅调整B相，
            Ton2D=2*tB-Ton2U;
            pm->foc.shunt_case = 1;
            if(Ton2U<Tdownlimit)//B相调整后仍不足，进入第二阶段：调整C相
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;
                Ton1U=Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                    pm->foc.shunt_case = 2;
                if(Ton1D<Tdownlimit)//A相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton1D=Tdownlimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton3U=Ton3U+0.5*Tdelta;
                    Ton3D=Ton3D+0.5*Tdelta;
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton2U=tA-Tmin;//调整第一阶段：调整B相，
            Ton2D=2*tB-Ton2U;
            Ton3U=tA+Tmin;//调整第三阶段：调整A相
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 4;
            if(Ton2U<Tdownlimit)
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;
                // Ton1U=Tmin;//调整A相
                // Ton1D=2*tA-Ton1U;  
                // Ton3U=Tmin;//调整C相
                // Ton3D=2*tC-Ton3U;
                // if(Ton3D<Tdownlimit)
                // {
                //     Ton3D=Tdownlimit;
                // }
                // if(Ton1D<Tdownlimit)
                // {
                //     Ton1D=Tdownlimit;
                // }
            }
            if(Ton3U>Tuplimit)
            {
                Ton3U=Tuplimit;
                Ton3D=2*tC-Ton3U;
                // Ton1U=Tuplimit-Tmin;//调整B相
                // Ton1D=2*tA-Ton1U;
                // Ton2U=Tuplimit-2*Tmin;//调整A相
                // Ton2D=2*tB-Ton2U;
                // if(Ton1D>Tuplimit)
                // {
                //     Ton1D=Tuplimit;
                // }
                // if(Ton2D>Tuplimit)
                // {
                //     Ton2D=Tuplimit;
                // }
            }
         }
        trig1=(Ton2U+Ton1U)/2;
        trig2=(Ton3U+Ton1U)/2;
    } break;
    case 5:
    {
        t1 = (int16_t)(X);
        t2 = (int16_t)(-Y);
        tB = ( arr - t1 - t2) * 0.5f;
        tC = tB + t1;
        tA = tC + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
        if(t2 < Tmin && t1 >= Tmin)//120-150°: 窗口1(t2)不足,推A相上移
         {
            Ton1U=tC+Tmin;//Stage1: 仅调整A相上移
            Ton1D=2*tA-Ton1U;  /* FIX: A相中心=tA,原2*tB错误 */
            pm->foc.shunt_case = 1;
            if(Ton1U>Tuplimit)//A相调整后仍不足,进入Stage2:调整C相
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;  /* FIX: A相中心=tA,原2*tB错误 */
                Ton3U=Tuplimit-Tmin;//Stage2: 调整C相
                Ton3D=2*tC-Ton3U;
                pm->foc.shunt_case = 2;
                if(Ton3D>Tuplimit)
                {
                    Ton3D=Tuplimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton2U=Ton2U-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    Ton3U=Ton3U-0.5*Tdelta;
                    Ton1D=Ton1D-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t2 >= Tmin && t1 < Tmin)//150-180°: 窗口2(t1)不足,推B相下移
         {
            Ton2U=tC-Tmin;//Stage1: 仅调整B相下移
            Ton2D=2*tB-Ton2U;  /* FIX: B相中心=tB,原2*tC错误 */
            pm->foc.shunt_case = 1;
            if(Ton2U<Tdownlimit)//B相调整后仍不足,进入Stage2:调整C相
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;  /* FIX: B相中心=tB,原2*tC错误 */
                Ton3U=Tmin;//Stage2: 调整C相为Tmin创建窗口
                Ton3D=2*tC-Ton3U;  /* C相中心=tC 正确 */
                pm->foc.shunt_case = 2;
                if(Ton3D<Tdownlimit)//C相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton3D=Tdownlimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton1D=Ton1D+0.5*Tdelta;
                    Ton3U=Ton3U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制: 两窗口都不足,同时移相
         {
            Ton2U=tC-Tmin;//B相下移
            Ton2D=2*tB-Ton2U;  /* FIX: B相中心=tB,原2*tC错误 */
            Ton1U=tC+Tmin;//A相上移
            Ton1D=2*tA-Ton1U;  /* FIX: A相中心=tA,原2*tC错误 */
            pm->foc.shunt_case = 4;
            if(Ton2U<Tdownlimit)
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;  /* FIX */
                // Ton3U=Tmin;//调整C相为Tmin创建窗口
                // Ton3D=2*tC-Ton3U;  /* FIX */
                // Ton1U=2*Tmin;//调整A相为2Tmin创建窗口
                // Ton1D=2*tA-Ton1U;  /* FIX */
                // if(Ton1D<Tdownlimit)
                // {
                //     Ton1D=Tdownlimit;
                // }
                // if(Ton3D<Tdownlimit)
                // {
                //     Ton3D=Tdownlimit;
                // }
            }
            if(Ton1U>Tuplimit)
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;  /* FIX */
                // Ton3U=Tuplimit-Tmin;//调整C相
                // Ton3D=2*tC-Ton3U;  /* FIX */
                // Ton2U=Tuplimit-2*Tmin;//调整B相
                // Ton2D=2*tB-Ton2U;  /* FIX */
                // if(Ton2D>Tuplimit)
                // {
                //     Ton2D=Tuplimit;
                // }
                // if(Ton3D>Tuplimit)
                // {
                //     Ton3D=Tuplimit;
                // }
            }
         }
        trig1=(Ton2U+Ton3U)/2;
        trig2=(Ton3U+Ton1U)/2;
    } break;
    case 4:
    {
        t1 = (int16_t)(-X);
        t2 = (int16_t)(Z);
        tC = ( arr - t1 - t2) * 0.5f;
        tB = tC + t1;
        tA = tB + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
        if(t2 < Tmin && t1 >= Tmin)//210-240度
         {
            Ton1U=tB+Tmin;//调整第一阶段：仅调整A相，
            Ton1D=2*tA-Ton1U;
            pm->foc.shunt_case = 1;
            if(Ton1U>Tuplimit)//A相调整后仍不足，进入第二阶段：调整B相
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;
                Ton2U=Tuplimit-Tmin;//调整B相
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 2;
                if(Ton2D>Tuplimit)//B相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton2D=Tuplimit;
                    Ton2U=2*tB-Ton2D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton3U=Ton3U-0.5*Tdelta;
                    Ton3D=Ton3D-0.5*Tdelta;
                    Ton1D=Ton1D-0.5*Tdelta;
                    Ton2U=Ton2U-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }

            }
         }
         else if (t2 >= Tmin && t1 < Tmin)//180-210度
         {
            Ton3U=tB-Tmin;//调整第一阶段：仅调整C相，
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 1;
            if(Ton3U<Tdownlimit)//C相调整后仍不足，进入第二阶段：调整B相
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
                Ton2U=Tmin;//调整B相
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 2;
                if(Ton2D<Tdownlimit)//B相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton2D=Tdownlimit;
                    Ton2U=2*tB-Ton2D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton1D=Ton1D+0.5*Tdelta;
                    Ton2U=Ton2U+0.5*Tdelta;
                    Ton3D=Ton3D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
            
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton3U=tB-Tmin;//调整第一阶段：调整C相，
            Ton3D=2*tC-Ton3U;
            Ton1U=tB+Tmin;//调整第三阶段：调整A相
            Ton1D=2*tA-Ton1U;
            pm->foc.shunt_case = 4;
            if(Ton3U<Tdownlimit)
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
                // Ton2U=Tmin;//调整B相
                // Ton2D=2*tB-Ton2U;
                // Ton1U=2*Tmin;//调整A相
                // Ton1D=2*tA-Ton1U;
                // if(Ton1D<Tdownlimit)
                // {
                //     Ton1D=Tdownlimit;
                // }
                // if(Ton2D<Tdownlimit)
                // {
                //     Ton2D=Tdownlimit;
                // }
            }
            if(Ton1U>Tuplimit)
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;
                // Ton2U=Tuplimit-Tmin;//调整B相
                // Ton2D=2*tB-Ton2U;
                // Ton3U=Tuplimit-2*Tmin;//调整C相
                // Ton3D=2*tC-Ton3U;
                // if(Ton2D>Tuplimit)
                // {
                //     Ton2D=Tuplimit;
                // }
                // if(Ton3D>Tuplimit)
                // {
                //     Ton3D=Tuplimit;
                // }
            }
         }
        trig1=(Ton2U+Ton3U)/2;
        trig2=(Ton2U+Ton1U)/2;
    } break;
    case 6:
    {
        t1 = (int16_t)(-Y);
        t2 = (int16_t)(-Z);
        tC = ( arr - t1 - t2) * 0.5f;
        tA = tC + t1;
        tB = tA + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
         if(t2 < Tmin && t1 >= Tmin)//240-270度
         {
            Ton2U=tA+Tmin;//调整第一阶段：仅调整B相，
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 1;
            if(Ton2U>Tuplimit)//B相调整后仍不足，进入第二阶段：调整C相
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
                Ton1U=Tuplimit-Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                pm->foc.shunt_case = 2;
                if(Ton1D>Tuplimit)
                {
                    Ton1D=Tuplimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton3U=Ton3U-0.5*Tdelta;
                    Ton3D=Ton3D-0.5*Tdelta;
                    Ton1U=Ton1U-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }

         }
         else if (t2 >= Tmin && t1 < Tmin)//270-300度
         {
            Ton3U=tA-Tmin;//调整第一阶段：仅调整C相，
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 1;
            if(Ton3U<Tdownlimit)//C相调整后仍不足，进入第二阶段：调整B相
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
                Ton1U=Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                pm->foc.shunt_case = 2;
                if(Ton1D<Tdownlimit)//A相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton1D=Tdownlimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton2U=Ton2U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton3D=Ton3D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton3U=tA-Tmin;//调整第一阶段：调整C相，
            Ton3D=2*tC-Ton3U;
            Ton2U=tA+Tmin;//调整第三阶段：调整B相
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 4;
            if(Ton3U<Tdownlimit)
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
                // Ton1U=Tmin;//调整A相
                // Ton1D=2*tA-Ton1U;
                // Ton2U=2*Tmin;//调整B相
                // Ton2D=2*tB-Ton2U;
                // if(Ton1D<Tdownlimit)
                // {
                //     Ton1D=Tdownlimit;
                // }
                // if(Ton2D<Tdownlimit)
                // {
                //     Ton2D=Tdownlimit;
                // }
            }
            if(Ton2U>Tuplimit)
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
                // Ton1U=Tuplimit-Tmin;//调整A相
                // Ton1D=2*tA-Ton1U;
                // Ton3U=Tuplimit-2*Tmin;//调整C相
                // Ton3D=2*tC-Ton3U;
                // if(Ton1D>Tuplimit)
                // {                 
                //     Ton1D=Tuplimit;
                // }             
                // if(Ton3D>Tuplimit)
                // {
                //     Ton3D=Tuplimit;
                // }   
            }
            
         }
        trig1=(Ton3U+Ton1U)/2;
        trig2=(Ton1U+Ton2U)/2;
    } break;
    case 2:
    default:
    {
        int16_t t1 = (int16_t)(Y);
        int16_t t2 = (int16_t)(-X);
        tA = (arr - t1 - t2) * 0.5f;
        tC = tA + t1;
        tB = tC + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
        if(t2 < Tmin && t1 >= Tmin)//330-360度
         {
            Ton2U=tC+Tmin;//调整第一阶段：仅调整B相，
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 1;
            if(Ton2U>Tuplimit)//B相调整后仍不足，进入第二阶段：调整A相
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
                Ton3U=Tuplimit-Tmin;//调整C相
                Ton3D=2*tC-Ton3U;
                pm->foc.shunt_case = 2;
                if(Ton3D>Tuplimit)
                {
                    Ton3D=Tuplimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton1U=Ton1U-0.5*Tdelta;
                    Ton1D=Ton1D-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    Ton3U=Ton3U-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t2 >= Tmin && t1 < Tmin)//300-330度
         {
            Ton1U=tC-Tmin;//调整第一阶段：仅调整A相，
            Ton1D=2*tA-Ton1U;
            pm->foc.shunt_case = 1;
            if(Ton1U<Tdownlimit)//A相调整后仍不足，进入第二阶段：调整A相
            {
                Ton1U=Tdownlimit;
                Ton1D=2*tA-Ton1U;
                Ton3U=Tmin;//调整C相
                Ton3D=2*tC-Ton3U;
                pm->foc.shunt_case = 2; 
                if(Ton3D<Tdownlimit)//C相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton3D=Tdownlimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton2U=Ton2U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    Ton3U=Ton3U+0.5*Tdelta;
                    Ton1D=Ton1D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton1U=tC-Tmin;//调整第一阶段：调整A相，
            Ton1D=2*tA-Ton1U;
            Ton2U=tC+Tmin;//调整第三阶段：调整B相
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 4;
            if(Ton1U<Tdownlimit)
            {
                Ton1U=Tdownlimit;
                Ton1D=2*tA-Ton1U;
                // Ton3U=Tmin;//调整C相
                // Ton3D=2*tC-Ton3U;
                // Ton2U=2*Tmin;//调整B相
                // Ton2D=2*tB-Ton2U;
                // if(Ton3D<Tdownlimit)
                // {
                //     Ton3D=Tdownlimit;
                // }
                // if(Ton2D<Tdownlimit)
                // {
                //     Ton2D=Tdownlimit;
                // }
            }
            if(Ton2U>Tuplimit)
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
                // Ton3U=Tuplimit-Tmin;//调整A相
                // Ton3D=2*tC-Ton3U;
                // Ton1U=Tuplimit-2*Tmin;//调整C相
                // Ton1D=2*tA-Ton1U;
                // if(Ton1D>Tuplimit)
                // {
                //     Ton1D=Tuplimit;
                // }
                // if(Ton3D>Tuplimit)
                // {                    
                //     Ton3D=Tuplimit;
                // }
            }
         }
        trig1=(Ton1U+Ton3U)/2;
        trig2=(Ton3U+Ton2U)/2;
    } break;
    }


    pm->foc.svm_sector = (uint8_t)Sextant;

    // float t1_norm = (Sextant & 1) ? tv2 : tv1;
    // float t2_norm = (Sextant & 1) ? tv1 : tv2;
    // t1_norm = foc_clamp_f32(t1_norm, 0.0f, 1.0f);
    // t2_norm = foc_clamp_f32(t2_norm, 0.0f, 1.0f);
    // pm->foc.t1_eff = t1_norm;
    // pm->foc.t2_eff = t2_norm;


    pm->foc.arr1_up = (uint16_t)Ton1U;
    pm->foc.arr2_up = (uint16_t)Ton2U;
    pm->foc.arr3_up = (uint16_t)Ton3U;
    pm->foc.arr1_down = (uint16_t)Ton1D;
    pm->foc.arr2_down = (uint16_t)Ton2D;
    pm->foc.arr3_down = (uint16_t)Ton3D;





    pm->foc.sample_trig1 = (uint16_t)trig1;
    pm->foc.sample_trig2 = (uint16_t)trig2;

    /* TODO: 这里预留给你写移相(SSPS)逻辑，基于 t1_eff/t2_eff + sector 直接修改 arr*_up/down 和 sample_trig* */
}

/* ================================================================
 *  O. 单电阻: 上/下半周期加载 + 双缓冲交换
 * ================================================================ */
/* 上半周期 CCR (伏秒补偿, 无ADC触发) */
_RAM_FUNC void foc_single_shunt_apply_up(pmsm_t *pm)
{
    htim1.Instance->CCR1 = pm->foc.arr1_up;
    htim1.Instance->CCR2 = pm->foc.arr2_up;
    htim1.Instance->CCR3 = pm->foc.arr3_up;
}

/* 下半周期 CCR (采样半周期) + ADC触发点
 * TRGO2 = OC4REF_RISING | OC6REF_RISING, PWM Mode 1:
 *   下计数时 OCxREF 在 CNT 降至 CCRx 时产生上升沿→触发ADC */
_RAM_FUNC void foc_single_shunt_apply_down(pmsm_t *pm)
{
    htim1.Instance->CCR1 = pm->foc.arr1_down;
    htim1.Instance->CCR2 = pm->foc.arr2_down;
    htim1.Instance->CCR3 = pm->foc.arr3_down;
    htim1.Instance->CCR4 = pm->foc.sample_trig1;
    htim1.Instance->CCR6 = pm->foc.sample_trig2;
}

/* ================================================================
 *  R. PWM 启停
 * ================================================================ */
_RAM_FUNC void foc_pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_6);

    foc_single_shunt_apply_up(&pm);
}

_RAM_FUNC void foc_pwm_stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_6);
}

_RAM_FUNC void foc_pwm_run(pmsm_t *pm)
{
    foc_single_shunt_schedule_update(pm);
}

_RAM_FUNC void foc_pwm_duty_set(pmsm_t *pm)
{
    uint16_t half = (uint16_t)(PWM_ARR() / 2U);
    pm->foc.dtc_a = 0.5f;
    pm->foc.dtc_b = 0.5f;
    pm->foc.dtc_c = 0.5f;
    pm->foc.svm_sector = 1U;
    pm->foc.t1_eff = 0.0f;
    pm->foc.t2_eff = 0.0f;

    pm->foc.arr1_up   = half; pm->foc.arr2_up   = half; pm->foc.arr3_up   = half;
    pm->foc.arr1_down = half; pm->foc.arr2_down = half; pm->foc.arr3_down = half;
    pm->foc.sample_trig1 = (uint16_t)(half - ADC_CONV_TICKS);
    pm->foc.sample_trig2 = (uint16_t)(half - 2U * ADC_CONV_TICKS);
}

/* ================================================================
 *  S. SMO 滑模观测器 + PLL 锁相环 (含 LPF 相位补偿)
 *
 *  数据流:
 *   1. 滑模控制量  Zα/Zβ = Kslide · sat(i_err / E0)
 *   2. LPF 提取反电势  Eα/Eβ  (截止频率 Kslf rad/s)
 *   3. 相位补偿: 将 Eα/Eβ 超前旋转 φ=atan(ωe/Kslf) 抵消 LPF 的相位滞后
 *   4. PLL 误差(cross-product 归一化):
 *        err_n = (Eαc·cosθ̂ + Eβc·sinθ̂) / |Ec|  ≈ sin(θ̂ − θe)
 *   5. PI → ωe_pll → 积分 → θ̂
 *
 *  参数设计 (电机: pn=6, flux=0.0065 Wb, wr_max=30 rad/s)
 *    we_max   = wr_max × pn = 180 rad/s
 *    Kslide   ≈ 1.5 × we_max × flux ≈ 1.75 V → 取 2.0
 *    Kslf     ≈ 3 × we_max ≈ 540 rad/s        → 取 500
 *    E0       = 额定电流 × 20% ≈ 1.0 A
 *    PLL带宽  ωn = 300 rad/s, ζ = 0.707
 *      kp = 2ζωn ≈ 424,  ki = ωn² = 90000
 * ================================================================ */
void foc_smo_init(pmsm_t *pm)
{
    pm->esmo.Rs     = &pm->para.Rs;
    pm->esmo.Ls     = &pm->para.Ls;
    pm->esmo.Ld     = &pm->para.Ld;
    pm->esmo.Lq     = &pm->para.Lq;
    pm->esmo.flux   = &pm->para.flux;
    pm->esmo.i_alph = &pm->foc.i_alph;
    pm->esmo.i_beta = &pm->foc.i_beta;
    pm->esmo.v_alph = &pm->foc.v_alph;
    pm->esmo.v_beta = &pm->foc.v_beta;
    pm->esmo.we     = &pm->foc.we;

    /* Kslide: BEMF倍数因子 (kslide_eff = |we| * flux * Kslide)
     *   需满足: BEMF_max < kslide_eff < (Vapplied_min - Rs*Ipeak)
     *   实际: BEMF_max=1.17V, Kslide=1.2 → kslide_eff=1.40V @ we=180
     *   Vapplied左右线快运行时: sqrt(1.57^2+0.3^2)=1.60V > 1.40V ✓ */
    pm->esmo.Kslide = 1.2f;    /* BEMF倍数因子 (1.1~1.3合适) */
    pm->esmo.Kslf   = 500.0f;
    pm->esmo.E0     = 0.5f;    /* 边界层宽度 (A), 需 < 典型电流误差才能进入滑模 */
    pm->esmo.ts     = pm->period.foc_ts;
    pm->esmo.fs     = pm->period.foc_fs;

    pm->esmo.pll_kp      = 424.0f;
    pm->esmo.pll_ki      = 90000.0f;
    pm->esmo.pll_err_int = 0.0f;
    pm->esmo.we_pll      = 0.0f;

    pm->esmo.we_est   = 0.0f;
    pm->esmo.pos_e    = 0.0f;
    pm->esmo.m_theta  = 0.0f;
    pm->esmo.EstIalph = 0.0f;
    pm->esmo.EstIbeta = 0.0f;
    pm->esmo.Ealph    = 0.0f;
    pm->esmo.Ebeta    = 0.0f;
}

_RAM_FUNC void foc_smo_run(pmsm_t *pm)
{
    const float ts = pm->esmo.ts;
    const float ls = max(*pm->esmo.Ls, 1e-6f);
    const float rs = *pm->esmo.Rs;

    /* ── 0. 自适应 Kslide: 随转速同步缩放 ─────────────────────────
     *   原理: BEMF_max = |ωe| × flux, Kslide 需略大于 BEMF 才能保持滑模
     *         低速时 BEMF≈0, 若 Kslide 取固定大值会导致 EstI 偏到
     *         (Vapplied - Kslide)/Rs >> 额定电流, SMO 永久饱和.
     *   IF 阶段用 we_set (已知命令速度); SMO 闭环用 we_pll.
     *   Kslide_eff = max(1.5 × |ωe| × flux, Kslide_min=0.05V)        */
    float we_eff = (pm->foc.run_stage < 2U)
                    ? pm->ctrl.we_set
                    : fabsf(pm->esmo.we_pll);
    float kslide_eff = we_eff * (*pm->esmo.flux) * pm->esmo.Kslide;
    if (kslide_eff < 0.3f) kslide_eff = 0.3f;

    /* ── 1. 电流估算误差 ─────────────────────────────────────────── */
    float i_err_a = pm->foc.i_alph - pm->esmo.EstIalph;
    float i_err_b = pm->foc.i_beta  - pm->esmo.EstIbeta;

    /* ── 2. 饱和函数 → 滑模控制量 ───────────────────────────────── */
    float sat_a = i_err_a / pm->esmo.E0;
    float sat_b = i_err_b / pm->esmo.E0;
    MIN_MAX_LIMT(sat_a, -1.0f, 1.0f);
    MIN_MAX_LIMT(sat_b, -1.0f, 1.0f);
    pm->esmo.Zalph = kslide_eff * sat_a;
    pm->esmo.Zbeta = kslide_eff * sat_b;

    /* ── 3. 估算电流积分 (前向欧拉) ──────────────────────────────── */
    pm->esmo.EstIalph += ((pm->foc.v_alph - rs * pm->esmo.EstIalph - pm->esmo.Zalph) / ls) * ts;
    pm->esmo.EstIbeta += ((pm->foc.v_beta  - rs * pm->esmo.EstIbeta  - pm->esmo.Zbeta)  / ls) * ts;

    /* ── 4. LPF 提取反电势 ────────────────────────────────────────
     *   H(s) = Kslf / (s + Kslf),  离散: E[k] = E[k-1] + (Z - E[k-1])·Kslf·ts */
    pm->esmo.Ealph += (pm->esmo.Zalph - pm->esmo.Ealph) * pm->esmo.Kslf * ts;
    pm->esmo.Ebeta += (pm->esmo.Zbeta  - pm->esmo.Ebeta)  * pm->esmo.Kslf * ts;

    /* ── 5. LPF 相位补偿 ──────────────────────────────────────────
     *   LPF 在 ωe 处的相位滞后: φ = atan(ωe / Kslf)
     *   将 Eα/Eβ 逆时针旋转 φ (超前补偿):
     *     Eαc = Eα·cosφ − Eβ·sinφ
     *     Eβc = Eα·sinφ + Eβ·cosφ */
    float comp    = atan2f(pm->esmo.we_pll, pm->esmo.Kslf);
    float cos_c   = cosf(comp);
    float sin_c   = sinf(comp);
    float Ealph_c = pm->esmo.Ealph * cos_c - pm->esmo.Ebeta * sin_c;
    float Ebeta_c = pm->esmo.Ealph * sin_c + pm->esmo.Ebeta * cos_c;

    /* ── 6. PLL 误差计算 (cross-product, 归一化至 [-1,1]) ──────────
     *   [Eαc, Eβc] = Emag·[−sinθe, cosθe]
     *   cross = Eαc·cosθ̂ + Eβc·sinθ̂ = Emag·sin(θ̂ − θe)
     *   取负号 → err = sin(θe − θ̂): θ̂ 滞后时 err>0 → we_pll 增大 → 收敛 */
    float Emag    = sqrtf(Ealph_c * Ealph_c + Ebeta_c * Ebeta_c);
    float cos_pe  = cosf(pm->esmo.pos_e);
    float sin_pe  = sinf(pm->esmo.pos_e);
    float err_raw = Ealph_c * cos_pe + Ebeta_c * sin_pe;
    float err_n   = (Emag > 0.01f) ? (-err_raw / Emag) : 0.0f;

    /* ── 7. PLL PI 调节器 → ωe_pll ──────────────────────────────── */
    pm->esmo.pll_err_int += pm->esmo.pll_ki * err_n * ts;
    MIN_MAX_LIMT(pm->esmo.pll_err_int, -3000.0f, 3000.0f);
    pm->esmo.we_pll = pm->esmo.pll_kp * err_n + pm->esmo.pll_err_int;

    /* ── 8. 积分得到角度 θ̂ ──────────────────────────────────────── */
    pm->esmo.pos_e += pm->esmo.we_pll * ts;
    wrap_0_2pi(pm->esmo.pos_e);

    /* ── 9. 写回 foc 结构体 ───────────────────────────────────────── */
    pm->esmo.we_est = pm->esmo.we_pll;
    pm->foc.p_e     = pm->esmo.pos_e;
    pm->foc.we      = pm->esmo.we_pll;
    pm->foc.wr      = pm->esmo.we_pll * pm->para.div_pn;
}

/* ================================================================
 *  TIM1 更新中断: FOC 主循环 (扁平化, 全部计算在一个函数内)
 *
 *  Overflow  (counting_down=1, 到达ARR):
 *    采样→重构→Clarke→Vbus→SMO→角度选择→Park→PI→iPark→SVM→调度→预加载UP
 *  Underflow (counting_down=0, 到达0):
 *    预加载DOWN + ADC触发点
 * ================================================================ */
_RAM_FUNC void foc_tim1_update_isr(pmsm_t *pm)
{
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
    {
        /* ===== Overflow: 到达ARR, 即将下计数 ===== */

        /* --- 1. 读取 ADC, 重构三相电流 --- */
        pm->adc.vbus = ADC2->JDR1;//读母线电压值

        float i_s1 = (pm->foc.i_shunt_raw_1 - pm->adc.ia_off) * pm->board.i_ratio;
        float i_s2 = (pm->foc.i_shunt_raw_2 - pm->adc.ib_off) * pm->board.i_ratio;
        pm->foc.i_shunt_1 = i_s1;
        pm->foc.i_shunt_2 = i_s2;
        foc_reconstruct_current(pm, i_s1, i_s2);


        /* --- 2. Clarke 变换 (2/3 等幅值) --- */
        pm->foc.i_alph = TWO_BY_THREE * (pm->foc.i_a - 0.5f * pm->foc.i_b - 0.5f * pm->foc.i_c);
        pm->foc.i_beta = ONE_BY_SQRT3 * (pm->foc.i_b - pm->foc.i_c);

        /* --- 3. 母线电压更新 ---
         *   vbus:     ADC → 实际电压 (V), 是所有电压计算的根
         *   inv_vbus: = √3/vbus, 归一化因子的修正值 
         *             将相电压转换为满量程为1的占空比基准
         *   vs:       = vbus/√3 × 0.96, SVPWM 线性最大相电压
         *             用于下方 PI 输出限幅和电压圆裁剪 */
        float vb = (float)pm->adc.vbus * pm->board.v_ratio;
        pm->foc.vbus     = vb;
        pm->foc.inv_vbus = SQRT3 / vb;
        pm->foc.vs       = vb * ONE_BY_SQRT3 * 0.96f;
        
    

        /* --- 4. SMO 观测器 --- */
        foc_smo_run(pm);

        /* --- 5. 角度 + 电流参考: IF开环 / SMO闭环 --- */
        /*   对齐阶段 0~IF_ALIGN: theta固定=0, 让转子锁到d轴
         *   爬速阶段 IF_ALIGN~IF_TOTAL: we从IF_WE_MIN线性增到目标
         *   闭环阶段 >= IF_TOTAL: SMO角度, 速度PI给iq */
        #define IF_ALIGN_CNT  1000U   /* 0.5s 对齐 */
        #define IF_TOTAL_CNT  100000U   /* 1.5s IF总时长 */
        #define IF_WE_MIN     0.0f    /* 起始电角速度 rad/s */

        float theta, id_ref, iq_ref;
        if (pm->period.start_cnt < IF_TOTAL_CNT)
        {
            pm->period.start_cnt++;

            /* 运行阶段标志 */
            if (pm->period.start_cnt <= IF_ALIGN_CNT)
                pm->foc.run_stage = 0;   /* 对齐 */
            else
                pm->foc.run_stage = 1;   /* IF爬速 */

            if (pm->period.start_cnt <= IF_ALIGN_CNT)
            {
                /* ---- 对齐阶段: 固定角度, 锁转子 (id_if A 压入d轴) ---- */
                pm->ctrl.drag_pe = 0.0f;
                theta  = 0.0f;
                id_ref = pm->ctrl.id_if;
                iq_ref = 0.0f;
            }
            else
            {
                /* ---- 爬速阶段: 线性加速 + q轴拖动电流 iq_if ---- */
                float we_target = max(15 * pm->para.pn, IF_WE_MIN);
                float ramp_k = (float)(pm->period.start_cnt - IF_ALIGN_CNT)
                             / (float)(IF_TOTAL_CNT - IF_ALIGN_CNT);
                pm->ctrl.we_set = IF_WE_MIN + (we_target - IF_WE_MIN) * ramp_k;

                pm->ctrl.drag_pe += pm->ctrl.we_set * pm->period.foc_ts;
                wrap_0_2pi(pm->ctrl.drag_pe);
                theta  = pm->ctrl.drag_pe;
                id_ref =0;   /* d轴维持励磁 */
                iq_ref = pm->ctrl.iq_if;   /* q轴固定拖动电流 */
            }

            /* 切换前一拍: 同步 SMO/PLL 状态, 清 PI 积分 */
            if (pm->period.start_cnt == IF_TOTAL_CNT - 1U)
            {
                float we_sync  = pm->ctrl.we_set;
                float theta_sw = pm->ctrl.drag_pe;
                float flux_val = *pm->esmo.flux;

                /* PLL 角度/速度同步 */
                pm->esmo.pos_e       = theta_sw;
                pm->esmo.m_theta     = theta_sw;
                pm->esmo.we_est      = we_sync;
                pm->esmo.we_pll      = we_sync;
                pm->esmo.pll_err_int = we_sync;  /* 稳态 err=0 时积分项维持速度 */

                /* SMO 状态复位: 消除开环期间 LPF/观测器积累的错误历史
                 *   EstI → 赋实际电流, 使 i_err=0, sat=0, Z=0 (平滑过渡)
                 *   Ealph/Ebeta → 理论 BEMF 初值, 避免 LPF 从错误值收敛 */
                pm->esmo.EstIalph = pm->foc.i_alph;
                pm->esmo.EstIbeta = pm->foc.i_beta;
                pm->esmo.Ealph    = -we_sync * flux_val * sinf(theta_sw);
                pm->esmo.Ebeta    =  we_sync * flux_val * cosf(theta_sw);

                pm->spd_pi.i_term = 0.0f;
                pm->id_pi.i_term  = 0.0f;
                pm->iq_pi.i_term  = 0.0f;
            }
            
        }
        else
        {
            /* SMO 闭环: id=0, iq=速度环输出 */
            pm->foc.run_stage = 2;   /* 闭环 */
            theta  = pm->esmo.pos_e;
            id_ref = 0.0f;
            iq_ref = pm->ctrl.iq_set;//速度环输出传入
        }

        /* --- 6. sin/cos + Park 变换 --- */
        pm->foc.theta = theta;
        wrap_0_2pi(pm->foc.theta);
        pm->foc.sin_val = sinf(pm->foc.theta);
        pm->foc.cos_val = cosf(pm->foc.theta);
        pm->foc.i_d =  pm->foc.i_alph * pm->foc.cos_val + pm->foc.i_beta * pm->foc.sin_val;
        pm->foc.i_q =  pm->foc.i_beta * pm->foc.cos_val - pm->foc.i_alph * pm->foc.sin_val;

        /* --- 7. d/q 轴电流 PI (IF阶段与闭环阶段统一用PI) ---
         *   IF对齐:  id_ref=id_if, iq_ref=0
         *   IF爬速:  id_ref=id_if, iq_ref=iq_if  (固定拖动电流)
         *   SMO闭环: id_ref=0,     iq_ref=iq_set  (速度环输出)
         *   PI输出 vd, vq (V), 逆Park后得到 v_alph/v_beta 供SMO使用 */
        float vd, vq;
        /* IF阶段与闭环阶段均使用电流PI, 保证 v_alph/v_beta 真实反映施加电压
         * 这样SMO积分器能正确收敛 (若用固定vq_set且vq_set<Kslide会导致SMO饱和死锁) */
        MIN_MAX_LIMT(iq_ref, -pm->ctrl.iq_lim, pm->ctrl.iq_lim);
        vd = 0;  /* d轴电压参考固定为0, 仅用于锁定和维持磁场 */
        vq = foc_pi_run(&pm->iq_pi, iq_ref, pm->foc.i_q);
    

        /* 电压圆限幅: 保证 √(vd²+vq²) ≤ vs = vbus/√3 × 0.96
         *   vs 是 SVPWM 六边形内切圆半径, 限幅到此范围
         *   确保在任意电角度下都不会过调制
         *   !! 必须启用 !! 否则PI饱和时产生过调制, 六边形裁剪导致扇区不均 */
        float vmax = pm->foc.vs;
        float vmag = sqrtf(vd * vd + vq * vq);
        if (vmag > vmax)
        {
            float s = vmax / (vmag + 1e-6f);
            vd *= s;  /* 等比例缩放, 保持 vd/vq 方向不变 */
            vq *= s;
        }
        pm->foc.v_d = vd;  /* dq 轴电压, 单位: V */
        pm->foc.v_q = vq;

        /* --- 8. 逆 Park 变换 ---
         *   将旋转 dq 坐标电压 → 静止 αβ 坐标电压 (仍然是实际电压 V)
         *   v_alph, v_beta 有两个去处:
         *     ① ×inv_vbus 归一化后 → SVM 计算占空比
         *     ② 直接作为 SMO 观测器的输入电压 */
        pm->foc.v_alph = vd * pm->foc.cos_val - vq * pm->foc.sin_val;
        pm->foc.v_beta = vd * pm->foc.sin_val + vq * pm->foc.cos_val;

        /* --- 9. SVM + T1/T2判定 + 规则移相 + ADC触发点 --- */
        foc_single_shunt_schedule_update(pm);

        /* --- 10. 上溢中断仅配置前半周期电流采样触发值 + 当前上半周三相值 --- */
        htim1.Instance->CCR1 = pm->foc.arr1_up;
        htim1.Instance->CCR2 = pm->foc.arr2_up;
        htim1.Instance->CCR3 = pm->foc.arr3_up;
        htim1.Instance->CCR4 = pm->foc.sample_trig1;
        htim1.Instance->CCR6 = pm->foc.sample_trig2;
    }
    else
    {
        /* ===== Underflow: 到达0, 即将上计数 ===== */
        /* 配置三相下降阶段触发值 */
        htim1.Instance->CCR1 = pm->foc.arr1_down;
        htim1.Instance->CCR2 = pm->foc.arr2_down;
        htim1.Instance->CCR3 = pm->foc.arr3_down;

    }
}

/* ================================================================
 *  速度环 PI (TIM3 1kHz, 仅 SMO 闭环阶段运行)
 * ================================================================ */
void foc_spd_pi_calc(pmsm_t *pm)
{
    if (pm->period.start_cnt < 30000U)
        return;

    pm->spd_pi.i_term_max =  pm->ctrl.iq_lim;
    pm->spd_pi.i_term_min = -pm->ctrl.iq_lim;

    float wr_fb  = pm->foc.wr;
    float iq_ref = foc_pi_run(&pm->spd_pi, pm->ctrl.wr_set, wr_fb);
    MIN_MAX_LIMT(iq_ref, -pm->ctrl.iq_lim, pm->ctrl.iq_lim);
    pm->ctrl.iq_set = iq_ref;
}


