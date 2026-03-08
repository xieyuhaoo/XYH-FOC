/**
 * @file    common.h
 * @brief   FOC 单电阻采样控制核心数据结构与函数声明
 * @details 精简版，仅保留单电阻FOC+SMO无感控制所需结构
 *          硬件平台: STM32G431RBT  AxDr_L驱动板
 *          电机: 2312S PMSM  7极对
 */
#ifndef __COMMON_H__
#define __COMMON_H__

#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

/* ======================== 编译器属性 ======================== */
#define _RAM_FUNC
#define _RAM_DATA

/* ======================== 类型别名 ======================== */
typedef float    f32;
typedef double   f64;
typedef int64_t  s64;
typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const float    cf32;
typedef const uint32_t cu32;
typedef const uint16_t cu16;
typedef const uint8_t  cu8;

typedef __IO float    vf32;
typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

/* ======================== 数学宏 ======================== */
#define SIGN(x)          (((x) < 0.0f) ? -1.0f : 1.0f)
#define NORM2_f(x, y)    (sqrtf(SQ(x) + SQ(y)))
#define SQ(x)            ((x) * (x))
#define ABS(x)           ((x) > 0 ? (x) : -(x))
#define min(x, y)        (((x) < (y)) ? (x) : (y))
#define max(x, y)        (((x) > (y)) ? (x) : (y))

#define MIN_MAX_LIMT(in, low, high) \
    (in = (in) > (high) ? (high) : (in) < (low) ? (low) : (in))
#define MAX_LIMT(in, outmax) \
    (in = (in) > (outmax) ? (outmax) : (in) < (-(outmax)) ? (-(outmax)) : (in))

#define wrap_pm_pi(theta)                          \
    theta = (theta > M_PI)  ? theta - M_2PI : theta; \
    theta = (theta < -M_PI) ? theta + M_2PI : theta;
#define wrap_0_2pi(theta)                          \
    theta = (theta > M_2PI) ? theta - M_2PI : theta; \
    theta = (theta < 0.0f)  ? theta + M_2PI : theta;

/* ======================== 数学常量 ======================== */
#define M_PI             (3.14159265358f)
#define M_2PI            (6.28318530716f)
#define SQRT3            (1.73205080757f)
#define SQRT3_BY_2       (0.86602540378f)
#define ONE_BY_SQRT3     (0.57735026919f)
#define TWO_BY_SQRT3     (1.15470053838f)
#define TWO_BY_THREE     (0.66666666667f)

/* ======================== 硬件抽象宏 ======================== */
#define Dead_Time  80

#define PWM_ARR()  __HAL_TIM_GET_AUTORELOAD(&htim1)

#define set_dtc_a(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value)
#define set_dtc_b(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value)
#define set_dtc_c(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value)

#define cs_down  HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
#define cs_up    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);

/* ======================== PI 控制器 ======================== */
typedef struct
{
    volatile float kp;
    volatile float ki;
    volatile float ts;          /* 采样周期 */

    volatile float p_term;
    volatile float i_term;
    volatile float i_term_max;
    volatile float i_term_min;

    volatile float out_value;
} pid_para_t;

/* ======================== PWM 调制模式 ======================== */
typedef enum
{
    pwm_svpwm    = 0,
    pwm_dpwm_min = 1,
    pwm_dpwm_max = 2,
    pwm_dpwm_60  = 3,
} pwm_mod_e;

/* ======================== FOC 运算结构体 ======================== */
typedef struct
{
    /* 调制模式 */
    pwm_mod_e pwm_mode;

    /* 母线电压 */
    float vs;           /* 可用电压幅值 = vbus*0.96/根号3 */
    float vbus;     /* 母线电压 */
    float inv_vbus;     /* 根号3 / vbus */

    /* 电角度 / 角速度 */
    float p_e;          /* 电角度 (SMO输出) */
    float we;           /* 电角速度 rad/s */
    float wr;           /* 机械角速度 rad/s */
    float theta;        /* 当前控制用电角度 */
    float sin_val;
    float cos_val;

    /* 三相电流 (重构后) */
    float i_a;
    float i_b;
    float i_c;

    /* αβ 电流 */
    float i_alph;
    float i_beta;

    /* dq 电流 */
    float i_d;
    float i_q;

    /* dq 电压 */
    float v_d;
    float v_q;

    /* αβ 电压 */
    float v_alph;
    float v_beta;

    /* 三相占空比 [0, 1] */
    float dtc_a;
    float dtc_b;
    float dtc_c;

    /* SVPWM 扇区 (1~6) */
    uint8_t svm_sector;

    /* ---- 单电阻重构相关 ---- */
    float   shunt_window_min;   /* 最小有效采样窗口阈值 */
    float   shunt_lpf_alpha;    /* 重构电流LPF系数 */

    /* ---- 有效矢量时间 (归一化 0~1) ---- */
    float   t1_eff;             /* 第一采样窗口 = CCR_max - CCR_mid */
    float   t2_eff;             /* 第二采样窗口 = CCR_mid - CCR_min */

    /* 注入ADC双采样原始值 (ADC ISR → TIM1 ISR 传递) */
    float   i_shunt_raw_1;// ADC采样1的原始值 (A)
    float   i_shunt_raw_2;// ADC采样2的原始值 (A)

    float  i_shunt_1;             /* 采样1的电流值 (A) */
    float  i_shunt_2;             /* 采样2的电流值 (A) */
    uint8_t shunt_pair_ready;   /* 1=两次采样都就绪, ADC ISR置位/TIM1 ISR清零 */

    /* ---- 规则平移PWM参数 ---- */
    uint16_t arr1_up;      /* CH1 上半周期 CCR (伏秒补偿) */
    uint16_t arr2_up;
    uint16_t arr3_up;
    uint16_t arr1_down;    /* CH1 下半周期 CCR */
    uint16_t arr2_down;
    uint16_t arr3_down;

    /* CH4 / CH6 ADC触发点 */
    uint16_t sample_trig1;
    uint16_t sample_trig2;

    /* 调试标志 */
    uint8_t  run_stage;    /* 运行阶段: 0=对齐 1=IF爬速 2=SMO闭环 */
    uint8_t  shunt_case;   /* 移相情况: 0=无移相 1=窗口1不足S1 2=窗口1S2 3=窗口1S3
                                       11=窗口2不足S1 12=窗口2S2 13=窗口2S3
                                       21=低调制 */
} pmsm_foc_t;

/* ======================== 状态枚举 ======================== */
typedef enum
{
    reset = 0,
    start = 1,
    opera = 2,
} pm_ctrl_bit_e;

/* ======================== 电机参数 ======================== */
typedef struct
{
    float Rs;       /* 定子电阻 Ω */
    float Ld;       /* d轴电感 H */
    float Lq;       /* q轴电感 H */
    float Ls;       /* 平均电感 H */
    float Ldif;     /* Lq - Ld */
    float flux;     /* 转子磁链 Wb */
    float B;        /* 粘滞摩擦系数 */
    float Js;       /* 转动惯量 kg·m² */
    float pn;       /* 极对数 */

    float ibw;      /* 电流环带宽 rad/s */
    float delta;

    float Gr;       /* 减速比 */
    float Gref;     /* 减速效率 */
    float Kt;       /* 转矩常数 = 1.5 * pn * flux */
    float div_Kt;   /* 1 / Kt */
    float div_pn;   /* 1 / pn */
    float pnd_2pi;  /* pn / (2π) */
    float div_Gr;   /* 1 / Gr */

    float e_off;    /* 编码器电角度偏移 */
    float r_off;    /* 编码器机械角度偏移 */
    float m_off;    /* 机械零位偏移 */
} pmsm_para_t;

/* ======================== 控制设定 ======================== */
typedef struct
{
    float drag_pe;  /* 拖动电角度 */
    float pos_acc;  /* 角度增量 / 周期 */
    float we_set;   /* 电角速度设定 */

    float vd_set;   /* d轴电压设定 (V/f模式) */
    float vq_set;   /* q轴电压设定 (V/f模式) */

    float id_set;   /* d轴电流参考 A */
    float iq_set;   /* q轴电流参考 A */
    float iq_lim;   /* q轴电流限幅 A */
    float id_if;    /* IF启动时的d轴电流参考 A */
    float iq_if;    /* IF爬速时的q轴拖动电流参考 A */

    float wr_set;   /* 机械角速度参考 rad/s */
    float wm_acc;   /* 加速度 rad/s² */
    float wm_dec;   /* 减速度 rad/s² */
} pmsm_ctrl_t;

/* ======================== 时基参数 ======================== */
typedef struct
{
    uint32_t start_cnt;     /* 启动计数器 */

    float foc_ts;           /* FOC 采样周期 s */
    float foc_fs;           /* FOC 采样频率 Hz */

    float cur_pid_ts;       /* 电流环周期 s */
    float cur_pid_fs;       /* 电流环频率 Hz */

    float spd_pid_ts;       /* 速度环周期 s (1/1000) */
    float spd_pid_fs;       /* 速度环频率 Hz (1000) */
} period_t;

/* ======================== ADC 原始值 ======================== */
typedef struct
{
    uint16_t ia;
    uint16_t ib;
    uint16_t ic;

    uint16_t vbus;
    uint16_t va;
    uint16_t vb;
    uint16_t vc;

    float ia_off;   /* A相偏置 (ADC counts) */
    float ib_off;   /* B相偏置 */
    float ic_off;   /* C相偏置 */
} pmsm_adc_val_t;

/* ======================== 板级参数 ======================== */
typedef struct
{
    float v_ref;    /* ADC 参考电压 3.3V */
    float v_adc;    /* ADC 满量程 4096 */

    float i_res;    /* 采样电阻 Ω */
    float i_op;     /* 运放增益倍数 */
    float i_ratio;  /* 电流换算系数 A/LSB */
    float i_max;    /* 可测最大电流 A */

    float v1_res;   /* 母线分压上桥电阻 Ω */
    float v2_res;   /* 母线分压下桥电阻 Ω */
    float v_op;     /* 电压分压比 */
    float v_ratio;  /* 电压换算系数 V/LSB */
    float v_max;    /* 可测最大电压 V */

    float dead_time; /* 死区时间 µs */
} pmsm_board_t;

/* ======================== SMO 滑模观测器 ======================== */
typedef struct
{
    /* 指针: 绑定到 pmsm_foc_t 对应字段 */
    float *Rs;
    float *Ls;
    float *Ld;
    float *Lq;
    float *flux;
    float *i_alph;
    float *i_beta;
    float *v_alph;
    float *v_beta;
    float *we;

    /* 内部状态 */
    float EstIalph;     /* 估算α轴电流 */
    float EstIbeta;     /* 估算β轴电流 */
    float Ealph;        /* α轴反电动势估算 */
    float Ebeta;        /* β轴反电动势估算 */
    float Zalph;        /* α轴滑模控制量 */
    float Zbeta;        /* β轴滑模控制量 */

    /* 参数 */
    float Kslide;       /* 滑模增益 */
    float Kslf;         /* 低通滤波增益 */
    float E0;           /* 饱和函数边界 */

    float m_theta;      /* 上一拍原始atan2角度 (用于PLL初始化) */
    float we_lpf_alpha; /* 速度估算低通滤波系数 (留作备用) */

    /* PLL 锁相环 ─────────────────────────────────────────────
     * 原理: err = Eα_c·cos(θ̂) + Eβ_c·sin(θ̂) = Emag·sin(θ̂ - θe)
     *   θ̂≈θe 时 err≈0；PI调节器驱动 ωe_pll → 积分 → θ̂
     * Eα_c/Eβ_c: 对LPF输出做超前旋转 atan(ωe/Kslf) 补偿相位滞后
     * 增益设计: PLL带宽 ωn_pll，阻尼 ζ=0.707
     *   kp = 2·ζ·ωn_pll   (已对误差做幅值归一化, 单位 rad/s/1)
     *   ki = ωn_pll²      (单位 rad/s²/1) */
    float pll_kp;       /* PLL 比例增益 (rad/s / normalized_err) */
    float pll_ki;       /* PLL 积分增益 (rad/s² / normalized_err) */
    float pll_err_int;  /* PLL 积分项 (rad/s) */
    float we_pll;       /* PLL 输出电角速度 rad/s */

    /* 输出 */
    float pos_e;        /* 估算电角度 rad (PLL输出) */
    float we_est;       /* 估算电角速度 rad/s (= we_pll) */

    float ts;           /* 采样周期 */
    float fs;           /* 采样频率 */
} esmo_t;

/* ======================== PMSM 总控制结构体 ======================== */
typedef struct
{
    pm_ctrl_bit_e  ctrl_bit;    /* 状态机: reset / start / opera */

    pmsm_board_t   board;       /* 板级参数 */
    pmsm_adc_val_t adc;         /* ADC 原始值 */
    pmsm_para_t    para;        /* 电机参数 */
    pmsm_ctrl_t    ctrl;        /* 控制设定 */
    pmsm_foc_t     foc;         /* FOC 运算变量 */
    period_t       period;      /* 时基参数 */
    esmo_t         esmo;        /* SMO 观测器 */

    pid_para_t     id_pi;       /* d轴电流PI */
    pid_para_t     iq_pi;       /* q轴电流PI */
    pid_para_t     spd_pi;      /* 速度PI */
} pmsm_t;

/* ======================== 全局实例 ======================== */
extern pmsm_t pm;

/* ======================== 函数声明 ======================== */

/* --- foc_calc.c: 纯数学算法 --- */
void sin_cos_val(pmsm_foc_t *foc);
void clarke_transform(pmsm_foc_t *foc);
void park_transform(pmsm_foc_t *foc);
void inverse_park(pmsm_foc_t *foc);

/* --- foc_drv.c: 初始化 --- */
void pmsm_init(void);
void pmsm_power_on_autostart(pmsm_t *pm);

/* --- foc_drv.c: 初始化 / ADC / PWM --- */
void foc_get_curr_off(void);
void foc_single_shunt_init(pmsm_t *pm);
void foc_single_shunt_schedule_update(pmsm_t *pm);
void foc_pwm_start(void);
void foc_pwm_stop(void);

/* --- foc_drv.c: 观测器 --- */
void foc_smo_init(pmsm_t *pm);
void foc_smo_run(pmsm_t *pm);

/* --- foc_drv.c: ISR + 速度环 --- */
void foc_spd_pi_calc(pmsm_t *pm);
void foc_tim1_update_isr(pmsm_t *pm);

#endif /* __COMMON_H__ */
