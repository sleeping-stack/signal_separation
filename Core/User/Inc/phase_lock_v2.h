#ifndef SIGNAL_SEPARATION_PHASE_LOCK_V2_H
#define SIGNAL_SEPARATION_PHASE_LOCK_V2_H

#include "ADS8688.h"
#include "ad9833.h"
#include "spi.h"
#include <stdint.h>

/* ======================== 可调参数 ======================== */

/* 分数频率合成 — 控制 base/alt 频率字切换周期 */
#define PL_V2_STEP_BEISHU 10

/* 相位搜索步进 (度) */
#define PL_V2_PHASE_SEARCH_STEP_DEG 50.0f

/* 相位搜索降采样: 每 N 次调用步进一次 (5kHz 下 N=50 → 100Hz 步进率，匹配 LPF) */
#define PL_V2_SEARCH_DECIMATE 50

/* 锁定后的电压偏差阈值 (mV, 双边 ±2.56V 量程) */
#define PL_V2_VOLTAGE_THRESHOLD_MV 8.0f

/* 过零点检测窗口 (mV) */
#define PL_V2_ZERO_CROSS_WINDOW_MV 1300.0f

/* 连续检测次数阈值 — 需要连续 N 次超阈值才调整频率字 */
#define PL_V2_CNT_CONSECUTIVE 5

/* 频率下限保护 (Hz) */
#define PL_V2_MIN_FREQ_HZ 5000.0f

/* 频率上限保护 (Hz) */
#define PL_V2_MAX_FREQ_HZ 100000.0f

/* ======================== 通道状态 ======================== */

typedef enum
{
    PL_V2_STATE_IDLE = 0,
    PL_V2_STATE_SEARCHING, /* 相位步进搜索过零点 */
    PL_V2_STATE_LOCKED /* 已锁定，频率微调维持 */
} PhaseLockV2_State_e;

typedef struct
{
    PhaseLockV2_State_e state;

    /* 信号参数 */
    float base_freq_hz; /* 目标基频 (Hz, 由 FFT 传入) */
    float current_phase_deg; /* 当前输出相位 (度) */
    unsigned short wave_type; /* 波形类型 */

    /* 锁定检测 */
    float target_voltage_mv; /* 锁定时的基准电压 (mV) */
    float last_voltage_mv; /* 上一次电压 (mV), 用于斜率检测 */
    uint8_t zero_cross_det; /* 过零预触发标志 */
    uint16_t search_decimate_cnt; /* 搜索步进降采样计数器 */

    /* 分数频率合成 */
    uint32_t freq_word_base; /* 基础频率字 (28-bit) */
    uint32_t freq_word_alt; /* 交替频率字 (base + 1) */
    int delay_counter; /* base/alt 占空比控制 */
    uint8_t toggle_req; /* 请求切换到 alt 频率字 */

    /* 锁定微调 */
    uint8_t cnt_up; /* 连续偏高计数 */
    uint8_t cnt_down; /* 连续偏低计数 */
    uint16_t call_cnt; /* 调用计数 (用于分数合成时序) */

} PhaseLockV2_Ch_t;

/* ======================== 公开接口 ======================== */

/**
 * @brief  初始化锁相模块 V2 (状态初始化，不涉及硬件)
 */
void phase_lock_v2_init(void);

/**
 * @brief  主循环轮询 — 替代 phase_lock_driver_poll()
 * @note   由 TIM7 ISR 置 g_phase_lock_tick_flag 标志位触发 (5kHz)
 *         内部一次性读取 ADS8688 → 更新双通道锁相状态 → 写入 AD9833
 */
void phase_lock_v2_poll(void);

/**
 * @brief  使能锁相闭环控制
 * @param  base_freq_a : 通道 A 基频 (Hz, 由 FFT 分析得到)
 * @param  type_a      : 通道 A 波形类型
 * @param  base_freq_b : 通道 B 基频 (Hz)
 * @param  type_b      : 通道 B 波形类型
 */
void phase_lock_v2_enable(float base_freq_a, unsigned short type_a, float base_freq_b, unsigned short type_b);

/**
 * @brief  禁能锁相闭环控制
 */
void phase_lock_v2_disable(void);

/**
 * @brief  查询锁相是否使能
 * @return 1: 使能中, 0: 已禁能
 */
uint8_t phase_lock_v2_is_enabled(void);

#endif /* SIGNAL_SEPARATION_PHASE_LOCK_V2_H */
