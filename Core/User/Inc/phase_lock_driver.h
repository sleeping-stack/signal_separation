#ifndef SIGNAL_SEPARATION_PHASE_LOCK_DRIVER_H
#define SIGNAL_SEPARATION_PHASE_LOCK_DRIVER_H

#include "ADS8688.h"
#include "ad9833.h"
#include "spi.h"
#include <math.h>

/* ======================== PID 控制器参数宏 ======================== */
/* 信号频率范围: 20kHz ~ 100kHz, 步进 5kHz */

/* 锁相放大器 A (ADS8688 CH0 → AD9833 hspi1) */
#define PHASE_LOCK_KP_A 0.1f /* 比例系数 (Hz/V) */
#define PHASE_LOCK_KI_A 0.0001f /* 积分系数 (Hz/V, 已预乘 dt=0.001s, Ki_raw=0.1) */
#define PHASE_LOCK_KD_A 0.01f /* 微分系数 (Hz/V, 已预除 dt=0.001s, Kd_raw=0.00001) */

/* 锁相放大器 B (ADS8688 CH1 → AD9833 hspi4) */
#define PHASE_LOCK_KP_B 0.1f /* 比例系数 (Hz/V) */
#define PHASE_LOCK_KI_B 0.0001f /* 积分系数 (Hz/V, 已预乘 dt=0.001s, Ki_raw=0.1) */
#define PHASE_LOCK_KD_B 0.01f /* 微分系数 (Hz/V, 已预除 dt=0.001s, Kd_raw=0.00001) */

/* 通用限幅 — 锁相放大器 LPF 带宽 ~100Hz，限幅必须在此范围内 */
#define PHASE_LOCK_INTEGRAL_LIMIT 100.0f /* 积分项限幅 (Hz), 缩窄至 ±100Hz 匹配 LPF 带宽 */
#define PHASE_LOCK_MAX_FREQ_STEP 10.0f /* 单次最大频率步进 (Hz) */

/* 锁相目标电压 (V) — 仅作为初始化回退值，实际目标由首次采样动态捕获 */
#define PHASE_LOCK_TARGET_VOLTAGE 0.0f

/* 目标电压捕获延迟 (SysTick 周期数) — 等待锁相放大器 LPF 稳定后再捕获 */
#define PHASE_LOCK_SETTLE_TICKS 10 /* 10ms @ 1kHz SysTick */

/* 频率下限保护 — 防止频率漂移到 0 Hz */
#define PHASE_LOCK_MIN_FREQ_HZ 5000.0f /* 最低允许输出频率 (Hz)，对应最小步进 5kHz */

/* 电压死区 (V) — 锁相放大器输出在此范围内时不进行 PID 修正，防止振荡 */
#define PHASE_LOCK_VOLTAGE_DEADBAND 0.01f

/* ADS8688 通道映射 */
#define PHASE_LOCK_ADC_CH_A 0 /* 锁相放大器A → ADS8688 CH0 */
#define PHASE_LOCK_ADC_CH_B 1 /* 锁相放大器B → ADS8688 CH1 */

/* ======================== PID 控制器结构体 ======================== */
typedef struct
{
    float Kp; /* 比例系数 (Hz/V) */
    float Ki; /* 积分系数 (Hz/V, 已预乘 dt) */
    float Kd; /* 微分系数 (Hz/V, 已预除 dt) */
    float integral; /* 积分累加器 (Hz) */
    float integral_limit; /* 积分限幅 (Hz) */
    float max_step; /* 单次最大频率步进 (Hz) */
    float prev_error; /* 上次误差 (V) */
    float current_offset_hz; /* 当前频率偏移量 (Hz) */
    float base_freq_hz; /* 基频 (Hz), 由FFT分析得到 */
    float target_voltage; /* 动态目标电压 (V)，锁相使能后延迟捕获 */
    uint8_t target_captured; /* 目标电压是否已捕获 (0=未捕获, 1=已捕获) */
    uint16_t settle_counter; /* 目标捕获延迟计数 (SysTick 周期) */
    float phase_deg; /* AD9833 输出相位 (度, 0~360)，锁相每次更新频率后同步写入 */
    unsigned short wave_type; /* AD9833 波形类型 */
} PIController_t;

/* ======================== SPI 句柄外部引用 ======================== */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;

/* SysTick 触发的锁相轮询标志，由 ISR 置 1，主循环检查并清零 */
extern volatile uint8_t g_phase_lock_tick_flag;

/* ======================== 函数声明 ======================== */
void phase_lock_driver_init(void);
float phase_lock_driver_read_adc(uint8_t channel);

/* PID 锁相控制 — 通过调节频率保持锁相放大器输出为 0V */
void phase_lock_driver_maintain_zero_output(void);
void phase_lock_driver_enable(float base_freq_a, unsigned short type_a, float phase_a, float base_freq_b,
                              unsigned short type_b, float phase_b);
void phase_lock_driver_disable(void);
uint8_t phase_lock_driver_is_enabled(void);
void phase_lock_driver_poll(void); /* 主循环轮询：SysTick 置标志 → 主循环执行锁相维护 */

#endif // SIGNAL_SEPARATION_PHASE_LOCK_DRIVER_H
