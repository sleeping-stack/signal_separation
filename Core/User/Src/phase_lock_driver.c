#include "phase_lock_driver.h"

ADS8688 ads8688;
uint16_t adc_data[NUM_CHANNELS] = {0}; // 存储ADS8688采集的原始数据

/* ======================== 静态全局变量 ======================== */
static PIController_t g_pid_A; /* 锁相放大器 A 的 PID 控制器 */
static PIController_t g_pid_B; /* 锁相放大器 B 的 PID 控制器 */
static uint8_t g_phase_lock_enabled = 0; /* 锁相使能标志 */
volatile uint8_t g_phase_lock_tick_flag = 0; /* SysTick ISR 置 1，主循环轮询清零 */

/* ======================== PID 控制器内部函数 ======================== */

/**
 * @brief  初始化 PID 控制器
 */
static void PIDController_Init(PIController_t *pid, float Kp, float Ki, float Kd, float i_limit, float max_s)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->integral_limit = i_limit;
    pid->max_step = max_s;
    pid->prev_error = 0.0f;
    pid->current_offset_hz = 0.0f;
    pid->base_freq_hz = 0.0f;
    pid->target_voltage = PHASE_LOCK_TARGET_VOLTAGE; /* 默认回退值 0V */
    pid->target_captured = 0; /* 未捕获 */
    pid->settle_counter = 0;
    pid->phase_deg = 0.0f;
    pid->wave_type = AD9833_OUT_SINUS;
}

/**
 * @brief  PID 控制器单次迭代 (位置式，带积分抗饱和)
 * @param  pid : PID 控制器指针
 * @param  sp  : 设定值 (目标电压，V)
 * @param  pv  : 过程变量 (当前测量电压，V)
 * @return 频率修正量 (Hz)
 *
 * 公式: output = Kp·e + Σ(Ki·e) + Kd·(e - e_prev)
 *       其中 Ki 已预乘 dt, Kd 已预除 dt
 */
static float PIDController_Update(PIController_t *pid, float sp, float pv)
{
    float error = sp - pv; /* 误差 = 目标 - 实测 */

    /* ---- 比例项: Kp * e ---- */
    float p_term = pid->Kp * error;

    /* ---- 积分项: Σ(Ki * e), 带限幅防饱和 ---- */
    pid->integral += pid->Ki * error;
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    /* ---- 微分项: Kd * (e - e_prev) ---- */
    float d_term = pid->Kd * (error - pid->prev_error);

    /* ---- PID 输出 ---- */
    float output = p_term + pid->integral + d_term;

    /* ---- 输出步进限幅 ---- */
    if (output > pid->max_step)
    {
        output = pid->max_step;
    }
    else if (output < -pid->max_step)
    {
        output = -pid->max_step;
    }

    pid->prev_error = error;

    return output; /* 单位: Hz */
}

/* ======================== 公开函数 ======================== */

/**
 * @brief  锁相模块初始化
 *         初始化 ADS8688 及两个 PID 控制器
 */
void phase_lock_driver_init(void)
{
    if (ADS8688_Init(&ads8688, &hspi3, ADC_SPI_CS_GPIO_Port, ADC_SPI_CS_Pin) != 0)
    {
        Error_Handler();
    }

    /* 初始化锁相放大器 A 的 PID 控制器 */
    PIDController_Init(&g_pid_A, PHASE_LOCK_KP_A, PHASE_LOCK_KI_A, PHASE_LOCK_KD_A, PHASE_LOCK_INTEGRAL_LIMIT,
                       PHASE_LOCK_MAX_FREQ_STEP);

    /* 初始化锁相放大器 B 的 PID 控制器 */
    PIDController_Init(&g_pid_B, PHASE_LOCK_KP_B, PHASE_LOCK_KI_B, PHASE_LOCK_KD_B, PHASE_LOCK_INTEGRAL_LIMIT,
                       PHASE_LOCK_MAX_FREQ_STEP);
}

/**
 * @brief  读取指定通道的电压值
 * @param  channel: ADS8688 通道号 (0-7)
 * @return 电压值 (V)
 */
float phase_lock_driver_read_adc(uint8_t channel)
{
    if (ADS_Read_All_Raw(&ads8688, adc_data) == HAL_OK)
    {
        return ADS8688_ConvertToVoltage(adc_data[channel], ads8688.channel_range[channel]);
    }
    return 0.0f;
}

/**
 * @brief  锁相闭环控制 — 调节 AD9833 输出频率，维持锁相放大器输出为 0V
 * @note   由 TIM7 以 5kHz 频率调用。
 *         一次性读取 ADS8688 全部通道 → 提取 CH0/CH1 电压 → PID 计算频率修正 → AD9833 更新频率。
 *         当 g_phase_lock_enabled == 0 时直接返回。
 */
void phase_lock_driver_maintain_zero_output(void)
{
    float voltage_a, voltage_b;
    float freq_correction_a, freq_correction_b;
    float target_freq_a, target_freq_b;

    if (!g_phase_lock_enabled)
    {
        return;
    }

    /* 一次性读取 ADS8688 全部 8 通道（避免两次 SPI 全读的冗余通信） */
    if (ADS_Read_All_Raw(&ads8688, adc_data) != HAL_OK)
    {
        return; /* SPI 通信失败，跳过本轮 */
    }
    voltage_a = ADS8688_ConvertToVoltage(adc_data[PHASE_LOCK_ADC_CH_A], ads8688.channel_range[PHASE_LOCK_ADC_CH_A]);
    voltage_b = ADS8688_ConvertToVoltage(adc_data[PHASE_LOCK_ADC_CH_B], ads8688.channel_range[PHASE_LOCK_ADC_CH_B]);

    /* ---- 锁相放大器 A (ADS8688 CH0 → AD9833 hspi1) ---- */

    /* 延迟捕获：等待锁相放大器 LPF 稳定后再捕获目标电压 */
    if (!g_pid_A.target_captured)
    {
        if (g_pid_A.settle_counter > 0)
        {
            g_pid_A.settle_counter--;
        }
        else
        {
            /* 延迟结束，捕获当前电压作为 PID 动态目标 */
            g_pid_A.target_voltage = voltage_a;
            g_pid_A.target_captured = 1;
        }
    }

    /* 仅目标已捕获后才运行 PID */
    if (g_pid_A.target_captured)
    {
        if (fabsf(voltage_a - g_pid_A.target_voltage) > PHASE_LOCK_VOLTAGE_DEADBAND)
        {
            freq_correction_a = PIDController_Update(&g_pid_A, g_pid_A.target_voltage, voltage_a);
            g_pid_A.current_offset_hz -= freq_correction_a;
        }
    }

    /* 频率钳位：不允许超出基频±积分限幅范围 */
    if (g_pid_A.current_offset_hz > PHASE_LOCK_INTEGRAL_LIMIT)
    {
        g_pid_A.current_offset_hz = PHASE_LOCK_INTEGRAL_LIMIT;
    }
    else if (g_pid_A.current_offset_hz < -PHASE_LOCK_INTEGRAL_LIMIT)
    {
        g_pid_A.current_offset_hz = -PHASE_LOCK_INTEGRAL_LIMIT;
    }

    target_freq_a = g_pid_A.base_freq_hz + g_pid_A.current_offset_hz;
    if (target_freq_a < PHASE_LOCK_MIN_FREQ_HZ)
    {
        target_freq_a = PHASE_LOCK_MIN_FREQ_HZ;
        g_pid_A.current_offset_hz = target_freq_a - g_pid_A.base_freq_hz;
    }
    AD9833_SetFrequencyQuick(hspi1, target_freq_a, g_pid_A.wave_type);
    /* 频率更新后同步写入相位，维持 A/B 相位差 */
    AD9833_SetPhaseQuick(hspi1, g_pid_A.phase_deg);

    /* ---- 锁相放大器 B (ADS8688 CH1 → AD9833 hspi4) ---- */

    /* 延迟捕获：等待锁相放大器 LPF 稳定后再捕获目标电压 */
    if (!g_pid_B.target_captured)
    {
        if (g_pid_B.settle_counter > 0)
        {
            g_pid_B.settle_counter--;
        }
        else
        {
            /* 延迟结束，捕获当前电压作为 PID 动态目标 */
            g_pid_B.target_voltage = voltage_b;
            g_pid_B.target_captured = 1;
        }
    }

    /* 仅目标已捕获后才运行 PID */
    if (g_pid_B.target_captured)
    {
        if (fabsf(voltage_b - g_pid_B.target_voltage) > PHASE_LOCK_VOLTAGE_DEADBAND)
        {
            freq_correction_b = PIDController_Update(&g_pid_B, g_pid_B.target_voltage, voltage_b);
            g_pid_B.current_offset_hz -= freq_correction_b;
        }
    }

    /* 频率钳位 */
    if (g_pid_B.current_offset_hz > PHASE_LOCK_INTEGRAL_LIMIT)
    {
        g_pid_B.current_offset_hz = PHASE_LOCK_INTEGRAL_LIMIT;
    }
    else if (g_pid_B.current_offset_hz < -PHASE_LOCK_INTEGRAL_LIMIT)
    {
        g_pid_B.current_offset_hz = -PHASE_LOCK_INTEGRAL_LIMIT;
    }

    target_freq_b = g_pid_B.base_freq_hz + g_pid_B.current_offset_hz;
    if (target_freq_b < PHASE_LOCK_MIN_FREQ_HZ)
    {
        target_freq_b = PHASE_LOCK_MIN_FREQ_HZ;
        g_pid_B.current_offset_hz = target_freq_b - g_pid_B.base_freq_hz;
    }
    AD9833_SetFrequencyQuick(hspi4, target_freq_b, g_pid_B.wave_type);
    /* 频率更新后同步写入相位，维持 A/B 相位差 */
    AD9833_SetPhaseQuick(hspi4, g_pid_B.phase_deg);
}

/**
 * @brief  使能锁相闭环控制
 * @param  base_freq_a : 锁相放大器 A 的基频 (Hz, 由 FFT 分析得到)
 * @param  type_a      : A 通道 AD9833 波形类型
 * @param  phase_a     : A 通道 AD9833 输出相位 (度)
 * @param  base_freq_b : 锁相放大器 B 的基频 (Hz, 由 FFT 分析得到)
 * @param  type_b      : B 通道 AD9833 波形类型
 * @param  phase_b     : B 通道 AD9833 输出相位 (度)
 */
void phase_lock_driver_enable(float base_freq_a, unsigned short type_a, float phase_a, float base_freq_b,
                              unsigned short type_b, float phase_b)
{
    /* 重置 PID 状态 */
    g_pid_A.integral = 0.0f;
    g_pid_A.prev_error = 0.0f;
    g_pid_A.current_offset_hz = 0.0f;
    g_pid_A.base_freq_hz = base_freq_a;
    g_pid_A.target_captured = 0; /* 标记目标电压待重新捕获（补偿锁相放大器直流偏移） */
    g_pid_A.settle_counter = PHASE_LOCK_SETTLE_TICKS; /* 启动延迟计数 */
    g_pid_A.phase_deg = phase_a;
    g_pid_A.wave_type = type_a;

    g_pid_B.integral = 0.0f;
    g_pid_B.prev_error = 0.0f;
    g_pid_B.current_offset_hz = 0.0f;
    g_pid_B.base_freq_hz = base_freq_b;
    g_pid_B.target_captured = 0; /* 标记目标电压待重新捕获 */
    g_pid_B.settle_counter = PHASE_LOCK_SETTLE_TICKS; /* 启动延迟计数 */
    g_pid_B.phase_deg = phase_b;
    g_pid_B.wave_type = type_b;

    g_phase_lock_enabled = 1;
}

/**
 * @brief  禁能锁相闭环控制
 */
void phase_lock_driver_disable(void)
{
    g_phase_lock_enabled = 0;
}

/**
 * @brief  查询锁相闭环是否使能
 * @return 1: 使能中, 0: 已禁能
 */
uint8_t phase_lock_driver_is_enabled(void)
{
    return g_phase_lock_enabled;
}

/**
 * @brief  主循环中轮询调用 — 由 TIM7 ISR 置标志位触发 (5kHz)
 * @note   将 SPI 阻塞操作移出中断上下文，放在主循环空闲时执行。
 */
void phase_lock_driver_poll(void)
{
    if (g_phase_lock_tick_flag)
    {
        g_phase_lock_tick_flag = 0;
        phase_lock_driver_maintain_zero_output();
    }
}
