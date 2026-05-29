#include "phase_lock_v2.h"
#include <math.h>

/* ======================== AD9833 底层 (晶振 20MHz) ======================== */
#define FCLK_HZ      20000000UL
#define FTW_SCALE    268435456.0 /* 2^28 */

/* SPI 句柄 (与 phase_lock_driver 共享) */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;

/* ADS8688 实例 (与 phase_lock_driver 共享) */
extern ADS8688 ads8688;
extern uint16_t adc_data[8];

/* TIM7 标志位 (与 phase_lock_driver 共享) */
extern volatile uint8_t g_phase_lock_tick_flag;

/* ======================== 静态变量 ======================== */
static PhaseLockV2_Ch_t g_ch[2];
static uint8_t g_enabled = 0;

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 将目标频率转为 28-bit AD9833 频率字 (FCLK=20MHz)
 */
static uint32_t freq_to_word(float freq_hz)
{
    if (freq_hz < 0.0f)
        freq_hz = 0.0f;
    if (freq_hz > (float)FCLK_HZ / 2.0f)
        freq_hz = (float)FCLK_HZ / 2.0f;
    return (uint32_t)((double)freq_hz * FTW_SCALE / (double)FCLK_HZ + 0.5);
}

/**
 * @brief 直接写入 28-bit 频率字到 AD9833 FREQ0 寄存器
 *        AD9833 在 B28 模式下连续写 FREQ0 低14位+高14位即可完成更新
 */
static void write_freq_word(SPI_HandleTypeDef hspi, uint32_t freq_word, unsigned short type)
{
    uint16_t cmd_b28  = (uint16_t)(AD9833_B28 | type);
    uint16_t freq_lo  = (uint16_t)(AD9833_REG_FREQ0 | (freq_word & 0x3FFF));
    uint16_t freq_hi  = (uint16_t)(AD9833_REG_FREQ0 | ((freq_word >> 14) & 0x3FFF));

    HAL_SPI_Transmit(&hspi, (uint8_t *)&cmd_b28, 1, 100);
    HAL_SPI_Transmit(&hspi, (uint8_t *)&freq_lo,  1, 100);
    HAL_SPI_Transmit(&hspi, (uint8_t *)&freq_hi,  1, 100);
}

/**
 * @brief 写入相位到 AD9833 PHASE0 寄存器 (12-bit, 0~4095)
 */
static void write_phase_word(SPI_HandleTypeDef hspi, float phase_deg)
{
    if (phase_deg >= 360.0f || phase_deg < 0.0f)
    {
        while (phase_deg >= 360.0f)
            phase_deg -= 360.0f;
        while (phase_deg < 0.0f)
            phase_deg += 360.0f;
    }
    uint16_t pw  = (uint16_t)(phase_deg * (4096.0f / 360.0f) + 0.5f) & 0x0FFF;
    uint16_t val = (uint16_t)(AD9833_REG_PHASE0 | pw);
    HAL_SPI_Transmit(&hspi, (uint8_t *)&val, 1, 100);
}

/**
 * @brief ADS8688 原始值 (0~65535, 0~5.12V) → 双极性 mV (±2560mV, 中心=0V)
 */
static float raw_to_mv_bipolar(uint16_t raw)
{
    float v = (float)raw * (5120.0f / 65535.0f); /* 0~5.12V */
    return (v - 2560.0f);                         /* 偏移 2.56V → ±2560mV */
}

/**
 * @brief 单通道锁相更新
 * @param ch_idx    : 0=A通道(hspi1), 1=B通道(hspi4)
 * @param voltage_mv: ADS8688 当前电压 (mV, 双极性 ±2560mV)
 */
static void update_channel(uint8_t ch_idx, float voltage_mv)
{
    PhaseLockV2_Ch_t *ch = &g_ch[ch_idx];
    SPI_HandleTypeDef   hspi = (ch_idx == 0) ? hspi1 : hspi4;

    /* 频率范围保护 */
    if (ch->base_freq_hz < PL_V2_MIN_FREQ_HZ)
        ch->base_freq_hz = PL_V2_MIN_FREQ_HZ;
    if (ch->base_freq_hz > PL_V2_MAX_FREQ_HZ)
        ch->base_freq_hz = PL_V2_MAX_FREQ_HZ;

    switch (ch->state)
    {

    /* ============ 状态 1：相位搜索 ============ */
    case PL_V2_STATE_SEARCHING: {
        /* 降采样: 匹配 100Hz LPF 响应速度 */
        if (ch->search_decimate_cnt > 0)
        {
            ch->search_decimate_cnt--;
            ch->last_voltage_mv = voltage_mv;
            break;
        }
        ch->search_decimate_cnt = PL_V2_SEARCH_DECIMATE;

        /* 过零点检测 */
        if (ch->zero_cross_det)
        {
            float dv = voltage_mv - ch->last_voltage_mv;

            /* 下降沿 + 零点窗口内 → 锁定! */
            if ((dv < 0.0f) && (voltage_mv < PL_V2_ZERO_CROSS_WINDOW_MV) &&
                (voltage_mv > -PL_V2_ZERO_CROSS_WINDOW_MV))
            {
                ch->target_voltage_mv = voltage_mv;
                ch->state             = PL_V2_STATE_LOCKED;
                ch->zero_cross_det    = 0;
                ch->delay_counter     = 2 * PL_V2_STEP_BEISHU;
                ch->cnt_up            = 0;
                ch->cnt_down          = 0;
                break;
            }
        }
        else if (voltage_mv > 0.0f)
        {
            ch->zero_cross_det = 1; /* 预触发 */
        }

        /* 步进相位继续搜索 */
        ch->current_phase_deg += PL_V2_PHASE_SEARCH_STEP_DEG;
        if (ch->current_phase_deg >= 360.0f)
            ch->current_phase_deg -= 360.0f;
        write_phase_word(hspi, ch->current_phase_deg);

        ch->last_voltage_mv = voltage_mv;
        break;
    }

    /* ============ 状态 2：已锁定，频率微调维持 ============ */
    case PL_V2_STATE_LOCKED: {
        float d_vot = voltage_mv - ch->target_voltage_mv;

        /* 慢速跟踪目标, 补偿直流漂移 */
        ch->target_voltage_mv += d_vot * 0.01f;

        /* 电压偏高 → 减小 delay_counter → 等效降低频率 */
        if (d_vot > PL_V2_VOLTAGE_THRESHOLD_MV)
        {
            ch->cnt_down = 0;
            ch->delay_counter--;
            if (ch->delay_counter < 0)
                ch->delay_counter = 0;

            ch->cnt_up++;
            if (ch->cnt_up >= PL_V2_CNT_CONSECUTIVE)
            {
                ch->freq_word_base++;
                ch->freq_word_alt++;
                ch->cnt_up        = 0;
                ch->delay_counter = PL_V2_STEP_BEISHU * 2;
            }
        }
        else
        {
            ch->cnt_up = 0;
        }

        /* 电压偏低 → 增大 delay_counter → 等效提高频率 */
        if (d_vot < -PL_V2_VOLTAGE_THRESHOLD_MV)
        {
            ch->cnt_up = 0;
            ch->delay_counter++;
            if (ch->delay_counter > PL_V2_STEP_BEISHU * 2)
                ch->delay_counter = PL_V2_STEP_BEISHU * 2;

            ch->cnt_down++;
            if (ch->cnt_down >= PL_V2_CNT_CONSECUTIVE)
            {
                if (ch->freq_word_base > 0)
                {
                    ch->freq_word_base--;
                    ch->freq_word_alt--;
                }
                ch->cnt_down      = 0;
                ch->delay_counter = 0;
            }
        }
        else
        {
            ch->cnt_down = 0;
        }

        /* 频率字上下限钳位 */
        {
            uint32_t fw_min = freq_to_word(PL_V2_MIN_FREQ_HZ);
            uint32_t fw_max = freq_to_word(PL_V2_MAX_FREQ_HZ);
            if (ch->freq_word_base < fw_min)
            {
                ch->freq_word_base = fw_min;
                ch->freq_word_alt  = fw_min + 1;
            }
            if (ch->freq_word_alt > fw_max)
            {
                ch->freq_word_alt  = fw_max;
                ch->freq_word_base = fw_max - 1;
            }
        }

        break;
    }

    default:
        break;
    }

    /* ============ 分数频率合成时序 (所有状态都执行) ============ */
    ch->call_cnt++;
    if (ch->call_cnt >= (uint16_t)(PL_V2_STEP_BEISHU * 4))
    {
        ch->call_cnt = 0;

        if (ch->toggle_req)
        {
            write_freq_word(hspi, ch->freq_word_alt, ch->wave_type);
            ch->toggle_req = 0;
        }

        if (ch->delay_counter > 0)
        {
            ch->delay_counter--;
        }
        else
        {
            ch->toggle_req = 1;
        }

        /* 死锁保护 */
        if (ch->delay_counter == 0 && !ch->toggle_req)
        {
            if (ch->cnt_up == 0 && ch->cnt_down == 0)
                ch->delay_counter = 2 * PL_V2_STEP_BEISHU;
        }

        /* 输出 base 频率字 */
        write_freq_word(hspi, ch->freq_word_base, ch->wave_type);
    }
}

/* ======================== 公开接口实现 ======================== */

void phase_lock_v2_init(void)
{
    g_ch[0].state = PL_V2_STATE_IDLE;
    g_ch[1].state = PL_V2_STATE_IDLE;
    g_enabled     = 0;
}

void phase_lock_v2_poll(void)
{
    if (!g_phase_lock_tick_flag)
        return;
    g_phase_lock_tick_flag = 0;

    if (!g_enabled)
        return;

    /* 一次性读取 ADS8688 所有通道 */
    if (ADS_Read_All_Raw(&ads8688, adc_data) != HAL_OK)
        return;

    float mv_a = raw_to_mv_bipolar(adc_data[0]);
    float mv_b = raw_to_mv_bipolar(adc_data[1]);

    update_channel(0, mv_a);
    update_channel(1, mv_b);
}

void phase_lock_v2_enable(float base_freq_a, unsigned short type_a,
                          float base_freq_b, unsigned short type_b)
{
    PhaseLockV2_Ch_t *chA = &g_ch[0];
    PhaseLockV2_Ch_t *chB = &g_ch[1];

    /* 通道 A */
    chA->state               = PL_V2_STATE_SEARCHING;
    chA->base_freq_hz        = base_freq_a;
    chA->wave_type           = type_a;
    chA->current_phase_deg   = 0.0f;
    chA->zero_cross_det      = 0;
    chA->search_decimate_cnt = PL_V2_SEARCH_DECIMATE;
    chA->last_voltage_mv     = 0.0f;
    chA->target_voltage_mv   = 0.0f;
    chA->freq_word_base      = freq_to_word(base_freq_a);
    chA->freq_word_alt       = chA->freq_word_base + 1;
    chA->delay_counter       = 2 * PL_V2_STEP_BEISHU;
    chA->toggle_req          = 0;
    chA->cnt_up              = 0;
    chA->cnt_down            = 0;
    chA->call_cnt            = 0;

    /* 通道 B */
    chB->state               = PL_V2_STATE_SEARCHING;
    chB->base_freq_hz        = base_freq_b;
    chB->wave_type           = type_b;
    chB->current_phase_deg   = 0.0f;
    chB->zero_cross_det      = 0;
    chB->search_decimate_cnt = PL_V2_SEARCH_DECIMATE;
    chB->last_voltage_mv     = 0.0f;
    chB->target_voltage_mv   = 0.0f;
    chB->freq_word_base      = freq_to_word(base_freq_b);
    chB->freq_word_alt       = chB->freq_word_base + 1;
    chB->delay_counter       = 2 * PL_V2_STEP_BEISHU;
    chB->toggle_req          = 0;
    chB->cnt_up              = 0;
    chB->cnt_down            = 0;
    chB->call_cnt            = 0;

    g_enabled = 1;
}

void phase_lock_v2_disable(void)
{
    g_enabled        = 0;
    g_ch[0].state    = PL_V2_STATE_IDLE;
    g_ch[1].state    = PL_V2_STATE_IDLE;
}

uint8_t phase_lock_v2_is_enabled(void)
{
    return g_enabled;
}
