#include "adc_dma_timer.h"

uint8_t g_adc1_dma_complete_flag = 0; // adc1 数据 dma 采集完成标志
uint32_t g_adc_sample_rate = 0; // 当前ADC采样率（Sps）
uint16_t g_adc1_dma_data[ADC_DATA_LENGTH] __attribute__((section(".dma_buffer"))); // adc1 dma 采集数据缓冲区

// ADC 回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        g_adc1_dma_complete_flag = 1; // 采集完成标志
        // 停止采集
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop(&htim6);
    }
}

void adc_timer_init()
{
    /* 校准ADC，采用偏移校准 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
}

// 进行一次adc采集
void adc_start_one_time()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc1_dma_data, ADC_DATA_LENGTH); // ADC 的 dma 开始采集
    HAL_TIM_Base_Start(&htim6); // 启动定时器，开始触发 ADC 采样
}

/**
 * @brief  动态修改 TIM6 的触发频率（即 ADC 采样率）
 * @param  target_fs: 期望的采样率（单位：Sps，例如 100000 表示 100 kSps）
 * @retval int: 0 成功，-1 参数错误或超出频率范围
 */
int32_t set_ADC_Sampling_Rate(uint32_t target_fs)
{
    // 定时器输入时钟线频率为 240 MHz
    const uint32_t tim_src_clk = 240000000U;

    if (target_fs == 0)
        return -1;

    /* * 计算总分频数 = (PSC + 1) * (ARR + 1) = tim_src_clk / target_fs
     * 为了保证定时器触发的抖动最小、定时精度最高：
     * 优先让 PSC = 0（即不分频），全靠 ARR 来调节。
     * 只有当频率太低，单个 ARR (最大 65535) 装不下时，才去增加 PSC。
     */
    uint32_t total_divider = tim_src_clk / target_fs;

    // 如果总分频数为 0（说明输入的 target_fs 超过了 240MHz，不合法）
    if (total_divider == 0)
        return -1;

    uint32_t psc_val = 0;
    uint32_t arr_val = total_divider - 1;

    // 如果总分频数超过了 16 位定时器的最大值 65536
    if (total_divider > 65536)
    {
        // 自动计算 PSC，使得 ARR 能够落在 0 ~ 65535 范围内
        psc_val = total_divider / 65536;
        arr_val = (total_divider / (psc_val + 1)) - 1;

        // 边界检查，防止定时器寄存器溢出
        if (psc_val > 65535)
            return -1;
    }

    __HAL_TIM_SET_PRESCALER(&htim6, psc_val); // 修改 PSC
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr_val); // 修改 ARR

    /* * 触发一次软件更新事件（UG位），让新的 PSC 和 ARR 立即生效。
     * 如果不加这句，修改后的 PSC 需要等到定时器下一次溢出后才会生效。
     */
    htim6.Instance->EGR = TIM_EGR_UG;

    g_adc_sample_rate = target_fs;
    return 0;
}
