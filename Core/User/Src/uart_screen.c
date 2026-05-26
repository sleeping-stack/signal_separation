#include "uart_screen.h"

uint8_t phase_diffrence = 0;
uint8_t g_receive_data_flag = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        g_receive_data_flag = 1; // 设置接收完成标志
    }
    HAL_UART_Receive_IT(&huart1, &phase_diffrence, 1);
}
