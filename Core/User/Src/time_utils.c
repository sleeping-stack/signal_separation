#include "time_utils.h"

uint32_t time_now_ms(void)
{
    return HAL_GetTick();
}
