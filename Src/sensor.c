#include "sensor.h"

uint8_t Find_situation(void)
{
    uint8_t L_1, L_4;
    L_1 = HAL_GPIO_ReadPin(L1_GPIO_Port, L1_Pin);
    L_4 = HAL_GPIO_ReadPin(L4_GPIO_Port, L4_Pin);
    if (L_1)
    {
        L_1 = 1;
    }
    else
    {
        L_1 = 0;
    }
    if (L_4)
    {
        L_4 = 1;
    }
    else
    {
        L_4 = 0;
    }
    if (L_4 && !L_1)
    {
        return TURN_RIGHT;
    }
    else if (!L_4 && L_1)
    {
        return TURN_LEFT;
    }
    else
    {
        return GO_STRAIGHT;
    }
}
