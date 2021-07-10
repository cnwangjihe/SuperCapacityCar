#include "sensor.h"

uint8_t Find_situation(void)
{
    uint8_t L_1, L_2, L_3, L_4;
    L_1 = HAL_GPIO_ReadPin(L1_GPIO_Port, L1_Pin);
    L_2 = HAL_GPIO_ReadPin(L2_GPIO_Port, L2_Pin);
    L_3 = HAL_GPIO_ReadPin(L3_GPIO_Port, L3_Pin);
    L_4 = HAL_GPIO_ReadPin(L4_GPIO_Port, L4_Pin);
    if (L_4 && !L_1)
        return TURN_LEFT;
    else if (!L_4 && L_1)
        return TURN_RIGHT;
    else if (L_1 && L_2 && L_3 && L_4)
        return CIRCLE;
    else if (!L_1 && !L_4)
        return POSITION_OK;
    else
        return GO_STRAIGHT;
}