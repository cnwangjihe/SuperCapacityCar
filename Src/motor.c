#include "main.h"
#include "tim.h"

//前进函数
void moto_straight(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 200);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 200);
}

//后退函数
void moto_retreat(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 200);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 200);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
}

//刹车函数
void moto_stop(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

//前进急行函数
void moto_sprint(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 10);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 10);
}

//后退急行函数
void moto_anti_sprint(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 10);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 10);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
}

//左转弯函数
void moto_left(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 425);
}

//右转弯函数
void moto_right(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 425);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 50);
}

//左急弯函数
void moto_sprint_left(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 100);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 350);
}

//右急弯函数
void moto_sprint_right(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 350);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
}

//顺时针原地旋转
void moto_clockwise(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 200);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 200);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
}

//逆时针原地旋转
void moto_anti_clockwise(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 200);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 200);
}

//掉头函数
void moto_turn_around(void)
{
	moto_anti_sprint();
	HAL_Delay(300);
	moto_stop();
	HAL_Delay(200);
	moto_clockwise();
	HAL_Delay(600);
	moto_stop();
	HAL_Delay(200);
    moto_sprint();
    HAL_Delay(300);
    moto_straight();
}