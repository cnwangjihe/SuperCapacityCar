#ifndef _MOTOR_H
#define _MOTOR_H

//前进函数
void moto_straight(void);
//后退函数
void moto_retreat(void);
//刹车函数
void moto_stop(void);
//前进急行函数
void moto_sprint(void);
//后退急行函数
void moto_anti_sprint(void);
//左转弯函数
void moto_left(void);
//右转弯函数
void moto_right(void);
//左急弯函数
void moto_sprint_left(void);
//右急弯函数
void moto_sprint_right(void);
//顺时针原地旋转
void moto_clockwise(void);
//逆时针原地旋转
void moto_anti_clockwise(void);
//掉头函数
void moto_turn_around(void);

#endif