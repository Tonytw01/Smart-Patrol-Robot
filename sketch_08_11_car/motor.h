#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

// 宣告外部可以呼叫的初始化、任務與控制函式
void motor_init();
void Task_Motor(void *pvParameters);
void stopRobot();

#endif
