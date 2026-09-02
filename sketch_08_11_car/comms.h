#ifndef COMMS_H
#define COMMS_H

#include "config.h"
#include "motor.h" // 因為收到切換模式指令時，需要呼叫 stopRobot()
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// 宣告外部可以呼叫的初始化與任務函式
void comms_init();
void Task_Comms(void *pvParameters);

#endif
