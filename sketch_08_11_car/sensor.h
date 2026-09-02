#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// 宣告外部可以呼叫的初始化與任務函式
void sensor_init();
void Task_Sensor(void *pvParameters);

#endif
