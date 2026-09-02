#ifndef RADAR_H
#define RADAR_H

#include "config.h"
#include <ESP32Servo.h>

void radar_init();
void Task_Radar(void *pvParameters);

#endif
