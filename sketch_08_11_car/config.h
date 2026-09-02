#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// 宣告一把全域的電力互斥鎖
extern SemaphoreHandle_t PowerMutex;

//  雙模式狀態機定義與安全 UART 腳位

enum ControlMode {//列舉
  MODE_GLOVE,  
  MODE_PI      
};
extern ControlMode currentMode; //宣告變數

#define RXD2 33 
#define TXD2 14 

// --- 1. 通訊結構與變數 ---
typedef struct message {
  int speed;
  int turn;
  int flex_pwm;
  int mode_switch;//模式切換
} message;

extern message myMessage;

extern volatile int target_throttle;
extern volatile int target_steer;
extern volatile int dynamic_speed; 

extern volatile int current_left_pwm;
extern volatile int current_right_pwm;

// --- 2. 腳位定義 ---
const int ENA = 26;  const int ENB = 27;
const int IN1 = 16;  const int IN2 = 17;
const int IN3 = 18;  const int IN4 = 19;
const int TRIG_PIN = 5;
const int ECHO_PIN = 32;

// --- 3. 控制參數設定 ---
const int freq = 5000;
const int resolution = 8;
const int channelA = 0;
const int channelB = 1;
const int TURN_SPEED = 185;

const int SAFE_DISTANCE_GLOVE = 7;
const int SAFE_DISTANCE_PI = 30;

extern float yaw_angle;
extern float gyro_z_offset;
extern float integral; 
extern float prev_error;

// 方形避障專用的狀態機變數
extern int bypass_step;
extern unsigned long bypass_timer;

// --- 4. 超音波非阻塞中斷變數 ---
extern volatile unsigned long echo_start_time;
extern volatile int raw_distance;
extern unsigned long last_trigger_time;


//  卡爾曼濾波器參數

extern float kf_est;
extern float kf_P;
extern float kf_Q;
extern float kf_R;
extern int filtered_distance;

// FreeRTOS 任務控制代碼 (Task Handles)
extern TaskHandle_t TaskSensor_Handle;
extern TaskHandle_t TaskComms_Handle;
extern TaskHandle_t TaskMotor_Handle;
extern bool is_radar_scanning;
extern TaskHandle_t TaskRader_Handle;

// 雷達掃描結果記憶區
extern volatile int best_scan_angle; // 紀錄最空曠的雷達角度 (0~180)
extern volatile int max_scan_dist;   // 紀錄該角度的距離


#endif
