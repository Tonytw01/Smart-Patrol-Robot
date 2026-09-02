#include "config.h"
#include "sensor.h"
#include "motor.h"
#include "comms.h"
#include "radar.h"

// 實體化 config.h 中的 extern 變數
ControlMode currentMode = MODE_GLOVE;
message myMessage;
volatile int target_throttle = 0;
volatile int target_steer = 0;
volatile int dynamic_speed = 0; 
volatile int current_left_pwm = 0;
volatile int current_right_pwm = 0;

float yaw_angle = 0.0;
float gyro_z_offset = 0.0;
float integral = 0.0; 
float prev_error = 0.0;
float target_yaw = 0.0;
int bypass_step = 0;
unsigned long bypass_timer = 0;
volatile unsigned long echo_start_time = 0;
volatile int raw_distance = 999;
volatile int best_scan_angle = 90; 
volatile int max_scan_dist = 0;
unsigned long last_trigger_time = 0;

float kf_est = 50.0;
float kf_P = 1.0;
float kf_Q = 1.0;
float kf_R = 30.0;
int filtered_distance = 999;

bool is_radar_scanning = false;

//實體化雷達相關變數
//int scan_dist_right = 0;
//int scan_dist_left = 0;
int turn_direction = 0;

// 宣告 Task Handles
TaskHandle_t TaskRadar_Handle =NULL;
TaskHandle_t TaskSensor_Handle = NULL;
TaskHandle_t TaskComms_Handle = NULL;
TaskHandle_t TaskMotor_Handle = NULL;
SemaphoreHandle_t PowerMutex;

void setup() {
  Serial.begin(115200);
  
  // 建立電力互斥鎖
  PowerMutex = xSemaphoreCreateMutex();
  
  delay(1000); //  給序列埠一點準備時間
  Serial.println("\n\n===============================");
  Serial.println(" 系統啟動中... 優先測試雷達");
  Serial.println("===============================");

  
  
  // (未來會在這裡呼叫 sensor_init(), motor_init(), comms_init() 等函式)
  motor_init();// 1.先讓直流馬達初始化，佔走 Channel 0 和 1
  sensor_init();//3.感測器與通訊最後載入
  comms_init();
  radar_init();// 2.雷達隨後初始化，它會自動分配到乾淨的 Channel 2 進行測試！  
  
  // ---------------------------------------------------------
  //  註冊 FreeRTOS 任務 (目前先指定空殼，後面我們會補上)
  // ---------------------------------------------------------
  // xTaskCreatePinnedToCore(任務函式, "名稱", 堆疊大小, 參數, 優先級, TaskHandle, 綁定核心)
  
  // Task 1: 感測器讀取 (綁在 Core 0)
   xTaskCreatePinnedToCore(Task_Sensor, "SensorTask", 4096, NULL, 1, &TaskSensor_Handle, 0);
  
  // Task 2: 通訊與大腦 (綁在 Core 1)
   xTaskCreatePinnedToCore(Task_Comms, "CommsTask", 4096, NULL, 2, &TaskComms_Handle, 1);
  
  // Task 3: 馬達 PID 控制 (綁在 Core 1，優先級最高！)
  xTaskCreatePinnedToCore(Task_Motor, "MotorTask", 4096, NULL, 3, &TaskMotor_Handle, 1);

  //  Task 4: 雷達伺服馬達控制 (綁在 Core 0，不干擾 Core 1 的馬達 PID)
  xTaskCreatePinnedToCore(Task_Radar, "RadarTask", 2048, NULL, 1, &TaskRadar_Handle, 0);
  
  Serial.println(" FreeRTOS 任務排程完畢");
}

void loop() {
  // FreeRTOS 環境下，原本的 loop() 任務優先級很低
  vTaskDelay(pdMS_TO_TICKS(1000));
}
