#include "sensor.h"

// 建立 MPU6050 物件 (只在這個檔案內使用)
Adafruit_MPU6050 mpu;

// ==========================================
//  超音波中斷服務常式 (ISR)
// ==========================================
void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echo_start_time = micros();
  } else {
    unsigned long echo_end_time = micros();
    unsigned long duration = echo_end_time - echo_start_time;
    int d = duration * 0.034 / 2;
    if (d > 0 && d < 400) {
      raw_distance = d;
    } else {
      raw_distance = 999;
    }
  }
}

//  非阻塞發射超音波
void triggerUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

// ==========================================
//  感測器硬體初始化
// ==========================================
void sensor_init() {
  // 1. 初始化超音波腳位與中斷
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  // 2. 初始化 I2C 與 MPU6050
  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println(" 找不到 MPU6050，系統停機");
    while (1) vTaskDelay(pdMS_TO_TICKS(100)); // FreeRTOS 防卡死迴圈
  }
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // 3. 陀螺儀校準 (將原本的 delay 換成 vTaskDelay)
  Serial.println(" 請保持車體靜止，2秒後開始校準陀螺儀...");
  vTaskDelay(pdMS_TO_TICKS(2000)); 

  float sum = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    vTaskDelay(pdMS_TO_TICKS(4)); 
  }
  gyro_z_offset = sum / 500.0;
  
  Serial.print(" 陀螺儀校準完成，Offset: ");
  Serial.println(gyro_z_offset, 4);
}

// ==========================================
//  FreeRTOS 專屬任務：感測器環境掃描
// ==========================================
void Task_Sensor(void *pvParameters) {
  //  使用 vTaskDelayUntil 確保迴圈絕對精準運行在 50Hz (每 20ms 一次)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); 
  
  int ultrasonic_timer = 0;

  while (1) {
    // --- 1. 讀取 MPU6050 並計算 Yaw 角度 ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float current_gyro_z = g.gyro.z - gyro_z_offset;
    if (abs(current_gyro_z) < 0.01) {
      current_gyro_z = 0;
    }

    // 將角速度積分成角度 (因為任務頻率是精準的 50Hz，dt 直接代入 0.02 秒)
    yaw_angle += (current_gyro_z * 57.2958) * 0.02;

    // --- 2. 控制超音波發射 (每 60ms 發射一次) ---
    ultrasonic_timer += 20;
    if (ultrasonic_timer >= 60) {
      triggerUltrasonic();
      ultrasonic_timer = 0;
    }

    // 讓任務進入精準休眠，直到下一個 20ms 週期到來
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
