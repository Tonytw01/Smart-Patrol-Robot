#include "comms.h"
#include <esp_wifi.h>



//  遙測回傳結構 (加入時間戳記)

typedef struct telemetry_data {
  unsigned long timestamp; //  時間戳記 (毫秒)
  float target_angle;
  float current_angle;
} telemetry_data;

telemetry_data testData;


extern float yaw_angle ;
extern float target_yaw ;

//  已更新：手套端的真實 MAC 位址
uint8_t glove_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t glovePeerInfo;

//  宣告記錄最後接收時間的變數 (Watchdog 計時用)
volatile unsigned long last_packet_time = 0;


//  ESP-NOW 接收回呼函式 (手套端)

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  // 檢查收到的封包是否為手套傳來的控制指令 (16 bytes)
  if (len == sizeof(myMessage)) {
    memcpy(&myMessage, incomingData, sizeof(myMessage));

    //  成功收到手套封包，立刻刷新時間戳記
    last_packet_time = millis();

    
    //  1. 處理模式切換訊號 (由手套端按鈕發送的脈衝)
   
    // 如果收到 1，代表手套端剛剛按下了按鈕
    if (myMessage.mode_switch == 1) {

      if (currentMode == MODE_GLOVE) {
        Serial.println(" 手套遠端指令：切換至 [智慧巡邏模式]");
        currentMode = MODE_PI;
      } else {
        Serial.println(" 手套遠端指令：切換至 [手套遙控模式]");
        currentMode = MODE_GLOVE;
      }

      // 換手瞬間強制煞車策安全
      stopRobot();
      target_throttle = 0;
      target_steer = 0;
    }

    
    //  2. 動力數據寫入 (僅在手套模式下生效)
  
    if (currentMode == MODE_GLOVE) {
      target_throttle = myMessage.speed;
      target_steer = myMessage.turn;
      dynamic_speed = myMessage.flex_pwm;
    }
  }
}


//  通訊硬體與網路初始化

void comms_init() {
  // 1. 初始化與樹莓派的 UART 連線
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(20);

  // 2. 初始化 ESP-NOW (手套連線)
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println(" ESP-NOW 初始化失敗！");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  //  3. 將手套端加入為「發送對象」，打通雙向通訊
  memcpy(glovePeerInfo.peer_addr, glove_mac, 6);
  glovePeerInfo.channel = 0;
  glovePeerInfo.encrypt = false;
  if (esp_now_add_peer(&glovePeerInfo) != ESP_OK) {
    Serial.println(" 無法新增手套端為發送對象");
  }

  //  系統啟動時，先初始化一次時間戳記
  last_packet_time = millis();

  Serial.println(" 通訊模組初始化完成 (UART & ESP-NOW 雙向)");
}




//  FreeRTOS 專屬任務：大腦通訊與決策

void Task_Comms(void *pvParameters) {
  int report_timer = 0;
  int telemetry_timer = 0; //  新增：遙測回傳計時器

  while (1) {
    unsigned long current_time = millis();

    //  3. Watchdog 核心邏輯：檢查手套是否失聯超過 500 毫秒
    if (currentMode == MODE_GLOVE && (current_time - last_packet_time > 500)) {
      // 只要發現動力不是 0，就立刻強制歸零
      if (target_throttle != 0 || target_steer != 0) {
        target_throttle = 0;
        target_steer = 0;
        dynamic_speed = 0;
        Serial.println(" [通訊警告] 遙控訊號遺失超過 0.5 秒，啟動強制煞車");
      }
    }

    // --- 1. 監聽樹莓派 UART 指令 ---
    if (Serial2.available()) {
      String piCommand = Serial2.readStringUntil('\n');
      piCommand.trim();

      if (piCommand.length() > 0) {
        Serial.print(" [大腦接收] UART 訊號: ");
        Serial.println(piCommand);

        if (piCommand == "SWITCH_MODE") {
          stopRobot();
          if (currentMode == MODE_GLOVE) {
            currentMode = MODE_PI;
            Serial.println(" 控制權切換：[智慧巡邏模式] (Pi 大腦掌權)");
          } else {
            currentMode = MODE_GLOVE;
            Serial.println(" 控制權切換：[手套遙控模式] (手套掌權)");
            target_throttle = 0; target_steer = 0; dynamic_speed = 0;
          }
        }
        else if (currentMode == MODE_PI) {
          if (bypass_step > 0) {
            // 正在避障中，忽略大腦的一般前進後退指令
          }
          else if (piCommand == "Forward") {
            target_throttle = 50; target_steer = 0; dynamic_speed = 140;
          }
          else if (piCommand == "Turn_Left_Slightly") {
            target_throttle = 0; target_steer = -60; dynamic_speed = 200;
          }
          else if (piCommand == "Turn_Right_Slightly") {
            target_throttle = 0; target_steer = 60; dynamic_speed = 200;
          }
          else if (piCommand == "Stop") {
            target_throttle = 0; target_steer = 0; dynamic_speed = 0;
          }
        }
      }
    }

    // --- 2. 狀態數據回報給樹莓派 (每 300ms 觸發) ---
    report_timer += 10;
    if (report_timer >= 300) {
      Serial2.print("DIST:");
      Serial2.print(filtered_distance);
      Serial2.print(":");
      Serial2.print((currentMode == MODE_PI) ? "PI" : "GLOVE");
      Serial2.print(":");
      Serial2.println(bypass_step);

      report_timer = 0;
    }

    // ---  3. 遙測數據回傳給手套 (每 40ms 觸發一次，25Hz) ---
    telemetry_timer += 10;
    if (telemetry_timer >= 40) {

      testData.timestamp = millis();
      testData.target_angle = target_yaw;  
      testData.current_angle = yaw_angle;

      esp_now_send(glove_mac, (uint8_t *) &testData, sizeof(testData));

      telemetry_timer = 0;
    }

    //  讓出 CPU 資源
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
