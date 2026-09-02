#include "radar.h"

Servo radarServo;
const int SERVO_PIN = 25;

void radar_init() {
  // 強制釋放所有計時器，並只允許伺服馬達使用 Timer 3
  ESP32PWM::allocateTimer(3);

  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 700, 2300); // 確保 MG90S 能轉滿 180 度

  Serial.println(" [硬體自我測試] 伺服馬達準備進行開機掃描...");

  radarServo.write(90);
  delay(1000);

  Serial.println(" 測試：緩慢轉向 0 度");
  for (int pos = 90; pos >= 0; pos -= 1) {
    radarServo.write(pos);
    delay(10);
  }
  delay(500);

  Serial.println(" 測試：緩慢轉向 180 度");
  for (int pos = 0; pos <= 180; pos += 1) {
    radarServo.write(pos);
    delay(10);
  }
  delay(500);

  Serial.println(" 測試：緩慢回正 90 度");
  for (int pos = 180; pos >= 90; pos -= 1) {
    radarServo.write(pos);
    delay(10);
  }
  delay(500);

  Serial.println(" 雷達伺服馬達自我測試 (POST) 通過，初始化完成");
}

void Task_Radar(void *pvParameters) {
  while (1) {
    if (is_radar_scanning) {
      if (xSemaphoreTake(PowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {

        Serial.println(" [雷達] 已取得電力鎖，開始動態環景掃描...");
        if (!radarServo.attached()) radarServo.attach(SERVO_PIN, 700, 2300);

        max_scan_dist = 0;
        best_scan_angle = 90;

        for (int pos = 90; pos >= 0; pos -= 5) {
          radarServo.write(pos);
          vTaskDelay(pdMS_TO_TICKS(15));
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        Serial.println("📡 [雷達任務] 執行高解析度 0~180 度掃描與質心運算...");

        //  1. 質心演算法專用變數
        //int max_scan_dist = 0;
        int best_angle_sum = 0;
        int best_angle_count = 0;
        const int SCAN_STEP = 15; // 解析度提升：每 15 度掃描一次 (共 13 個點)
        const int MAX_VALID_DIST = 200; // 距離上限：超過 200cm 就視為絕對安全的「無限遠」

        //  2. 開始高解析度掃描
        for (int angle = 0; angle <= 180; angle += SCAN_STEP) {
          radarServo.write(angle);

          // 等待伺服馬達轉到位 (角度越小，需要的機械穩定時間越短)
          vTaskDelay(pdMS_TO_TICKS(50));

          long dist_sum = 0;
          int valid_count = 0;
          const int SAMPLES_PER_ANGLE = 2; // 為了加快整體掃描速度，單點採樣降為 2 次

          // 取樣
          for (int i = 0; i < SAMPLES_PER_ANGLE; i++) {
            if (raw_distance != 999) {
              dist_sum += (long)(raw_distance * 0.8);
              valid_count++;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
          }

          int current_dist = (valid_count > 0) ? (int)(dist_sum / valid_count) : 999;

          // 如果前方非常空曠，強制收斂到上限值，方便後續的平均計算
          if (current_dist > MAX_VALID_DIST && current_dist < 900) {
            current_dist = MAX_VALID_DIST;
          }

          Serial.print(" 視角 "); Serial.print(angle);
          Serial.print(" 度 | 距離: "); Serial.print(current_dist); Serial.println(" cm");

          //  3. 核心邏輯：尋找最寬廣的連續缺口
          if (current_dist > max_scan_dist && current_dist < 900) {
            // 發現更遠的距離！重置平均池，以這個新距離為霸主
            max_scan_dist = current_dist;
            best_angle_sum = angle;
            best_angle_count = 1;
          }
          else if (current_dist == max_scan_dist || abs(max_scan_dist - current_dist) <= 10) {
            // 距離跟目前最遠的差不多 (容許 10cm 誤差)
            // 代表這是同一個「寬廣缺口」的延伸，將角度加入平均池！
            best_angle_sum += angle;
            best_angle_count++;
          }
        }

        //  4. 計算質心 (目標角度 = 所有最遠角度的平均值)
        if (best_angle_count > 0) {
          best_scan_angle = best_angle_sum / best_angle_count;
        } else {
          best_scan_angle = 90; // 防呆：如果都掃不到，預設看正前方
        }

        for (int pos = 180; pos >= 90; pos -= 5) {
          radarServo.write(pos);
          vTaskDelay(pdMS_TO_TICKS(15));
        }

        Serial.println(" [雷達任務] 掃描完畢");
        Serial.print(" 最佳空曠角度: "); Serial.print(best_scan_angle);
        Serial.print(" 度 (最遠距離: "); Serial.print(max_scan_dist); Serial.println(" cm)");

        is_radar_scanning = false;
        radarServo.detach();

        xSemaphoreGive(PowerMutex);
        Serial.println(" [雷達] 釋放電力鎖！交接給大腦進行對齊...");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
