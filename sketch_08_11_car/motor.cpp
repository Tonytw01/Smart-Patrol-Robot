#include "motor.h"


// 動態路徑與防禦機制全域變數

static float target_turn_angle = 0.0;        // 雷達目標轉向角 (-90~90度)
const float INERTIA_OFFSET = 25.0;           // 慣性補償角度 (避免過轉)
static unsigned long continuous_straight_timer = 0; // 連續直走時間
static int small_turn_count = 0;             // 角落死結計數器
static int straight_10s_count = 0;           // 連續10秒無障礙次數

// PID 與姿態控制全域變數
extern float target_yaw;                     // 預期目標角度
extern float integral ;                      // PID 積分誤差
extern float prev_error ;                    // PID 前次誤差

// 哨兵模式專用變數
static int sentry_turn_count = 0;   // 記錄轉45度次數 (0~7)
static int sentry_max_dist = 0;     // 環視最遠距離
static float sentry_best_angle = 0.0; // 最遠距離對應角度

// ==========================================
// 電子主動煞車系統
// ==========================================
void stopRobot() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(channelA, 0);
  ledcWrite(channelB, 0);

  current_left_pwm = 0;
  current_right_pwm = 0;
  integral = 0;
  prev_error = 0;
  target_yaw = yaw_angle;
}

const int IR_LEFT_PIN = 34;
const int IR_RIGHT_PIN = 35;

// ==========================================
// 馬達硬體初始化
// ==========================================
void motor_init() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // 啟動紅外線感測器
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  const int SERVO_PIN = 13; // 伺服馬達
  ledcSetup(channelA, freq, resolution);
  ledcSetup(channelB, freq, resolution);
  ledcAttachPin(ENA, channelA);
  ledcAttachPin(ENB, channelB);

  stopRobot();
  Serial.println("[Motor] 初始化完成！");
}

// ==========================================
// FreeRTOS 任務：馬達 PID 與避障決策
// ==========================================
void Task_Motor(void *pvParameters) {
  // 設定精準的 20ms (50Hz) 迴圈
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  int print_timer = 0;
  static int lost_signal_count = 0;
  static bool is_defensive_scan = false; // 紀錄是否為15秒觸發掃描

  while (1) {
    unsigned long current_time = millis();

    // --- 1. 卡爾曼濾波器 (處理超音波數據) ---
    if (raw_distance != 999) {
      lost_signal_count = 0;
      float calibrated_dist = raw_distance * 0.8;
      kf_P = kf_P + kf_Q;
      float K = kf_P / (kf_P + kf_R);
      kf_est = kf_est + K * (calibrated_dist - kf_est);
      kf_P = (1 - K) * kf_P;
      filtered_distance = (int)kf_est;
    } else {
      lost_signal_count++;
      if (lost_signal_count > 10) {
        filtered_distance = 999;
        kf_est = 50.0;
      }
    }

    // --- 2. 姿態 PID 核心計算 ---
    float error = target_yaw - yaw_angle;

    // 條件式積分：誤差<10度才累積，避免大彎累積無效誤差
    if (abs(error) < 10.0) {
      integral += (error * 0.02);
    }
    // 誤差大或非直行時清空歷史誤差
    else {
      integral = 0;
    }

    // 手套放開油門時，積分歸零
    if (abs(target_throttle) <= 15 && abs(target_steer) <= 10) {
      integral = 0;
      prev_error = 0;
    }

    if (integral > 50)  integral = 50;
    if (integral < -50) integral = -50;

    int target_gear_pwm = 0;
    // 初始化 Ki，由檔位決定
    float dynamic_Kp = 0, dynamic_Ki = 0, dynamic_Kd = 0;
    int LEFT_BOOST = 0;

    if (dynamic_speed < 140) {
      target_gear_pwm = 0;
    }
    else if (dynamic_speed == 140) {
      target_gear_pwm = 165;
      dynamic_Kp = 5;
      dynamic_Ki = 0.0009; // 中速檔 Ki
      dynamic_Kd = 1;
      LEFT_BOOST = 10;
    }
    else if (dynamic_speed >= 141 && dynamic_speed <= 180) {
      target_gear_pwm = 150;
      dynamic_Kp = 19;
      dynamic_Ki = 0.0009; // 高速檔 Ki
      dynamic_Kd = 1.5;
      LEFT_BOOST = 15;
    }
    else if (dynamic_speed > 180 && dynamic_speed <= 200) {
      target_gear_pwm = 180;
      dynamic_Kp = 16;
      dynamic_Ki = 0.001; // 極速檔 Ki
      dynamic_Kd = 1.7;
      LEFT_BOOST = 15;
    }
    else {
      target_gear_pwm = 220;
      dynamic_Kp = 8;
      dynamic_Ki = 0.0009;
      dynamic_Kd = 2;
      LEFT_BOOST = 15;
    }

    float pid_correction = (dynamic_Kp * error) + (dynamic_Ki * integral) + (dynamic_Kd * (error - prev_error) / 0.02);
    prev_error = error;

    // ==========================================
    // 3. 決策區 (目標油門與轉向)
    // ==========================================
    if (bypass_step > 0) {
      unsigned long elapsed = current_time - bypass_timer;

      // --------------------------------------------------
      // [狀態 99] 伺服雷達掃描與決策分流
      // --------------------------------------------------
      if (bypass_step == 99) {
        if (!is_radar_scanning) {
          Serial.println("[Decision] 雷達掃描結束，分析地形...");

          target_turn_angle = best_scan_angle - 90.0;

          // 核心升級：動態避障轉向角度限制在正負 70 度內
          target_turn_angle = constrain(target_turn_angle, -80.0, 80.0);

          // 極端情況防護
          if (max_scan_dist < 20) {
            target_turn_angle = -90.0;
          }

          // 判斷是否為小角度微調 (<45度)
          if (abs(target_turn_angle) < 45.0) {
            small_turn_count++;
            Serial.print("[Alert] 狹窄角落！計數：");
            Serial.println(small_turn_count);
          } else {
            small_turn_count = 0; // 轉大彎代表出口寬廣，計數歸零
          }

          // 檢查是否陷入死結
          if (small_turn_count >= 3) {
            Serial.println("[Alert] 陷入死結！強制倒車U轉！");
            target_turn_angle = 150.0; // 強制轉 150 度
            small_turn_count = 0;      // 脫困後歸零
            bypass_step = 5;           // 統一進入倒車狀態
          }
          else if (abs(target_turn_angle) < 15.0) {
            Serial.println("[Info] 前方空曠，解除警報...");
            bypass_step = 4; // 進入緩衝期恢復直走
          }
          else {
            Serial.print("[Info] 準備向 ");
            Serial.print(target_turn_angle);
            Serial.println(" 度脫離...");
            bypass_step = 5; // 統一進入倒車狀態
          }

          bypass_timer = current_time;
        }
      }
      // --------------------------------------------------
      // [狀態 9] 哨兵模式前置：防禦性倒車
      // --------------------------------------------------
      else if (bypass_step == 9) {
        target_throttle = -45; // 打倒檔
        target_steer = 0;

        // 倒車 600 毫秒
        if (elapsed >= 600) {
          Serial.println("[Sentry] 後退完成，啟動環視掃描...");
          target_throttle = 0;
          stopRobot(); // 煞車停穩

          bypass_step = 10; // 進入哨兵靜止測量
          bypass_timer = current_time;
          yaw_angle = 0.0; target_yaw = 0.0; // 重置陀螺儀
        }
      }
      // --------------------------------------------------
      // [狀態 10] 哨兵模式：靜止3秒 (人臉與距離偵測)
      // --------------------------------------------------
      else if (bypass_step == 10) {
        target_throttle = 0;
        target_steer = 0;

        if (elapsed >= 3000) { // 停3秒讓相機抓人臉

          // 記錄當下方位超音波距離
          Serial.print("[Scan] 角度: ");
          Serial.print(sentry_turn_count * 45);
          Serial.print(" 度 | 測量距離: ");
          Serial.println(filtered_distance);

          // 找出最空曠方向
          if (filtered_distance > sentry_max_dist) {
            sentry_max_dist = filtered_distance;
            sentry_best_angle = sentry_turn_count * 45.0; // 記錄對應角度
          }

          sentry_turn_count++; // 準備下一個方位

          if (sentry_turn_count < 8) {
            // 還沒轉完一圈，進入狀態 11 繼續轉
            bypass_step = 11;
            bypass_timer = current_time;
            yaw_angle = 0.0; target_yaw = 0.0;
          } else {
            // 轉完一圈回到原點
            Serial.print("[Scan] 結束！最佳角度："); Serial.println(sentry_best_angle);

            // 將 0~315 度轉換為底盤最高效率轉向 (-180 到 +180度)
            if (sentry_best_angle > 180.0) {
              target_turn_angle = sentry_best_angle - 360.0;
            } else {
              target_turn_angle = sentry_best_angle;
            }

            // 如果四周塞滿 (<20cm)，強制作U型迴轉
            if (sentry_max_dist < 20) target_turn_angle = 175.0;

            bypass_step = 1; // 切換至動態轉向對齊
            bypass_timer = current_time;
            yaw_angle = 0.0; target_yaw = 0.0;
          }
        }
      }
      // --------------------------------------------------
      // [狀態 11] 哨兵模式：旋轉45度
      // --------------------------------------------------
      else if (bypass_step == 11) {
        if (elapsed < 300) {
          target_throttle = 0; target_steer = 0;
        }
        else if (elapsed > 2000) {
          // 防呆：45度轉2秒未到位代表卡住，啟動脫困
          Serial.println("[Alert] 環視卡死，啟動蠕動救援...");
          bypass_step = 6;
          bypass_timer = current_time;
        }
        else {
          target_throttle = 0;
          target_steer = 60; // 統一向左轉45度

          // 最小煞車角度防呆
          float actual_stop_angle = 55.0 - INERTIA_OFFSET;
          if (actual_stop_angle < 5.0) {
            actual_stop_angle = 5.0; // 強制最少轉5度
          }

          // 加上慣性補償轉45度
          if (abs(yaw_angle) >= actual_stop_angle) {
            bypass_step = 10; // 回到狀態 10 靜止測量
            bypass_timer = current_time;
            yaw_angle = 0.0; target_yaw = 0.0; integral = 0; prev_error = 0;
            target_throttle = 0; target_steer = 0;
          }
        }
      }
      // --------------------------------------------------
      // [狀態 5] 防禦性倒車脫離
      // --------------------------------------------------
      else if (bypass_step == 5) {
        target_throttle = -45; // 稍微打倒檔
        target_steer = 0;

        if (elapsed >= 800) {
          Serial.println("[Info] 後退完畢，準備對齊...");
          target_throttle = 0;
          bypass_step = 1;             // 進入動態轉向狀態
          bypass_timer = current_time;
          yaw_angle = 0.0; target_yaw = 0.0; // 重置陀螺儀
        }
      }
      // --------------------------------------------------
      // [狀態 6] 卡死救援：左右扭動脫困
      // --------------------------------------------------
      else if (bypass_step == 6) {
        // 向左扭 0.3 秒
        if (elapsed < 300) {
          target_throttle = 0; target_steer = 60;
        }
        // 向右扭 0.3 秒
        else if (elapsed < 600) {
          target_throttle = 0; target_steer = -60;
        }
        // 再次向左扭 0.3 秒
        else if (elapsed < 900) {
          target_throttle = 0; target_steer = 60;
        }
        // 再次向右扭 0.3 秒
        else if (elapsed < 1200) {
          target_throttle = 0; target_steer = -60;
        }
        // 扭鬆後直線倒車拉開距離
        else if (elapsed < 1700) {
          target_throttle = -50; target_steer = 0;
        }
        // 跳出轉彎迴圈，重新雷達評估
        else {
          Serial.println("[Rescue] 扭動脫困完畢！重啟雷達掃描...");
          target_throttle = 0;
          target_steer = 0;

          is_radar_scanning = true; // 喚醒雷達伺服馬達
          bypass_step = 99;         // 回到狀態99重新決策
          bypass_timer = current_time;
          yaw_angle = 0.0; target_yaw = 0.0; // 陀螺儀歸零
        }
      }
      // --------------------------------------------------
      // [狀態 1] 精準動態轉向對齊
      // --------------------------------------------------
      else if (bypass_step == 1) {
        if (elapsed < 300) {
          target_throttle = 0; target_steer = 0; // 對齊前先停穩
        }
        else if (elapsed > 2500) {
          Serial.println("[Rescue] 轉彎超時！可能卡住，啟動二次倒車！");
          target_throttle = 0; target_steer = 0;
          bypass_step = 6;             // 退回重試
          bypass_timer = current_time; 
          yaw_angle = 0.0; target_yaw = 0.0; 
        }
        else {
          target_throttle = 0;

          if (target_turn_angle > 0) {
            target_steer = 60; // 左轉
          } else {
            target_steer = -60; // 右轉
          }
          // 最小煞車角度防呆
          float actual_stop_angle = abs(target_turn_angle) - INERTIA_OFFSET;
          if (actual_stop_angle < 10.0) {
            actual_stop_angle = 10.0; // 強制最少轉10度
          }
          // 加入慣性補償
          if (abs(yaw_angle) >= (abs(target_turn_angle) - INERTIA_OFFSET)) {
            Serial.println("[Info] 對齊缺口！進入緩衝期...");
            bypass_step = 4;
            bypass_timer = current_time;
            yaw_angle = 0.0; target_yaw = 0.0; integral = 0; prev_error = 0;
            target_throttle = 0; target_steer = 0;
          }
        }
      }
      // --------------------------------------------------
      // [狀態 4] 轉向後的穩定緩衝期 (Settling Time)
      // --------------------------------------------------
      else if (bypass_step == 4) {
        target_throttle = 0;
        target_steer = 0;

        if (elapsed >= 500) {
          Serial.println("[Info] 緩衝結束，平穩起步！");
          bypass_step = 0;
          target_throttle = 50;
          dynamic_speed = 140;
          continuous_straight_timer = current_time; // 重新開始直走計時
        }
      }

      // --------------------------------------------------
      // [狀態 20] 左紅外線觸發：倒車後右微調
      // --------------------------------------------------
      else if (bypass_step == 20) {
        if (elapsed < 400) {
          // 階段一：倒車拉開距離
          target_throttle = -45;
          target_steer = 0;
        }
        else {
          // 階段二：原地轉彎
          target_throttle = 0;
          target_steer = 60;    // 向右轉

          // 利用陀螺儀轉15度
          if (abs(yaw_angle) >= 15.0) {
            Serial.println("[Info] 左側盲區解除，恢復直走！");
            bypass_step = 4; // 進入緩衝期
            bypass_timer = current_time;
            yaw_angle = 0.0; target_yaw = 0.0; integral = 0; prev_error = 0;
          }
        }
      }
      // --------------------------------------------------
      // [狀態 21] 右紅外線觸發：倒車後左微調
      // --------------------------------------------------
      else if (bypass_step == 21) {
        if (elapsed < 400) {
          // 階段一：倒車
          target_throttle = -45;
          target_steer = 0;
        }
        else {
          // 階段二：原地轉彎
          target_throttle = 0;
          target_steer = -60;     // 向左轉

          if (abs(yaw_angle) >= 15.0) {
            Serial.println("[Info] 右側盲區解除，恢復直走！");
            bypass_step = 4;
            bypass_timer = current_time;
            yaw_angle = 0.0; target_yaw = 0.0; integral = 0; prev_error = 0;
          }
        }
      }
    }
    // ==========================================
    // 一般避障與15秒防禦檢查
    // ==========================================

    // 1. 一般超音波避障
    else if (currentMode == MODE_PI && filtered_distance < SAFE_DISTANCE_PI && target_throttle > 15 && bypass_step == 0) {
      Serial.println("[PI] 偵測到前方障礙物！");
      target_throttle = 0; target_steer = 0;
      stopRobot();
      is_radar_scanning = true;
      straight_10s_count = 0;   // 計數器歸零
      bypass_step = 99;
    }
    // 紅外線盲區防護
    else if (currentMode == MODE_PI && bypass_step == 0 && target_throttle > 15 &&
             (digitalRead(IR_LEFT_PIN) == LOW || digitalRead(IR_RIGHT_PIN) == LOW)) {
      
      bool ir_left_alert = (digitalRead(IR_LEFT_PIN) == LOW);
      bool ir_right_alert = (digitalRead(IR_RIGHT_PIN) == LOW);

      if (ir_left_alert && ir_right_alert) {
        // 雙側皆阻擋，觸發主避障
        Serial.println("[IR] 雙側盲區阻擋！啟動主避障...");
        target_throttle = 0; target_steer = 0;
        stopRobot();
        is_radar_scanning = true;
        bypass_step = 99;
      }
      else if (ir_left_alert) {
        Serial.println("[IR] 左側障礙！向右微調...");
        bypass_step = 20;
        bypass_timer = current_time;       // 重置計時器
        yaw_angle = 0.0; target_yaw = 0.0; // 陀螺儀歸零
      }
      else if (ir_right_alert) {
        Serial.println("[IR] 右側障礙！向左微調...");
        bypass_step = 21;
        bypass_timer = current_time;       // 重置計時器
        yaw_angle = 0.0; target_yaw = 0.0;
      }
    }
    // 2. 15秒哨兵模式 (環視/人臉偵測)
    else if (currentMode == MODE_PI && bypass_step == 0 && target_throttle > 15 && target_steer == 0) {
      if (current_time - continuous_straight_timer > 10000) {
        Serial.println("[Sentry] 直走滿10秒，啟動環視！");

        // 給予初始倒車動力
        target_throttle = -45;
        target_steer = 0;

        // 初始化哨兵記憶
        sentry_turn_count = 0;
        sentry_max_dist = 0;
        sentry_best_angle = 0.0;
        yaw_angle = 0.0; target_yaw = 0.0; // 陀螺儀歸零

        bypass_step = 9; // 進入測量狀態
        bypass_timer = current_time;
        continuous_straight_timer = current_time;
      }
    }

    // 3. 手套防撞保護
    else if (currentMode == MODE_GLOVE && filtered_distance < SAFE_DISTANCE_GLOVE && target_throttle > 15) {
      target_throttle = 0;
    }

    // 4. 重置直走計時器
    else if (target_throttle <= 15 || target_steer != 0) {
      continuous_straight_timer = current_time;
    }

    // 5. 穩定直走超過3秒，解除角落警報
    else if (bypass_step == 0 && target_throttle > 15 && target_steer == 0) {
      if (current_time - continuous_straight_timer > 3000) {
        if (small_turn_count > 0) {
          Serial.println("[Info] 恢復平穩直行，角落計數器歸零！");
          small_turn_count = 0;
        }
      }
    }

    /*
        // ==========================================
        // 專題實驗：PID步階產生器
        // ==========================================
        static unsigned long exp_start_time = millis();
        static bool step_applied = false;
        static bool init_locked = false;

        // 1. 強制接管控制權：維持中低速直走
        target_throttle = 50;
        dynamic_speed = 140;
        target_steer = 0;

        // 2. 開機前3秒：鎖定初始角度，確保平穩直走
        if (current_time - exp_start_time <= 3000) {
          if (!init_locked) {
            target_yaw = yaw_angle; // 記錄落地角度
            init_locked = true;
            Serial.println("[Test] 初始角度已鎖定...");
          }
        }
        // 3. 三秒鐘一到：注入30度的標準步階
        else if (!step_applied) {
          target_yaw = yaw_angle + 90.0; // 步階設定
          step_applied = true;
          Serial.println("[Test] 步階觸發！");
        }
        // ==========================================
    */

    // ==========================================
    // 4. 硬體輸出區 (決策轉PWM)
    // ==========================================
    if (xSemaphoreTake(PowerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

      int hardware_steer = target_steer;
      int glove_threshold = 70;

      if (currentMode == MODE_GLOVE) {
        if (abs(hardware_steer) <= glove_threshold) {
          hardware_steer = 0;
        }
      }

      // 航向鎖定緩衝計時器
      static unsigned long straight_start_time = 0;
      static bool was_turning_or_stopped = true;

      // ------------------------------------
      // 轉彎狀態
      // ------------------------------------
      if (abs(hardware_steer) > 10) {
        was_turning_or_stopped = true; // 標記車子正在轉彎

        int turn_pwm;
        if (currentMode == MODE_PI) {
          turn_pwm = 215;
        } else {
          turn_pwm = 200;
        }

        current_left_pwm = turn_pwm;
        current_right_pwm = turn_pwm;

        if (hardware_steer > 10) { // 左轉
          digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        } else { // 右轉
          digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
        }
        ledcWrite(channelA, current_right_pwm); ledcWrite(channelB, current_left_pwm);

        integral = 0; prev_error = 0;
        target_yaw = yaw_angle;
      }
      // ------------------------------------
      // 直走狀態 (含緩衝)
      // ------------------------------------
      else if (abs(target_throttle) > 15) {

        // 剛進入直走狀態時
        if (was_turning_or_stopped) {
          straight_start_time = millis(); // 記錄直走開始時間
          was_turning_or_stopped = false;
        }

        // 給予300ms收斂適應期，動態更新目標角度避免PID暴衝
        if (current_time - straight_start_time < 300) {
          target_yaw = yaw_angle;
          integral = 0;
          prev_error = 0;
          pid_correction = 0; // 強制不補償
        }

        // 輸出馬力
        if (target_throttle > 15) {
          // 前進輸出
          current_left_pwm  = constrain(target_gear_pwm + LEFT_BOOST - pid_correction, 0, 255);
          current_right_pwm = constrain(target_gear_pwm + pid_correction, 0, 255);
          digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
        } else {
          // 倒車輸出：固定PWM，無視PID
          current_left_pwm  = 180 + LEFT_BOOST;
          current_right_pwm = 180;
          digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        }
        ledcWrite(channelA, current_right_pwm); ledcWrite(channelB, current_left_pwm);
      }
      // ------------------------------------
      // 停止狀態
      // ------------------------------------
      else {
        was_turning_or_stopped = true; // 標記車子停止
        stopRobot();
      }

      xSemaphoreGive(PowerMutex);

    } else {
      stopRobot();
    }

    // --- 5. 定期顯示資訊 ---
    print_timer += 20;
    if (print_timer >= 300) {
      Serial.print("[Motor Task] ");
      Serial.print(currentMode == MODE_GLOVE ? "手套" : "大腦");
      Serial.print(" | 油門: "); Serial.print(target_throttle);
      Serial.print(" | 轉向: "); Serial.print(target_steer);
      Serial.print(" | 距離: "); Serial.print(filtered_distance);
      Serial.print(" cm | 角度: "); Serial.println(yaw_angle, 1);
      print_timer = 0;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}