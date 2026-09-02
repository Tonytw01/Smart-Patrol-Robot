#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- 1. 定義通訊結構 
typedef struct message {
  int speed;       // 前後速度 (-255 ~ 255)
  int turn;        // 左右轉向 (-255 ~ 255)
  int flex_pwm;    // 彎曲感測檔位 PWM 值
  int mode_switch; // 模式切換訊號 (0=手套模式, 1=自主巡邏模式)
} message;


//  遙測回傳結構 (加入時間戳記)

typedef struct telemetry_data {
  unsigned long timestamp; // 時間戳記 (毫秒)
  float target_angle;
  float current_angle;
} telemetry_data;

telemetry_data incomingTelemetry;


//3. 新增：接收回呼函式 (把收到的數據印成 CSV)

void OnTelemetryRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // 確認收到的封包大小是否等於遙測結構的大小
  if (len == sizeof(telemetry_data)) {
    memcpy(&incomingTelemetry, incomingData, sizeof(incomingTelemetry));

    //  印出最純粹的 CSV 格式： 時間,目標值,實際值
    Serial.print(incomingTelemetry.timestamp);
    Serial.print(",");
    Serial.print(incomingTelemetry.target_angle);
    Serial.print(",");
    Serial.println(incomingTelemetry.current_angle);
  }
}

message myMessage;
Adafruit_MPU6050 mpu;

// 接收端的 MAC 位址
uint8_t peer1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;


//  腳位與全域變數設定

const int FLEX_PIN = 34;   // 彎曲感測器接腳
const int BUTTON_PIN = 4;  //  切換模式按鈕接腳 

//  中斷與防震盪專用變數 
volatile unsigned long last_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 300; // 防震盪時間：300毫秒內的連續觸發都會被過濾
volatile bool button_pressed_flag = false; // 紀錄按鈕是否被有效按下的旗標



//  硬體中斷函式 (Interrupt Service Routine)

// ESP32 的中斷函式必須加上 IRAM_ATTR，讓它跑在最快的 RAM 裡面
void IRAM_ATTR handleButtonInterrupt() {
  unsigned long current_time = millis();

  // 防震盪邏輯：如果現在時間距離上次觸發超過 300 毫秒，才視為有效按壓
  if (current_time - last_debounce_time > DEBOUNCE_DELAY) {
    button_pressed_flag = true; // 立起旗標，交給主迴圈處理
    last_debounce_time = current_time;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "傳送成功" : "傳送失敗");
}

void setup() {
  Serial.begin(115200);

  //  更改 CSV 標題列 (新增時間欄位)
  Serial.println("Time(ms),Target_Angle,Current_Angle");

  //  1. 恢復 MPU6050 初始化 ( 如果沒接硬體，程式會卡在這裡保護系統)
  if (!mpu.begin()) {
    Serial.println("找不到 MPU6050，請檢查接線。");
    while (1) yield();
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(FLEX_PIN, INPUT);

  //  初始化按鈕腳位
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);

  //  2. 啟動 WiFi 與 ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失敗");
    return;
  }

  //  3. 註冊發送與接收的回呼函式
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnTelemetryRecv);

  //  4. 恢復註冊車載端為 Peer (必須執行，否則 loop 發送會報錯)
  memcpy(peerInfo.peer_addr, peer1, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("無法新增接收端");
    return;
  }
}

void loop() {
  
  //  處理按鈕切換邏輯 (改為脈衝觸發)
  
  if (button_pressed_flag) {
    myMessage.mode_switch = 1; //  1 代表「發送一次切換請求」
    button_pressed_flag = false; // 放下旗標
    //    Serial.println(" 傳送模式切換脈衝指令");
  } else {
    myMessage.mode_switch = 0; //  0 代表「平時無動作」
  }

  // --- 讀取 MPU6050 數據 ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float z_accel = a.acceleration.z;
  float y_accel = a.acceleration.y;

  // --- 讀取彎曲感測器數據 ---
  int flex_raw = analogRead(FLEX_PIN);

  // --- 數據轉換與死區設定 ---
  int speed_val = map(y_accel * 10, -98, 98, -255, 255);
  int turn_val  = map(z_accel * 10, -98, 98, -255, 255);

  if (abs(y_accel) < 1.5) {
    speed_val = 0;
  }
  if (abs(z_accel) < 1.5) {
    turn_val = 0;
  }

  // --- 彎曲感測器檔位邏輯 ---
  int target_gear = 0;
  if (flex_raw < 500) {
    target_gear = 0;
  } else if (flex_raw >= 500 && flex_raw < 800) {
    target_gear = 150;
  } else if (flex_raw >= 800 && flex_raw < 905) {
    target_gear = 195;
  } else {
    target_gear = 230;
  }

  // --- 打包封包 ---
  myMessage.speed = -speed_val;
  myMessage.turn  = turn_val;
  myMessage.flex_pwm = target_gear;



  // --- 正式發送 ---
  esp_now_send(peer1, (uint8_t *) &myMessage, sizeof(myMessage));

  // --- 除錯印出 ---
  //  Serial.print("傳送 -> [Speed: "); Serial.print(myMessage.speed);
  //  Serial.print(", Turn: "); Serial.print(myMessage.turn);
  //  Serial.print(", Gear: "); Serial.println(myMessage.flex_pwm);


  delay(50);
}
