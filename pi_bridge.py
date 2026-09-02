import cv2
import socket
import serial
from picamera2 import Picamera2

# 初始化 UART
try:
    esp32_serial = serial.Serial('/dev/serial0', 9600, timeout=0.05)
    print(" UART 連線成功")
except Exception as e:
    print(f" UART 初始化失敗: {e}")
    esp32_serial = None

# 初始化網路與相機
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
laptop_address = ('YOUR_LAPTOP_IP', 8000) 
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

try:
    while True:
        # 1. 發送影像
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 50])
        client_socket.sendto(buffer.tobytes(), laptop_address)

        # 2. 轉發超音波數據 (讀取 ESP32 -> 傳給筆電)
        # 升級版：轉發超音波數據 (加入忽略亂碼防護)
        if esp32_serial and esp32_serial.in_waiting > 0:
            try:
                # 就算收到亂碼也先印出來看看
                raw_bytes = esp32_serial.readline()
                print(f" [底層偵錯] 收到原始位元組: {raw_bytes}")
                
                sensor_data = raw_bytes.decode('utf-8', errors='ignore').strip()
                if sensor_data.startswith("DIST:"):
                    print(f" [樹莓派中繼] 成功拿到距離: {sensor_data}") 
                    client_socket.sendto(sensor_data.encode('utf-8'), laptop_address)
            except Exception as e:
                # 把原本的 pass 換成印出錯誤
                print(f" 讀取 UART 發生異常: {e}")

        # 3. 轉發控制指令 (筆電 -> ESP32)
        client_socket.settimeout(0.01)
        try:
            data, _ = client_socket.recvfrom(1024)
            if esp32_serial:
                esp32_serial.write((data.decode('utf-8') + '\n').encode('utf-8'))
        except:
            pass

except KeyboardInterrupt:
    picam2.stop()
    if esp32_serial: esp32_serial.close()
