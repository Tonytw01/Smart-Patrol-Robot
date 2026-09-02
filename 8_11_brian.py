import cv2
import time
import socket
import numpy as np
import face_recognition
import os 
import select

# 系統與通訊設定

HOST = '0.0.0.0'
PORT = 8000

KNOWN_FACES_DIR = "known_faces"  
INTRUDERS_DIR = "intruders"      
COOLDOWN_SECONDS = 5             
CONFIRM_FRAMES = 1
AI_COOLDOWN_SECONDS = 10            

os.makedirs(INTRUDERS_DIR, exist_ok=True)
os.makedirs(KNOWN_FACES_DIR, exist_ok=True)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


#  1. 載入人臉白名單資料庫

print("系統啟動中... 正在載入組員白名單...")
known_face_encodings = []
known_face_names = []

#  確保 known_faces 資料夾有放這些照片
team_database = {
    "Team Member A": ["member_A1.jpg"], 
    "Team Member B": ["member_B1.jpg"], 
}

for name, photo_list in team_database.items():
    for photo_name in photo_list:
        photo_path = os.path.join(KNOWN_FACES_DIR, photo_name)
        if os.path.exists(photo_path):
            try:
                image = face_recognition.load_image_file(photo_path)
                encoding = face_recognition.face_encodings(image)[0]
                known_face_encodings.append(encoding)
                known_face_names.append(name)
                print(f" 成功載入特徵: {name} ({photo_name})")
            except IndexError:
                print(f" 警告: 在 {photo_name} 中找不到人臉，已跳過。")
        else:
            print(f" 警告: 找不到照片檔案 {photo_path}")

print(" 人臉特徵載入完畢！")


#  2. 網路與狀態初始化
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))

print(f" 高階視覺大腦啟動，監聽 Port {PORT}...")

current_distance = 999
last_command = ""
is_auto_mode = False 
hardware_bypass_step = 0  

frame_counter = 0
AI_INTERVAL = 5
face_locations = []
face_names = []
ai_locked_state = "None"  # 狀態會有："None", "Stranger", "Member"
last_face_detect_time = 0

#  三個變數來控制 1 秒緩衝
is_waiting_for_face = False
face_wait_start_time = 0
STABILIZE_DELAY =0.4  #1 秒鐘的站定緩衝時間

#  將這兩個變數拉到迴圈外，確保跳過幀時能保留上一次的記憶
current_frame_has_stranger = False
current_frame_has_member = False

while True:
   
    #  1. 接收與解析封包 ( 升級：緩衝區抽乾技術 Buffer Draining)
    # 檢查是否有封包等待接收 (最多等 0.01 秒)
    ready = select.select([server_socket], [], [], 0.01)
    if not ready[0]:
        continue 
        
    latest_frame_data = None
    
    # 只要緩衝區裡還有積壓的舊封包，就一直把它們抽出來
    while True:
        ready = select.select([server_socket], [], [], 0.0)
        if ready[0]:
            message, address = server_socket.recvfrom(65536)
            
            # 攔截感測器數據並即時更新 (確保拿到最新距離)
            if message.startswith(b"DIST:"):
                try:
                    data_parts = message.decode('utf-8').strip().split(":")
                    current_distance = int(data_parts[1]) 
                    if len(data_parts) >= 4:
                        hardware_mode = data_parts[2]
                        hardware_bypass_step = int(data_parts[3]) 
                        is_auto_mode = True if hardware_mode == "PI" else False
                except ValueError:
                    pass
            else:
                # 記住最新的影像封包，覆蓋掉舊的
                latest_frame_data = message
        else:
            break # 緩衝區已被抽乾，跳出抽取迴圈
            
    # 如果這批封包裡剛好沒有影像，就回到最上層繼續等
    if latest_frame_data is None:
        continue
    
    # 確保永遠只解碼最新的那一張影像
    frame = cv2.imdecode(np.frombuffer(latest_frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None: 
        continue

    frame =cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # 修正顏色通道順序，確保 OpenCV 正確顯示


    #  3. 人臉辨識核心邏輯 ( 輕量雷達喚醒 + 1秒防抖版)
    current_time = time.time()
    is_in_cooldown = (current_time - last_face_detect_time) < AI_COOLDOWN_SECONDS

    frame_counter += 1

    if not is_in_cooldown:
        
     
        # 階段 A：待機巡邏 (使用極速 Haar Cascade 雷達)

       
        if not is_waiting_for_face:
            if frame_counter % 5 == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                potential_faces = face_cascade.detectMultiScale(
                    gray, scaleFactor=1.2, minNeighbors=3, minSize=(20, 20)
                )                
                
                if len(potential_faces) > 0:
                    is_waiting_for_face = True
                    face_wait_start_time = current_time

                    #  宣告一個全域或外層變數來存放截圖
                    global best_snapshot 
                    best_snapshot = None 
                    

        # 階段 B：短暫防抖後，鎖定截圖定生死

        else:
            if (current_time - face_wait_start_time) < STABILIZE_DELAY:

                #  0.4 秒防抖期：車子正在急煞，不斷更新最新的一張圖
                # 確保拿到的是車子完全停穩那一刻的畫面
                best_snapshot = frame.copy() 
                
                # 螢幕上依然畫 Scanning 讓你知道它正在鎖定
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                face_locations = face_recognition.face_locations(rgb_frame, model="hog")
                face_names = ["Scanning..."] * len(face_locations)
                
            else:
                #  時間到直接拿出剛才防抖期最後一刻拍下的 best_snapshot 來算
                
                if best_snapshot is not None:
                    # 把保留的照片轉成 RGB 給模型辨識
                    ai_rgb_frame = cv2.cvtColor(best_snapshot, cv2.COLOR_BGR2RGB)
                    ai_face_locations = face_recognition.face_locations(ai_rgb_frame, model="hog")
                    
                    if len(ai_face_locations) > 0:
                        face_encodings = face_recognition.face_encodings(ai_rgb_frame, ai_face_locations)
                        face_names = []
                        current_frame_has_stranger = False
                        current_frame_has_member = False
                        
                        for face_encoding in face_encodings:
                            matches = face_recognition.compare_faces(known_face_encodings, face_encoding, tolerance=0.6)
                            name = "Stranger"

                            if True in matches:
                                first_match_index = matches.index(True)
                                name = known_face_names[first_match_index]
                                current_frame_has_member = True
                            else:
                                current_frame_has_stranger = True
                            
                            face_names.append(name)

                        #  更新狀態並存檔，存檔也是存保留的最清晰截圖
                        if current_frame_has_stranger:
                            ai_locked_state = "Stranger"
                            timestamp = time.strftime("%Y%m%d_%H%M%S")
                            snap_filename = os.path.join(INTRUDERS_DIR, f"Intruder_{timestamp}.jpg")
                            cv2.imwrite(snap_filename, best_snapshot) # 存下完美犯罪證據
                            print(f" [警報] 發現陌生人，已拍下存證：{snap_filename}")
                            
                        elif current_frame_has_member:
                            ai_locked_state = "Member"
                            
                        is_waiting_for_face = False
                        last_face_detect_time = current_time
                        
                    else:
                        # 如果照片裡居然沒抓到人臉，退回雷達模式繼續找
                        ai_locked_state = "None"
                        is_waiting_for_face = False
                
    elif is_in_cooldown:
        # 階段 C：冷卻中，清空狀態
        is_waiting_for_face = False
        face_locations = []
        face_names = []

    
    #  4. 繪製人臉方框
 
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        
        #  修改：加入橘色的 Scanning 判斷
        if name == "Stranger":
            color = (0, 0, 255)       # 紅色警報
        elif name == "Scanning...":
            color = (0, 165, 255)     # 橘色掃描中
        else:
            color = (0, 255, 0)       # 綠色白名單通行

        cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
        cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)


  
    #  5. 決策與 UI 顯示 (新增除錯文字)
  
    command = "Standby" 
    status_text = ""
    status_color = (255, 255, 255)

    if not is_auto_mode:
        status_text = "[Mode: Glove] Standby"
        status_color = (255, 191, 0)
        last_command = ""  #  關鍵修復：在手套模式待機時清空記憶，確保下次切換為大腦時，能強制發出第一道指令
    else:
        command = "Forward"
        status_text = "[Auto] Patrolling (Forward)..."
        status_color = (255, 255, 0) 

        #  視覺化判定 1：看到自己人 
        if ai_locked_state == "Member":
            status_text = f"[Auto] MEMBER VERIFIED -> Action: Forward"
            status_color = (0, 255, 0)

        #  視覺化判定 2：確認為陌生人 
        elif ai_locked_state == "Stranger":
            status_text = "[Auto]  STRANGER DETECTED -> Action: STOP!"
            status_color = (0, 0, 255) # 紅色警報
            command = "Forward"



        # 覆寫狀態：硬體避障優先級最高
        if hardware_bypass_step > 0 or current_distance < 50:
            action_str = ""
            if hardware_bypass_step == 1: action_str = " (Turn Right)"
            elif hardware_bypass_step == 2: action_str = " (Forward)" 
            elif hardware_bypass_step == 3: action_str = " (Turn Left)"
            elif hardware_bypass_step == 4: action_str = " (Recovering)"
            status_text = f"[Auto] ESP32 Bypassing!{action_str}"
            status_color = (0, 165, 255) 

    if is_auto_mode and command != last_command and command != "Standby":
        server_socket.sendto((command + '\n').encode('utf-8'), address)
        last_command = command
        print(f"[{time.strftime('%H:%M:%S')}] 發送決策指令: {command}")


    #  6. 繪製螢幕文字 (距離、懷疑度、狀態)

    cv2.putText(frame, f"Dist: {current_distance} cm", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
    
    #  修改冷卻狀態顯示，並在倒數結束時重置狀態
    if is_in_cooldown:
        remain_time = int(AI_COOLDOWN_SECONDS - (current_time - last_face_detect_time))
        cv2.putText(frame, f"AI Cooldown: {remain_time}s", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    else:
        # 冷卻結束了，狀態恢復為尋找中
        ai_locked_state = "None"


    # 左下角：顯示具體的行動指令 (Forward / Stop / Bypassing)
    cv2.putText(frame, status_text, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)
    
    cv2.imshow('Final Patrol Brain', frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        server_socket.sendto("Stop\n".encode('utf-8'), address)
        break
    elif key == ord('m'):
        server_socket.sendto("SWITCH_MODE\n".encode('utf-8'), address)
        last_command = "" 

server_socket.close()
cv2.destroyAllWindows()