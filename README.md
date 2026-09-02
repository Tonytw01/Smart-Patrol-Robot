 智慧巡邏車機電控制系統 (Smart Patrol Robot)



這是一個結合機電整合、邊緣運算與電腦視覺的智慧巡邏機器人專案。系統採用「三層式硬體架構」，並搭載 FreeRTOS 雙核心即時作業系統，實現 50Hz 之 PID 姿態控制與自主避障。



 系統架構

大腦 (PC 端): 負責高負載之 OpenCV/dlib 人臉辨識與最高決策。

腦幹 (Raspberry Pi 3B+): 負責影像 JPEG 壓縮串流與 UART/UDP 指令橋接。

小腦 (ESP32): 運行 FreeRTOS 雙核心多工，執行 50Hz PID 航向鎖定、超音波卡爾曼濾波與伺服馬達環景掃描避障。

手套遙控器 (ESP32 + MPU6050): 透過 ESP-NOW 無線協定實現低延遲體感控制。



 環境建置與執行

1\. *PC 端人臉辨識環境: 需安裝 Python 3.11 與相關視覺套件。

&#x20;  bash

&#x20;  pip install opencv-python face\_recognition

註：Windows 環境需預先配置 C++ 編譯環境或下載對應之 dlib 檔進行安裝。

2\. 參數設定: 執行前請至 pi\_bridge.py、sketch\_08\_04\_send.ino 與 comms.cpp 中，將 IP 位址與 MAC 位址替換為使用者當下實際的網路環境參數。

