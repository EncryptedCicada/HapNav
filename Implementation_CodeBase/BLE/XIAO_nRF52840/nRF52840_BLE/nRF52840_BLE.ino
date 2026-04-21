#include <bluefruit.h>

void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t buffer[32]; // 用來存放抓取到的名稱
  memset(buffer, 0, sizeof(buffer));

  /* 嘗試從廣播封包中解析名稱
     類型 0x09 (COMPLETE_LOCAL_NAME) 是最標準的名稱存放位置
  */
  int len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer));
  
  // 如果找不到完整名稱，嘗試抓取簡短名稱 (0x08)
  if (len == 0) {
    len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer));
  }

  // 如果成功解析出名稱（長度大於 0）
  if (len > 0) {
    buffer[len] = '\0'; // 確保字串結尾正確，避免亂碼
    
    Serial.print("Device Name Found: ");
    Serial.print((char*)buffer); // 印出名稱
    Serial.print(" | RSSI: ");
    Serial.println(report->rssi); // 印出訊號強度
  } 
  /* 註：如果 len 為 0，代表該封包是匿名的（例如某些背景廣播），
     在「單純 parse 名字」的邏輯下，我們就直接跳過不處理。
  */
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); 

  Serial.println("--- Hardcore Scan Mode ---");

  // 1. 初始化 Bluefruit，確保有足夠的資源給 Central 模式
  if (!Bluefruit.begin(0, 1)) {
    Serial.println("Error: Bluefruit failed to start!");
  }
  
  // 2. 設定電源功率 (設為最大值確保接收靈敏度)
  Bluefruit.setTxPower(4); 

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setInterval(160, 160); 
  Bluefruit.Scanner.useActiveScan(true); 
  Bluefruit.Scanner.start(0); 

  Serial.println("Scanning started...");
}

void loop() {}