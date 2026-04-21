#include <bluefruit.h>
#include <Wire.h>
#include "Haptic_Driver.h" // 引用 Haptic Driver 函式庫 [cite: 14]

// 定義目標裝置名稱與 I2C 地址
#define DEV_NAME "ESP32_S3_TX"
#define MUX_ADDR 0x70 // Multiplexer 地址 [cite: 15]

Haptic_Driver hapDrive; // 建立驅動器物件 [cite: 14]

// 多工器通道切換函式 [cite: 15, 16]
void selectMuxChannel(uint8_t channel) {
  if (channel > 3) return; 
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel); 
  Wire.endTransmission();
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));

  int name_len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer));
  
  uint8_t mdata[32];
  int mdata_len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, mdata, sizeof(mdata));

  if (name_len > 0 && strcmp((char*)buffer, DEV_NAME) == 0) {
    buffer[name_len] = '\0';
    
    if (mdata_len >= 3) {
      uint8_t status = mdata[2]; // 獲取傳送端的狀態值 [cite: 1, 5]
      Serial.printf("[%lu] Device: %s | Status: %d\n", millis(), buffer, status);

      // --- 核心邏輯：收到 10 時觸發振動 ---
      if (status == 10) {
        Serial.println(">>> 觸發 Haptic 振動！");
        
        // selectMuxChannel(0);       // 選擇通道 0 
        hapDrive.setVibrate(80);   // 設定振動強度 (最大 127) 
        delay(200);                // 振動 200ms
        hapDrive.setVibrate(0);    // 停止振動 
      }
    }
  }

  Bluefruit.Scanner.stop();
  Bluefruit.Scanner.start(0);
}

void setup() {
  delay(3000);
  Serial.begin(115200);
  Wire.begin(); // 初始化 I2C 總線 [cite: 14]

  // 1. Haptic Driver 初始化 [cite: 17, 18, 19]
  if (!hapDrive.begin()) {
    Serial.println("無法連接到 Haptic Driver！");
  } else {
    Serial.println("Qwiic Haptic Driver DA7280 已就緒！");
    hapDrive.defaultMotor();      // 載入預設馬達設定 [cite: 18]
    hapDrive.enableFreqTrack(false); // 停用頻率追蹤以避免壓力報錯 
    hapDrive.setOperationMode(DRO_MODE); // 設定為直接輸出模式 
  }

  // 2. BLE 掃描器初始化
  Bluefruit.begin(0, 1);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setInterval(200, 150);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);
}

void loop() {
  // loop 保持空置或處理其他任務
}