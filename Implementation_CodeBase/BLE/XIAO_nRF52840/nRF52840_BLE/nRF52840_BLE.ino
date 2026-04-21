#include <bluefruit.h>


// Define the advertising name. Ensure the Receiver's filter matches this name.
#define DEV_NAME "ESP32_S3_TX"
void scan_callback(ble_gap_evt_adv_report_t* report) {
uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));

  // 1. 解析「完整裝置名稱」 (AD Type: 0x09)
  int name_len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer));
  
  // 2. 解析「製造商自定義資料」 (AD Type: 0xFF)
  uint8_t mdata[32];
  memset(mdata, 0, sizeof(mdata));
  int mdata_len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, mdata, sizeof(mdata));

  // 3. 判斷邏輯：檢查名稱是否符合目標
  if (name_len > 0 && strcmp((char*)buffer, DEV_NAME) == 0) {
    buffer[name_len] = '\0';
    Serial.printf("[%lu] Device: %s | RSSI: %d", millis(), buffer, report->rssi);

    // 如果同時有製造商資料，就印出來
    if (mdata_len >= 3) {
      // 根據您的 Transmitter 設定：mdata[0] & [1] 是 ID, mdata[2] 是狀態碼
      Serial.printf(" | M-Data ID: %02X%02X | Status: %02X", mdata[1], mdata[0], mdata[2]);
    }
    Serial.println();
  }

  // 4. 維持您的重啟掃描機制以持續接收
  Bluefruit.Scanner.stop();
  Bluefruit.Scanner.start(0);
}

void setup() {
  // 1. 強制延遲，給 USB 連接埠列舉的時間
  delay(3000); 
  Serial.begin(115200);
  while (!Serial) delay(10); 

  Serial.println("--- Continuous Scan Test ---");

  // 2. 初始化 LED 作為心跳監測
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH); // 關閉 (nRF52840 LED 是低電位觸發)

  // 3. 調整掃描參數：不要 100% 佔滿，給系統留 25% 的時間處理 Serial
  Bluefruit.begin(0, 1);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setInterval(200, 150); // 每 125ms 掃描其中的 93ms
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);
}

void loop() {
  // 如果這個 LED 一直閃，代表 CPU 沒有當機
  digitalWrite(LED_RED, LOW);  // 點亮
  delay(50);
  digitalWrite(LED_RED, HIGH); // 熄滅
  delay(450);
}