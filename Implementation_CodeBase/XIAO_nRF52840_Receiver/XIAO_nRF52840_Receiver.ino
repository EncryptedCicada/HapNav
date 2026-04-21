#include <bluefruit.h>

#include <Wire.h>

#include "Haptic_Driver.h" // Include Haptic Driver library

// Define target device name and I2C address

#define DEV_NAME "ESP32_S3_TX"

#define MUX_ADDR 0x70 // Multiplexer address

Haptic_Driver hapDrive; // Create haptic driver object

// Function to switch multiplexer channel

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

      uint8_t status = mdata[2]; // Get status value from transmitter

      Serial.printf("[%lu] Device: %s | Status: %d | RSSI: %d\n", millis(), buffer, status, report->rssi );

    //   Serial.print(" | RSSI: ");
    // Serial.println(report->rssi); // 印出訊號強度
      // --- Core logic: trigger vibration when status == 10 ---

      if (status == 10) {

        Serial.println(">>> Trigger Haptic Vibration!");

        

        // selectMuxChannel(0);       // Select channel 0

        hapDrive.setVibrate(10);   // Set vibration intensity (max 127)

        delay(200);                // Vibrate for 200 ms

        hapDrive.setVibrate(0);    // Stop vibration

      }

    }

  }

  Bluefruit.Scanner.stop();

  Bluefruit.Scanner.start(0);

}

void setup() {

  delay(3000);

  Serial.begin(115200);

  Wire.begin(); // Initialize I2C bus

  // 1. Initialize Haptic Driver

  if (!hapDrive.begin()) {

    Serial.println("Failed to connect to Haptic Driver!");

  } else {

    Serial.println("Qwiic Haptic Driver DA7280 is ready!");

    hapDrive.defaultMotor();      // Load default motor configuration

    hapDrive.enableFreqTrack(false); // Disable frequency tracking to avoid pressure-related errors

    hapDrive.setOperationMode(DRO_MODE); // Set to direct output mode

  }

  // 2. Initialize BLE scanner

  Bluefruit.begin(0, 1);

  Bluefruit.Scanner.setRxCallback(scan_callback);

  Bluefruit.Scanner.setInterval(200, 150);

  Bluefruit.Scanner.useActiveScan(true);

  Bluefruit.Scanner.start(0);

}

void loop() {

  // Keep loop empty or handle other tasks

}