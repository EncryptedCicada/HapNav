#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

// Define the advertising name. Ensure the Receiver's filter matches this name.
#define DEV_NAME "ESP32_VAL_TX" 

BLEAdvertising *pAdvertising;
uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("--- BLE Transmitter Verification Start ---");

  // Initialize the BLE device [cite: 3]
  BLEDevice::init(DEV_NAME);
  pAdvertising = BLEDevice::getAdvertising();

  // Configure advertising data
  BLEAdvertisementData advData;
  advData.setName(DEV_NAME);
  
  // Set Manufacturer Data (using 0xFFFF as a testing Company ID)
  // Followed by a status value to verify the Receiver gets the latest data
  char mData[4];
  mData[0] = 0xFF; 
  mData[1] = 0xFF;
  mData[2] = 0x00; // Initial status value
  advData.setManufacturerData(String(mData, 3));

  pAdvertising->setAdvertisementData(advData);

  // Optimization: Adjust advertising parameters for better scanning
  // Shortening the interval helps the Receiver capture signals within its scan window
  pAdvertising->setMinInterval(0x20); // 20ms (0x20 * 0.625ms)
  pAdvertising->setMaxInterval(0x40); // 40ms (0x40 * 0.625ms)
  
  // Enable Scan Response to ensure the device name is fully transmitted
  pAdvertising->setScanResponse(true); 

  // Start advertising
  pAdvertising->start();
  Serial.println("Advertising is running...");
}

void loop() {
  // Update advertising data every 2 seconds to observe changes on the Receiver [cite: 5, 6]
  delay(2000);
  counter++;

  BLEAdvertisementData newAdvData;
  newAdvData.setName(DEV_NAME);
  
  // Toggle status between 0 and 1
  uint8_t statusByte = (counter % 2 == 0) ? 0x01 : 0x00; 
  
  uint8_t mData[3];
  mData[0] = 0xFF;
  mData[1] = 0xFF;
  mData[2] = statusByte;

  newAdvData.setManufacturerData(String((char*)mData, 3));
  
  // Update the actual advertising payload
  pAdvertising->setAdvertisementData(newAdvData);

  Serial.print("Update Payload - Counter: ");
  Serial.print(counter);
  Serial.print(" | Status: ");
  Serial.println(statusByte);
}