/*
Date: 5/2021
Author: Elias Santistevan @ SparkFun Electronics
Writing vibration values via I2C to vibrate the motor. 

This vibrates *extremely* vigurously, adhere the motor to something or it will
produce a fault and stop functioning. 

*/

#include <Wire.h>
#include "Haptic_Driver.h"
#include <ArduinoBLE.h>

Haptic_Driver hapDrive;

int event = 0; 

// for Multiplexer
#define MUX_ADDR 0x70
void selectMuxChannel(uint8_t channel) {
  if (channel > 3) return; // 這款只有 4 個通道 (0-3)

  Wire.beginTransmission(MUX_ADDR);
  // 使用位元左移操作來選擇通道 (1 << 0 = 1, 1 << 1 = 2...)
  Wire.write(1 << channel); 
  Wire.endTransmission();
}

void setup(){

  Wire.begin();
  Serial.begin(115200);

  // bluetooth setup
  BLE.begin();
  BLE.scanForName("ESP32_Sensor");


  // haptic driver setup
  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Qwiic Haptic Driver DA7280 found!");

  if( !hapDrive.defaultMotor() ) 
    Serial.println("Could not set default settings.");

  // Frequency tracking is done by the IC to ensure that the motor is hitting
  // its resonant frequency. I found that restricting the PCB (squeezing)
  // raises an error which stops operation because it can not reach resonance.
  // I disable here to avoid this error. 
  hapDrive.enableFreqTrack(false);

  Serial.println("Setting I2C Operation.");
  hapDrive.setOperationMode(DRO_MODE);
  Serial.println("Ready.");

  delay(1000);

}

void loop(){

  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    if (peripheral.connect()) {
      // after connection, look for the characteristic with the specified UUID
      BLECharacteristic dataChar = peripheral.characteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8");
      
      if (dataChar) {
        uint8_t value = 0;
        dataChar.read(); // read data
        dataChar.readValue(value);
        // control multiplexer channel based on received value
      }
    }
  }
  // If uploading often the Haptic Driver IC will throw a fault. Let's
  // clear that error (0x10), just in case.
  //event = hapDrive.getIrqEvent();
  //Serial.print("Interrupt: ");
  //Serial.println(event, HEX);
  //Serial.println("Clearing event.");
  //hapDrive.clearIrq(event);

  // Max value is 127 with acceleration on (default).
  // set the front motor
  selectMuxChannel(0);
  hapDrive.setVibrate(25);
  delay(500); 
  hapDrive.setVibrate(0); 
  delay(500);

  // set the back motor
  selectMuxChannel(2);
  hapDrive.setVibrate(25);
  delay(500); 
  hapDrive.setVibrate(0); 
  delay(500);
}
