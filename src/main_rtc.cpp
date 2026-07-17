#include <Arduino.h>
#include "rtc.h"

RTC rtc;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize RTC (default I2C pins: SDA=21, SCL=22)
  if (!rtc.init()) {
    Serial.println("Failed to initialize RTC!");
    return;
  }
  
  // Set RTC to compilation time (uncomment if needed)
  // rtc.setToCompileTime();
  
  Serial.println("RTC initialized successfully!\n");
}

void loop() {
  // Method 1: Get formatted string
  Serial.println(rtc.getDateTimeString());
  
  // Method 2: Get DateTime object for more control
  DateTime now = rtc.now();
  Serial.printf("%04d/%02d/%02d %02d:%02d:%02d\n",
                now.year(), now.month(), now.day(),
                now.hour(), now.minute(), now.second());
  
  // Method 3: Get Unix timestamp
  Serial.printf("Unix: %lu\n", rtc.getUnixTime());
  
  // Method 4: Get ISO 8601 format
  Serial.println(rtc.getISO8601());
  
  delay(1000);
}