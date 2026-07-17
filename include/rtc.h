#ifndef RTC_H
#define RTC_H

#include <Wire.h>
#include "RTClib.h"

class RTC {
private:
  RTC_PCF8523 _rtc;
  bool _initialized;
  
public:
  // Constructor
  RTC();
  
  // Initialize RTC (call in setup)
  bool init(uint8_t sda_pin = 21, uint8_t scl_pin = 22);
  
  // Set RTC to compilation date/time
  bool setToCompileTime();
  
  // Get current date/time as DateTime object
  DateTime now();
  
  // Get formatted date/time string (YYYY-MM-DD HH:MM:SS)
  String getDateTimeString();
  
  // Get ISO 8601 format (YYYY-MM-DDTHH:MM:SS)
  String getISO8601();
  
  // Get Unix timestamp
  uint32_t getUnixTime();
  
  // Check if initialized
  bool isInitialized() const { return _initialized; }
};

#endif