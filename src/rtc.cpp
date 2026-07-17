#include "rtc.h"

#define SDA 22
#define SCL 27

RTC::RTC() : _initialized(false) {
}

bool RTC::init(uint8_t sda_pin=SDA, uint8_t scl_pin=SCL) {
  Serial.println("\n=== RTC Initialization ===");
  
  // Initialize I2C
  Wire.begin(sda_pin, scl_pin);
  
  // Initialize RTC
  if (!_rtc.begin()) {
    Serial.println("ERROR: Couldn't find PCF8523 RTC!");
    _initialized = false;
    return false;
  }
  
  Serial.println("PCF8523 RTC found!");
  
  // Check if RTC lost power
  if (!_rtc.initialized() || _rtc.lostPower()) {
    Serial.println("WARNING: RTC lost power!");
    Serial.println("Call setToCompileTime() to set the time.");
  } else {
    Serial.println("RTC is running with valid time");
    Serial.print("Current time: ");
    Serial.println(getDateTimeString());
  }
  
  Serial.println("=== RTC Ready ===\n");
  
  _initialized = true;
  return true;
}

bool RTC::setToCompileTime(int32_t offset_seconds) {
  if (!_initialized) {
    Serial.println("ERROR: RTC not initialized!");
    return false;
  }
  
  Serial.println("Setting RTC to compile time...");
  
  DateTime compileTime = DateTime(F(__DATE__), F(__TIME__));
  DateTime adjustedTime = DateTime(compileTime.unixtime() + offset_seconds);
  
  _rtc.adjust(adjustedTime);
  
  Serial.print("RTC set to: ");
  Serial.println(getDateTimeString());
  
  return true;
}

DateTime RTC::now() {
  if (!_initialized) {
    Serial.println("ERROR: RTC not initialized!");
    return DateTime((uint32_t) 0);  // Return epoch time
  }
  
  return _rtc.now();
}

String RTC::getDateTimeString() {
  if (!_initialized) {
    return "RTC not initialized";
  }
  
  DateTime dt = _rtc.now();
  
  char buffer[20];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());
  
  return String(buffer);
}

String RTC::getISO8601() {
  if (!_initialized) {
    return "RTC not initialized";
  }
  
  DateTime dt = _rtc.now();
  
  char buffer[20];
  sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());
  
  return String(buffer);
}

uint32_t RTC::getUnixTime() {
  if (!_initialized) {
    return 0;
  }
  
  return _rtc.now().unixtime();
}