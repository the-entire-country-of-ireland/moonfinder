#include "rtc.h"
#include <sys/time.h>

#define SDA 22
#define SCL 27

RTC::RTC()
  : _initialized(false),
    _fallbackEpoch(DateTime(F(__DATE__), F(__TIME__)).unixtime()),
    _fallbackStartMillis(0),
    _hasValidRtcTime(false) {
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
    Serial.println("Using compile time plus uptime.");
  } else {
    Serial.println("RTC is running with valid time");
    Serial.print("Current time: ");
    Serial.println(getDateTimeString());
    _hasValidRtcTime = true;
  }
  
  Serial.println("=== RTC Ready ===\n");
  
  _initialized = true;
  _fallbackStartMillis = millis();
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
  _fallbackEpoch = adjustedTime.unixtime();
  _fallbackStartMillis = millis();
  _hasValidRtcTime = true;
  
  Serial.print("RTC set to: ");
  Serial.println(getDateTimeString());
  
  return true;
}

DateTime RTC::now() {
  if (!_initialized) {
    Serial.println("ERROR: RTC not initialized!");
    return currentTime();
  }

  return currentTime();
}

DateTime RTC::currentTime() {
  if (_initialized && _hasValidRtcTime) return _rtc.now();
  return DateTime(_fallbackEpoch + (millis() - _fallbackStartMillis) / 1000);
}

bool RTC::syncSystemClock() {
  DateTime value = currentTime();
  timeval tv;
  tv.tv_sec = static_cast<time_t>(value.unixtime());
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  return value.unixtime() != 0;
}

String RTC::getDateTimeString() {
  DateTime dt = currentTime();
  
  char buffer[20];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());
  
  return String(buffer);
}

String RTC::getISO8601() {
  DateTime dt = currentTime();
  
  char buffer[20];
  sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());
  
  return String(buffer);
}

uint32_t RTC::getUnixTime() {
  return currentTime().unixtime();
}