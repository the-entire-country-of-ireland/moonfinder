#include "rtc.h"
#include "i2c_config.h"
#include <sys/time.h>
#include <time.h>

RTC::RTC()
  : _initialized(false),
    _fallbackEpoch(DateTime(F(__DATE__), F(__TIME__)).unixtime()),
    _fallbackStartMillis(0),
    _hasValidRtcTime(false),
    _useSystemClock(false) {
}

bool RTC::init(uint8_t sda_pin=MoonlightI2cSda, uint8_t scl_pin=MoonlightI2cScl) {
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
  if (_useSystemClock) return DateTime(static_cast<uint32_t>(time(nullptr)));
  if (_initialized && _hasValidRtcTime) return _rtc.now();
  return DateTime(_fallbackEpoch + (millis() - _fallbackStartMillis) / 1000);
}

bool RTC::syncSystemClock() {
  DateTime value = currentTime();
  timeval tv;
  tv.tv_sec = static_cast<time_t>(value.unixtime());
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  _useSystemClock = true;
  return value.unixtime() != 0;
}

bool RTC::useSystemClock() {
  const time_t systemTime = time(nullptr);
  if (systemTime <= 0) return false;
  _useSystemClock = true;
  return true;
}

String RTC::getDateTimeString() {
  DateTime dt = currentTime();
  
  char buffer[20];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());
  
  return String(buffer);
}

String RTC::getEasternDateTimeString() {
  static bool timezoneConfigured = false;
  if (!timezoneConfigured) {
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();
    timezoneConfigured = true;
  }

  const time_t timestamp = static_cast<time_t>(currentTime().unixtime());
  struct tm easternTime;
  localtime_r(&timestamp, &easternTime);

  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
           easternTime.tm_year + 1900, easternTime.tm_mon + 1,
           easternTime.tm_mday, easternTime.tm_hour, easternTime.tm_min,
           easternTime.tm_sec);
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