#ifndef SD_CARD_H
#define SD_CARD_H

#include <FS.h>
#include <SD.h>
#include <SPI.h>

class SDCard {
private:
  SPIClass* _spi;
  uint8_t _cs_pin;
  String _current_filename;
  bool _initialized;
  
public:
  // Constructor
  SDCard(uint8_t cs_pin = 5, 
         uint8_t sck_pin = 18, 
         uint8_t miso_pin = 19, 
         uint8_t mosi_pin = 23);
  
  // Initialize SD card and print info
  bool init();
  
  // Create new file with prefix and auto-increment counter
  // Returns the filename created, or empty string on failure
  String createFile(const String& prefix, const String& folder = "/data");
  
  // Append line to current file
  bool appendLine(const String& data);
  
  // Append formatted data to current file
  bool appendLinef(const char* format, ...);
  
  // Get current filename
  String getCurrentFilename() const { return _current_filename; }
  
  // Check if initialized
  bool isInitialized() const { return _initialized; }
  
  // Get card info
  uint64_t getCardSize();
  uint64_t getUsedSpace();
  uint64_t getFreeSpace();
  String getCardType();
};

#endif