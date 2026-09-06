#include "sd_card.h"
#include <ArduinoEigen.h>
#include <fstream>
#include <iostream>

SDCard::SDCard(uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t mosi_pin) 
  : _cs_pin(cs_pin), _initialized(false) {
  _spi = new SPIClass(VSPI);
  _spi->begin(sck_pin, miso_pin, mosi_pin, cs_pin);
}

bool SDCard::init() {
  Serial.println("\n=== SD Card Initialization ===");
  
  if (!SD.begin(_cs_pin, *_spi)) {
    Serial.println("ERROR: SD Card mount failed!");
    _initialized = false;
    return false;
  }
  
  // Check card type
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("ERROR: No SD card attached!");
    _initialized = false;
    return false;
  }
  
  // Print card type
  Serial.print("Card Type: ");
  switch(cardType) {
    case CARD_MMC:
      Serial.println("MMC");
      break;
    case CARD_SD:
      Serial.println("SDSC");
      break;
    case CARD_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("UNKNOWN");
  }
  
  // Print card size
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card Size: %llu MB\n", cardSize);
  
  // Print space info
  uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
  uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
  uint64_t freeBytes = totalBytes - usedBytes;
  
  Serial.printf("Total Space: %llu MB\n", totalBytes);
  Serial.printf("Used Space: %llu MB\n", usedBytes);
  Serial.printf("Free Space: %llu MB\n", freeBytes);
  
  Serial.println("=== SD Card Ready ===\n");
  
  _initialized = true;
  return true;
}

String SDCard::createFolder(const String& prefix, const String& folder) {
  if (!_initialized) {
    Serial.println("ERROR: SD card not initialized!");
    return "";
  }
  
  // Create folder if it doesn't exist
  if (!SD.exists(folder)) {
    if (SD.mkdir(folder)) {
      Serial.println("Created parent folder: " + folder);
    } else {
      Serial.println("ERROR: Failed to create parent folder: " + folder);
      return "";
    }
  }
  
  // Find next available filename
  int counter = 0;
  String filename;
  
  while (counter < 10000) {  // Safety limit
    filename = folder + "/" + prefix + "_" + String(counter);
    if (!SD.exists(filename)) {
      break;
    }
    counter++;
  }
  
  if (counter >= 10000) {
    Serial.println("ERROR: Too many folders with this prefix!");
    return "";
  }
  
  // Create the folder
  if (!SD.mkdir(filename)) {
    Serial.println("ERROR: Failed to create folder: " + filename);
    return "";
  }
  
  Serial.println("Created folder: " + filename);
  
  return filename;
}

String SDCard::createFile(const String& prefix, const String& folder) {
  if (!_initialized) {
    Serial.println("ERROR: SD card not initialized!");
    return "";
  }
  
  // Create folder if it doesn't exist
  if (!SD.exists(folder)) {
    if (SD.mkdir(folder)) {
      Serial.println("Created parent folder: " + folder);
    } else {
      Serial.println("ERROR: Failed to create parent folder: " + folder);
      return "";
    }
  }
  
  // Find next available filename
  int counter = 0;
  String filename;
  
  while (counter < 10000) {  // Safety limit
    filename = folder + "/" + prefix + "_" + String(counter) + ".txt";
    if (!SD.exists(filename)) {
      break;
    }
    counter++;
  }
  
  if (counter >= 10000) {
    Serial.println("ERROR: Too many files with this prefix!");
    return "";
  }
  
  // Create the file (open and close to create it)
  fs::File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("ERROR: Failed to create file: " + filename);
    return "";
  }
  file.close();
  
  _current_filename = filename;
  Serial.println("Created file: " + filename);
  
  return filename;
}

bool SDCard::appendLine(const String& data) {
  
  if (!_initialized) {
    Serial.println("ERROR: SD card not initialized!");
    return false;
  }
  
  if (_current_filename.isEmpty()) {
    Serial.println("ERROR: No file selected! Call createFile() first.");
    return false;
  }
  
  fs::File file = SD.open(_current_filename, FILE_APPEND);
  if (!file) {
    Serial.println("ERROR: Failed to open file for writing: " + _current_filename);
    return false;
  }
  
  file.println(data);
  file.close();
  
  return true;
}

bool SDCard::appendLinef(const char* format, ...) {
  if (!_initialized) {
    Serial.println("ERROR: SD card not initialized!");
    return false;
  }
  
  if (_current_filename.isEmpty()) {
    Serial.println("ERROR: No file selected! Call createFile() first.");
    return false;
  }
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  fs::File file = SD.open(_current_filename, FILE_APPEND);
  if (!file) {
    Serial.println("ERROR: Failed to open file for writing: " + _current_filename);
    return false;
  }
  
  file.println(buffer);
  file.close();
  
  return true;
}

uint64_t SDCard::getCardSize() {
  return _initialized ? SD.cardSize() / (1024 * 1024) : 0;
}

uint64_t SDCard::getUsedSpace() {
  return _initialized ? SD.usedBytes() / (1024 * 1024) : 0;
}

uint64_t SDCard::getFreeSpace() {
  if (!_initialized) return 0;
  uint64_t total = SD.totalBytes() / (1024 * 1024);
  uint64_t used = SD.usedBytes() / (1024 * 1024);
  return total - used;
}

String SDCard::getCardType() {
  if (!_initialized) return "Not initialized";
  
  uint8_t cardType = SD.cardType();
  switch(cardType) {
    case CARD_MMC:  return "MMC";
    case CARD_SD:   return "SDSC";
    case CARD_SDHC: return "SDHC";
    default:        return "UNKNOWN";
  }
}

void SDCard::printDirectory(const char* path, uint8_t levels) {
  File root = SD.open(path);
  if (!root) {
    Serial.printf("Failed to open directory: %s\n", path);
    return;
  }

  if (!root.isDirectory()) {
    Serial.printf("%s is not a directory\n", path);
    root.close();
    return;
  }

  Serial.printf("Listing directory: %s\n", path);

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.printf("DIR : %s\n", file.name());

      // Recurse into subdirectories if levels > 0
      if (levels > 0) {
        printDirectory(file.path(), levels - 1);
      }
    } else {
      Serial.printf("FILE: %s  SIZE: %lu bytes\n",
                    file.name(),
                    (unsigned long)file.size());
    }

    file = root.openNextFile();
  }

  root.close();
}

void SDCard::printFileLines(const char* filename) {
  File file = SD.open(filename);
  if (!file) {
    Serial.printf("Failed to open %s\n", filename);
    return;
  }

  if (file.isDirectory()) {
    Serial.printf("%s is a directory, not a file\n", filename);
    file.close();
    return;
  }

  Serial.printf("Contents of %s:\n", filename);

  while (file.available()) {
    Serial.println(file.readStringUntil('\n'));
  }

  file.close();
}

bool SDCard::loadCalibrationData(const std::string& filename, 
                                  Eigen::Matrix3d& M, 
                                  Eigen::Vector3d& c) {
  File file = SD.open(filename.c_str());
  if (!file) {
    Serial.printf("Failed to open file: %s\n", filename.c_str());
    return false;
  }

  if (file.isDirectory()) {
    Serial.printf("%s is a directory, not a file\n", filename.c_str());
    file.close();
    return false;
  }

  // Read 3x3 matrix (3 lines, 3 values per line)
  for (int i = 0; i < 3; i++) {
    if (!file.available()) {
      Serial.printf("Error: unexpected end of file at matrix row %d\n", i);
      file.close();
      return false;
    }
    
    String line = file.readStringUntil('\n');
    line.trim();
    
    // Parse space-separated values
    int startIdx = 0;
    for (int j = 0; j < 3; j++) {
      int spaceIdx = line.indexOf(' ', startIdx);
      String value;
      
      if (spaceIdx == -1) {
        value = line.substring(startIdx);  // last value on line
      } else {
        value = line.substring(startIdx, spaceIdx);
        startIdx = spaceIdx + 1;
      }
      
      M(i, j) = value.toDouble();
    }
  }

  // Read 3D vector (1 line, 3 values)
  if (!file.available()) {
    Serial.println("Error: unexpected end of file while reading vector c");
    file.close();
    return false;
  }
  
  String line = file.readStringUntil('\n');
  line.trim();
  
  int startIdx = 0;
  for (int i = 0; i < 3; i++) {
    int spaceIdx = line.indexOf(' ', startIdx);
    String value;
    
    if (spaceIdx == -1) {
      value = line.substring(startIdx);
    } else {
      value = line.substring(startIdx, spaceIdx);
      startIdx = spaceIdx + 1;
    }
    
    c(i) = value.toDouble();
  }

  file.close();
  Serial.printf("Successfully loaded calibration from %s\n", filename.c_str());
  
  // Optional: print what was loaded
  Serial.println("Matrix M:");
  for (int i = 0; i < 3; i++) {
    Serial.printf("  %f %f %f\n", M(i, 0), M(i, 1), M(i, 2));
  }
  Serial.println("Vector c:");
  Serial.printf("  %f %f %f\n", c(0), c(1), c(2));
  
  return true;
}

