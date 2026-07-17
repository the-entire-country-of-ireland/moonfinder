// #include <Arduino.h>
// #include "sd_card.h"

// SDCard sdCard;  // Uses default VSPI pins

// void setup() {
//   Serial.begin(115200);
//   delay(100);
  
//   // Initialize SD card
//   if (!sdCard.init()) {
//     Serial.println("Failed to initialize SD card!");
//     return;
//   }
  
//   // Create a new file with prefix "sensor_readings"
//   String filename = sdCard.createFile("sensor_readings", "/data");
//   if (filename.isEmpty()) {
//     Serial.println("Failed to create file!");
//     return;
//   }
  
//   Serial.println("Writing data to: " + filename);
  
//   // Write header
//   sdCard.appendLine("timestamp,temp,humidity,pressure");
  
//   // Write 5 lines of random sensor data
//   for (int i = 0; i < 5; i++) {
//     float temp = random(200, 300) / 10.0;      // 20-30°C
//     float humidity = random(300, 800) / 10.0;  // 30-80%
//     float pressure = random(9800, 10200) / 10.0; // 980-1020 hPa
    
//     // Method 1: Using String
//     // String line = String(millis()) + "," + String(temp, 1) + "," + 
//     //               String(humidity, 1) + "," + String(pressure, 1);
//     // sdCard.appendLine(line);
    
//     // Method 2: Using printf-style formatting
//     sdCard.appendLinef("%lu,%.1f,%.1f,%.1f", millis(), temp, humidity, pressure);
    
//     Serial.printf("Written: %lu,%.1f,%.1f,%.1f\n", millis(), temp, humidity, pressure);
    
//     delay(100);
//   }
  
//   Serial.println("\nData written successfully!");
//   Serial.println("Current file: " + sdCard.getCurrentFilename());
// }

// void loop() {
//   // Nothing here
// }