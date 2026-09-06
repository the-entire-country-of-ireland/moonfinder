#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>
#include <MPU9250_WE.h>

// IMU
#define SDA_PIN 6
#define SCL_PIN 5

// I2C Addresses
#define ICM20948_ADDR 0x69
#define MPU9250_ADDR 0x68

Adafruit_ICM20948 myIMU20948;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);


void myscanI2C() {
  Serial.println("\nScanning I2C bus...");
  byte count = 0;
  
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" device(s)");
}


void setup_myIMU20948() {
    // Initialize ICM-20948
  Serial.println("\n--- Initializing ICM-20948 ---");
  if (!myIMU20948.begin_I2C(ICM20948_ADDR)) {
    Serial.println("Failed to find ICM-20948!");
  } else {
    Serial.println("ICM-20948 Found!");
    myIMU20948.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    myIMU20948.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
    myIMU20948.setMagDataRate(AK09916_MAG_DATARATE_20_HZ);
  }
}

void setup_mpu9250() {
  Serial.println("\n--- Initializing MPU-9250 ---");
  
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

}




void readMPU9250() {  
  Serial.println("--- MPU-9250 ---");

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);
}

void readICM20948() {
  sensors_event_t accel, gyro, mag, temp;
  myIMU20948.getEvent(&accel, &gyro, &temp, &mag);
  
  Serial.println("--- ICM-20948 ---");
  Serial.print("Accel: X="); Serial.print(accel.acceleration.x);
  Serial.print(" Y="); Serial.print(accel.acceleration.y);
  Serial.print(" Z="); Serial.println(accel.acceleration.z);
  
  Serial.print("Gyro:  X="); Serial.print(gyro.gyro.x);
  Serial.print(" Y="); Serial.print(gyro.gyro.y);
  Serial.print(" Z="); Serial.println(gyro.gyro.z);
  
  Serial.print("Mag:   X="); Serial.print(mag.magnetic.x);
  Serial.print(" Y="); Serial.print(mag.magnetic.y);
  Serial.print(" Z="); Serial.println(mag.magnetic.z);
}




void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Multiple IMU Test");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz
  
  // Scan I2C bus
  myscanI2C();
    
  setup_mpu9250();
  setup_myIMU20948();

  delay(100);
}

void loop() {
  Serial.println("\n========================================");
  
  readICM20948();
  readMPU9250();
  
  delay(100);
}


