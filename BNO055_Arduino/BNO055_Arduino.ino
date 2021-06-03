#include "Wire.h"

#define ADDR 0x28 //BNO055 address
#define CHIP_ID 0x00 //Chip ID register
#define Page_ID 0x07 //Page ID register
#define OPR_MODE 0x3D //Operation mode register
#define ACC_Config 0x08 //Accelerometer configuration register
#define MAG_Config 0x09 //Magnetometer configuration register
#define GYR_Config_0 0x0A //Gyroscope configuration register 1
#define GYR_Config_1 0x0B  //Gyroscope configuration register 2
#define GYRO_DATA 0x14 //Start of the 6 registers that returns gyro data
#define MAG_DATA 0x0E //Start of the 6 registers that returns magnetometer data
#define ACC_DATA 0x08 //Start of the 6 registers that returns accelerometer data
#define UNIT_SEL 0x3B // Unit Selection register

//define writeToBNO as the commands required to write to the BNO just to reduce clutter
#define writeToBNO(reg, data) Wire.beginTransmission(ADDR);Wire.write(reg);Wire.write(data);Wire.endTransmission(true);

//raw accel values
double accX = 0, accY = 0, accZ = 0;

//raw gyro values
double gyroX = 0, gyroY = 0, gyroZ = 0;

unsigned long prevTime;

void setup() {
  Serial.begin(19200); //begins the serial monitor

  Wire.begin(); //begins communication with all i2c devices

  Wire.setClock(10000000); //changed the i2c clock to 1MHz

  bnoInitialize(); //initialized the BNO055
}

void loop() {
  prevTime = micros();
  
  gyroData();
  accData();

  /*
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.print(gyroZ);
  Serial.print(",");
  Serial.print(accX);
  Serial.print(",");
  Serial.print(accY);
  Serial.print(",");
  Serial.println(accZ);
  Serial.print(",");
  */
  
  Serial.println(micros() - prevTime);
}

/*
 * Program functions from this point forwards
 */

//get data from the gyroscopes
void gyroData() {
  Wire.beginTransmission(ADDR);
  Wire.write(GYRO_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, 6, false);
  
  gyroX = (int16_t)(Wire.read()|Wire.read()<<8) / 900.0;
  gyroY = (int16_t)(Wire.read()|Wire.read()<<8) / 900.0;
  gyroZ = (int16_t)(Wire.read()|Wire.read()<<8) / 900.0; 
}

//get data from the accelerometer
void accData() {
  Wire.beginTransmission(ADDR);
  Wire.write(ACC_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, 6, false);
  
  accY = (int16_t)(Wire.read()|Wire.read()<<8) / 100.0;
  accX = (int16_t)(Wire.read()|Wire.read()<<8) / 100.0;
  accZ = (int16_t)(Wire.read()|Wire.read()<<8) / 100.0;
}

//initialize the bno055
void bnoInitialize() {
  writeToBNO(0x07, 0x00); //change to page 0 of the register map
  
  delay(10);
  Wire.beginTransmission(ADDR); //begin talking to the BNO055
  Wire.write(CHIP_ID); //ask to talk to the CHIP_ID
  Wire.endTransmission(false); //ends the transmission, the false makes sure that no other master uses the BNO
  Wire.requestFrom(ADDR, 1, false); // request 1 byte from the BNO, sends true to release the BNO
  while(Wire.read() != 0xA0); //waits until the value returned is equal to the default value of the register

  writeToBNO(OPR_MODE, 0b00001000); //changes the operating mode to IMU
  
  writeToBNO(Page_ID, 0x01); //change to page 1 of the register map

  writeToBNO(ACC_Config, 0b00010011); //changes the accelerometer to normal power mode, 125hz refreshrate, and +- 16g

  writeToBNO(MAG_Config, 0b00001111); //changes the magnetometer to normal power, regular operation mode, and 30hz refreshrate

  writeToBNO(GYR_Config_0, 0b00010010); //changes the gyroscope to 116hz refreshrate and +- 500dps

  writeToBNO(GYR_Config_1, 0x00); //changes the gyroscope to normal power mode

  writeToBNO(Page_ID, 0x00); //change to page 0 of the register map
}
