#include <Wire.h>
#include "Gyro.h"

Gyro gyro;

double gyroValues[4];
int i = 0;

void handleGyroInterrupt() {
  gyro.onDmpDataReady();
}

void enableGyroInterrupt() {
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
  attachInterrupt(0, handleGyroInterrupt, RISING);
  Serial.println("Interrupt attached");
}

void setup() {
  Serial.begin(9600);
  enableGyroInterrupt();
  if (!gyro.initialize()) {
    Serial.println("Gyro initialization failed!");
  }
  if (!gyro.startDmp()) {
    Serial.println("Gyro DMP starting failed!");
  }
}

void loop() {
  if (gyro.interruptReady()) {
    bool result = gyro.getGyroData(Gyro::DataType::QUATERNION, gyroValues);
    if (!result) {
      Serial.println("Failed to get Gyro data");
    }
    if ((i % 500) == 0) {
      Serial.println(gyroValues[0]);
      Serial.println(gyroValues[1]);
      Serial.println(gyroValues[2]);
      Serial.println(gyroValues[3]);
      Serial.println("");
      i = 0;
    }
    i += 1;
  }
  delay(1);
}












/*

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <helper_3dmath.h>


// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t X_offset = 155;
int16_t Y_offset = -690;
int16_t Z_offset = -30;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read() + X_offset;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read() + Y_offset;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read() + Z_offset;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(333);
}






/*
//Adafruit_MMA8451 mma = Adafruit_MMA8451();

char blueToothVal;           //value sent over via bluetooth
char lastValue;              //stores last state of device (on/off)

void setup()
{
 Serial.begin(9600);

 if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
 pinMode(13,OUTPUT);

  //mma.setRange(MMA8451_RANGE_2_G);
  
  //Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  //Serial.println("G");
}

void loop()
{
  // BLUETOOTH CODE
  if(Serial.available())
  {//if there is data being recieved
    blueToothVal=Serial.read(); //read it
  }
  if (blueToothVal=='n')
  {//if value from bluetooth serial is n
    digitalWrite(13,HIGH);            //switch on LED
    if (lastValue!='n')
      Serial.println(F("LED is on")); //print LED is on
    lastValue=blueToothVal;
  }
  else if (blueToothVal=='f')
  {//if value from bluetooth serial is n
    digitalWrite(13,LOW);             //turn off LED
    if (lastValue!='f')
      Serial.println(F("LED is off")); //print LED is on
    lastValue=blueToothVal;
  }
}*/
/*
// ACCELEROMETER

  // Read the 'raw' data in 14-bit counts
  mma.read();
  Serial.print("X:\t"); Serial.print(mma.x); 
  Serial.print("\tY:\t"); Serial.print(mma.y); 
  Serial.print("\tZ:\t"); Serial.print(mma.z); 
  Serial.println();

  // Get a new sensor event
  sensors_event_t event; 
  mma.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2) 
  Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");
  
  // Get the orientation of the sensor
  uint8_t o = mma.getOrientation();
  
  switch (o) {
    case MMA8451_PL_PUF: 
      Serial.println("Portrait Up Front");
      break;
    case MMA8451_PL_PUB: 
      Serial.println("Portrait Up Back");
      break;    
    case MMA8451_PL_PDF: 
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB: 
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF: 
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB: 
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF: 
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB: 
      Serial.println("Landscape Left Back");
      break;
    }
  Serial.println();
  delay(500);
  */
//}*/
