#include <Wire.h>
#include "Gyro.h"
#include "Motor.h"
#include "Pwm.h"
#include "Bluetooth.h"
#include <PID_v1.h>

//Define Variables we'll be connecting to
double pitchSetpoint, pitchInput, pitchOutput;
double rollSetpoint, rollInput, rollOutput;

//Define the aggressive and conservative Tuning Parameters
double consKp = 1, consKi = 0.05, consKd = 0.25;
PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);

int targetSpeed[4];

double dataBuffer[10];

Gyro gyro;
double quaternionData[4];
double yprData[3];

Bluetooth bluetooth;

Motor motorA;
Motor motorB;
Motor motorC;
Motor motorD;

void setup() {
  Serial.begin(9600);
  initializeGyro();
  initializeMotors();

  pitchInput = 0.0;
  rollInput = 0.0;

  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;

  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);
}

void loop() {
  targetSpeed[0] = 10;
  targetSpeed[1] = 10;
  targetSpeed[2] = 10;
  targetSpeed[3] = 10;
  
  if (gyro.interruptReady()) {
    bool success = gyro.readGyroData(Gyro::DataType::YAWPITCHROLL, dataBuffer);
    if (success) {
      copyBuffer(dataBuffer, yprData, 3);

      pitchInput = yprData[1];
      rollInput = yprData[2];
      pitchPID.Compute();
      rollPID.Compute();

      int actSpeed[4];
      stabilise(targetSpeed, actSpeed, rollOutput, pitchOutput);

      runIndividual(actSpeed);
    }
  }
}

void runIndividual (int* actSpeed) {
  motorA.setSpeed(actSpeed[0]);
  motorB.setSpeed(actSpeed[1]);
  motorC.setSpeed(actSpeed[2]);
  motorD.setSpeed(actSpeed[3]);
}

void stabilise (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff) {
  actSpeed[0] = (int) currSpeed[0] + (rollDiff / 2) - (pitchDiff / 2);  //each motor has actual Speed and speed at which we want them to fly...
  actSpeed[1] = (int) currSpeed[1] + (rollDiff / 2) + (pitchDiff / 2);
  actSpeed[2] = (int) currSpeed[2] - (rollDiff / 2) + (pitchDiff / 2);  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  actSpeed[3] = (int) currSpeed[3] - (rollDiff / 2) - (pitchDiff / 2);

  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0 )
      actSpeed[i] = 0;
  }
}


void initializeGyro() {
  Serial.print("Enabling interrupt detection on interrupt 0...");
  attachInterrupt(0, handleGyroInterrupt, RISING);
  Serial.println(" done.");

  if (!gyro.initialize()) {
    Serial.println("Gyro initialization failed.");
  }
  if (!gyro.startDmp()) {
    Serial.println("Gyro DMP starting failed.");
  }
}

void initializeMotors() {
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  motorA.initialize(6);
  motorB.initialize(9);
  motorC.initialize(10);
  motorD.initialize(11);
}

void handleGyroInterrupt() {
  gyro.onDmpDataReady();
}

void copyBuffer(double from[], double to[], int length) {
  for (int i = 0; i < length; i++) {
    to[i] = from[i];
  }
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
