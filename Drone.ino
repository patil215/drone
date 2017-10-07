#include <Wire.h>
#include "Gyro.h"
#include "Motor.h"
#include "Pwm.h"
#include "Bluetooth.h"
#include <PID_v1.h>

// Define Variables we'll be connecting to
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
  targetSpeed[0] = 20;
  targetSpeed[1] = 20;
  targetSpeed[2] = 20;
  targetSpeed[3] = 20;
  
  bool gyroRead = gyro.readGyroData(Gyro::DataType::YAWPITCHROLL, dataBuffer);
  if (gyroRead) {
    double yprData[3];
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

void runIndividual (int * actSpeed) {
  motorA.setSpeed(actSpeed[0]);
  motorB.setSpeed(actSpeed[1]);
  motorC.setSpeed(actSpeed[2]);
  motorD.setSpeed(actSpeed[3]);
}

void stabilise (int * currSpeed, int * actSpeed, float rollDiff, float pitchDiff) {
  actSpeed[0] = (int) currSpeed[0] + (rollDiff / 2) - (pitchDiff / 2);  //each motor has actual Speed and speed at which we want them to fly...
  actSpeed[1] = (int) currSpeed[1] + (rollDiff / 2) + (pitchDiff / 2);
  actSpeed[2] = (int) currSpeed[2] - (rollDiff / 2) + (pitchDiff / 2);  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  actSpeed[3] = (int) currSpeed[3] - (rollDiff / 2) - (pitchDiff / 2);

  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0 ) {
      Serial.println("actspeed negative");
      actSpeed[i] = 0;
    }
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
