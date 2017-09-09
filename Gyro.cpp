#include "Gyro.h"

/* 
 * NOTE:
 * In addition to 3.3v, GND, SDA, and SCL connections, this depends on
 * MPU-6050 INT pin being connected to interrupt #0 (pin #2 on Arduino nano)
 */

void Gyro::onDmpDataReady() {
  mpuInterrupt = true;
}

/*
* Sets up the MPU Gyroscope controller.
* Returns false if setup fails.
*/
bool Gyro::initialize() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  mpu.initialize();

  // verify connection
  if (!mpu.testConnection()) {
  	return false;
  }
  return true;
}
  
// TODO have this be calibrated
int16_t * Gyro::getCalibratedOffsets() {
	// 0: x, 1: y, 2: z, 3: z_accel
  int16_t vals[] = {220, 76, 85, 1788};
	return vals;
}

  
bool Gyro::startDmp() {
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // initialize offsets
  int16_t * offsets = getCalibratedOffsets();
  mpu.setXGyroOffset(offsets[0]);
  mpu.setYGyroOffset(offsets[1]);
  mpu.setZGyroOffset(offsets[2]);
  mpu.setZAccelOffset(offsets[3]);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // INTERRUPT DETECTION ENABLE CODE HERE
      
      
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      return true;
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      return false;
  }
}

bool Gyro::interruptReady() {
	return mpuInterrupt || !(fifoCount >= packetSize);
}

void Gyro::getQuaternion(double * valueArray) {
	// 0: w, 1: x, 2: y, 3: z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  valueArray[0] = q.w;
  valueArray[1] = q.x;
  valueArray[2] = q.y;
  valueArray[3] = q.z;
}

void Gyro::getEuler(double * valueArray) {
  // Give Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  valueArray[0] = euler[0] * 180 / M_PI;
  valueArray[1] = euler[1] * 180/M_PI;
  valueArray[2] = euler[2] * 180/M_PI;
}

void Gyro::getYawPitchRoll(double * valueArray) {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  valueArray[0] = ypr[0] * 180/M_PI;
  valueArray[1] = ypr[1] * 180/M_PI;
  valueArray[2] = ypr[2] * 180/M_PI;
}

void Gyro::getRealAccel(double * valueArray) {
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  valueArray[0] = aaReal.x;
  valueArray[1] = aaReal.y;
  valueArray[2] = aaReal.z;
}

void Gyro::getWorldAccel(double * valueArray) {
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  valueArray[0] = aaWorld.x;
  valueArray[1] = aaWorld.y;
  valueArray[2] = aaWorld.z;
}

bool Gyro::getGyroData(DataType type, double * valueArray) {
  // if programming failed, don't try to do anything and check MPU interrupt or extra packet(s) available
	if (!dmpReady || !interruptReady()) {
    Serial.println("DMP not ready! Returning null.");
		return false;
	}

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow! Returning null."));
      return false;

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait

      // TODO examine this
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;


		// Quaternion: actual quaternion components in a [w, x, y, z] format (not best for parsing
		// on a remote host such as Processing or something though)

		// Euler: Euler angles (in degrees) calculated from the quaternions coming from the FIFO.
		// Note that Euler angles suffer from gimbal lock (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)

		// YawPitchRoll: Yaw/pitch/roll angles (in degrees) calculated from the quaternions coming
		// from the FIFO. Note this also requires gravity vector calculations.
		// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
		// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

		// RealAccel: acceleration components with gravity removed. This acceleration reference frame is
		// not compensated for orientation, so +X is always +X according to the
		// sensor, just without the effects of gravity. If you want acceleration
		// compensated for orientation, us WorldAccel instead.

		// WorldAccel: Acceleration components with gravity removed and adjusted for the world frame of
		// reference (yaw is relative to initial orientation, since no magnetometer
		// is present in this case). Could be quite handy in some cases.
      switch (type) {
      	case Gyro::DataType::QUATERNION:
      		getQuaternion(valueArray);
          return true;
      	case Gyro::DataType::EULERANGLE:
          getEuler(valueArray);
          return true;
      	case Gyro::DataType::YAWPITCHROLL:
      		getYawPitchRoll(valueArray);
          return true;
      	case Gyro::DataType::REALACCEL:
      		getRealAccel(valueArray);
          return true;
      	case Gyro::DataType::WORLDACCEL:
      		getWorldAccel(valueArray);
          return true;
      }
  }
  Serial.println("No correct type, so returning null.");
  return false;
}
