#ifndef _GYRO_H_
#define _GYRO_H_

#include "MPU6050_6Axis_MotionApps20.h"

class Gyro {

  public:
  	bool initialize();
  	bool startDmp();
    
    enum class DataType {
      /* Actual quaternion components in a [w, x, y, z] format (not best for parsing
      on a remote host such as Processing or something) */
    	QUATERNION, 

      /* Euler angles (in degrees) calculated from the quaternions coming from the FIFO.
      Note that Euler angles suffer from gimbal lock */
      EULERANGLE,

      /* Yaw/pitch/roll angles (in degrees) calculated from the quaternions coming
      from the FIFO. Note this also requires gravity vector calculations.
      Also note that yaw/pitch/roll angles suffer from gimbal lock (for
      more info, see: http://en.wikipedia.org/wiki/Gimbal_lock) */
      YAWPITCHROLL,

      /* Acceleration components with gravity removed. This acceleration reference frame is
      not compensated for orientation, so +X is always +X according to the
      sensor, just without the effects of gravity. If you want acceleration
      compensated for orientation, us WorldAccel instead. */
      REALACCEL,

      /*Acceleration components with gravity removed and adjusted for the world frame of
      reference (yaw is relative to initial orientation, since no magnetometer
      is present in this case). Could be quite handy in some cases. */
      WORLDACCEL
    };

    bool interruptReady();
    void onDmpDataReady();
    bool readGyroData(DataType type, double * valueArray);

  private:
    MPU6050 mpu;	  
  	// packet structure for InvenSense teapot demo
  	uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
  
  	// MPU control/status vars
  	bool dmpReady = false;  // set true if DMP init was successful
  	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  	uint16_t fifoCount;     // count of all bytes currently in FIFO
  	uint8_t fifoBuffer[64]; // FIFO storage buffer
  
  	// orientation/motion vars
  	Quaternion q;           // [w, x, y, z]         quaternion container
  	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  	VectorFloat gravity;    // [x, y, z]            gravity vector
  	float euler[3];         // [psi, theta, phi]    Euler angle container
  	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  
  	volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  
  	int16_t * getCalibratedOffsets();
    void readQuaternion(double * valueArray);
    void readEuler(double * valueArray);
    void readYawPitchRoll(double * valueArray);
    void readRealAccel(double * valueArray);
    void readWorldAccel(double * valueArray);

};
#endif
