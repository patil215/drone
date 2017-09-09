#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_6AXIS_MOTIONAPPS20_H_

#include "I2Cdev.h"
#include "helper_3dmath.h"

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include "MPU6050.h"

// Tom Carpenter's conditional PROGMEM code
// http://forum.arduino.cc/index.php?topic=129407.0
#ifdef __AVR__
    #include <avr/pgmspace.h>
#else
    // Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
    #ifndef __PGMSPACE_H_
        #define __PGMSPACE_H_ 1
        #include <inttypes.h>

        #define PROGMEM
        #define PGM_P  const char *
        #define PSTR(str) (str)
        #define F(x) x

        typedef void prog_void;
        typedef char prog_char;
        typedef unsigned char prog_uchar;
        typedef int8_t prog_int8_t;
        typedef uint8_t prog_uint8_t;
        typedef int16_t prog_int16_t;
        typedef uint16_t prog_uint16_t;
        typedef int32_t prog_int32_t;
        typedef uint32_t prog_uint32_t;
        
        #define strcpy_P(dest, src) strcpy((dest), (src))
        #define strcat_P(dest, src) strcat((dest), (src))
        #define strcmp_P(a, b) strcmp((a), (b))
        
        #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
        #define pgm_read_word(addr) (*(const unsigned short *)(addr))
        #define pgm_read_dword(addr) (*(const unsigned long *)(addr))
        #define pgm_read_float(addr) (*(const float *)(addr))
        
        #define pgm_read_byte_near(addr) pgm_read_byte(addr)
        #define pgm_read_word_near(addr) pgm_read_word(addr)
        #define pgm_read_dword_near(addr) pgm_read_dword(addr)
        #define pgm_read_float_near(addr) pgm_read_float(addr)
        #define pgm_read_byte_far(addr) pgm_read_byte(addr)
        #define pgm_read_word_far(addr) pgm_read_word(addr)
        #define pgm_read_dword_far(addr) pgm_read_dword(addr)
        #define pgm_read_float_far(addr) pgm_read_float(addr)
    #endif
#endif

//#define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]

extern const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM;
extern const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] PROGMEM;
extern const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE] PROGMEM;
uint8_t MPU6050::dmpInitialize();
bool MPU6050::dmpPacketAvailable();
uint8_t MPU6050::dmpGetAccel(int32_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetAccel(int16_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(int16_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q);
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
uint8_t MPU6050::dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);
uint16_t MPU6050::dmpGetFIFOPacketSize();

#endif /* _MPU6050_6AXIS_MOTIONAPPS20_H_ */
