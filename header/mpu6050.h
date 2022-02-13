/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  - most of the code is a port of the arduino mpu6050 library by Jeff Rowberg
    https://github.com/jrowberg/i2cdevlib
  - Mahony complementary filter for attitude estimation
    http://www.x-io.co.uk
*/


#ifndef MPU6050_H
#define MPU6050_H


//definitions
//enable the getattitude functions
//because we do not have a magnetometer, we have to start the chip always in the same position
//then to obtain your object attitude you have to apply the aerospace sequence
//0 disabled
//1 mahony filter
//2 dmp chip processor
#define MPU6050_GETATTITUDE 2

//definitions for raw data
//gyro and acc scale
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_2000
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_2

#define MPU6050_GYRO_LSB_250 131.0
#define MPU6050_GYRO_LSB_500 65.5
#define MPU6050_GYRO_LSB_1000 32.8
#define MPU6050_GYRO_LSB_2000 16.4
#if MPU6050_GYRO_FS == MPU6050_GYRO_FS_250
#define MPU6050_GGAIN MPU6050_GYRO_LSB_250
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_500
#define MPU6050_GGAIN MPU6050_GYRO_LSB_500
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_1000
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_2000
#endif

#define MPU6050_ACCEL_LSB_2 16384.0
#define MPU6050_ACCEL_LSB_4 8192.0
#define MPU6050_ACCEL_LSB_8 4096.0
#define MPU6050_ACCEL_LSB_16 2048.0
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_2
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_4
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_8
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_16
#endif

#define MPU6050_CALIBRATEDACCGYRO 1 //set to 1 if is calibrated
#if MPU6050_CALIBRATEDACCGYRO == 1
#define MPU6050_AXOFFSET 0
#define MPU6050_AYOFFSET 0
#define MPU6050_AZOFFSET 0
#define MPU6050_AXGAIN 16384.0
#define MPU6050_AYGAIN 16384.0
#define MPU6050_AZGAIN 16384.0
#define MPU6050_GXOFFSET -42
#define MPU6050_GYOFFSET 9
#define MPU6050_GZOFFSET -29
#define MPU6050_GXGAIN 16.4
#define MPU6050_GYGAIN 16.4
#define MPU6050_GZGAIN 16.4
#endif


void initMPU6050_DMP(void);


#if MPU6050_GETATTITUDE == 2
//dmp definitions
//packet size
#define MPU6050_DMP_dmpPacketSize 42
//define INT0 rise edge interrupt
#define MPU6050_DMP_INT0SETUP EICRB |= (1<<ISC61) | (1<<ISC60) //<--------------------------------- change to INT6
//define enable and disable INT0 rise edge interrupt
#define MPU6050_DMP_INT0DISABLE XIntruptRegs.XINT1CR.bit.ENABLE = 0			//<--------------------------------- change to INT6
#define MPU6050_DMP_INT0ENABLE XIntruptRegs.XINT1CR.bit.ENABLE = 1				//<--------------------------------- change to INT6
extern volatile Uint16 mpu6050_mpuInterrupt;
#endif

//functions
extern void mpu6050_init();
extern Uint16 mpu6050_testConnection();

extern void mpu6050_setSleepDisabled();
extern void mpu6050_setSleepEnabled();

extern int16 mpu6050_readBytes(Uint16 regAddr, Uint16 length, Uint16 *data);
extern int16 mpu6050_readByte(Uint16 regAddr, Uint16 *data);
extern void mpu6050_writeBytes(Uint16 regAddr, Uint16 length, Uint16* data);
extern void mpu6050_writeByte(Uint16 regAddr, Uint16 data);
extern int16 mpu6050_readBits(Uint16 regAddr, Uint16 bitStart, Uint16 length, Uint16 *data);
extern int16 mpu6050_readBit(Uint16 regAddr, Uint16 bitNum, Uint16 *data);
extern void mpu6050_writeBits(Uint16 regAddr, Uint16 bitStart, Uint16 length, Uint16 data);
extern void mpu6050_writeBit(Uint16 regAddr, Uint16 bitNum, Uint16 data);


#if MPU6050_GETATTITUDE == 2
extern void mpu6050_writeWords(Uint16 regAddr, Uint16 length, Uint16* data);
extern void mpu6050_setMemoryBank(Uint16 bank, Uint16 prefetchEnabled, Uint16 userBank);
extern void mpu6050_setMemoryStartAddress(Uint16 address);
extern void mpu6050_readMemoryBlock(Uint16 *data, Uint16 dataSize, Uint16 bank, Uint16 address);
extern Uint16 mpu6050_writeMemoryBlock(const Uint16 *data, Uint16 dataSize, Uint16 bank, Uint16 address, Uint16 verify, Uint16 useProgMem);
extern Uint16 mpu6050_writeDMPConfigurationSet(const Uint16 *data, Uint16 dataSize, Uint16 useProgMem);
extern Uint16 mpu6050_getFIFOCount();
extern void mpu6050_getFIFOBytes(Uint16 *data, Uint16 length);
extern Uint16 mpu6050_getIntStatus();
extern void mpu6050_resetFIFO();
extern int16 mpu6050_getXGyroOffset();
extern void mpu6050_setXGyroOffset(int16 offset);
extern int16 mpu6050_getYGyroOffset();
extern void mpu6050_setYGyroOffset(int16 offset);
extern int16 mpu6050_getZGyroOffset();
extern void mpu6050_setZGyroOffset(int16 offset);
//base dmp
extern Uint16 mpu6050_dmpInitialize();
extern void mpu6050_dmpEnable();
extern void mpu6050_dmpDisable();
extern void mpu6050_getQuaternion(const Uint16* packet, float *qw, float *qx, float *qy, float *qz);
extern void mpu6050_getRollPitchYaw(float qw, float qx, float qy, float qz, float *roll, float *pitch, float *yaw);
extern Uint16 mpu6050_getQuaternionWait(float *qw, float *qx, float *qy, float *qz);
#endif

#endif
