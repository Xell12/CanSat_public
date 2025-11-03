#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>


#define MPU6050_ADDR        0x68 //I2C addr

///Registers
#define MPU_WHO_AM_I        0x75
#define MPU_PWR_MGMT_1      0x6B
#define MPU_PWR_MGMT_2      0x6C
#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C

#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_ACCEL_XOUT_L  0x3C
#define MPU_ACCEL_YOUT_H  0x3D
#define MPU_ACCEL_YOUT_L  0x3E
#define MPU_ACCEL_ZOUT_H  0x3F
#define MPU_ACCEL_ZOUT_L  0x40
#define MPU_GYRO_XOUT_H  0x43
#define MPU_GYRO_XOUT_L  0x44
#define MPU_GYRO_YOUT_H  0x45
#define MPU_GYRO_YOUT_L  0x46
#define MPU_GYRO_ZOUT_H  0x47
#define MPU_GYRO_ZOUT_L  0x48

typedef struct {
    double accel_x;  // x accel in m/s^2 
    double accel_y;  // y accel in m/s^2
    double accel_z;  // z accel in m/s^2 
    double gyro_x;   // x gyro in deg/s 
    double gyro_y;   // y gyro in deg/s 
    double gyro_z;   // z gyro in deg/s 
} mpu6050_data_t;



/* 
        Open I2C bus and verify MPU6050 
    bus   I²C bus number (e.g. 1)
    addr  MPU6050 address
 */
int  mpu6050_init(int bus, uint8_t addr);

/*
        Configure power management, DLPF, sample rate, accel & gyro ranges
    i2c_handle  pigpio I2C handle 
    dlpf_cfg    0–6 digital low‑pass filter setting
    smplrt_div  sample rate divider (rate = 1 kHz/(1+smplrt_div))
    accel_fs    accel full‑scale: 0= +-2g,1=+-4g,2=+-8g,3=±16g
    gyro_fs     gyro full‑scale:  0=+-250 deg/s,1=+-500 deg/s,2=+-1000 deg/s,3=+-2000 deg/s
 */
bool mpu6050_config(int i2c_handle, uint8_t dlpf_cfg, uint8_t smplrt_div, uint8_t accel_fs, uint8_t gyro_fs);
                       
/*
        Read one sensor sample (accelerometer + gyroscope).
    handle     pigpio I2C handle
    out        output data
 */
void mpu6050_read(int handle, mpu6050_data_t *out);

/*
        Close the I2C handle for the MPU6050.
    handle     pigpio I2C handle
 */
void mpu6050_close(int handle);
#endif // MPU6050_H
