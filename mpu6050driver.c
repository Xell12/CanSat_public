#include <stdio.h>
#include <pigpio.h>
#include <stdint.h>
#include "mpu6050.h"

static float accel_sens_div   = 8192.0f;  // raw units per g
static float gyro_sens_div    = 65.5f;    // raw units per deg/s


static bool writeByte(int i2c_handle, unsigned char reg, unsigned char data)
{
    int res = i2cWriteByteData(i2c_handle, reg, data);
    if (res != 0)
    {
        printf("Failed to WRITE byte to I2C device MPU6050: %d\n", res);
        return false;
    }
    return true;
}

static bool readBytes(int i2c_handle, unsigned char reg, unsigned char *data, int length)
{
    int res = i2cReadI2CBlockData(i2c_handle, reg, (char*) data, length);
    if (res < 0 || res != length)
    {
        printf("Failed to READ bytes from I2C device MPU6050: %d\n", res);
        return false;
    }
    return true;
}
//MPU6050 initialization
int mpu6050_init(int bus, uint8_t addr)
{
    int i2c_handle = i2cOpen(bus, addr, 0);
    if (i2c_handle < 0)
    {
        printf("MPU6050: i2cOpen failed (%d)\n", i2c_handle);
        return i2c_handle;
    }
    int who = i2cReadByteData(i2c_handle, MPU_WHO_AM_I);
    if (who < 0)
    {
        printf("MPU6050: WHO_AM_I read error (%d)\n", who);
        i2cClose(i2c_handle);
        return who;
    }
    if (who != MPU6050_ADDR)
    {
        printf("MPU6050: WHO_AM_I mismatch (0x%02X)\n", who);
        i2cClose(i2c_handle);
        return -2;
    }

    return i2c_handle;
}

//MPU6050 configuration
bool mpu6050_config(int i2c_handle, uint8_t dlpf_cfg, uint8_t smplrt_div, uint8_t accel_fs, uint8_t gyro_fs)
{
    // Wake up & disable sleep 
    if (!writeByte(i2c_handle, MPU_PWR_MGMT_1, 0b00000001)) return false;
    if (!writeByte(i2c_handle, MPU_PWR_MGMT_2, 0b00000000)) return false;

    // DLPF (mask lower 3 bits: 0b00000111) 
    if (dlpf_cfg > 6) dlpf_cfg = 6;
    if (!writeByte(i2c_handle, MPU_CONFIG, dlpf_cfg & 0b00000111)) return false;

    //Sample rate divider
    if (!writeByte(i2c_handle, MPU_SMPLRT_DIV, smplrt_div)) return false;

    // Accel FS_SEL: bits 4:3 of ACCEL_CONFIG (mask 0b00000011) then shift 
    accel_fs &= 0b00000011;
    if (!writeByte(i2c_handle, MPU_ACCEL_CONFIG, (accel_fs << 3))) return false;

    // Gyro FS_SEL: bits 4:3 of GYRO_CONFIG (mask 0b00000011) then shift
    gyro_fs &= 0b00000011;
    if (!writeByte(i2c_handle, MPU_GYRO_CONFIG, (gyro_fs << 3))) return false;

    switch (accel_fs) {
        case 0: accel_sens_div = 16384.0f; break;  // +-2g
        case 1: accel_sens_div = 8192.0f;  break;  // +-4g
        case 2: accel_sens_div = 4096.0f;  break;  // +-8g
        case 3: accel_sens_div = 2048.0f;  break;  // +-16g
    }
    switch (gyro_fs) {
        case 0: gyro_sens_div  = 131.0f;   break;  // +-250 deg/s
        case 1: gyro_sens_div  = 65.5f;    break;  // +-500 deg/s
        case 2: gyro_sens_div  = 32.8f;    break;  // +-1000 deg/s
        case 3: gyro_sens_div  = 16.4f;    break;  // +-2000 deg/s
    }
    printf("MPU6050 configured: DLPF=%d, SMPLRT_DIV=%d, ACCEL_FS=%d, GYRO_FS=%d\n",
           dlpf_cfg, smplrt_div, accel_fs, gyro_fs);
    printf("Accel Sensitivity: %.2f raw units/g, Gyro Sensitivity: %.2f raw units/°/s\n",accel_sens_div, gyro_sens_div);
    return true;
}

//MPU6050 read data
void mpu6050_read(int i2c_handle, mpu6050_data_t *values)
{
    unsigned char data[14];
    if (!readBytes(i2c_handle, MPU_ACCEL_XOUT_H, data, 14)){printf("Failed to READ MPU6050 data\n"); return;}

    int16_t raw_accel_x = (data[0] << 8) | data[1];
    int16_t raw_accel_y = (data[2] << 8) | data[3];
    int16_t raw_accel_z = (data[4] << 8) | data[5];
    int16_t raw_gyro_x = (data[8] << 8) | data[9];
    int16_t raw_gyro_y = (data[10] << 8) | data[11];
    int16_t raw_gyro_z = (data[12] << 8) | data[13];

    const double g_mps2 = 9.80665; // Gravity in m/s^2
    // Convert raw values to physical units
    values->accel_x = (double) raw_accel_x / accel_sens_div * g_mps2; // Convert from (raw/div) [g]  to [m/s^2]
    values->accel_y = (double) raw_accel_y / accel_sens_div * g_mps2; // Convert from (raw/div) [g]  to [m/s^2]
    values->accel_z = (double) raw_accel_z / accel_sens_div * g_mps2; // Convert from (raw/div) [g]  to [m/s^2]
    values->gyro_x = (double) raw_gyro_x / gyro_sens_div; // Convert to degrees/s
    values->gyro_y = (double) raw_gyro_y / gyro_sens_div; // Convert to degrees/s 
    values->gyro_z = (double) raw_gyro_z / gyro_sens_div; // Convert to degrees



}

void mpu6050_close(int i2c_handle)
{
    if (i2c_handle >= 0)
    {
        i2cClose(i2c_handle);
    }
}
