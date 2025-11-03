#include <stdio.h>
#include <pigpio.h>
#include <stdint.h>
#include "bmp280.h"

static bool writeByte(int i2c_handle, unsigned char reg, unsigned char data)
{
    int res = i2cWriteByteData(i2c_handle, reg, data);
    if (res != 0)
    {
        printf("Failed to WRITE byte to I2C device BMP: %d\n", res);
        return false;
    }
    return true;
}

static bool readBytes(int i2c_handle, unsigned char reg, unsigned char *data, int length)
{
    int res = i2cReadI2CBlockData(i2c_handle, reg, (char*) data, length);
    if (res < 0 || res != length)
    {
        printf("Failed to READ bytes from I2C device BMP: %d\n", res);
        return false;
    }
    return true;
}

int bmp280_init(int bus, uint8_t addr, bmp280_calib_t *cal)
{
    int i2c_handle = i2cOpen(bus, addr, 0);
    if (i2c_handle < 0)
    {
        printf("BMP280: i2cOpen failed (%d)\n", i2c_handle);
        return i2c_handle;
    }
    int who = i2cReadByteData(i2c_handle, BMP280_REG_ID);
    if (who < 0)
    {
        printf("BMP280: WHO_AM_I read error (%d)\n", who);
        i2cClose(i2c_handle);
        return who;
    }
    if (who != BMP280_CHIP_ID)
    {
        printf("BMP280: WHO_AM_I mismatch (0x%02X)\n", who);
        i2cClose(i2c_handle);
        return -2;
    }

    uint8_t buf[24];
    if (!readBytes(i2c_handle, BMP280_COMPENSATION_REGS, buf, 24)) {
      i2cClose(i2c_handle);
      return -3;
    }
    // Unpack into *cal little‑endian
    cal->dig_T1 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    cal->dig_T2 = (int16_t) buf[2] | ((int16_t) buf[3] << 8);
    cal->dig_T3 = (int16_t) buf[4] | ((int16_t) buf[5] << 8);
    cal->dig_P1 = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    cal->dig_P2 = (int16_t) buf[8] | ((int16_t) buf[9] << 8);
    cal->dig_P3 = (int16_t) buf[10]| ((int16_t) buf[11]<< 8);
    cal->dig_P4 = (int16_t) buf[12]| ((int16_t) buf[13]<< 8);
    cal->dig_P5 = (int16_t) buf[14]| ((int16_t) buf[15]<< 8);
    cal->dig_P6 = (int16_t) buf[16]| ((int16_t) buf[17]<< 8);
    cal->dig_P7 = (int16_t) buf[18]| ((int16_t) buf[19]<< 8);
    cal->dig_P8 = (int16_t) buf[20]| ((int16_t) buf[21]<< 8);
    cal->dig_P9 = (int16_t) buf[22]| ((int16_t) buf[23]<< 8);
    cal->t_fine = 0;  // initi

    return i2c_handle;
}

bool bmp280_config(int i2c_handle, uint8_t osrs_t, uint8_t osrs_p, uint8_t filter_coeff, uint8_t standby_ms) {
  // ctrl_meas: osrs_t[7:5], osrs_p[4:2], mode[1:0]=0b11 normal
  uint8_t ctrl = (osrs_t << 5) | (osrs_p << 2) | 0b11;
  if (!writeByte(i2c_handle, BMP280_REG_CTRL, ctrl)) return false;

  // config: t_sb[7:5], filter[4:2], spi3w_en[0]=0
  uint8_t cfg = (standby_ms << 5) | (filter_coeff << 2);
  if (!writeByte(i2c_handle, BMP280_REG_CONFIG, cfg)) return false;

  return true;
}


void bmp280_read(int i2c_handle, bmp280_calib_t *cal, bmp280_data_t *values)
{
    uint8_t data[6];
    if(!readBytes(i2c_handle, BMP280_REG_PRESS_MSB, data, 6)){printf("Failed to READ BMP280 data\n"); return;}
    int32_t adc_P = ((int32_t)data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // Temperature compensation
    double var1 = ((adc_T / 16384.0) - ((double)cal->dig_T1 / 1024.0))* (double)cal->dig_T2;
    double var2 = (((adc_T / 131072.0) - ((double)cal->dig_T1 / 8192.0))* ((adc_T / 131072.0) - ((double)cal->dig_T1 / 8192.0))) * (double)cal->dig_T3;

    cal->t_fine = (int32_t)(var1 + var2);
    values->temperature = (float)((var1 + var2) / 5120.0);
    
    // Pressure compensation
    var1 = ((double)cal->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * (double)cal->dig_P6 / 32768.0;
    var2 = var2 + var1 * (double)cal->dig_P5 * 2.0;
    var2 = (var2 / 4.0) + ((double)cal->dig_P4 * 65536.0);
    var1 = ((double)cal->dig_P3 * var1 * var1 / 524288.0 + (double)cal->dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * (double)cal->dig_P1;

    if (var1 == 0.0) {
        values->pressure = 0; //in case of div
        return;
    }
    double p = 1048576.0 - adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = (double)cal->dig_P9 * p * p / 2147483648.0;
    var2 = p * (double)cal->dig_P8 / 32768.0;

    values->pressure = (float)(p + (var1 + var2 + (double)cal->dig_P7) / 16.0);

}

void bmp280_close(int i2c_handle)
{
    if (i2c_handle >= 0)
    {
        i2cClose(i2c_handle);
    }
}
