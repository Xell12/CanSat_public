#include "qmc5883l.h"
#include <pigpio.h>
#include <stdio.h>

// LSB per Gauss for range 2^15
static const float _lsb_per_gauss[] = {
    [QMC5883L_RANGE_2G] = 8192.0f,  
    [QMC5883L_RANGE_8G] = 2048.0f  
};

static bool writeByte(int i2c_handle, uint8_t reg, uint8_t data) {
    return (i2cWriteByteData(i2c_handle, reg, data) == 0);
}

static bool readBytes(int i2c_handle, uint8_t reg, char *data, unsigned len) {
    int r = i2cReadI2CBlockData(i2c_handle, reg, data, len);
    return (r == (int)len);
}

int qmc5883l_init(int bus, uint8_t addr) {
    int i2c_handle = i2cOpen(bus, addr, 0);
    if (i2c_handle < 0) {
        printf("QMC5883L: i2cOpen failed (%d)\n", i2c_handle);
    }
    return i2c_handle;
}

bool qmc5883l_config(int i2c_handle,
                     qmc5883l_osr_t   osr,
                     qmc5883l_range_t range,
                     qmc5883l_odr_t   odr,
                     qmc5883l_mode_t  mode)
{
    // CTRL1: [7:6]=OSR, [5:4]=RNG, [3:2]=ODR, [1:0]=MODE 
    uint8_t ctrl1 = ((osr   & 0x03) << 6)
                  | ((range & 0x03) << 4)
                  | ((odr   & 0x03) << 2)
                  | ((mode  & 0x03) << 0);
    if (! writeByte(i2c_handle, QMC5883L_REG_CTRL1, ctrl1)) return false;

    // CTRL2: soft-reset = bit0, no reset=0
    if (! writeByte(i2c_handle, QMC5883L_REG_CTRL2, 0x00)) return false;

    return true;
}

void qmc5883l_read(int i2c_handle, qmc5883l_data_t *values) {
    uint8_t st;
    if (! readBytes(i2c_handle, QMC5883L_REG_STATUS, (char*)&st, 1)) return;
    //bit0 = DRDY, bit1 = overflow
    if (! (st & 0x01)) return;
    if (st & 0x02) {
        printf("QMC5883L: data overflow\n");
    }

    uint8_t data[6];
    if (! readBytes(i2c_handle, QMC5883L_REG_OUT_X_LSB,(char*) data, 6)) return;

    int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);

    // extract range bits from last CTRL1
    uint8_t rng = (i2cReadByteData(i2c_handle, QMC5883L_REG_CTRL1) >> 4) & 0x03;
    float lsb = _lsb_per_gauss[rng];

    values->x = (float)rawX / lsb;
    values->y = (float)rawY / lsb;
    values->z = (float)rawZ / lsb;
}

void qmc5883l_close(int i2c_handle) {
    if (i2c_handle >= 0) i2cClose(i2c_handle);
}
