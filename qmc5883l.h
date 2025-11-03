#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdint.h>
#include <stdbool.h>

// QMC5883L I2C address 
#define QMC5883L_ADDR          0x0D

// Register map 
#define QMC5883L_REG_OUT_X_LSB 0x00
#define QMC5883L_REG_OUT_X_MSB 0x01
#define QMC5883L_REG_OUT_Y_LSB 0x02
#define QMC5883L_REG_OUT_Y_MSB 0x03
#define QMC5883L_REG_OUT_Z_LSB 0x04
#define QMC5883L_REG_OUT_Z_MSB 0x05
#define QMC5883L_REG_STATUS    0x06
#define QMC5883L_REG_CTRL1     0x09
#define QMC5883L_REG_CTRL2     0x0A

// Oversampling ratio (OSR) 
typedef enum {
    QMC5883L_OSR_512 = 0,  // 00b = ×512
    QMC5883L_OSR_256 = 1,  // 01b = ×256
    QMC5883L_OSR_128 = 2,  // 10b = ×128
    QMC5883L_OSR_64  = 3   // 11b = ×64
} qmc5883l_osr_t;

// Full‐scale range 
typedef enum {
    QMC5883L_RANGE_2G  = 0,  // 00b = +-2 Gauss
    QMC5883L_RANGE_8G  = 1   // 01b = +-8 Gauss
} qmc5883l_range_t;

// Output data rate (ODR)
typedef enum {
    QMC5883L_ODR_10HZ  = 0,  // 00b = 10hz
    QMC5883L_ODR_50HZ  = 1,  // 01b = 50hz
    QMC5883L_ODR_100HZ = 2,  // 10b = 100hz
    QMC5883L_ODR_200HZ = 3   // 11b = 200hz
} qmc5883l_odr_t;

// Operating mode
typedef enum {
    QMC5883L_MODE_STANDBY   = 0,  // 00b 
    QMC5883L_MODE_CONTINUOUS= 1   // 01b
} qmc5883l_mode_t;

// Magnetometer sample (Gauss)
typedef struct {
    float x;
    float y;
    float z;
} qmc5883l_data_t;

/*
        Initialize the QMC5883L
    bus   I2C bus number
    addr  QMC5883L_ADDR
 */
int  qmc5883l_init(int bus, uint8_t addr);

/*
        Configure oversampling, range, data rate, and mode
    handle   pigpio I2C handle
    osr      oversampling ratio
    range    +-2g / +-8g
    odr      output data rate
    mode     continuous / standby
 */
bool qmc5883l_config(int i2c_handle,
                     qmc5883l_osr_t   osr,
                     qmc5883l_range_t range,
                     qmc5883l_odr_t   odr,
                     qmc5883l_mode_t  mode);

/*
        Read one magnetometer sample.
    handle   pigpio I2C handle
    values      output data
 */
void qmc5883l_read(int i2c_handle, qmc5883l_data_t *values);

/*
        Close the I2C handle
 handle   pigpio I2C handle
 */
void qmc5883l_close(int i2c_handle);

#endif
