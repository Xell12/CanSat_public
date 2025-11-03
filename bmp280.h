#ifndef BMP280_H
#define BMP280_H

#include <stdbool.h>
#include <stdint.h>
#define BMP280_ADDR 0x76   // or 0x77
#define BMP280_CHIP_ID     0x58
#define BMP280_COMPENSATION_REGS 0x88 // 24 bytes of calib data
#define BMP280_REG_ID      0xD0
#define BMP280_REG_RESET   0xE0
#define BMP280_REG_STATUS  0xF3
#define BMP280_REG_CTRL    0xF4
#define BMP280_REG_CONFIG  0xF5
#define BMP280_REG_PRESS_MSB 0xF7  // 0xF7..0xF9  pressure
#define BMP280_REG_PRESS_LSB 0xF8 
#define BMP280_REG_PRESS_XLSB 0xF9  
#define BMP280_REG_TEMP_MSB  0xFA  // 0xFA..0xFC  temperature
#define BMP280_REG_TEMP_LSB  0xFB 
#define BMP280_REG_TEMP_XLSB  0xFC  

typedef struct {
  // Calibration coefficients from 0x88..0x9F
  uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
  uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
  int16_t  dig_P4; int16_t dig_P5; int16_t dig_P6;
  int16_t  dig_P7; int16_t dig_P8; int16_t dig_P9;
  int32_t  t_fine;           // intermediate for compensation
} bmp280_calib_t;

typedef struct {
  float temperature;  // deg C
  float pressure;     // Pa
} bmp280_data_t;

/*
    Open I2C, verify ID, and read calibration into *cal.
  bus   I2C bus number
  addr  BMP280 I2C address
  cal   pointer to a bmp280_calib_t to fill
  */
int  bmp280_init(int bus, uint8_t addr, bmp280_calib_t *cal);

/*
  Configure oversampling & IIR filter
 handle        pigpio I2C handle
 osrs_t        temp oversampling (0–5)
 osrs_p        pressure oversampling (0–5)
 filter_coeff  IIR filter (0–4)
 standby_ms    standby time in ms (0–7)
 */
bool bmp280_config(int i2c_handle, uint8_t osrs_t, uint8_t osrs_p, uint8_t filter_coeff, uint8_t standby_ms);

/*
  Read & compensate temperature and pressure
handle    pigpio I2C handle
cal       calibration data from bmp280_init
values    output data
*/
void bmp280_read(int i2c_handle, bmp280_calib_t *cal, bmp280_data_t *values);

//Close I2C handle
void bmp280_close(int i2c_handle);

#endif 
