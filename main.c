#include <stdio.h>
#include <pigpio.h>
#include <signal.h>
#include <stdbool.h>
#include "mpu6050.h"
#include "bmp280.h"
#include "qmc5883l.h"

static volatile bool isRunning = true;

void signalHandler(int signal)
{
    printf("Signal received\n");
    isRunning = false;
}

int main(void)
{

    const unsigned int i2c_bus = 1;
    int pigpio_res;

    unsigned int cfg = gpioCfgGetInternals();
    cfg |= PI_CFG_NOSIGHANDLER;
    gpioCfgSetInternals(cfg);

    signal(SIGINT, signalHandler);

    printf("Initializing pigpio...\n");
    pigpio_res = gpioInitialise();
    if (pigpio_res == PI_INIT_FAILED)
    {
        printf("pigpio init failed: %d\n", pigpio_res);
        gpioTerminate();
        return -1;
    }
    //sensos init
 
    int i2c_handle = mpu6050_init(i2c_bus, MPU6050_ADDR);
    if (i2c_handle < 0) {
        printf("mpu6050_init failed: %d\n", i2c_handle);
        mpu6050_close(i2c_handle);
        gpioTerminate();
        return 1;
    }
    bmp280_calib_t cal;
    int i2c_handle1 = bmp280_init(1, BMP280_ADDR, &cal);
    if (i2c_handle1 < 0) {
        printf("bmp280_init failed: %d\n", i2c_handle1);
        bmp280_close(i2c_handle1);
        gpioTerminate();
        return 1;
    }
    int i2c_handle2 = qmc5883l_init(i2c_bus, QMC5883L_ADDR);
    if (i2c_handle2 < 0) {
        printf("qmc5883l_init failed: %d\n", i2c_handle2);
        qmc5883l_close(i2c_handle2);
        gpioTerminate();
        return 1;
    }
    /*sensor configuration
    
     dlpf_cfg
     smplrt_div
     accel_fs=±4g
      gyro_fs=±500°/s */
    if (!mpu6050_config(i2c_handle,2,9,1,1))  
    {
        printf("MPU6050 config failed\n");
        mpu6050_close(i2c_handle);
        gpioTerminate();
        return 1;
    }
    printf("MPU6050 initialized and configured.\n");
    /*osrs_t=
    osrs_p=
    filter=
    standby=*/
    if (!bmp280_config(i2c_handle1,1,4,4,2)) {
        printf("bmp280_config failed\n");
        bmp280_close(i2c_handle1);
        gpioTerminate();
        return 1;
    }
    if (!qmc5883l_config(i2c_handle2,QMC5883L_OSR_512,QMC5883L_RANGE_2G,QMC5883L_ODR_100HZ,QMC5883L_MODE_CONTINUOUS))
    {
        printf("qmc5883l_config failed\n");
        qmc5883l_close(i2c_handle2);
        gpioTerminate();
        return 1;
    }

    //sensor data reading loop
    mpu6050_data_t mpu;
    bmp280_data_t bmp;
    qmc5883l_data_t qmc;
    while (isRunning) {
        mpu6050_read(i2c_handle, &mpu);
        printf("Accel: X=%.2f m/s2, Y=%.2f m/s2, Z=%.2f m/s2 | Gyro: X=%.2f deg/s, Y=%.2f deg/s, Z=%.2f deg/s\n",
               mpu.accel_x, mpu.accel_y, mpu.accel_z,
               mpu.gyro_x, mpu.gyro_y, mpu.gyro_z);
        bmp280_read(i2c_handle1, &cal, &bmp);
        printf("BMP280: Temperature=%.2f degC, Pressure=%.2f Pa\n",
               bmpt.temperature, bmp.pressure);
        qmc5883l_read(i2c_handle2, &qmc);
        printf("Mag (Gauss):  X = %.3f,  Y = %.3f,  Z = %.3f\n",
               qmc.x, qmc.y, qmc.z);
        time_sleep(3);
    }
    
    mpu6050_close(i2c_handle);
    bmp280_close(i2c_handle1);
    qmc5883l_close(i2c_handle2);
    gpioTerminate();
    return 0;
}
