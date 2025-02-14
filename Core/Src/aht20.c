/*
 * aht20.c
 *
 *  Created on: 15 янв. 2025 г.
 *      Author: bryki
 */
#include "main.h"
#include "aht20.h"

enum REG{
  DEFAULT_ADDRESS = 0x70,//0x38,
  CMD_CALIBRATE = 0xE1,     ///< Calibration command
  CMD_INITIALIZE = 0xBE,     ///< Calibration command
  CMD_TRIGGER = 0xAC,       ///< Trigger reading command
  CMD_SOFTRESET = 0xBA     ///< Soft reset command
};

I2C_HandleTypeDef *hi2c;

volatile int32_t temperature;
volatile uint32_t humidity;

aht20_status_t aht20_write(uint8_t *buf, uint8_t size);
aht20_status_t aht20_read(uint8_t *buf, uint8_t size);


aht20_status_t aht20_Init(I2C_HandleTypeDef *h)
{
    uint8_t buf[4];

    hi2c = h;

    // проверка связи с датчиком
    if(aht20_write(0, 0) != AHT20_OK)
        return AHT20_NACK;

    aht20_read(buf,1);

    //check if calibrated
    if((buf[0] & 0x08) == 0)
    {
        buf[0] = CMD_INITIALIZE;
        buf[1] = 0x08;
        buf[2] = 0;
        aht20_write(buf, 3);

        HAL_Delay(20);

        buf[0] = CMD_TRIGGER;
        buf[1] = 0x33;
        buf[2] = 0x00;
        aht20_write(buf, 3);

        HAL_Delay(80);
        aht20_read(buf,1);
    }
//    //Check if the calibrated bit is set. If not, init the sensor.
//    HAL_I2C_Master_Receive(hi2c, 0x38<<1, buf, 1, 100);
//
//    if((buf[0] & 0x80) == 0)
//    {
//        buf[0] = CMD_INITIALIZE;
//        buf[1] = 0x08;
//        buf[2] = 0;
//        HAL_I2C_Master_Transmit(hi2c, 0x38<<1, buf, 3, 100);
//
//
//
//        HAL_I2C_Master_Transmit(hi2c, 0x38<<1, buf, 3, 100);
//        HAL_Delay(80);
//
//        HAL_I2C_Master_Receive(hi2c, 0x38<<1, buf, 1, 100);
//    }

//
//    if (!getStatus(BIT_CALIBRATED)){
//        initialize();
//    delay(10);
//    triggerMeasurement();
//    delay(80); //Wait for measurement to complete
//    uint8_t counter = 0;
//    while (getStatus(BIT_BUSY)){
//    delay(1);
//    if (counter++ > 100) return false;
//    }
//    if (!getStatus(BIT_CALIBRATED)) return false;
//    }

    return (buf[1] & 0x80) ? AHT20_OK : AHT20_ERROR;
}

aht20_status_t aht20_Reset(void)
{
    uint8_t cmd = CMD_SOFTRESET;

    return aht20_write(&cmd, 1);
}

aht20_status_t aht20_Measure(void)
{
    aht20_status_t ret;
    uint8_t buf[7];

    buf[0] = CMD_TRIGGER;
    buf[1] = 0x33;
    buf[2] = 0x00;
    ret = aht20_write(buf, 3);

    if(ret != AHT20_OK)
        return ret;

    HAL_Delay(80);

    ret = aht20_read(buf, 7);
    if(ret != AHT20_OK)
        return ret;

    temperature = ((uint32_t )buf[3] << 16) | ((uint32_t )buf[4] << 8) | (uint32_t )buf[5];
    temperature &= 0x000FFFFF;

    temperature =  (temperature * 2000/ 1048576) - 500;

    humidity = ((uint32_t )buf[1] << 12) | ((uint32_t )buf[2] << 4) | ((uint32_t )buf[3] >> 4);
    humidity = (humidity * 1000/ 1048576);

    return ret;
}

int16_t aht20_GetTemp(void)
{
    return (int16_t )temperature;
}

uint16_t aht20_GetHumidity(void)
{
    return (uint16_t )humidity;
}

aht20_status_t aht20_write(uint8_t *buf, uint8_t size)
{
    uint32_t err;

    HAL_I2C_Master_Transmit(hi2c, DEFAULT_ADDRESS, buf, size, 100);

    err = HAL_I2C_GetError(hi2c);

    if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF)
    {
        return AHT20_NACK;
    }
    else if (err != HAL_I2C_ERROR_NONE)
    {
        return AHT20_ERROR;
    }

    return AHT20_OK;
}

aht20_status_t aht20_read(uint8_t *buf, uint8_t size)
{
    uint32_t err;

    HAL_I2C_Master_Receive(hi2c, DEFAULT_ADDRESS, buf, size, 100);

    err = HAL_I2C_GetError(hi2c);

    if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF)
    {
        return AHT20_NACK;
    }
    else if (err != HAL_I2C_ERROR_NONE)
    {
        return AHT20_ERROR;
    }

    return AHT20_OK;
}
