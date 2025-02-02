/*
 * aht20.h
 *
 *  Created on: 15 янв. 2025 г.
 *      Author: bryki
 */

#ifndef INC_AHT20_H_
#define INC_AHT20_H_

typedef enum
{
    AHT20_OK,
    AHT20_NACK,
    AHT20_ERROR
} aht20_status_t;

aht20_status_t aht20_Init(I2C_HandleTypeDef *h);
aht20_status_t aht20_Reset(void);
aht20_status_t aht20_Measure(void);
int16_t aht20_GetTemp(void);
uint16_t aht20_GetHumidity(void);

#endif /* INC_AHT20_H_ */
