/*
 * spi.h
 *
 *  Created on: Feb 9, 2025
 *      Author: bryki
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#include <stddef.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------------*/
typedef enum spi_status_t
{
    SPI_OK = 0, // 0: Successful
    SPI_ERROR,  // 1: R/W Error
} spi_status_t;

/* ---------------------------------------------------------------------------------------------------------*/
#define SPI_PORT                        GPIOA
#define SPI_MOSI                        LL_GPIO_PIN_7
#define SPI_MISO                        LL_GPIO_PIN_6
#define SPI_SCLK                        LL_GPIO_PIN_5
#define SPI_CS                          LL_GPIO_PIN_4


void spi_init(void);

void spi_cs_activate(void);
void spi_cs_deactivate(void);
uint8_t spi_transfer(uint8_t data);

spi_status_t spi_rx(uint8_t *rx_buf, uint16_t size);
spi_status_t spi_tx(uint8_t *tx_buf, uint16_t size);
spi_status_t spi_tx_rx(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t size);
#endif /* SRC_SPI_H_ */
