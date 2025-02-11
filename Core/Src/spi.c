/*
 * spi.c
 *
 *  Created on: Feb 9, 2025
 *      Author: bryki
 */
#include "main.h"
#include "spi.h"

#define SPI_PORT_CLOCK  LL_IOP_GRP1_PERIPH_##port


void spi_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    GPIO_InitStruct.Pin = SPI_MOSI | SPI_SCLK | SPI_CS;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SPI_PORT, &GPIO_InitStruct);

    LL_GPIO_SetOutputPin(SPI_PORT, SPI_CS);

    GPIO_InitStruct.Pin = SPI_MISO;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SPI_PORT, &GPIO_InitStruct);
}

void spi_cs_activate(void)
{
    LL_GPIO_ResetOutputPin(SPI_PORT, SPI_CS);
}

void spi_cs_deactivate(void)
{
    LL_GPIO_SetOutputPin(SPI_PORT, SPI_CS);
}

uint8_t spi_transfer(uint8_t data)
{
    uint8_t mask = 0x80;
    uint8_t reg = 0;
    volatile uint8_t delay;

    while(mask)
    {
        //reg <<= 1;

        if(data & mask)
            LL_GPIO_SetOutputPin(SPI_PORT, SPI_MOSI);
        else
            LL_GPIO_ResetOutputPin(SPI_PORT, SPI_MOSI);

        for(delay = 0; delay < 20; delay++);

        LL_GPIO_SetOutputPin(SPI_PORT, SPI_SCLK);

        if(LL_GPIO_IsInputPinSet(SPI_PORT, SPI_MISO))
            reg |= mask;

        mask >>= 1;

        for(delay = 0; delay < 20; delay++);

        LL_GPIO_ResetOutputPin(SPI_PORT, SPI_SCLK);
    }

    LL_GPIO_ResetOutputPin(SPI_PORT, SPI_MOSI);

    return reg;
}

spi_status_t spi_tx_rx(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t size)
{
    uint16_t i;

    for(i = 0; i < size; i++)
        rx_buf[i] = spi_transfer(tx_buf[i]);

    return SPI_OK;
}

spi_status_t spi_tx(uint8_t *tx_buf, uint16_t size)
{
    uint16_t i;

    for(i = 0; i < size; i++)
        spi_transfer(tx_buf[i]);

    return SPI_OK;
}

spi_status_t spi_rx(uint8_t *rx_buf, uint16_t size)
{
    uint16_t i;

    for(i = 0; i < size; i++)
        rx_buf[i] = spi_transfer(0xff);

    return SPI_OK;
}
