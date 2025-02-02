/*
 * wire.c
 *
 *  Created on: 14 янв. 2025 г.
 *      Author: bryki
 */
#include "Wire.h"
#include "twi.h"

uint8_t rxBuffer[32];
uint16_t rxBufferAllocated;
uint16_t rxBufferIndex;
uint16_t rxBufferLength;

uint8_t txAddress;
uint8_t txBuffer[32];
uint16_t txBufferAllocated;
uint16_t txDataSize;

uint8_t transmitting;

uint8_t ownAddress;
i2c_t _i2c;

void wire_begin(uint8_t address)
{
  rxBufferIndex = 0;
  rxBufferLength = 0;
  resetRxBuffer();

  txDataSize = 0;
  txAddress = 0;
  resetTxBuffer();

  //user_onRequest = NULL;
  transmitting = 0;

  ownAddress = address << 1;

  recoverBus(); // in case I2C bus (device) is stuck after a reset for example

  i2c_init(&_i2c, 100000, ownAddress);

}

void wire_beginTransmission(uint8_t address)
{
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address << 1;
  // reset tx data size
  txDataSize = 0;
}

uint8_t wire_endTransmission(uint8_t sendStop)
{
  int8_t ret = 4;

    // transmit buffer (blocking)

  HAL_I2C_Master_Transmit

    switch (i2c_master_write(&_i2c, txAddress, txBuffer, txDataSize)) {
      case I2C_OK :
        ret = 0; // Success
        break;
      case I2C_DATA_TOO_LONG :
        ret = 1;
        break;
      case I2C_NACK_ADDR:
        ret = 2;
        break;
      case I2C_NACK_DATA:
        ret = 3;
        break;
      case I2C_TIMEOUT:
      case I2C_BUSY:
      case I2C_ERROR:
      default:
        ret = 4;
        break;
    }

    // reset Tx buffer
    resetTxBuffer();

    // reset tx buffer data size
    txDataSize = 0;

    // indicate that we are done transmitting
    transmitting = 0;

  return ret;
}

void resetRxBuffer(void)
{
  if (rxBuffer != 0) {
    memset(rxBuffer, 0, rxBufferAllocated);
  }
}

void resetTxBuffer(void)
{
  if (txBuffer != 0) {
    memset(txBuffer, 0, txBufferAllocated);
  }
}

// Send clear bus (clock pulse) sequence to recover bus.
// Useful in case of bus stuck after a reset for example
// a mix implementation of Clear Bus from
// https://www.nxp.com/docs/en/user-guide/UM10204.pdf
// https://bits4device.wordpress.com/2017/07/28/i2c-bus-recovery/
void recoverBus(void)
{
//  pinMode(pinNametoDigitalPin(_i2c.sda), INPUT);
//
//  if (digitalReadFast(_i2c.sda) == LOW) {
//    pinMode(pinNametoDigitalPin(_i2c.scl), OUTPUT);
//
//    for (int i = 0; i < 20; i++) {
//      digitalWriteFast(_i2c.scl, LOW);
//      delayMicroseconds(10);
//      digitalWriteFast(_i2c.scl, HIGH);
//      delayMicroseconds(10);
//    }
//    pinMode(pinNametoDigitalPin(_i2c.scl), INPUT);
//  }
}
