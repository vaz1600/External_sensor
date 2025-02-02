/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <stdint.h>

#include "twi.h"

// Minimal buffer length. Buffers length will be increased when needed,
// but TX buffer is limited to a maximum to avoid too much stack consumption
// Note: Buffer length and max buffer length are limited by uin16_t type
#define BUFFER_LENGTH 32
#if !defined(WIRE_MAX_TX_BUFF_LENGTH)
  #define WIRE_MAX_TX_BUFF_LENGTH       1024U
#endif

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1


    void allocateRxBuffer(size_t length);
    size_t allocateTxBuffer(size_t length);

    void resetRxBuffer(void);
    void resetTxBuffer(void);
    void recoverBus(void);

    void wire_begin(uint8_t address);
    void wire_beginTransmission(uint8_t address);
    uint8_t wire_endTransmission(uint8_t sendStop);

#endif
