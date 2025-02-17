/*
 * RF24_mesh.h
 *
 *  Created on: 20 янв. 2025 г.
 *      Author: bryki
 */

#ifndef INC_RF24_MESH_H_
#define INC_RF24_MESH_H_

#include <stdint.h>

void mesh_Init(void);
void mesh_Begin(uint16_t addr);
uint8_t mesh_AddressRequest(void);
uint8_t mesh_Lookup(void);
uint8_t mesh_Write(uint8_t type, uint8_t *buf, uint8_t size);
uint8_t mesh_Read(uint8_t *type, uint8_t *buf, uint8_t size);

#endif /* INC_RF24_MESH_H_ */
