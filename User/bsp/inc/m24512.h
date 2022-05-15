#ifndef _M24512_H__
#define _M24512_H__

#include "stm32f4xx_hal.h"
#include <string.h>

#include "eeprom_i2c.h"
#include "eeprom_address.h"
#include "main.h"

#define M24512_ADDR 0xA0

uint8_t M24512_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint8_t buffersize);
uint8_t M24512_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint8_t NumByteToRead);
uint8_t M24512_ReadAddr(uint8_t *addr);
void M24512_TEST(void);
uint8_t M24512_CHECK(void);

#endif//_M24512_H__
