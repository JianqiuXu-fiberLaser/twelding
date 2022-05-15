#ifndef _EEPROM_I2C_H__

#define _EEPROM_I2C_H__

#include "stm32f4xx_hal.h"
#include "main.h"

//#define EEPROM_I2C_SCL(x) (x ? EEPROM_SCL_GPIO_Port->BSRR=(uint32_t)EEPROM_SCL_Pin : EEPROM_SCL_GPIO_Port->BSRR=(uint32_t)(EEPROM_SCL_Pin<<16U))
//#define EEPROM_I2C_SDA(x) (x ? EEPROM_SDA_GPIO_Port->BSRR=(uint32_t)EEPROM_SDA_Pin : EEPROM_SDA_GPIO_Port->BSRR=(uint32_t)(EEPROM_SDA_Pin<<16U))
//#define EEPROM_I2C_SCL(x) (x ? HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port,EEPROM_SCL_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port,EEPROM_SCL_Pin,GPIO_PIN_RESET))
#define EEPROM_I2C_SCL_HIGH (EEPROM_SCL_GPIO_Port->BSRR=(uint32_t)EEPROM_SCL_Pin)
#define EEPROM_I2C_SCL_LOW (EEPROM_SCL_GPIO_Port->BSRR=(((uint32_t)EEPROM_SCL_Pin)<<16U))
//#define EEPROM_I2C_SDA(x) (x ? HAL_GPIO_WritePin(EEPROM_SDA_GPIO_Port,EEPROM_SDA_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(EEPROM_SDA_GPIO_Port,EEPROM_SDA_Pin,GPIO_PIN_RESET))
#define EEPROM_I2C_SDA_HIGH (EEPROM_SDA_GPIO_Port->BSRR=(uint32_t)EEPROM_SDA_Pin)
#define EEPROM_I2C_SDA_LOW (EEPROM_SDA_GPIO_Port->BSRR=(((uint32_t)EEPROM_SDA_Pin)<<16U))
#define EEPROM_I2C_SDA_IN {EEPROM_SDA_GPIO_Port->MODER&=0xfff3ffff;}//clear to zero
#define EEPROM_I2C_SDA_OUT {EEPROM_SDA_GPIO_Port->MODER&=0xfff3ffff;EEPROM_SDA_GPIO_Port->MODER|=0x00040000;}
//#define EEPROM_READ_I2C_SDA (((EEPROM_SDA_GPIO_Port->IDR)&EEPROM_SDA_Pin)>0)
#define EEPROM_READ_I2C_SDA (HAL_GPIO_ReadPin(EEPROM_SDA_GPIO_Port,EEPROM_SDA_Pin)==GPIO_PIN_SET)
#define EEPROM_WC_HIGH (EEPROM_WC_GPIO_Port->BSRR=(uint32_t)EEPROM_WC_Pin)
#define EEPROM_WC_LOW (EEPROM_WC_GPIO_Port->BSRR=(((uint32_t)EEPROM_WC_Pin)<<16U))

void i2c_delay(uint16_t del);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_waitack(void);
void i2c_sendack(void);
void i2c_sendnack(void);
void i2c_write_one_byte(uint8_t data);
uint8_t i2c_read_one_byte(void);

#endif//_EEPROM_I2C_H__
