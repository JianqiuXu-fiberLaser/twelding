#include "eeprom_i2c.h"

void i2c_delay(uint16_t del)
{
	uint16_t i;
	for(i=0;i<del;i++)
	{
        __NOP();    
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
	}
}

void i2c_start(void)
{
	EEPROM_I2C_SCL_HIGH;
	EEPROM_I2C_SDA_HIGH;
	i2c_delay(2);
	EEPROM_I2C_SDA_LOW;
	i2c_delay(2);
	EEPROM_I2C_SCL_LOW;
}

void i2c_stop(void)
{
	EEPROM_I2C_SCL_LOW;
	EEPROM_I2C_SDA_LOW;
	i2c_delay(2);
	EEPROM_I2C_SCL_HIGH;
	i2c_delay(2);
	EEPROM_I2C_SDA_HIGH;
}

uint8_t i2c_waitack(void)
{
	uint8_t i=200;
	
	EEPROM_I2C_SCL_LOW;
//	EEPROM_I2C_SDA_LOW;
	EEPROM_I2C_SDA_IN;
	i2c_delay(2);
	EEPROM_I2C_SCL_HIGH;
	i2c_delay(1);
	
	do
	{
		i--;
	}while((i>0)&&(EEPROM_READ_I2C_SDA==1));
	
	i2c_delay(1);
	EEPROM_I2C_SCL_LOW;
	EEPROM_I2C_SDA_OUT;
	
	return i;
}

void i2c_sendack(void)
{
	EEPROM_I2C_SCL_LOW;
	EEPROM_I2C_SDA_LOW;
	i2c_delay(2);
	EEPROM_I2C_SCL_HIGH;
	i2c_delay(2);
	EEPROM_I2C_SCL_LOW;
}

void i2c_sendnack(void)
{
	EEPROM_I2C_SCL_LOW;
	EEPROM_I2C_SDA_LOW;
	i2c_delay(1);
	EEPROM_I2C_SDA_HIGH;
	i2c_delay(1);
	EEPROM_I2C_SCL_HIGH;
	i2c_delay(2);
	EEPROM_I2C_SCL_LOW;
	i2c_delay(1);
	EEPROM_I2C_SDA_LOW;
}

void i2c_write_one_byte(uint8_t data)
{
	uint8_t i;
	
	EEPROM_I2C_SCL_LOW;
	EEPROM_I2C_SDA_LOW;
	i2c_delay(1);
	
	for(i=0;i<8;i++)
	{
		if((data&0x80)!=0x00)
		{
			EEPROM_I2C_SDA_HIGH;
		}
		else
		{
			EEPROM_I2C_SDA_LOW;
		}
		i2c_delay(1);
		data<<=1;
		EEPROM_I2C_SCL_HIGH;
		i2c_delay(2);
		EEPROM_I2C_SCL_LOW;
		i2c_delay(1);
	}
}

uint8_t i2c_read_one_byte(void)
{
	uint8_t i,data;
	
	EEPROM_I2C_SCL_LOW;
//	EEPROM_I2C_SDA_LOW;
	EEPROM_I2C_SDA_IN;
	i2c_delay(1);
	
	for(i=0;i<8;i++)
	{
		data<<=1;
		EEPROM_I2C_SCL_LOW;
		i2c_delay(1);
		EEPROM_I2C_SCL_HIGH;
		i2c_delay(1);
		if(EEPROM_READ_I2C_SDA!=0x00)
		{
			data|=0x01;
		}
		i2c_delay(1);
		EEPROM_I2C_SCL_LOW;
		i2c_delay(1);
	}
	
	EEPROM_I2C_SDA_OUT;
	
	return data;
}
