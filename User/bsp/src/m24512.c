#include "m24512.h"
#include "cmsis_os.h"

static uint8_t M24512_WriteByte(uint8_t* pBuffer, uint16_t WriteAddr);
static uint8_t M24512_ReadByte(uint8_t* pBuffer, uint16_t ReadAddr);

uint8_t M24512_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr,uint8_t buffersize)
{
	osMutexAcquire(	(osMutexId_t)M24512MutexHandle,
					(uint32_t)portMAX_DELAY);
	uint8_t i;
	uint8_t status;
	EEPROM_WC_LOW;
	i2c_delay(10);
	i2c_stop();
	i2c_delay(50);
	EEPROM_WC_HIGH;
	for(i=0;i<buffersize;i++)
	{
		status=M24512_WriteByte(pBuffer+i,WriteAddr+i);
		if(status!=0)
			return status;
	}
	osMutexRelease((osMutexId_t)M24512MutexHandle);
	return 0;
}

static uint8_t M24512_WriteByte(uint8_t* pBuffer, uint16_t WriteAddr)
{
	volatile uint8_t temp,i;
	EEPROM_WC_LOW;
	i2c_delay(10);
	for(i=0;i<100;i++)
	{
		i2c_start();
		i2c_write_one_byte(M24512_ADDR&0xfe);
		temp=i2c_waitack();
		if(temp!=0)
			break;
		i2c_stop();
	}
	if(temp==0)
	{
		i2c_delay(10);
		EEPROM_WC_HIGH;
		return 1;
	}
	i2c_write_one_byte(WriteAddr>>8);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		i2c_delay(10);
		EEPROM_WC_HIGH;
		return 2;
	}
	i2c_write_one_byte((uint8_t)WriteAddr);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		i2c_delay(10);
		EEPROM_WC_HIGH;
		return 3;
	}
	i2c_write_one_byte(*pBuffer);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		i2c_delay(10);
		EEPROM_WC_HIGH;
		return 4;
	}
	i2c_stop();
	i2c_delay(10);
	EEPROM_WC_HIGH;
	return 0;
}

uint8_t M24512_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint8_t NumByteToRead)
{
	volatile uint8_t temp,i;
	osMutexAcquire(	(osMutexId_t)M24512MutexHandle,
					(uint32_t)portMAX_DELAY);
	EEPROM_WC_LOW;
	i2c_delay(10);
	i2c_stop();
	i2c_delay(50);
	EEPROM_WC_HIGH;
	for(i=0;i<100;i++)
	{
		i2c_start();
		i2c_write_one_byte(M24512_ADDR&0xfe);
		temp=i2c_waitack();
		if(temp!=0)
			break;
		i2c_stop();
	}
	if(temp==0)
	{
		return 1;
	}
	i2c_write_one_byte(ReadAddr>>8);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		return 2;
	}
	i2c_write_one_byte((uint8_t)ReadAddr);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		return 3;
	}
	i2c_start();
	i2c_write_one_byte(M24512_ADDR|0x01);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		return 4;
	}
	for(i=0;i<NumByteToRead;i++)
	{
		pBuffer[i]=i2c_read_one_byte();
		if(i==NumByteToRead-1)
		{
			i2c_sendnack();
		}
		else
		{
			i2c_sendack();
		}
	}
	i2c_stop();
	osMutexRelease((osMutexId_t)M24512MutexHandle);
	return 0;
}

static uint8_t M24512_ReadByte(uint8_t* pBuffer, uint16_t ReadAddr)
{
	volatile uint8_t temp,i;
	for(i=0;i<100;i++)
	{
		i2c_start();
		i2c_write_one_byte(M24512_ADDR&0xfe);
		temp=i2c_waitack();
		if(temp!=0)
			break;
		i2c_stop();
	}
	if(temp==0)
	{
		return 1;
	}
	i2c_write_one_byte(ReadAddr>>8);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		return 2;
	}
	i2c_write_one_byte((uint8_t)ReadAddr);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		return 3;
	}
	i2c_start();
	i2c_write_one_byte(M24512_ADDR|0x01);
	temp=i2c_waitack();
	if(temp==0)
	{
		i2c_stop();
		return 4;
	}
	*pBuffer=i2c_read_one_byte();
	i2c_sendnack();
	i2c_stop();
	return 0;
}

uint8_t M24512_ReadAddr(uint8_t *addr)
{
	volatile uint8_t temp;
	uint8_t pBuffer;
	EEPROM_WC_LOW;
	i2c_delay(10);
	i2c_stop();
	i2c_delay(50);
	EEPROM_WC_HIGH;
	
	i2c_start();
	i2c_write_one_byte(M24512_ADDR|0x01);
	temp=i2c_waitack();
	if(temp==0)
	{
		*addr=0;
		i2c_stop();
		return 1;
	}
	pBuffer=i2c_read_one_byte();
	*addr=pBuffer;
	i2c_sendnack();
	i2c_stop();
	return 0;
}

void M24512_TEST(void)
{
	uint8_t i,temp[255];
	for(i=0;i<255;i++)
	{
		temp[i]=i;
	}
	M24512_WriteBuffer(temp,0,sizeof(temp));
	memset(temp,0,sizeof(temp));
	for(i=0;i<sizeof(temp);i++)
	{
		M24512_ReadByte(&temp[i],i);
	}
	while(1)
	{
		i2c_delay(100);
	}
}

uint8_t M24512_CHECK(void)
{
//	uint8_t writebyte[6]={0x01,0x02,0x03,0x01,0x02,0x03};
//	M24512_WriteBuffer(&writebyte[0],EEPROM_CHECK1_ADDRESS,sizeof(writebyte[0]));
//	M24512_WriteBuffer(&writebyte[1],EEPROM_CHECK2_ADDRESS,sizeof(writebyte[1]));
//	M24512_WriteBuffer(&writebyte[2],EEPROM_CHECK3_ADDRESS,sizeof(writebyte[2]));
//	M24512_WriteBuffer(&writebyte[3],EEPROM_CHECK1_ADDRESS_BACKUP,sizeof(writebyte[3]));
//	M24512_WriteBuffer(&writebyte[4],EEPROM_CHECK2_ADDRESS_BACKUP,sizeof(writebyte[4]));
//	M24512_WriteBuffer(&writebyte[5],EEPROM_CHECK3_ADDRESS_BACKUP,sizeof(writebyte[5]));
	uint8_t readbyte;
	M24512_ReadBuffer(&readbyte,EEPROM_CHECK1_ADDRESS,sizeof(readbyte));
	if(readbyte!=0x01)
	{
		return 1;
	}
	M24512_ReadBuffer(&readbyte,EEPROM_CHECK2_ADDRESS,sizeof(readbyte));
	if(readbyte!=0x02)
	{
		return 1;
	}
	M24512_ReadBuffer(&readbyte,EEPROM_CHECK3_ADDRESS,sizeof(readbyte));
	if(readbyte!=0x03)
	{
		return 1;
	}
	M24512_ReadBuffer(&readbyte,EEPROM_CHECK1_ADDRESS_BACKUP,sizeof(readbyte));
	if(readbyte!=0x01)
	{
		return 1;
	}
	M24512_ReadBuffer(&readbyte,EEPROM_CHECK2_ADDRESS_BACKUP,sizeof(readbyte));
	if(readbyte!=0x02)
	{
		return 1;
	}
	M24512_ReadBuffer(&readbyte,EEPROM_CHECK3_ADDRESS_BACKUP,sizeof(readbyte));
	if(readbyte!=0x03)
	{
		return 1;
	}
	return 0;
}
