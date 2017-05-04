#ifndef __I2C_H__
#define __I2C_H__
#include "stm32f10x.h"
#include "usart_utils.h"

#define DEFAULT_I2C                  I2C1
#define DEFAULT_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define DEFAULT_I2C_Port             GPIOB
#define DEFAULT_I2C_SCL_Pin          GPIO_Pin_6
#define DEFAULT_I2C_SDA_Pin          GPIO_Pin_7
#define DEFAULT_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define DEFAULT_I2C_Speed            100000

#define I2C_DEBUG
#ifdef I2C_DEBUG
	#define I2C_DEBUG_PRINT(x) USART_UTILS::write_string(x)
  #define I2C_DEBUG_PRINTHEX(x) USART_UTILS::write_format("%x",x)
  #define I2C_DEBUG_PRINTDEC(x) USART_UTILS::write_format("%d",x)
#else
    #define I2C_DEBUG_PRINT(x)
    #define I2C_DEBUG_PRINTHEX(x)
    #define I2C_DEBUG_PRINTDEC(x)
#endif
class I2Cdev
{
	public:
		I2Cdev();
	
		static void Init(void);

		static void BitsWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
		static void BitWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
		static void ByteWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
		static void ByteWriteDebug(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
		static void BytesWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t* pBuffer);
		static void BytesWriteDebug(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t* pBuffer);
	
		static void WordWrite(uint8_t devAddr, uint8_t regAddr, uint16_t data);
		static void WordsWrite(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);
	
		static void BitsRead(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data); 
		static void BitRead(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);	
		static void BytesRead(uint8_t slaveAddr, uint8_t regAddr, u16 NumByteToRead, uint8_t* pBuffer);
		static void ByteRead(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
};

#endif
