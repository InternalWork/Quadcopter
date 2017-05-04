#include "I2Cdev.h"

I2Cdev::I2Cdev()
{
}

/**
* @brief  Initializes the I2C peripheral used to drive the HMC5883L
* @param  None
* @retval None
*/
void I2Cdev::Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(DEFAULT_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(DEFAULT_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  DEFAULT_I2C_SCL_Pin | DEFAULT_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(DEFAULT_I2C_Port, &GPIO_InitStructure);
 
  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = DEFAULT_I2C_Speed;
  
  /* Apply I2C configuration after enabling it */
  I2C_Init(DEFAULT_I2C, &I2C_InitStructure);
  
  I2C_Cmd(DEFAULT_I2C, ENABLE);
}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr DEFAULT_I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void I2Cdev::BitsWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
	uint8_t tmp;
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	
	BytesRead(slaveAddr, regAddr, 1, &tmp);
	
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	BytesWrite(slaveAddr, regAddr, 1, &tmp);
}

/** write a single bit in an 8-bit device register.
 * @param slaveAddr DEFAULT_I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void I2Cdev::BitWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    BytesRead(slaveAddr, regAddr, 1, &tmp);  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    BytesWrite(slaveAddr, regAddr, 1, &tmp); 
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr DEFAULT_I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void I2Cdev::BitsRead(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
	uint8_t tmp;
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);

	BytesRead(slaveAddr, regAddr, 1, &tmp);  
	tmp &= mask;
	tmp >>= (bitStart - length + 1);
	*data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr DEFAULT_I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void I2Cdev::BitRead(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) 
{
    uint8_t tmp;
	BytesRead(slaveAddr, regAddr, 1, &tmp);  
    *data = tmp & (1 << bitNum);
}

void I2Cdev::ByteRead(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data) 
{
    BytesRead(slaveAddr, regAddr, 1, data);
}

void I2Cdev::ByteWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
	BytesWrite(slaveAddr, regAddr, 1, &data);
}

void I2Cdev::ByteWriteDebug(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
	BytesWriteDebug(slaveAddr, regAddr, 1, &data);
}

/**
* @brief  Reads a block of data from the device.
* @param  slaveAddr  : slave address
* @param  pBuffer : pointer to the buffer that receives the data read from the device.
* @param  ReadAddr : device's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the device.
* @retval None
*/

void I2Cdev::BytesRead(uint8_t slaveAddr, uint8_t regAddr, u16 NumByteToRead, uint8_t* pBuffer)
{
	// ENTR_CRT_SECTION(void);
  /* While the bus is busy */
  while(I2C_GetFlagStatus(DEFAULT_I2C, I2C_FLAG_BUSY));
	
  /* Send START condition */
  I2C_GenerateSTART(DEFAULT_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send device's address for write */
  I2C_Send7bitAddress(DEFAULT_I2C, slaveAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(DEFAULT_I2C, ENABLE);

  /* Send the device's internal address to write to */
  I2C_SendData(DEFAULT_I2C, regAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send START condition a second time */
  I2C_GenerateSTART(DEFAULT_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send device's address for read */
  I2C_Send7bitAddress(DEFAULT_I2C, slaveAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(DEFAULT_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(DEFAULT_I2C, ENABLE);
			while(I2C_GetFlagStatus(DEFAULT_I2C, I2C_FLAG_STOPF));
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the device */
      *pBuffer = I2C_ReceiveData(DEFAULT_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(DEFAULT_I2C, ENABLE);
//  EXT_CRT_SECTION(void);

}

void I2Cdev::BytesWriteDebug(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t* pBuffer)
{
	//  ENTR_CRT_SECTION(void);

	/* Send START condition */
	I2C_GenerateSTART(DEFAULT_I2C, ENABLE);

	/* Test on EV5 and clear it */
	I2C_DEBUG_PRINT("\n/* Test on EV5 and clear it */");
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_MODE_SELECT));
	
	/* Send device's address for write */
	I2C_Send7bitAddress(DEFAULT_I2C, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	I2C_DEBUG_PRINT("\n/* Test on EV6 and clear it */");
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	/* Send the device's internal address to write to */
	I2C_SendData(DEFAULT_I2C, regAddr);

	/* Test on EV8 and clear it */
	I2C_DEBUG_PRINT("\n/* Test on EV8 and clear it */");
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	for(uint8_t i = 0; i < length; i ++)
	{
		/* Send the byte to be written */
		I2C_SendData(DEFAULT_I2C, pBuffer[i]);
		/* Test on EV8 and clear it */
		I2C_DEBUG_PRINT("\n/* Test on EV8 and clear it */ (");
		I2C_DEBUG_PRINTDEC(i);
		I2C_DEBUG_PRINT(")");
		while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	/* Send STOP condition */
	I2C_GenerateSTOP(DEFAULT_I2C, ENABLE);
	while(I2C_GetFlagStatus(DEFAULT_I2C , I2C_FLAG_STOPF));
	// EXT_CRT_SECTION(void);

}

void I2Cdev::BytesWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t* pBuffer)
{
	//  ENTR_CRT_SECTION(void);

	/* Send START condition */
	I2C_GenerateSTART(DEFAULT_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_MODE_SELECT));
	
	/* Send device's address for write */
	I2C_Send7bitAddress(DEFAULT_I2C, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	/* Send the device's internal address to write to */
	I2C_SendData(DEFAULT_I2C, regAddr);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	for(uint8_t i = 0; i < length; i ++)
	{
		/* Send the byte to be written */
		I2C_SendData(DEFAULT_I2C, pBuffer[i]);
		/* Test on EV8 and clear it */
		while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	/* Send STOP condition */
	I2C_GenerateSTOP(DEFAULT_I2C, ENABLE);
	while(I2C_GetFlagStatus(DEFAULT_I2C , I2C_FLAG_STOPF));
	// EXT_CRT_SECTION(void);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
void I2Cdev::WordWrite(uint8_t devAddr, uint8_t regAddr, uint16_t data) 
{
    WordsWrite(devAddr, regAddr, 1, &data);
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
void I2Cdev::WordsWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint16_t *data)
{
	//  ENTR_CRT_SECTION(void);

	/* Send START condition */
	I2C_GenerateSTART(DEFAULT_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send device's address for write */
	I2C_Send7bitAddress(DEFAULT_I2C, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the device's internal address to write to */
	I2C_SendData(DEFAULT_I2C, regAddr);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	for(uint8_t i = 0; i < length * 2; i ++)
	{
		/* Send the byte to be written */
		I2C_SendData(DEFAULT_I2C, (uint8_t)(data[i] >> 8)); //MSB
		/* Test on EV8 and clear it */
		while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		
		I2C_SendData(DEFAULT_I2C, (uint8_t)(data[i++])); //LSB
		/* Test on EV8 and clear it */
		while(!I2C_CheckEvent(DEFAULT_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}



	/* Send STOP condition */
	I2C_GenerateSTOP(DEFAULT_I2C, ENABLE);
	// EXT_CRT_SECTION(void);

}
