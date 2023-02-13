/*
 * nrf24.c
 *
 *  Created on: Oct 5, 2022
 *      Author: Mehmet Dincer
 */


#include <nrf24.h>

bool dynamic_payloads_enabled = false;
bool ack_payload_available = false;
uint8_t ack_payload_length;

//*******************************************************************
void RF24_Begin()
{
	RF24_CE_State(LOW);
	RF24_CS_State(HIGH);

	//	Standby_I mode
	//  Every new command must be started by a high to low transition of CSN

	HAL_Delay(5);
	RF24_ResetState();

	// Set 1250us timeout and auto Auto Retransmit Count is 15
	RF24_WriteRegister(SETUP_RETR, (0x05 << ARD) | (0x0F << ARC));

	RF24_SetPALevel();				//Set PA level 0dBm

	RF24_SetDataRate();				//Set Air data rate to 1Mhz

	RF24_SetCrcLength();			//Set crc setting(enable and 2byte)

	RF24_WriteRegister(DYNPD, 0);	//Disable the Dynamic Payload Length(DPL)

	/*_____________________________Reset Current Status___________________________*/
	RF24_WriteRegister(STATUS, 0x01 << RX_DR);	//write 1 to clear RX FIFO
	RF24_WriteRegister(STATUS, 0x01 << TX_DS);	//write 1 to clear TX FIFI
	RF24_WriteRegister(STATUS, 0x01 << MAX_RT);	//write 1 to clear max number of TX

	RF24_SetChannel(76);		//// This channel should be universally safe and not bleed over into adjacent

	RF24_FlushRX();
	RF24_FlushTX();
}


//**********************************************************************
void RF24_ResetState()
{
	RF24_WriteRegister(CONFIG, 0x08);
	RF24_WriteRegister(EN_AA, 0x3F);
	RF24_WriteRegister(EN_RXADDR, 0x03);
	RF24_WriteRegister(SETUP_AW, 0x03);
	RF24_WriteRegister(SETUP_RETR, 0x03);
	RF24_WriteRegister(RF_CH, 0x02);
	RF24_WriteRegister(RF_SETUP, 0x0F);
	RF24_WriteRegister(STATUS, 0x0E);
	RF24_WriteRegister(OBSERVE_TX, 0x00);
	RF24_WriteRegister(CD, 0x00);
	RF24_WriteRegister(RX_PW_P0, 0x00);
	RF24_WriteRegister(RX_PW_P1, 0x00);
	RF24_WriteRegister(RX_PW_P2, 0x00);
	RF24_WriteRegister(RX_PW_P3, 0x00);
	RF24_WriteRegister(RX_PW_P4, 0x00);
	RF24_WriteRegister(RX_PW_P5, 0x00);
	RF24_WriteRegister(FIFO_STATUS, 0x11);
}


//**********************************************************************
uint16_t RF24_WriteRegister(uint8_t reg, uint8_t value)
{
	uint8_t regSend = W_REGISTER | (reg & REGISTER_MASK);
	uint8_t regRead = 0;

	RF24_CS_State(LOW);

	HAL_SPI_Transmit(&hspi1, &regSend, 1, 100);
	HAL_SPI_Transmit(&hspi1, &value, 1, 100);

	RF24_CS_State(HIGH);

	return regRead;
}


//******************************************************************************
uint16_t RF24_WriteMultiReg(uint8_t reg, uint8_t *pData, uint8_t len)
{
	uint8_t regSend = W_REGISTER | (reg & REGISTER_MASK);
	uint8_t regRead = 0;
	HAL_StatusTypeDef status;

	RF24_CS_State(LOW);

	status = HAL_SPI_Transmit(&hspi1, &regSend, 1, 1000);
	if (status != HAL_OK) return error;

	status = HAL_SPI_Transmit(&hspi1, pData, len, 1000);
	if (status != HAL_OK) return error;

	RF24_CS_State(HIGH);

	return regRead;
}


//******************************************************************************
uint16_t RF24_ReadRegister(uint8_t reg)
{
	uint8_t regSend = R_REGISTER | (reg & REGISTER_MASK);
	uint8_t regRead;

	RF24_CS_State(LOW);

	HAL_SPI_Transmit(&hspi1, &regSend, 1, 100);
	HAL_SPI_Receive(&hspi1, &regRead, 1, 100);

	RF24_CS_State(HIGH);

	return regRead;

}


//******************************************************************************
uint16_t RF24_ReadMultiReg(uint8_t reg, uint8_t *pData, uint8_t len)
{
	HAL_StatusTypeDef status;
	uint8_t rReg = R_REGISTER | (reg & REGISTER_MASK);
	RF24_CS_State(LOW);

	status = HAL_SPI_Transmit(&hspi1, &rReg, 1, 1000);
	if (status != HAL_OK) return Error;

	status = HAL_SPI_Receive(&hspi1, pData, len, 1000);
	if (status != HAL_OK) return Error;

	RF24_CS_State(HIGH);

	return status;
}


//******************************************************************************
void RF24_SetPALevel()
{
	uint8_t setup = RF24_ReadRegister(RF_SETUP);
	setup &= ~( 0x03 << RF_PWR);					//Set rf output power -18dBm
	setup |= ( 0x03 << RF_PWR);						// Set rf ouput power 0dBm

	RF24_WriteRegister(RF_SETUP, setup);

	setup = RF24_ReadRegister(RF_SETUP);
}


//******************************************************************************
void RF24_SetDataRate()
{
	uint8_t regRead = RF24_ReadRegister(RF_SETUP);

	regRead &= ~(0x01 << RF_DR);
	regRead |= (0x00 << RF_DR);
	RF24_WriteRegister(RF_SETUP, regRead);
	regRead = RF24_ReadRegister(RF_SETUP);
}


//******************************************************************************
void RF24_SetCrcLength()
{
	uint8_t regRead = RF24_ReadRegister(CONFIG);

	regRead &= ~(0x01<< EN_CRC | 0x01 << CRCO);
	regRead |= 0x01<< EN_CRC | 0x01 << CRCO;

	RF24_WriteRegister(CONFIG, regRead);
	regRead = RF24_ReadRegister(CONFIG);
}


//******************************************************************************
void RF24_SetChannel(uint8_t channel)
{
	if (channel > 127)		channel = 127;		// channel can not bigger than 127
	RF24_WriteRegister(RF_CH, channel);
}

//******************************************************************************
void RF24_FlushRX()
{
	/*
	 * flux Rx fifo, used in Rx mode
	 */
	uint8_t reg = FLUSH_RX;

	RF24_CS_State(LOW);

	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);

	RF24_CS_State(HIGH);
}


//******************************************************************************
void RF24_FlushTX()
{
	/*
	 * Flux Tx Fifo, used in Tx mode
	 */
	uint8_t reg = FLUSH_TX;

	RF24_CS_State(LOW);

	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);

	RF24_CS_State(HIGH);
}


//******************************************************************************
void RF24_OpenWritingPipe(uint64_t value)
{
	uint8_t pData[5];
	pData[0] = (value & 0xFF);
	pData[1] = (value & 0xFF00) >>8;
	pData[2] = (value & 0xFF0000)>>16;
	pData[3] = (value & 0xFF000000)>>24;
	pData[4] = (value & 0xFF00000000)>>32;

	RF24_WriteMultiReg(RX_ADDR_P0, pData, 5);
	RF24_WriteMultiReg(TX_ADDR, pData, 5);

	RF24_WriteRegister(RX_PW_P0, payload_size);
}


//******************************************************************************
bool RF24_Write(const void* buf, uint8_t len)
{
	bool result = false , tx_ok, tx_fail;

	RF24_StartWrite(buf, len);		//Begin to write

	uint8_t status;
	uint32_t sent_at = HAL_GetTick();
	const uint32_t timeOut = 1000; // 500ms wait for the timeout value

	do
	{
		RF24_ReadMultiReg(OBSERVE_TX, &status, 1);

	}while(!(status & (0x01 << TX_DS | 0x01 << MAX_RT))
			&& ((HAL_GetTick() - sent_at) < timeOut));

	// the part above is wait up to tx full or tx interrupt flag to be high

	RF24_WhatHappened(&tx_ok, &tx_fail, &ack_payload_available);

	result = tx_ok;

	if(ack_payload_available)
	{
		ack_payload_length = RF24_GetDynamicPayloadSize();
	}

	RF24_PowerDown();		// power down

	RF24_FlushTX();			//TX fifo used in TX mode

	return result;
}


//******************************************************************************
void RF24_StartWrite(const void* buf, uint8_t len)
{
	uint8_t regRead = (uint8_t)RF24_ReadRegister(CONFIG);
	regRead |= (0x01 << PWR_UP) & ~(0x01 << PRIM_RX);
	RF24_WriteRegister(CONFIG, regRead);

	RF24_DelayUs(150);

	RF24_WritePayload(buf , len);

	RF24_CE_State(HIGH);
	RF24_DelayUs(15);
	RF24_CE_State(LOW);
}


//******************************************************************************
void RF24_DelayUs(uint32_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < time);

}


//******************************************************************************
void RF24_WritePayload(const void *buf, uint8_t len)
{
	/*
	 * ilk başta gelen buffer'ın +1 dediğimiz zaman kaç değiştiğini bul
	 * sonrasında bu değişime göre gelen verileri uint8_t formatında bir diziye ata
	 * */

	uint8_t regWrite[32] = {0};
	HAL_StatusTypeDef halState;
	uint8_t writingReg = W_TX_PAYLOAD;

	uint8_t dataLen = RF24_Min(len , payload_size);

	for(int i = 0; i < 32; i++)
	{
		if(i < dataLen)
		{
			regWrite[i] = *((uint8_t*)(buf++));	//bu kısımdaki hata gönderilen array'i adress olarak alıyor
		}
		else
			regWrite[i] = 0;
	}

	RF24_CS_State(LOW);
	halState = HAL_SPI_Transmit(&hspi1, &writingReg , 1, 10);
	halState = HAL_SPI_Transmit(&hspi1, regWrite, 32, 10);
	RF24_CS_State(HIGH);
	UNUSED(halState);
}


//******************************************************************************
void RF24_WhatHappened(bool* tx_ok, bool *tx_fail, bool *rx_ready)
{
	RF24_WriteRegister(STATUS, (0x01 << RX_DR | 0x01 << TX_DS | 0x01 << MAX_RT));
	uint16_t status = RF24_ReadRegister(STATUS);

	*tx_ok = status & (0x01 << TX_DS);
	*tx_fail = status & (0x01 << MAX_RT);
	*rx_ready = status & (0x01 << RX_DR);
}


//******************************************************************************
uint8_t RF24_GetDynamicPayloadSize(void)
{
	uint8_t result = 0;

	RF24_CS_State(LOW);
	result = RF24_ReadRegister(R_RX_PL_WID);
	RF24_CS_State(HIGH);

	return result;
}


//******************************************************************************
void RF24_PowerDown(void)
{
	uint8_t regRead;
	regRead = RF24_ReadRegister(CONFIG);
	regRead &= ~(0x01 << PWR_UP);
	RF24_WriteRegister(CONFIG, regRead);
}


//******************************************************************************
uint8_t RF24_GetStatus(void)
{
	uint8_t status;
	uint8_t regWrite = NOP;
	RF24_CS_State(LOW);
	HAL_SPI_Transmit(&hspi1, &regWrite, 1, 100);
	HAL_SPI_Receive(&hspi1, &status, 1, 100);
	return status;
}


//******************************************************************************
bool RF24_Available(uint8_t *pipeNum)
{
	uint8_t status = RF24_GetStatus();
	bool result = status & (0x01 << RX_DR);

	if(result)
	{
		if(pipeNum)
		{
			*pipeNum = (status >> RX_P_NO) & 0b111;
		}

		RF24_WriteRegister(STATUS, 0x01 << RX_DR);

		if(status & 0x01 << TX_DS)
		{
			RF24_WriteRegister(STATUS, 0x01 << TX_DS);
		}

	}
	return result;
}


//******************************************************************************
void RF24_StopListening()
{
	RF24_CE_State(LOW);
	RF24_FlushTX();
	RF24_FlushRX();
}


//******************************************************************************


/*
 * 17/10/2022
 * hatam write_payload kısmında
 * buffera aldığım adresi değişken olarak atamamak
 * */



/*
 * 10/10/2022
 *
 * https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
 * https://www.youtube.com/watch?v=vCPjqeDLlfY&t=458s
 * https://www.youtube.com/watch?v=OERIT7xq3Gw
 *
 * uint64 den uint'e convert ederken reinterpret diye bir komut kullanmış
 * bunu copy paste yaparak diretkman kullanabilirsin
 * */

















