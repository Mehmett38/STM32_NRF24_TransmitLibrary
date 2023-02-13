/*
 * nrf24.c
 *
 *  Created on: Oct 5, 2022
 *      Author: Mehmet Dincer
 */

#ifndef INC_NRF24_C_
#define INC_NRF24_C_

#include <main.h>
#include <stdbool.h>

typedef enum
{
	LOW,
	HIGH,
	Error = 404,
	error = 404
}Pin_State;

typedef enum
{
	RF24_PA_MIN,
	RF24_PA_LOW,
	RF24_PA_HIGH,
	RF24_PA_MAX,
	RF24_PA_ERROR
}rf24_pa_dbm_e;

typedef enum
{
	RF24_1MBPS,
	RF24_2MBPS,
	RF24_250KBPS
}rf24_datarate_e;

typedef enum
{
	RF24_CRC_DISABLED,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;




// enable or disable the Chip select pin
#define RF24_CS_State(x)			HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, x);

// enable or disable the chip enable pin
#define RF24_CE_State(x)			HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, x);

// take the min value

#define RF24_Min(x,y)				(x<y ? x: y)


/*Functions*/
void RF24_Begin(void);
void RF24_ResetState();
uint16_t RF24_WriteRegister(uint8_t reg, uint8_t value);
uint16_t RF24_WriteMultiReg(uint8_t reg, uint8_t *pData, uint8_t len);
uint16_t RF24_ReadRegister(uint8_t reg);
uint16_t RF24_ReadMultiReg(uint8_t reg, uint8_t *pData, uint8_t len);
void RF24_SetPALevel();			// aldığım kodlarda farlı configure edilmiş
void RF24_SetDataRate();		// aldığım kodlarda farklı configure edilmiş benim işimi 1Mhz görüyor
void RF24_SetCrcLength();		// Setting for CRC set as 2byte
void RF24_SetChannel(uint8_t channel);	//take the channel data
void RF24_FlushRX();
void RF24_FlushTX();
void RF24_OpenWritingPipe(uint64_t value);
bool RF24_Write(const void* buf, uint8_t len);
void RF24_StartWrite(const void* buf, uint8_t len);
void RF24_WritePayload(const void * buf, uint8_t len);
void RF24_WhatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready);
uint8_t RF24_GetDynamicPayloadSize(void);
void RF24_PowerDown(void);
uint8_t RF24_GetStatus(void);
bool RF24_Available(uint8_t *pipeNum);
void RF24_StopListening();
void RF24_WritePayload(const void *buf, uint8_t len);

void RF24_DelayUs(uint32_t time);

//special Variables
#define payload_size 					32

/* Instruction Mnemonics */
#define R_REGISTER    					0x00
#define W_REGISTER    					0x20
#define REGISTER_MASK 					0x1F
#define ACTIVATE      					0x50
#define R_RX_PL_WID   					0x60
#define R_RX_PAYLOAD  					0x61
#define W_TX_PAYLOAD  					0xA0
#define W_ACK_PAYLOAD 					0xA8
#define FLUSH_TX      					0xE1
#define FLUSH_RX      					0xE2
#define REUSE_TX_PL   					0xE3
#define NOP          					0xFF

/* nRF24L01+ Register map table*/
#define CONFIG     					 	0x00
#define EN_AA       					0x01
#define EN_RXADDR   					0x02
#define SETUP_AW    					0x03
#define SETUP_RETR  					0x04
#define RF_CH      						0x05
#define RF_SETUP    					0x06
#define STATUS      					0x07
#define OBSERVE_TX  					0x08
#define CD          					0x09
#define RX_ADDR_P0  					0x0A
#define RX_ADDR_P1  					0x0B
#define RX_ADDR_P2  					0x0C
#define RX_ADDR_P3  					0x0D
#define RX_ADDR_P4  					0x0E
#define RX_ADDR_P5  					0x0F
#define TX_ADDR     					0x10
#define RX_PW_P0    					0x11
#define RX_PW_P1    					0x12
#define RX_PW_P2    					0x13
#define RX_PW_P3    					0x14
#define RX_PW_P4    					0x15
#define RX_PW_P5    					0x16
#define FIFO_STATUS 					0x17
#define DYNPD	    					0x1C
#define FEATURE	   						0x1D

/*Config registers bit's names*/
#define MASK_RX_DR						06U
#define MASK_TX_DS						05U
#define MASK_MAX_RT						04U
#define EN_CRC							03U
#define CRCO							02U
#define PWR_UP							01U
#define PRIM_RX							00U

/*EN_AA registers bit's names*/
#define ENAA_P5							05U
#define ENAA_P4							04U
#define ENAA_P3							03U
#define ENAA_P2							02U
#define ENAA_P1							01U
#define ENAA_P0							00U

/*EN_RXADDR registers bit's names*/
#define ERX_P5							05U
#define ERX_P4							04U
#define ERX_P3							03U
#define ERX_P2							02U
#define ERX_P1							01U
#define ERX_P0							00U

/*SETUP_AW registers bit's names*/
#define AW								00U

/*SETUP_RETR registers bit's names*/
#define ARD								04U
#define ARC								00U


/*RF_SETUP registers bit's names*/
#define PLL_LOCK						04U
#define RF_DR							03U
#define RF_PWR							01U
#define LNA_HCURR						00U

/*Status registers bit's names*/
#define RX_DR							06U
#define TX_DS							05U
#define MAX_RT							04U
#define RX_P_NO							01U
#define TX_FULL							00U

/*OBSERVE_TX registers bit's names*/
#define PLOS_CNT						04U
#define ARC_CNT							00U
#define NOP								0xFF

#endif /* INC_NRF24_C_ */
