/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H1DR5.h
 Description   : Header file for module H1DR5.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H1DR5_H
#define H1DR5_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H1DR5_MemoryMap.h"
#include "H1DR5_uart.h"
#include "H1DR5_gpio.h"
#include "H1DR5_dma.h"
#include "H1DR5_inputs.h"
#include "H1DR5_eeprom.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H1DR5

/* Port-related Definitions */
#define	NUM_OF_PORTS	5
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5

/* Define available USARTs */
#define _USART2
#define _USART3
#define _USART4
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart5
#define UART_P5 &huart6

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* Ethernet SPI Pin Definitions */
#define SCK_PIN             GPIO_PIN_5
#define MISO_PIN            GPIO_PIN_6
#define MOSI_PIN            GPIO_PIN_7
#define SPI_PORT            GPIOA

#define ETH_SPI_HANDLER     &hspi1

/* Ethernet GPIO Definitions */
#define C_SELECT_PIN        GPIO_PIN_4
#define C_SELECT_PORT       GPIOA

#define RST_PIN             GPIO_PIN_0
#define RST_PORT            GPIOB

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_3

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS		1
#define MX_SIZE_USER_BUFFER     512
/* Enable User Data from external ports */
#define __USER_DATA_BUFFER

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H1DR5_OK =0,
	H1DR5_ERR_UNKNOWNMESSAGE,
	H1DR5_ERR_WRONGPARAMS,
	H1DR5_ERROR =255
} Module_Status;

/* Default Configuration Type Definition */
typedef struct DefaultValues {
	uint8_t LocalMac[6];
	uint8_t RemoteMac[6];
	uint8_t LocalIP[4];
	uint8_t RemoteIP[4];
	uint8_t SubnetMask[4];
	uint8_t DestIP[4];
	uint8_t LocalPort;
	uint8_t RemotePort;
} EthernetDefaultValues;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status DefaultValues(void);
Module_Status SetRemoteIPRemoteMAC(void);
Module_Status SetLocalIP(uint8_t *localIP);
Module_Status SetRemoteIP(uint8_t *remoteIP);
Module_Status SetLocalMAC(uint8_t *localMAC);
Module_Status SetLocalPORT(uint8_t localPort);
Module_Status SetRemotePORT(uint8_t remotePort);
Module_Status SetSubnetMask(uint8_t *SubnetMask);
Module_Status EthernetSendData(char *data ,uint16_t length);

#endif /* H1DR5_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
