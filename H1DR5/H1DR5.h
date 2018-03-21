/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2017 Hexabitz
    All rights reserved

    File Name     : H1DR5.h
    Description   : Header file for module H1DR5.
										Ethernet-SPI module (ENC28J60)
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H1DR5_H
#define H1DR5_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H1DR5_uart.h"
#include "H1DR5_gpio.h"
#include "H1DR5_dma.h"
#include "H1DR5_spi.h"
#include "FreeRTOS_UDP_IP.h"

/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H1DR5

/* Port-related definitions */
#define	NumOfPorts		5
#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1

/* Port-UART mapping */
#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT	GPIOA
#define	USART1_RX_PORT	GPIOA
#define	USART1_AF				GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT	GPIOA
#define	USART2_RX_PORT	GPIOA
#define	USART2_AF				GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT	GPIOB
#define	USART3_RX_PORT	GPIOB
#define	USART3_AF				GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT	GPIOA
#define	USART4_RX_PORT	GPIOA
#define	USART4_AF				GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT	GPIOB
#define	USART5_RX_PORT	GPIOB
#define	USART5_AF				GPIO_AF4_USART5

/* Module-specific Definitions */
#define _ETH_MISO_PORT			GPIOA
#define _ETH_MISO_PIN				GPIO_PIN_6
#define _ETH_SCK_PORT				GPIOA
#define _ETH_SCK_PIN				GPIO_PIN_5
#define _ETH_CS_PORT				GPIOA
#define _ETH_CS_PIN					GPIO_PIN_4
#define _ETH_MOSI_PORT			GPIOA
#define _ETH_MOSI_PIN				GPIO_PIN_7
#define	_ETH_RST_PORT				GPIOC
#define	_ETH_RST_PIN				GPIO_PIN_13
#define	_ETH_INT_PORT				GPIOB
#define	_ETH_INT_PIN				GPIO_PIN_0


/* H1DR5_Status Type Definition */
typedef enum
{
  H1DR5_OK = 0,
	H1DR5_ERR_UnknownMessage = 1,
	H1DR5_ERROR = 255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_14


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);



/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   -----------------------------------------------------------------------
*/




/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   -----------------------------------------------------------------------
*/



/* -----------------------------------------------------------------------
	|															Commands																 	|
   -----------------------------------------------------------------------
*/




#endif /* H1DR5_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
