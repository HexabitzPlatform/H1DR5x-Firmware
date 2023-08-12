/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved
 
 File Name     : H1DR5.h
 Description   : Header file for module H1DR5.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H1DR5_H
#define H1DR5_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H1DR5_MemoryMap.h"
#include "H1DR5_uart.h"
#include "H1DR5_gpio.h"
#include "H1DR5_dma.h"
#include "H1DR5_inputs.h"
#include "H1DR5_eeprom.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H1DR5


/* Port-related definitions */
#define	NumOfPorts			5

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 


/* Define available USARTs */

#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */

#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart5
#define P5uart &huart6


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

/* Module-specific Definitions */


#define NUM_MODULE_PARAMS		13

/*..........Enable User Data from external ports (like USB, Ethernet, BLE ...)......*/
#define __USER_DATA_BUFFER

/* Module EEPROM Variables */

// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* Module_Status Type Definition */
typedef enum {
	 H1DR5_OK =0, H1DR5_ERR_UnknownMessage,  H1DR5_ERR_WrongParams, H1DR5_ERROR =255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_3

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
extern void ExecuteMonitor(void);

/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */
extern Module_Status Ethernet_Receive_Data();
extern Module_Status EthernetSendData(char *data ,uint16_t length);
Module_Status Set_reseve_mac_and_ip_Remote();
extern Module_Status Set_Local_IP(uint8_t *IP);
extern Module_Status Set_Local_PORT(uint16_t to_port);
extern Module_Status Set_Remote_PORT(uint16_t from_port);
Module_Status Set_SubnetMask(uint8_t *SubnetMask);
Module_Status Set_Remote_IP(uint8_t *Gateway);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */
extern const CLI_Command_Definition_t CLI_Ethernet_Send_DataCommandDefinition;
extern const CLI_Command_Definition_t CLI_Set_IP_AddressCommandDefinition;
extern const CLI_Command_Definition_t CLI_Set_SubnetMaskCommandDefinition;
extern const CLI_Command_Definition_t CLI_Set_DefaultGatewayCommandDefinition;

#endif /* H1DR5_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/