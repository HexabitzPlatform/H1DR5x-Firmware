/*
    BitzOS (BOS) V0.1.6 - Copyright (C) 2017-2019 Hexabitz
    All rights reserved
		
    File Name     : H1DR5_dma.h
    Description   : Peripheral DMA setup header file.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H1DR5_DMA_H
#define H1DR5_DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
	 
	 
/* Check which DMA interrupt occured */	 
#define HAL_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  ((((__HANDLE__)->ISR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)


/* External variables --------------------------------------------------------*/

/* Export DMA structs */
extern DMA_HandleTypeDef msgRxDMA[6];
extern DMA_HandleTypeDef msgTxDMA[3];
extern DMA_HandleTypeDef streamDMA[6];
extern DMA_HandleTypeDef frontendDMA[3];
extern CRC_HandleTypeDef hcrc;
	 
/* External function prototypes ----------------------------------------------*/
extern void DMA_Init(void);
extern void DMA_MSG_RX_CH_Init(DMA_HandleTypeDef *hDMA, DMA_Channel_TypeDef *ch);
extern void DMA_MSG_TX_CH_Init(DMA_HandleTypeDef *hDMA, DMA_Channel_TypeDef *ch);
extern void DMA_STREAM_CH_Init(DMA_HandleTypeDef *hDMA, DMA_Channel_TypeDef *ch);
extern void SetupMessagingRxDMAs(void);
extern void DMA_MSG_RX_Setup(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hDMA);
extern void DMA_MSG_TX_Setup(UART_HandleTypeDef *huart);
extern void DMA_MSG_TX_UnSetup(UART_HandleTypeDef *huart);
extern void CRC_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* H1DR5_DMA_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
