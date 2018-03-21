/**
  ******************************************************************************
  * File Name          : H1DR5_spi.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
	 
/*
		MODIFIED by Hexabitz for BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"


/* Define variable SPIs */
#define _Spi1                       (1)
/* #define _Spi2                       (1) */

/* Port-SPI mapping */
#define P1spi &hspi1
/* #define P2spi &hspi2 */

/* Define macros use for SPIs */
#define Spi1Port        (0)
#define Spi2Port        (1)


/* Semaphore for SPIs */
extern SemaphoreHandle_t SPIxSemaphoreHandle[2];


/* External function prototypes -----------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
void MX_SPI1_Init(void);


HAL_StatusTypeDef readSPIxMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout);
HAL_StatusTypeDef writeSPIxMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout);
HAL_StatusTypeDef writereadSPIxMutex(uint8_t port, uint8_t *txbuffer, uint8_t *rxbuffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout);

HAL_StatusTypeDef readSPIxITMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout);
HAL_StatusTypeDef writeSPIxITMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout);
HAL_StatusTypeDef writereadSPIxITMutex(uint8_t port, uint8_t *txbuffer, uint8_t *rxbuffer, uint16_t n, uint32_t mutexTimeout);

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
