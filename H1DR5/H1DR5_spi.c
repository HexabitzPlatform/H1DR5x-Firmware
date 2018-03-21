/**
  ******************************************************************************
  * File Name          : H1DR5_spi.c
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
		MODIFIED by Hexabitz for BitzOS (BOS) V0.0.0 - Copyright (C) 2017 Hexabitz
    All rights reserved
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure SPI                                                              */
/*----------------------------------------------------------------------------*/


SPI_HandleTypeDef hspi1;

/* Semaphores */
SemaphoreHandle_t SPIxSemaphoreHandle[2];

/* SPI1 init function */
void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 0;
    HAL_SPI_Init(&hspi1);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    __SPI1_CLK_ENABLE();
    
    /* ENC28J60 reset pin */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /* SPI pins configuration ************************************************/
    /*
        CS  ---------------------> PA4
        SCK ---------------------> PA5
        MISO --------------------> PA6
        MOSI --------------------> PA7
        INT ---------------------> PG2
    */

    /*Configure GPIO pins : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Deselect ENC28J60 module */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /*Configure GPIO pins : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    
    /*Configure GPIO pins : PA6 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    GPIO_InitStruct.Pull = GPIO_PULLUP;    
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
 	/* HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); */
    
    /*Configure GPIO pins : PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    
    /* Create semaphores to protect module SPIs (FreeRTOS vSemaphoreCreateBinary didn't work) */
#ifdef _Spi1	
	osSemaphoreDef(SemaphoreSpi1); SPIxSemaphoreHandle[Spi1Port] = osSemaphoreCreate(osSemaphore(SemaphoreSpi1), 1);
#endif
#ifdef _Spi2
	osSemaphoreDef(SemaphoreSpi2); SPIxSemaphoreHandle[Spi2Port] = osSemaphoreCreate(osSemaphore(SemaphoreSpi2), 1);
#endif

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{


} 

/* --- Get the SPI for a given port. 
*/
SPI_HandleTypeDef* GetSpi(uint8_t port)
{
	switch (port)
	{
	#ifdef _Spi1
		case Spi1Port : 
			return P1spi;	
	#endif
	#ifdef _Spi2
		case Spi2Port :
			return P2spi;
	#endif
		default:
			return 0;
	}		
}

/* --- Blocking (polling-based) read protected with a semaphore --- 
*/
HAL_StatusTypeDef readSPIxMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
	if (GetSpi(port) != NULL) {
		/* Wait for the semaphore to be available. */
		if (osSemaphoreWait(SPIxSemaphoreHandle[port], mutexTimeout) == osOK) {
			while( result != HAL_OK && result != HAL_TIMEOUT ) {
				result = HAL_SPI_Receive(GetSpi(port), (uint8_t *)buffer, n, portTimeout);
			}
			/* Give back the semaphore. */
			osSemaphoreRelease(SPIxSemaphoreHandle[port]);
		}
	}
	
	return result;
}

/* --- Blocking (polling-based) write protected with a semaphore --- 
*/
HAL_StatusTypeDef writeSPIxMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
	if (GetSpi(port) != NULL) {
		/*/ Wait for the semaphore to be available. */
		if (osSemaphoreWait(SPIxSemaphoreHandle[port], mutexTimeout) == osOK) {
			while( result != HAL_OK && result !=  HAL_TIMEOUT ) {
				result = HAL_SPI_Transmit(GetSpi(port), (uint8_t *)buffer, n, portTimeout);
			}
			/* Give back the semaphore. */
			osSemaphoreRelease(SPIxSemaphoreHandle[port]);
		}
	}
	
	return result;
}

/* --- Blocking (polling-based) simultaneously write, read protected with a semaphore --- 
*/
HAL_StatusTypeDef writereadSPIxMutex(uint8_t port, uint8_t *txbuffer, uint8_t *rxbuffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
    if((txbuffer != NULL) || (rxbuffer != NULL))
    {    
        if (GetSpi(port) != NULL) {
            /*/ Wait for the semaphore to be available. */
            if (osSemaphoreWait(SPIxSemaphoreHandle[port], mutexTimeout) == osOK) {
                while( result != HAL_OK && result !=  HAL_TIMEOUT ) {
                    result = HAL_SPI_TransmitReceive(GetSpi(port), (uint8_t *)txbuffer, \
                                                    (uint8_t *)rxbuffer, n, portTimeout);
                }
                /* Give back the semaphore. */
                osSemaphoreRelease(SPIxSemaphoreHandle[port]);
            }
        }
	}
    
	return result;
}

/* --- Non-blocking (interrupt-based) read protected with a semaphore --- 
*/
HAL_StatusTypeDef readSPIxITMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR; 
	
	if (GetSpi(port) != NULL) {
		/* Wait for the mutex to be available. */
		if (osSemaphoreWait(SPIxSemaphoreHandle[port], mutexTimeout) == osOK) {
			result = HAL_SPI_Receive_IT(GetSpi(port), (uint8_t *)buffer, n);
		}
	}
	
	return result;
}

/* --- Non-blocking (interrupt-based) write protected with a semaphore --- 
*/
HAL_StatusTypeDef writeSPIxITMutex(uint8_t port, uint8_t *buffer, uint16_t n, uint32_t mutexTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR; 

	if (GetSpi(port) != NULL) {	
		/* Wait for the mutex to be available. */
		if (osSemaphoreWait(SPIxSemaphoreHandle[port], mutexTimeout) == osOK) {
			result = HAL_SPI_Transmit_IT(GetSpi(port), (uint8_t *)buffer, n);
		}
	}
	
	return result;
}

/* --- Non-blocking (polling-based) simultaneously write, read protected with a semaphore --- 
*/
HAL_StatusTypeDef writereadSPIxITMutex(uint8_t port, uint8_t *txbuffer, uint8_t *rxbuffer, uint16_t n, uint32_t mutexTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
    if((txbuffer != NULL) || (rxbuffer != NULL))
    {    
        if (GetSpi(port) != NULL) {
            /*/ Wait for the semaphore to be available. */
            if (osSemaphoreWait(SPIxSemaphoreHandle[port], mutexTimeout) == osOK) {
                result = HAL_SPI_TransmitReceive_IT(GetSpi(port), (uint8_t *)txbuffer, \
                                                       (uint8_t *)rxbuffer, n);
            }
        }
	}
    
	return result;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
