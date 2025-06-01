/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H1DR5.c
 Description   : Source code for module H1DR5.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H1DR5_inputs.h"
#include "H1DR5_spi.h"
#include "lan.h"

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

EthernetDefaultValues DefaultValue;

TaskHandle_t ProcessEthernetDataTaskHandle = NULL;

/* Private Variables *******************************************************/
uint8_t DataBuffer[MX_SIZE_USER_BUFFER] ={0};
uint8_t UserethernetData[MX_SIZE_USER_BUFFER] ={0};
uint16_t TXDataLength =0;
uint32_t ReceivedDataIndex =0;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={0};

/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void ProcessEthernetDataTask(void *argument);

/* Create CLI commands *****************************************************/
portBASE_TYPE CLI_EthernetSendDataCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetLocalIPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetSubnetMaskCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetRemoteIPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_DefaultValuesCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetRemoteIPRemoteMACCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetLocalPORTCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetRemotePORTCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
/* CLI command structure : EthernetSendData */
const CLI_Command_Definition_t CLI_EthernetSendDataCommandDefinition =
{
	( const int8_t * ) "ethernetsenddata", /* The command string to type. */
	( const int8_t * ) "ethernetsenddata :\r\n Parameters required to execute a EthernetSendData: my data \r\n\r\n",
	CLI_EthernetSendDataCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : SetLocalIP */
const CLI_Command_Definition_t CLI_SetLocalIPCommandDefinition =
{
	( const int8_t * ) "setlocalip", /* The command string to type. */
	( const int8_t * ) "setlocalip :\r\n Parameters required to execute a SetLocalIP: localiP is  \r\n\r\n",
	CLI_SetLocalIPCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : SetSubnetMask */
const CLI_Command_Definition_t CLI_SetSubnetMaskCommandDefinition =
{
	( const int8_t * ) "setsubnetmask", /* The command string to type. */
	( const int8_t * ) "setsubnetmask :\r\n Parameters required to execute a SetSubnetMask: my SubnetMask \r\n\r\n",
	CLI_SetSubnetMaskCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : SetRemoteIP */
const CLI_Command_Definition_t CLI_SetRemoteIPCommandDefinition =
{
	( const int8_t * ) "setremoteip", /* The command string to type. */
	( const int8_t * ) "setremoteip :\r\n Parameters required to execute a SetRemoteIP: my RemoteIP is \r\n\r\n",
	CLI_SetRemoteIPCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : DefaultValues */
const CLI_Command_Definition_t CLI_DefaultValuesCommandDefinition =
{
	( const int8_t * ) "defaultvalues", /* The command string to type. */
	( const int8_t * ) "defaultvalues :\r\n Parameters required to execute a DefaultValues: my DefaultValues \r\n\r\n",
	CLI_DefaultValuesCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : SetRemoteIPRemoteMAC */
const CLI_Command_Definition_t CLI_SetRemoteIPRemoteMACCommandDefinition =
{
	( const int8_t * ) "setremoteipremotemac", /* The command string to type. */
	( const int8_t * ) "setremoteipremotemac :\r\n Parameters required to execute a SetRemoteIPRemoteMAC: RemoteIPRemoteMAC \r\n\r\n",
	CLI_SetRemoteIPRemoteMACCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : SetLocalPORT */
const CLI_Command_Definition_t CLI_SetLocalPORTCommandDefinition =
{
	( const int8_t * ) "setlocalport", /* The command string to type. */
	( const int8_t * ) "setlocalport :\r\n Parameters required to execute a SetLocalPORT: LocalPORT is \r\n\r\n",
	CLI_SetLocalPORTCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : SetRemotePORT */
const CLI_Command_Definition_t CLI_SetRemotePORTCommandDefinition =
{
	( const int8_t * ) "setremoteport", /* The command string to type. */
	( const int8_t * ) "setremoteport :\r\n Parameters required to execute a SetRemotePORT: RemotePORT is \r\n\r\n",
	CLI_SetRemotePORTCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;

	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/***************************************************************************/
/* H1DR5 module initialization */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	EthernetGPIOInit();
	MX_SPI1_Init();
	lan_init();

    /*Circulating DMA Channels ON All Module */
	for(int i =1; i <= NUM_OF_PORTS; i++){
		if(GetUart(i) == &huart1){
			dmaIndex[i - 1] =&(DMA1_Channel1->CNDTR);
		}
		else if(GetUart(i) == &huart2){
			dmaIndex[i - 1] =&(DMA1_Channel2->CNDTR);
		}
		else if(GetUart(i) == &huart3){
			dmaIndex[i - 1] =&(DMA1_Channel3->CNDTR);
		}
		else if(GetUart(i) == &huart4){
			dmaIndex[i - 1] =&(DMA1_Channel4->CNDTR);
		}
		else if(GetUart(i) == &huart5){
			dmaIndex[i - 1] =&(DMA1_Channel5->CNDTR);
		}
		else if(GetUart(i) == &huart6){
			dmaIndex[i - 1] =&(DMA1_Channel6->CNDTR);
		}
	}
	/* Create module special task (if needed) */
	if(ProcessEthernetDataTaskHandle == NULL)
		xTaskCreate(ProcessEthernetDataTask,(const char* )"ProcessEthernetDataTask",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&ProcessEthernetDataTaskHandle);

}

/***************************************************************************/
/* H1DR5 message processing task */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H1DR5_OK;

	uint8_t LocalIP[4] ={};
	uint8_t RemoteIP[4] ={};
	uint8_t LocalPORT;
	uint8_t RemotePORT;
	uint8_t Subnet[4] ={};
	uint8_t IpGate[4] ={};

	switch(code){

		case CODE_H1DR5_ETHERNET_SEND_DATA:
			TXDataLength =(uint16_t )cMessage[port - 1][shift];
			EthernetSendData(&cMessage[port - 1][1 + shift],TXDataLength);
			break;

		case CODE_H1DR5_SET_LOCAL_IP:
			LocalIP[0] =cMessage[port - 1][0 + shift];
			LocalIP[1] =cMessage[port - 1][1 + shift];
			LocalIP[2] =cMessage[port - 1][2 + shift];
			LocalIP[3] =cMessage[port - 1][3 + shift];
			SetLocalIP(LocalIP);
			break;

		case CODE_H1DR5_SET_REMOTE_IP:
			RemoteIP[0] =cMessage[port - 1][0 + shift];
			RemoteIP[1] =cMessage[port - 1][1 + shift];
			RemoteIP[2] =cMessage[port - 1][2 + shift];
			RemoteIP[3] =cMessage[port - 1][3 + shift];
			SetRemoteIP(RemoteIP);
			break;

		case CODE_H1DR5_SET_SUBNET_MASK:
			Subnet[0] =cMessage[port - 1][0 + shift];
			Subnet[1] =cMessage[port - 1][1 + shift];
			Subnet[2] =cMessage[port - 1][2 + shift];
			Subnet[3] =cMessage[port - 1][3 + shift];
			SetSubnetMask(Subnet);
			break;

		case CODE_H1DR5_SET_LOCAL_PORT:
			Local_PORT =cMessage[port - 1][0 + shift];
			SetLocalPORT(LocalPORT);
			break;

		case CODE_H1DR5_SET_REMOTE_PORT:
			Remote_PORT =cMessage[port - 1][0 + shift];
			SetRemotePORT(RemotePORT);
			break;

		case CODE_H1DR5_SET_REMOTE_IP_REMOTE_MAC:
			SetRemoteIPRemoteMAC();
			break;
		case CODE_H1DR5_DEFAULT_VALUES:
//			 DefaultValues();
//			 memcpy(&messageParams[0], DefaultValue.LocalMac, sizeof(DefaultValue.LocalMac));
//			 memcpy(&messageParams[6],DefaultValue.RemoteMac, sizeof(DefaultValue.RemoteMac));
//			 memcpy(&messageParams[12], DefaultValue.LocalIP, sizeof(DefaultValue.LocalIP));
//			 memcpy(&messageParams[16], DefaultValue.RemoteIP, sizeof(DefaultValue.RemoteIP));
//			 memcpy(&messageParams[20], DefaultValue.SubnetMask, sizeof(DefaultValue.SubnetMask));
//             memcpy(&messageParams[24], DefaultValue.DestIP, sizeof(DefaultValue.DestIP));
//			 messageParams[28]=DefaultValue.LocalPort;
//			 messageParams[29]=DefaultValue.RemotePort ;
//			 SendMessageToModule(src, CODE_H1DR5_RECEIVE_DEFAULT_VALUE, 30);
			break;

		default:
			result =H1DR5_ERR_UNKNOWNMESSAGE;
			break;
	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART5)
		return P4;
	else if(huart->Instance == USART6)
		return P5;

	return 0;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&CLI_EthernetSendDataCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SetLocalIPCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SetSubnetMaskCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SetRemoteIPCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_DefaultValuesCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SetRemoteIPRemoteMACCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SetLocalPORTCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SetRemotePORTCommandDefinition);
}

/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex,float *value){
	Module_Status status =BOS_OK;

	switch(paramIndex){

		/* Invalid parameter index */
		default:
			status =BOS_ERR_WrongParam;
			break;
	}

	return status;
}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* EthernetTask function */
void ProcessEthernetDataTask(void *argument){

	for(;;){
		lan_poll(DataBuffer,&TXDataLength);
		Delay_ms(10);
		for(uint16_t i =0; i < TXDataLength; i++){
			IND_ON();
			Delay_ms(10);
			IND_OFF();
			Delay_ms(10);
			UserethernetData[ReceivedDataIndex] =DataBuffer[i];
			ReceivedDataIndex++;
			if(ReceivedDataIndex == MX_SIZE_USER_BUFFER)
				ReceivedDataIndex =0;
		}

		TXDataLength =0;

		taskYIELD();
	}
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/* Send data from Ethernet module */
Module_Status EthernetSendData(char *data,uint16_t length){
	Module_Status status =H1DR5_OK;

	if(data != NULL && length != 0){
		ether_send_udp(data,length);
		memset(data,0,length);
	}
	else{
		status =H1DR5_ERROR;
	}
	return status;
}

/***************************************************************************/
/* Set the connection settings */
Module_Status SetRemoteIPRemoteMAC(void){
	Module_Status status =H1DR5_OK;
	EthernetSendData("0",1);
	Delay_ms(10);
	return status;
}

/***************************************************************************/
/* SetLocalPORT */
Module_Status SetLocalPORT(uint8_t localPort){
	Module_Status status =H1DR5_OK;
	if(localPort > 255 || localPort < 1){
		status =H1DR5_ERROR;
	}
	Local_PORT =localPort;
	return status;
}

/***************************************************************************/
/* SetRemotePORT */
Module_Status SetRemotePORT(uint8_t remotePort){
	Module_Status status =H1DR5_OK;
	if(remotePort > 255 || remotePort < 1){
		status =H1DR5_ERROR;
	}
	Remote_PORT =remotePort;
	return status;
}

/***************************************************************************/
/* SetLocalMAC */
Module_Status SetLocalMAC(uint8_t *localMAC){
	Module_Status status =H1DR5_OK;

	for(int i =0; i < 6; i++){
		mac_addr[i] =localMAC[i];
	}

	return status;
}

/***************************************************************************/
/* Set IP address of the Ethernet module */
Module_Status SetLocalIP(uint8_t *localIP){
	Module_Status status =H1DR5_OK;
	uint32_t ip[4] ={};

	if(localIP == NULL)
		status =H1DR5_ERROR;

	for(int i =0; i < 4; i++){
		ip[i] =localIP[i];
	}

	Local_IP =inet_addr(ip[0],ip[1],ip[2],ip[3]);

	return status;

}

/***************************************************************************/
/* Set SetSubnet Mask of the Ethernet network */
Module_Status SetSubnetMask(uint8_t *SubnetMask){
	Module_Status status =H1DR5_OK;
	uint32_t subnetmask[4] ={};

	if(SubnetMask == NULL)
		status =H1DR5_ERROR;

	for(int i =0; i < 4; i++){
		subnetmask[i] =SubnetMask[i];
	}

	ip_mask =inet_addr(subnetmask[0],subnetmask[1],subnetmask[2],subnetmask[3]);

	return status;

}

/***************************************************************************/
/* Set the default gateway of the device to which the Ethernet module is connected */
Module_Status SetRemoteIP(uint8_t *remoteIP){
	Module_Status status =H1DR5_OK;
	uint32_t gateway[4] ={};

	if(remoteIP == NULL)
		status =H1DR5_ERROR;

	for(int i =0; i < 4; i++){
		gateway[i] =remoteIP[i];
	}

	Remote_IP =inet_addr(gateway[0],gateway[1],gateway[2],gateway[3]);
	ip_dest =Remote_IP;

	return status;

}

/***************************************************************************/
/* View Default_Values */
Module_Status DefaultValues(void){
	Module_Status status =H1DR5_OK;

	uint8_t Localip[4];
	uint8_t RemoteiP[4];
	uint8_t Destip[4];
	uint8_t Subnetmask[4];

	memcpy(DefaultValue.LocalMac,mac_addr,sizeof(mac_addr));
	memcpy(DefaultValue.RemoteMac,arp_cache[0].mac_addr,sizeof(mac_addr));

	Localip[0] =Local_IP;
	Localip[1] =(Local_IP >> 8);
	Localip[2] =(Local_IP >> 16);
	Localip[3] =(Local_IP >> 24);
	memcpy(DefaultValue.LocalIP,Localip,sizeof(Localip));

	RemoteiP[0] =Remote_IP;
	RemoteiP[1] =(Remote_IP >> 8);
	RemoteiP[2] =(Remote_IP >> 16);
	RemoteiP[3] =(Remote_IP >> 24);
	memcpy(DefaultValue.RemoteIP,RemoteiP,sizeof(RemoteiP));

	Subnetmask[0] =ip_mask;
	Subnetmask[1] =(ip_mask >> 8);
	Subnetmask[2] =(ip_mask >> 16);
	Subnetmask[3] =(ip_mask >> 24);
	memcpy(DefaultValue.SubnetMask,Subnetmask,sizeof(Subnetmask));

	Destip[0] =ip_dest;
	Destip[1] =(ip_dest >> 8);
	Destip[2] =(ip_dest >> 16);
	Destip[3] =(ip_dest >> 24);
	memcpy(DefaultValue.DestIP,Destip,sizeof(Destip));

	DefaultValue.LocalPort =Local_PORT;
	DefaultValue.RemotePort =Remote_PORT;

	return status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
portBASE_TYPE CLI_EthernetSendDataCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;

	static const int8_t *pcOKMessage=(int8_t* )"Ethernet is on \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );

	status =EthernetSendData(pcParameterString1,xParameterStringLength1);
	if(status == H1DR5_OK){
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,pcParameterString1);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_SetRemoteIPRemoteMACCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status =H1DR5_OK;

	static const int8_t *pcOKMessage =(int8_t* )"The connection has been opened \r\n  \n\r";

	(void )xWriteBufferLen;

	SetRemoteIPRemoteMAC();
	if(status == H1DR5_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_SetLocalIPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	int size=15;
	portBASE_TYPE xParameterStringLength1 =0;

	char Local_IP[size];
	uint8_t IP[4]={};
	int x;
	int k=0;
	int r=0,f=0;

	static const int8_t *pcOKMessage=(int8_t* )"The LocalIP has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );

	for(int y =0; y < xParameterStringLength1; y++){
		Local_IP[y] =(char )pcParameterString1[y];
	}

	size =xParameterStringLength1 - 1;
	x =xParameterStringLength1 - 1;

	for(int i =3; i >= 0; i--){
		while(x >= 0){
			if(Local_IP[x] == '.'){
				k =0;
				x--;
				break;
			}
			else{
				r =(Local_IP[x] - '0');
				f =pow(10,k);
				IP[i] +=r * f;
				k++;
				x--;
			}
		}
	}

	SetLocalIP(IP);

	if(status == H1DR5_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_SetSubnetMaskCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	int size=15;
	portBASE_TYPE xParameterStringLength1 =0;

	char SubnetMask[size];
	uint8_t Subnet[4]={};
	int x;
	int k=0;
	int r=0,f=0;

	static const int8_t *pcOKMessage=(int8_t* )"The SubnetMask has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );

	for(int y =0; y < xParameterStringLength1; y++){
		SubnetMask[y] =(char )pcParameterString1[y];
	}

	size =xParameterStringLength1 - 1;
	x =xParameterStringLength1 - 1;

	for(int i =3; i >= 0; i--){
		while(x >= 0){
			if(SubnetMask[x] == '.'){
				k =0;
				x--;
				break;
			}
			else{

				r =(SubnetMask[x] - '0');
				f =pow(10,k);
				Subnet[i] +=r * f;
				k++;
				x--;
			}
		}
	}

	SetSubnetMask(Subnet);

	if(status == H1DR5_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_SetRemoteIPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	int size=15;
	portBASE_TYPE xParameterStringLength1 =0;

	char Remote_IP[size];
	uint8_t Gateway[4]={};
	int x;
	int k=0;
	int r=0,f=0;

	static const int8_t *pcOKMessage=(int8_t* )"The RemoteIP has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );

	for(int y =0; y < xParameterStringLength1; y++){
		Remote_IP[y] =(char )pcParameterString1[y];
	}
	size =xParameterStringLength1 - 1;
	x =xParameterStringLength1 - 1;

	for(int i =3; i >= 0; i--){

		while(x >= 0){
			if(Remote_IP[x] == '.'){
				k =0;
				x--;
				break;
			}
			else{
				r =(Remote_IP[x] - '0');
				f =pow(10,k);
				Gateway[i] +=r * f;
				k++;

				x--;
			}
		}
	}

	SetRemoteIP(Gateway);

	if(status == H1DR5_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_DefaultValuesCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status =H1DR5_OK;

	static const int8_t *pcMessage1 =(int8_t* )"the DefaultValues of Params: \r\n"
		"the LocalMac: %d.%d.%d.%d.%d.%d\r\n"
		"the RemoteMac: %d.%d.%d.%d.%d.%d\r\n"
		"the LocalIP: %d.%d.%d.%d\r\n"
		"the RemoteIP: %d.%d.%d.%d\r\n"
		"the SubnetMask: %d.%d.%d.%d\r\n"
		"the DestIP: %d.%d.%d.%d\r\n"
		"the LocalPort: %d\r\n"
		"the RemotePort: %d\r\n";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	status =DefaultValues();

	if(status == H1DR5_OK){
		sprintf((char* )pcWriteBuffer,(char* )pcMessage1,DefaultValue.LocalMac[0],
			DefaultValue.LocalMac[1],DefaultValue.LocalMac[2],DefaultValue.LocalMac[3],
			DefaultValue.LocalMac[4],DefaultValue.LocalMac[5],DefaultValue.RemoteMac[0],
			DefaultValue.RemoteMac[1],DefaultValue.RemoteMac[2],DefaultValue.RemoteMac[3],
			DefaultValue.RemoteMac[4],DefaultValue.RemoteMac[5],DefaultValue.LocalIP[0],
			DefaultValue.LocalIP[1],DefaultValue.LocalIP[2],DefaultValue.LocalIP[3],
			DefaultValue.RemoteIP[0],DefaultValue.RemoteIP[1],DefaultValue.RemoteIP[2],
			DefaultValue.RemoteIP[3],DefaultValue.SubnetMask[0],DefaultValue.SubnetMask[1],
			DefaultValue.SubnetMask[2],DefaultValue.SubnetMask[3],DefaultValue.DestIP[0],
			DefaultValue.DestIP[1],DefaultValue.DestIP[2],DefaultValue.DestIP[3],
			DefaultValue.LocalPort,DefaultValue.RemotePort);
	}

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_SetLocalPORTCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	uint8_t LocalPORT =0;
	static const int8_t *pcOKMessage=(int8_t* )"The LocalPORT has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	LocalPORT =(uint8_t )atol((char* )pcParameterString1);
	status =SetLocalPORT(LocalPORT);

	if(status == H1DR5_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,pcParameterString1);

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_SetRemotePORTCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	uint8_t RemotePORT =0;
	static const int8_t *pcOKMessage=(int8_t* )"The RemotePORT has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	RemotePORT =(uint8_t )atol((char* )pcParameterString1);
	status=SetRemotePORT(RemotePORT);
	if(status == H1DR5_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,pcParameterString1);

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
