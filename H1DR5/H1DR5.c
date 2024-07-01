/*
 BitzOS (BOS) V0.3.4 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H1DR5.c
 Description   : Source code for module H1DR5.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H1DR5_inputs.h"
#include"H1DR5_spi.h"
#include "lan.h"





/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Module exported parameters ------------------------------------------------*/
uint16_t length;


module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};
defaultValues defaultValue;
/* Private variables ---------------------------------------------------------*/
#define MX_SIZE_USER_BUFFER 512
 uint16_t length;
 uint8_t UserethernetData[MX_SIZE_USER_BUFFER]={0};
 uint8_t DataBuffer[MX_SIZE_USER_BUFFER]={0};
 uint32_t indexInput=0;
 uint32_t indexProcess=0;
 TaskHandle_t ProcessEthernetDataTaskHandle = NULL;

/* Private function prototypes -----------------------------------------------*/
void  ether_send_udp(char *data ,uint16_t length);
void lan_poll(uint8_t* pData,uint16_t* length);
void ProcessEthernetDataTask(void *argument);
void ExecuteMonitor(void);

/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_EthernetSendDataCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetLocalIPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetSubnetMaskCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetRemoteIPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_DefaultValuesCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetRemoteIPRemoteMACCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetLocalPORTCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_SetRemotePORTCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);


/* CLI command structure : EthernetSendData */
const CLI_Command_Definition_t CLI_EthernetSendDataCommandDefinition =
{
	( const int8_t * ) "ethernetsenddata", /* The command string to type. */
	( const int8_t * ) "ethernetsenddata :\r\n Parameters required to execute a EthernetSendData: my data \r\n\r\n",
	CLI_EthernetSendDataCommand, /* The function to run. */
	1 /* one parameters are expected. */
};
/* CLI command structure : SetLocalIP */
const CLI_Command_Definition_t CLI_SetLocalIPCommandDefinition =
{
	( const int8_t * ) "setlocalip", /* The command string to type. */
	( const int8_t * ) "setlocalip :\r\n Parameters required to execute a SetLocalIP: localiP is  \r\n\r\n",
	CLI_SetLocalIPCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* CLI command structure : SetSubnetMask */
const CLI_Command_Definition_t CLI_SetSubnetMaskCommandDefinition =
{
	( const int8_t * ) "setsubnetmask", /* The command string to type. */
	( const int8_t * ) "setsubnetmask :\r\n Parameters required to execute a SetSubnetMask: my SubnetMask \r\n\r\n",
	CLI_SetSubnetMaskCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* CLI command structure : SetRemoteIP */
const CLI_Command_Definition_t CLI_SetRemoteIPCommandDefinition =
{
	( const int8_t * ) "setremoteip", /* The command string to type. */
	( const int8_t * ) "setremoteip :\r\n Parameters required to execute a SetRemoteIP: my RemoteIP is \r\n\r\n",
	CLI_SetRemoteIPCommand, /* The function to run. */
	1 /* one parameters are expected. */
};
/* CLI command structure : DefaultValues */
const CLI_Command_Definition_t CLI_DefaultValuesCommandDefinition =
{
	( const int8_t * ) "defaultvalues", /* The command string to type. */
	( const int8_t * ) "defaultvalues :\r\n Parameters required to execute a DefaultValues: my DefaultValues \r\n\r\n",
	CLI_DefaultValuesCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/* CLI command structure : SetRemoteIPRemoteMAC */
const CLI_Command_Definition_t CLI_SetRemoteIPRemoteMACCommandDefinition =
{
	( const int8_t * ) "setremoteipremotemac", /* The command string to type. */
	( const int8_t * ) "setremoteipremotemac :\r\n Parameters required to execute a SetRemoteIPRemoteMAC: RemoteIPRemoteMAC \r\n\r\n",
	CLI_SetRemoteIPRemoteMACCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

/* CLI command structure : SetLocalPORT */
const CLI_Command_Definition_t CLI_SetLocalPORTCommandDefinition =
{
	( const int8_t * ) "setlocalport", /* The command string to type. */
	( const int8_t * ) "setlocalport :\r\n Parameters required to execute a SetLocalPORT: LocalPORT is \r\n\r\n",
	CLI_SetLocalPORTCommand, /* The function to run. */
	1 /* one parameters are expected. */
};
/* CLI command structure : SetRemotePORT */
const CLI_Command_Definition_t CLI_SetRemotePORTCommandDefinition =
{
	( const int8_t * ) "setremoteport", /* The command string to type. */
	( const int8_t * ) "setremoteport :\r\n Parameters required to execute a SetRemotePORT: RemotePORT is \r\n\r\n",
	CLI_SetRemotePORTCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 -----------------------------------------------------------------------
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 12;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
      HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;

	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);

}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO ---
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =8, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

	HAL_FLASH_Unlock();

	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1,RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
		       	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS + add,array[i - 1][j]);
				 //HALFWORD 	//TOBECHECKED
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						add +=8;
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				//HALFWORD
				//TOBECHECKED
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
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[s].cmd + j*4 ));
				//HALFWORD
				//TOBECHECKED
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

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO ---
 */
uint8_t ClearROtopology(void){
	// Clear the array
	memset(array,0,sizeof(array));
	N =1;
	myID =0;

	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
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

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H1DR5 module initialization.
 */
void Module_Peripheral_Init(void){

	/* Array ports */

	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	Ethernet_GPIO_Init();
	MX_SPI1_Init();
	lan_init();

//Circulating DMA Channels ON All Module
	 for(int i=1;i<=NumOfPorts;i++)
		{
		  if(GetUart(i)==&huart1)
				   { index_dma[i-1]=&(DMA1_Channel1->CNDTR); }
		  else if(GetUart(i)==&huart2)
				   { index_dma[i-1]=&(DMA1_Channel2->CNDTR); }
		  else if(GetUart(i)==&huart3)
				   { index_dma[i-1]=&(DMA1_Channel3->CNDTR); }
		  else if(GetUart(i)==&huart4)
				   { index_dma[i-1]=&(DMA1_Channel4->CNDTR); }
		  else if(GetUart(i)==&huart5)
				   { index_dma[i-1]=&(DMA1_Channel5->CNDTR); }
		  else if(GetUart(i)==&huart6)
				   { index_dma[i-1]=&(DMA1_Channel6->CNDTR); }
		}
	 /* Create module special task (if needed) */
	 if(ProcessEthernetDataTaskHandle == NULL)
	 xTaskCreate(ProcessEthernetDataTask,(const char* ) "ProcessEthernetDataTask",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&ProcessEthernetDataTaskHandle);

}

/*-----------------------------------------------------------*/
/* --- H1DR5 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H1DR5_OK;

	uint8_t LocalIP[4]={};
	uint8_t RemoteIP[4]={};
	uint8_t LocalPORT;
	uint8_t RemotePORT;
	uint8_t Subnet[4]={};
	uint8_t IpGate[4]={};
	switch(code){
		case CODE_H1DR5_EthernetSendData:
			length=(uint16_t)cMessage[port - 1][shift];
			EthernetSendData(&cMessage[port - 1][1+shift],length);
			break;
		case CODE_H1DR5_SetLocalIP:
			LocalIP[0]= cMessage[port - 1][0 + shift];
			LocalIP[1]= cMessage[port - 1][1 + shift];
			LocalIP[2]= cMessage[port - 1][2 + shift];
			LocalIP[3]= cMessage[port - 1][3 + shift];
			SetLocalIP(LocalIP);
             break;
		case CODE_H1DR5_SetRemoteIP:
			RemoteIP[0]= cMessage[port - 1][0 + shift];
			RemoteIP[1]= cMessage[port - 1][1 + shift];
			RemoteIP[2]= cMessage[port - 1][2 + shift];
			RemoteIP[3]= cMessage[port - 1][3 + shift];
			SetRemoteIP(RemoteIP);
			 break;
		case CODE_H1DR5_SetSubnetMask:
			Subnet[0]= cMessage[port - 1][0 + shift];
			Subnet[1]= cMessage[port - 1][1 + shift];
			Subnet[2]= cMessage[port - 1][2 + shift];
			Subnet[3]= cMessage[port - 1][3 + shift];
			SetSubnetMask(Subnet);
			 break;
		case CODE_H1DR5_SetLocalPORT:
			Local_PORT= cMessage[port - 1][0 + shift];
			SetLocalPORT(LocalPORT);
			 break;
		case CODE_H1DR5_SetRemotePORT:
			Remote_PORT= cMessage[port - 1][0 + shift];
			SetRemotePORT(RemotePORT);
			 break;
		case CODE_H1DR5_SetRemoteIPRemoteMAC:
			SetRemoteIPRemoteMAC();
		case CODE_H1DR5_DefaultValues:
//			 DefaultValues();
//			 memcpy(&messageParams[0], defaultValue.LocalMac, sizeof(defaultValue.LocalMac));
//			 memcpy(&messageParams[6],defaultValue.RemoteMac, sizeof(defaultValue.RemoteMac));
//			 memcpy(&messageParams[12], defaultValue.LocalIP, sizeof(defaultValue.LocalIP));
//			 memcpy(&messageParams[16], defaultValue.RemoteIP, sizeof(defaultValue.RemoteIP));
//			 memcpy(&messageParams[20], defaultValue.SubnetMask, sizeof(defaultValue.SubnetMask));
//             memcpy(&messageParams[24], defaultValue.DestIP, sizeof(defaultValue.DestIP));
//			 messageParams[28]=defaultValue.LocalPort;
//			 messageParams[29]=defaultValue.RemotePort ;
//			 SendMessageToModule(src, CODE_H1DR5_receive_Defalt_Value, 30);
		default:
			result =H1DR5_ERR_UnknownMessage;
			break;
	}

	return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
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

/*-----------------------------------------------------------*/
/* --- Get the port for a given UART.
 */
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
	else if(huart->Instance == USART1)
		return P6;

	return 0;
}
/* EthernetTask function */
void ProcessEthernetDataTask(void *argument){

	for(;;){
	lan_poll(DataBuffer, &length);
	Delay_ms(10);
	for(uint16_t i=0; i<length; i++){
		IND_ON();
		Delay_ms(10);
		IND_OFF();
		Delay_ms(10);
		UserethernetData[indexInput]=DataBuffer[i];
		indexInput++;
		if(indexInput==MX_SIZE_USER_BUFFER)
			indexInput=0;
	}

	length=0;

	taskYIELD();
	}
}
/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */

/*
       Send data from Ethernet module
 */
Module_Status EthernetSendData(char *data ,uint16_t length){
	Module_Status status=H1DR5_OK;

	if(data!=NULL && length!=0){
		ether_send_udp(data,length);
		 memset (data, 0, length);
	}
	else{
		status=H1DR5_ERROR;
	}
	return status;
}

/*-----------------------------------------------------------*/

/*
 * Set the connection settings
 */
Module_Status SetRemoteIPRemoteMAC()
{
	Module_Status status=H1DR5_OK;
 EthernetSendData("0",1);
     Delay_ms(10);
     return status;
}
/*-----------------------------------------------------------*/

/*
     SetLocalPORT
 */
Module_Status SetLocalPORT(uint8_t localPort){
	Module_Status status=H1DR5_OK;
	if(localPort>255 || localPort<1)
	{
			status=H1DR5_ERROR;
	}
	Local_PORT=localPort;
	return status;
}
/*-----------------------------------------------------------*/

/*
     SetRemotePORT
 */
Module_Status SetRemotePORT(uint8_t remotePort){
	Module_Status status=H1DR5_OK;
	if(remotePort>255 || remotePort<1)
	{
			status=H1DR5_ERROR;
	}
	Remote_PORT=remotePort;
	return status;
}
/*-----------------------------------------------------------*/

/*
      SetLocalMAC
 */
Module_Status SetLocalMAC(uint8_t *localMAC){
	Module_Status status=H1DR5_OK;

	for(int i=0; i<6;i++){

		mac_addr[i]=localMAC[i];
		}


	return status;
}
/*-----------------------------------------------------------*/

/*
    Set IP address of the Ethernet module.
 */
Module_Status SetLocalIP(uint8_t *localIP){
	Module_Status status=H1DR5_OK;
	uint32_t ip[4]={};

	if(localIP==NULL){
		status=H1DR5_ERROR;
	}
	for(int i=0; i<4;i++){
		ip[i]=localIP[i];

	}

	Local_IP=inet_addr(ip[0],ip[1],ip[2],ip[3]);

	return status;

}
/*-----------------------------------------------------------*/

/*
     Set SetSubnet Mask of the Ethernet network.
 */
Module_Status SetSubnetMask(uint8_t *SubnetMask){
	Module_Status status=H1DR5_OK;
	uint32_t subnetmask[4]={};

	if(SubnetMask==NULL){
		status=H1DR5_ERROR;
	}
	for(int i=0; i<4;i++){
		subnetmask[i]=SubnetMask[i];

	}

	ip_mask=inet_addr(subnetmask[0],subnetmask[1],subnetmask[2],subnetmask[3]);

	return status;

}
/*-----------------------------------------------------------*/

/*
 Set the default gateway of the device to which the Ethernet module is connected
 */
Module_Status SetRemoteIP(uint8_t *remoteIP){
	Module_Status status=H1DR5_OK;
	uint32_t gateway[4]={};
	if(remoteIP==NULL){
		status=H1DR5_ERROR;
	}
	for(int i=0; i<4;i++){
		gateway[i]=remoteIP[i];
	}

	Remote_IP=inet_addr(gateway[0],gateway[1],gateway[2],gateway[3]);
	ip_dest=Remote_IP;
	return status;

}
/*-----------------------------------------------------------*/
/*
   View Default_Values
 */
Module_Status DefaultValues(){
	Module_Status status=H1DR5_OK;

	 memcpy(defaultValue.LocalMac, mac_addr, sizeof(mac_addr));
	 memcpy(defaultValue.RemoteMac, arp_cache[0].mac_addr, sizeof(mac_addr));
	 uint8_t Localip[4];
	 Localip[0]=Local_IP;
	 Localip[1]=(Local_IP >> 8);
	 Localip[2]=(Local_IP >> 16);
	 Localip[3]=(Local_IP >> 24);
	 memcpy(defaultValue.LocalIP, Localip, sizeof(Localip));
	 uint8_t RemoteiP[4];
	 RemoteiP[0]=Remote_IP;
	 RemoteiP[1]=(Remote_IP >> 8);
	 RemoteiP[2]=(Remote_IP >> 16);
	 RemoteiP[3]=(Remote_IP >> 24);
	 memcpy(defaultValue.RemoteIP, RemoteiP, sizeof(RemoteiP));
	 uint8_t Subnetmask[4];
	 Subnetmask[0]=ip_mask;
	 Subnetmask[1]=(ip_mask >> 8);
	 Subnetmask[2]=(ip_mask >> 16);
	 Subnetmask[3]=(ip_mask >> 24);
	 memcpy(defaultValue.SubnetMask, Subnetmask, sizeof(Subnetmask));
	 uint8_t Destip[4];
	 Destip[0]=ip_dest;
	 Destip[1]=(ip_dest >> 8);
	 Destip[2]=(ip_dest >> 16);
	 Destip[3]=(ip_dest >> 24);
	 memcpy(defaultValue.DestIP, Destip, sizeof(Destip));

	 defaultValue.LocalPort = Local_PORT;
	 defaultValue.RemotePort = Remote_PORT;
	 return status;
}

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
portBASE_TYPE CLI_EthernetSendDataCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;

	static const int8_t *pcOKMessage=(int8_t* )"Ethernet is on \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );


	status=EthernetSendData(pcParameterString1, xParameterStringLength1);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,pcParameterString1);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_SetRemoteIPRemoteMACCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static const int8_t *pcOKMessage=(int8_t* )"The connection has been opened \r\n  \n\r";


	(void )xWriteBufferLen;

	SetRemoteIPRemoteMAC();
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	}

	return pdFALSE;

}

/*-----------------------------------------------------------*/
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

	for(int y=0;y<xParameterStringLength1;y++)
	{
		Local_IP[y] = (char )pcParameterString1[y];
	}

	size=xParameterStringLength1-1;
	x=xParameterStringLength1-1;
	for(int i=3;i>=0;i--)
	{
		while(x>=0)
		{
			if(Local_IP[x]=='.')
			{
				k=0;
				x--;
				break;
			}
			else
			{
				r=(Local_IP[x]-'0');
				f=pow(10,k);
				IP[i]+=r*f;
				k++;
				x--;
			}
		}
	 }

	SetLocalIP(IP);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);
	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;

}
/*-----------------------------------------------------------*/
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

	for(int y=0;y<xParameterStringLength1;y++){
		SubnetMask[y] = (char )pcParameterString1[y];
	}

	size=xParameterStringLength1-1;
	x=xParameterStringLength1-1;
	for(int i=3;i>=0;i--){

		while(x>=0){
			if(SubnetMask[x]=='.'){
				k=0;
				x--;
				break;

			}
			else{

				r=(SubnetMask[x]-'0');
				f=pow(10,k);
				Subnet[i]+=r*f;
				k++;

				x--;
			}
		}


		 }

	SetSubnetMask(Subnet);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;

}
/*-----------------------------------------------------------*/
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

	for(int y=0;y<xParameterStringLength1;y++){
		Remote_IP[y] = (char )pcParameterString1[y];
	}
	size=xParameterStringLength1-1;
	x=xParameterStringLength1-1;
	for(int i=3;i>=0;i--){

		while(x>=0){
			if(Remote_IP[x]=='.'){
				k=0;
				x--;
				break;

			}
			else{

				r=(Remote_IP[x]-'0');
				f=pow(10,k);
				Gateway[i]+=r*f;
				k++;

				x--;
			}
		}


		 }

	SetRemoteIP(Gateway);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_DefaultValuesCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

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

		 	status=DefaultValues();


		 	if(status == H1DR5_OK)
		 		 {
		 		sprintf((char* )pcWriteBuffer,(char* )pcMessage1,defaultValue.LocalMac[0],defaultValue.LocalMac[1],defaultValue.LocalMac[2],defaultValue.LocalMac[3],defaultValue.LocalMac[4],defaultValue.LocalMac[5]
			    ,defaultValue.RemoteMac[0],defaultValue.RemoteMac[1],defaultValue.RemoteMac[2],defaultValue.RemoteMac[3],defaultValue.RemoteMac[4],defaultValue.RemoteMac[5]
				,defaultValue.LocalIP[0],defaultValue.LocalIP[1],defaultValue.LocalIP[2],defaultValue.LocalIP[3]
                ,defaultValue.RemoteIP[0],defaultValue.RemoteIP[1],defaultValue.RemoteIP[2],defaultValue.RemoteIP[3]
			    ,defaultValue.SubnetMask[0],defaultValue.SubnetMask[1],defaultValue.SubnetMask[2],defaultValue.SubnetMask[3]
			    ,defaultValue.DestIP[0],defaultValue.DestIP[1],defaultValue.DestIP[2],defaultValue.DestIP[3]
			     ,defaultValue.LocalPort
			     ,defaultValue.RemotePort);
		 		 }



		return pdFALSE;
}
/*-----------------------------------------------------------*/
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
	status=SetLocalPORT(LocalPORT);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,pcParameterString1);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;
}
/*-----------------------------------------------------------*/
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
	Remote_PORT =(uint8_t )atol((char* )pcParameterString1);
	status=SetRemotePORT(RemotePORT);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,pcParameterString1);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;
}
/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
