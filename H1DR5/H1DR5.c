/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
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
defalt_value defalt;
/* Private variables ---------------------------------------------------------*/
#define MX_SIZE_USER_BUFFER 512
 uint16_t length;
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
portBASE_TYPE CLI_Ethernet_Send_DataCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_Set_Local_IPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_Set_SubnetMaskCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_Set_Remote_IPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_Defalt_ValueCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_Set_reseve_mac_and_ip_RemoteCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);

portBASE_TYPE CLI_Set_Local_PORTCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE CLI_Set_Remote_PORTCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);


/* CLI command structure : Transmit_Data */
const CLI_Command_Definition_t CLI_Ethernet_Send_DataCommandDefinition =
{
	( const int8_t * ) "ethernet_send_data", /* The command string to type. */
	( const int8_t * ) "ethernet_send_data :\r\n Parameters required to execute a EthernetSendData: my data \r\n\r\n",
	CLI_Ethernet_Send_DataCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* CLI command structure : Set_Local_IP */
const CLI_Command_Definition_t CLI_Set_Local_IPCommandDefinition =
{
	( const int8_t * ) "set_local_ip", /* The command string to type. */
	( const int8_t * ) "set_local_ip :\r\n Parameters required to execute a Set_Local_IP: local_iP is  \r\n\r\n",
	CLI_Set_Local_IPCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* CLI command structure : Set_SubnetMask */
const CLI_Command_Definition_t CLI_Set_SubnetMaskCommandDefinition =
{
	( const int8_t * ) "set_subnet_mask", /* The command string to type. */
	( const int8_t * ) "set_subnet_mask :\r\n Parameters required to execute a Set_SubnetMask: my Subnet mask \r\n\r\n",
	CLI_Set_SubnetMaskCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* CLI command structure : Set_Remote_IP */
const CLI_Command_Definition_t CLI_Set_Remote_IPCommandDefinition =
{
	( const int8_t * ) "set_remote_ip", /* The command string to type. */
	( const int8_t * ) "set_remote_ip :\r\n Parameters required to execute a Set_Remote_IP: my Remote_IP is \r\n\r\n",
	CLI_Set_Remote_IPCommand, /* The function to run. */
	1 /* one parameters are expected. */
};
/* CLI command structure : Defalt_Value */
const CLI_Command_Definition_t CLI_Defalt_ValueCommandDefinition =
{
	( const int8_t * ) "defalt_value", /* The command string to type. */
	( const int8_t * ) "defalt_value :\r\n Parameters required to execute a Defalt_Value: my Defalt_Value \r\n\r\n",
	CLI_Defalt_ValueCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
const CLI_Command_Definition_t CLI_Set_reseve_mac_and_ip_RemoteCommandDefinition =
{
	( const int8_t * ) "set_reseve_mac_and_ip_remote", /* The command string to type. */
	( const int8_t * ) "set_reseve_mac_and_ip_remote :\r\n Parameters required to execute a Set_reseve_mac_and_ip_Remote: reseve_mac_and_ip_Remote \r\n\r\n",
	CLI_Set_reseve_mac_and_ip_RemoteCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

/* CLI command structure : Set_Local_PORT */
const CLI_Command_Definition_t CLI_Set_Local_PORTCommandDefinition =
{
	( const int8_t * ) "set_local_port", /* The command string to type. */
	( const int8_t * ) "set_local_port :\r\n Parameters required to execute a Set_Local_PORT: Local_PORT is \r\n\r\n",
	CLI_Set_Local_PORTCommand, /* The function to run. */
	1 /* one parameters are expected. */
};
/* CLI command structure : Set_Remote_PORT */
const CLI_Command_Definition_t CLI_Set_Remote_PORTCommandDefinition =
{
	( const int8_t * ) "set_remote_port", /* The command string to type. */
	( const int8_t * ) "set_remote_port :\r\n Parameters required to execute a Set_Remote_PORT: Remote_PORT is \r\n\r\n",
	CLI_Set_Remote_PORTCommand, /* The function to run. */
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

}

/*-----------------------------------------------------------*/
/* --- H1DR5 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H1DR5_OK;

	uint8_t Local_IP[4]={};
	uint8_t Remote_IP[4]={};
	uint8_t Local_PORT;
	uint8_t Remote_PORT;
	uint8_t Subnet[4]={};
	uint8_t IpGate[4]={};
	switch(code){
		case CODE_H1DR5_Ethernet_Send_Data:
			length=(uint16_t)cMessage[port - 1][shift];
			EthernetSendData(&cMessage[port - 1][1+shift],length);
			break;
		case CODE_H1DR5_Ethernet_Receive_Data:
			Ethernet_Receive_Data();
			break;
		case CODE_H1DR5_Set_Local_IP:
			Local_IP[0]= cMessage[port - 1][0 + shift];
			Local_IP[1]= cMessage[port - 1][1 + shift];
			Local_IP[2]= cMessage[port - 1][2 + shift];
			Local_IP[3]= cMessage[port - 1][3 + shift];
			Set_Local_IP(Local_IP);
             break;
		case CODE_H1DR5_Set_Remote_IP:
			Remote_IP[0]= cMessage[port - 1][0 + shift];
			Remote_IP[1]= cMessage[port - 1][1 + shift];
			Remote_IP[2]= cMessage[port - 1][2 + shift];
			Remote_IP[3]= cMessage[port - 1][3 + shift];
			Set_Remote_IP(Remote_IP);
			 break;
		case CODE_H1DR5_Set_Subnet_Mask:
			Subnet[0]= cMessage[port - 1][0 + shift];
			Subnet[1]= cMessage[port - 1][1 + shift];
			Subnet[2]= cMessage[port - 1][2 + shift];
			Subnet[3]= cMessage[port - 1][3 + shift];
			Set_SubnetMask(Subnet);
			 break;
		case CODE_H1DR5_Set_Local_PORT:
			Local_PORT= cMessage[port - 1][0 + shift];
			Set_Local_PORT(Local_PORT);
			 break;
		case CODE_H1DR5_Set_Remote_PORT:
			Remote_PORT= cMessage[port - 1][0 + shift];
			Set_Remote_PORT(Remote_PORT);
			 break;
		case CODE_H1DR5_reseve_mac_and_ip_Remote:
			Set_reseve_mac_and_ip_Remote();

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
	FreeRTOS_CLIRegisterCommand(&CLI_Ethernet_Send_DataCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Set_Local_IPCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Set_SubnetMaskCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Set_Remote_IPCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Defalt_ValueCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Set_reseve_mac_and_ip_RemoteCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Set_Local_PORTCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Set_Remote_PORTCommandDefinition);


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
/*
         Receive data from Ethernet module
 */
Module_Status Ethernet_Receive_Data()
{
	Module_Status status=H1DR5_OK;
	lan_poll(DataBuffer, &length);
	for(uint16_t i=0; i<length; i++){
		UserBufferData[indexInput]=DataBuffer[i];
//		ReceiveData[indexInput]=DataBuffer[i];
		indexInput++;
		if(indexInput==USER_RX_BUF_SIZE)
			indexInput=0;
	}
	length=0;
	}

/*-----------------------------------------------------------*/

/*
 * Set the connection settings
 */
Module_Status Set_reseve_mac_and_ip_Remote()
{
	Module_Status status=H1DR5_OK;
 EthernetSendData("0",1);
     Delay_ms(10);
     Ethernet_Receive_Data();
     return status;
}
/*-----------------------------------------------------------*/

/*
     Set Local_PORT
 */
Module_Status Set_Local_PORT(uint8_t from_port){
	Module_Status status=H1DR5_OK;
	if(from_port>255 || from_port<1)
	{
			status=H1DR5_ERROR;
	}
	Local_PORT=from_port;
	return status;
}
/*-----------------------------------------------------------*/

/*
     Set Remote_PORT
 */
Module_Status Set_Remote_PORT(uint8_t to_port){
	Module_Status status=H1DR5_OK;
	if(to_port>255 || to_port<1)
	{
			status=H1DR5_ERROR;
	}
	Remote_PORT=to_port;
	return status;
}
/*-----------------------------------------------------------*/

/*
     Set Local_Mac_addr
 */
Module_Status Set_Local_mac_addr(uint8_t *Mac_addr){
	Module_Status status=H1DR5_OK;

	for(int i=0; i<6;i++){

		mac_addr[i]=Mac_addr[i];
		}


	return status;
}
/*-----------------------------------------------------------*/

/*
    Set IP address of the Ethernet module.
 */
Module_Status Set_Local_IP(uint8_t *IP){
	Module_Status status=H1DR5_OK;
	uint32_t ip[4]={};

	if(IP==NULL){
		status=H1DR5_ERROR;
	}
	for(int i=0; i<4;i++){
		ip[i]=IP[i];

	}

	Local_IP=inet_addr(ip[0],ip[1],ip[2],ip[3]);

	return status;

}
/*-----------------------------------------------------------*/

/*
     Set subnet mask of the Ethernet network.
 */
Module_Status Set_SubnetMask(uint8_t *SubnetMask){
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
Module_Status Set_Remote_IP(uint8_t *Gateway){
	Module_Status status=H1DR5_OK;
	uint32_t gateway[4]={};
	if(Gateway==NULL){
		status=H1DR5_ERROR;
	}
	for(int i=0; i<4;i++){
		gateway[i]=Gateway[i];
	}

	Remote_IP=inet_addr(gateway[0],gateway[1],gateway[2],gateway[3]);
	ip_dest=Remote_IP;
	return status;

}
/*-----------------------------------------------------------*/
/*
   View Defalt_Value
 */
Module_Status Defalt_Value(){
	Module_Status status=H1DR5_OK;

	 memcpy(defalt.Local_mac_addr, mac_addr, sizeof(mac_addr));
	 memcpy(defalt.Remote_mac_addr, arp_cache[0].mac_addr, sizeof(mac_addr));
	 uint8_t Local_ip[4];
	 Local_ip[0]=Local_IP;
	 Local_ip[1]=(Local_IP >> 8);
	 Local_ip[2]=(Local_IP >> 16);
	 Local_ip[3]=(Local_IP >> 24);
	 memcpy(defalt.Local_IP, Local_ip, sizeof(Local_ip));
	 uint8_t Remote_iP[4];
	 Remote_iP[0]=Remote_IP;
	 Remote_iP[1]=(Remote_IP >> 8);
	 Remote_iP[2]=(Remote_IP >> 16);
	 Remote_iP[3]=(Remote_IP >> 24);
	 memcpy(defalt.Remote_IP, Remote_iP, sizeof(Remote_iP));
	 uint8_t ip_Mask[4];
	 ip_Mask[0]=ip_mask;
	 ip_Mask[1]=(ip_mask >> 8);
	 ip_Mask[2]=(ip_mask >> 16);
	 ip_Mask[3]=(ip_mask >> 24);
	 memcpy(defalt.ip_mask, ip_Mask, sizeof(ip_Mask));
	 uint8_t ip_Dest[4];
	 ip_Dest[0]=ip_dest;
	 ip_Dest[1]=(ip_dest >> 8);
	 ip_Dest[2]=(ip_dest >> 16);
	 ip_Dest[3]=(ip_dest >> 24);
	 memcpy(defalt.ip_dest, ip_Dest, sizeof(ip_Dest));

	 defalt.Local_PORT = Local_PORT;
	 defalt.Remote_PORT = Remote_PORT;
	 return status;
}

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
portBASE_TYPE CLI_Ethernet_Send_DataCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
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
portBASE_TYPE CLI_Set_reseve_mac_and_ip_RemoteCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static const int8_t *pcOKMessage=(int8_t* )"The connection has been opened \r\n  \n\r";


	(void )xWriteBufferLen;

	Set_reseve_mac_and_ip_Remote();
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	}

	return pdFALSE;

}

/*-----------------------------------------------------------*/
portBASE_TYPE CLI_Set_Local_IPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	int size=15;
	portBASE_TYPE xParameterStringLength1 =0;

	char Local_IP[size];
	uint8_t IP[4]={};
	int x;
	int k=0;
	int r=0,f=0;

	static const int8_t *pcOKMessage=(int8_t* )"The Local IP has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );

	for(int y=0;y<xParameterStringLength1;y++){
		Local_IP[y] = (char )pcParameterString1[y];
	}

	size=xParameterStringLength1-1;
	x=xParameterStringLength1-1;
	for(int i=3;i>=0;i--){

		while(x>=0){
			if(Local_IP[x]=='.'){
				k=0;
				x--;
				break;

			}
			else{

				r=(Local_IP[x]-'0');
				f=pow(10,k);
				IP[i]+=r*f;
				k++;

				x--;
			}
		}


		 }

	Set_Local_IP(IP);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_Set_SubnetMaskCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	int size=15;
	portBASE_TYPE xParameterStringLength1 =0;

	char SubnetMask[size];
	uint8_t Subnet[4]={};
	int x;
	int k=0;
	int r=0,f=0;

	static const int8_t *pcOKMessage=(int8_t* )"The Subnet Mask has been changed successfully \r\n  \n\r";
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

	Set_SubnetMask(Subnet);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_Set_Remote_IPCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	int size=15;
	portBASE_TYPE xParameterStringLength1 =0;

	char Remote_IP[size];
	uint8_t Gateway[4]={};
	int x;
	int k=0;
	int r=0,f=0;

	static const int8_t *pcOKMessage=(int8_t* )"The Remote_IP has been changed successfully \r\n  \n\r";
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

	Set_Remote_IP(Gateway);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_Defalt_ValueCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status status = H1DR5_OK;

	static const int8_t *pcMessage1 =(int8_t* )"the Defalt_Value of Params: \r\n"
	"the Local_mac_addr: %d.%d.%d.%d.%d.%d\r\n"
	"the Remote_mac_addr: %d.%d.%d.%d.%d.%d\r\n"
	"the Local_IP: %d.%d.%d.%d\r\n"
	"the Remote_IP: %d.%d.%d.%d\r\n"
	"the ip_mask: %d.%d.%d.%d\r\n"
	"the ip_dest: %d.%d.%d.%d\r\n"
	"the Local_PORT: %d\r\n"
	"the Remote_PORT: %d\r\n";

	     (void )xWriteBufferLen;
			configASSERT(pcWriteBuffer);

		 	status=Defalt_Value();


		 	if(status == H1DR5_OK)
		 		 {
		 		sprintf((char* )pcWriteBuffer,(char* )pcMessage1,defalt.Local_mac_addr[0],defalt.Local_mac_addr[1],defalt.Local_mac_addr[2],defalt.Local_mac_addr[3],defalt.Local_mac_addr[4],defalt.Local_mac_addr[5]
			    ,defalt.Remote_mac_addr[0],defalt.Remote_mac_addr[1],defalt.Remote_mac_addr[2],defalt.Remote_mac_addr[3],defalt.Remote_mac_addr[4],defalt.Remote_mac_addr[5]
				,defalt.Local_IP[0],defalt.Local_IP[1],defalt.Local_IP[2],defalt.Local_IP[3]
                ,defalt.Remote_IP[0],defalt.Remote_IP[1],defalt.Remote_IP[2],defalt.Remote_IP[3]
			    ,defalt.ip_mask[0],defalt.ip_mask[1],defalt.ip_mask[2],defalt.ip_mask[3]
			    ,defalt.ip_dest[0],defalt.ip_dest[1],defalt.ip_dest[2],defalt.ip_dest[3]
			     ,defalt.Local_PORT
			     ,defalt.Remote_PORT);
		 		 }



		return pdFALSE;
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_Set_Local_PORTCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	uint8_t Local_PORT =0;
	static const int8_t *pcOKMessage=(int8_t* )"The Local_PORT has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	Local_PORT =(uint8_t )atol((char* )pcParameterString1);
	status=Set_Local_PORT(Local_PORT);
	if(status == H1DR5_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,pcParameterString1);

	}

	else if(status == H1DR5_ERROR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_Set_Remote_PORTCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H1DR5_OK;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	uint8_t Remote_PORT =0;
	static const int8_t *pcOKMessage=(int8_t* )"The Remote_PORT has been changed successfully \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	Remote_PORT =(uint8_t )atol((char* )pcParameterString1);
	status=Set_Remote_PORT(Remote_PORT);
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
