/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint8_t myGateway[4]={192,168,0,15};
	uint8_t mySubnet[4]={255,255,255,0};
	uint8_t myIP[4]={192,168,0,16};
	uint8_t data_res[512];
/* Private function prototypes -----------------------------------------------*/


/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

//	lan_init();

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){
		Set_DefaultGateway(myGateway); //ip laptop
		Set_SubnetMask(mySubnet);	// SubnetMask laptop
		Set_IP_Address(myIP);  // ip ethernet

	// put your code here, to run repeatedly.
	while(1){

		Ethernet_Receive_Data();
				EthernetSendData("moham :) moham",14);
						Delay_ms(100);

	}
}

/*-----------------------------------------------------------*/
