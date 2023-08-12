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
	  Set_Remote_IP(myGateway); //ip laptop
		Set_SubnetMask(mySubnet);	// SubnetMask laptop
		Set_Local_IP(myIP);  // ip ethernet
		Set_Local_PORT(255);
		Set_Remote_PORT(1);
		Set_reseve_mac_and_ip_Remote();
	// put your code here, to run repeatedly.
	while(1){
//		Ethernet_Receive_Data();

				EthernetSendData(" abcdefg",8);
						Delay_ms(1000);

	}
}

/*-----------------------------------------------------------*/