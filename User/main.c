/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Private variables ---------------------------------------------------------*/
    uint8_t myGateway[4]={192,168,0,15};
	uint8_t mySubnet[4]={255,255,255,0};
	uint8_t myIP[4]={192,168,0,17};
	uint8_t Mac_addres[6]={0x07,0x6,0x2,0x3,0x4,0x5};

/* Private function prototypes -----------------------------------------------*/


/* Main function ------------------------------------------------------------*/

int main(void){
	Set_Local_mac_addr(Mac_addres); // Mac_addr ethernet

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){

	Set_Local_IP(myIP);  // ip ethernet
	Set_SubnetMask(mySubnet);	// SubnetMask laptop
	Set_Remote_IP(myGateway); //ip laptop
	Set_Local_PORT(90);// port ethernet
	Set_Remote_PORT(95);//port laptop
    Set_reseve_mac_and_ip_Remote();

    Defalt_Value();
		// put your code here, to run repeatedly.
	while(1){

		        Ethernet_Receive_Data();
//		        EthernetSendData("zamaeel",7);
//		            Delay_ms(1000);
	}
}

/*-----------------------------------------------------------*/
