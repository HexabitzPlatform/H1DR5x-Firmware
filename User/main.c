/*
 BitzOS (BOS) V0.3.3 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){
//	EthernetSendData("MOHAMAD", 7);
	// put your code here, to run repeatedly.
	while(1){
		EthernetSendData(ip1,"MOHAMAD", 7);
		Delay_ms(1000);
		EthernetSendData(ip2,"khatib", 6);
		Delay_ms(1000);
//		EthernetSendData(ip3,"MOHAMAD", 7);
//		Delay_ms(1000);
	}
}

/*-----------------------------------------------------------*/
