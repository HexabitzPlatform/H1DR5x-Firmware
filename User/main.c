/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
//extern SPI_HandleTypeDef hspi1;

/* Private variables ---------------------------------------------------------*/
    uint8_t myGateway[4]={192,168,0,15};
	uint8_t mySubnet[4]={255,255,255,0};
	uint8_t myIP[4]={192,168,0,17};
	uint8_t data_res[512];
	uint8_t value=0;
	uint8_t valuee=0x01;
	 char data_to_send[2];
	 uint8_t Mac_addres[6]={0x07,0x6,0x2,0x3,0x4,0x5};

/* Private function prototypes -----------------------------------------------*/


/* Main function ------------------------------------------------------------*/

int main(void){
	Set_Remote_mac_addr(Mac_addres); // Mac_addr ethernet

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

	/*
//		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//		HAL_SPI_Transmit(&hspi1,&valuee, 1, 100);
//		HAL_SPI_Receive(&hspi1,&value, 1, 100);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); */
     Defalt_Value();
		// put your code here, to run repeatedly.
	while(1){
//		lan_echo();

		Ethernet_Receive_Data();
		EthernetSendData("zamaeel",7);
			Delay_ms(1000);

//			char dets[7]="zaamel";
//			  EthernetSendData(dets,sizeof(dets));
//			  Delay_ms(1000);
//			  char myqq[100]={10};
//			   EthernetSendData(myqq[0],2);
//			   Delay_ms(1000);
//			   uint8_t x=20;
//			   EthernetSendData((char*)&x,2);
//			   Delay_ms(1000);


//			    sprintf(data_to_send, "%d", value);
//			   EthernetSendData(data_to_send, strlen(data_to_send));
//			   Delay_ms(100);
//			   value++;
	}
}

/*-----------------------------------------------------------*/
