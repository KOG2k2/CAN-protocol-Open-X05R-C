/*
 * CAN.c
 *
 *  Created on: Aug 21, 2023
 *      Author: Admin
 */

#include "CAN.h"

uint8_t led_switch = 0;

void CAN_manual_send(){
	switch(led_switch){
	  case 0:
		  if(isButtonPressed(USER_BUTT) == 1){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,0);
		  }
		  led_switch = 1;
	  case 1:
		  if(isButtonPressed(USER_BUTT) == 1){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,1);
		  }
		  led_switch = 0;
	  }
}
