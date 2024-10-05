/*
 * led.c
 *
 *  Created on: Aug 16, 2024
 *      Author: movic
 */

#include "led.h"
#include "main.h"

#define MILLIS_ON 50
#define MILLIS_OFF 50


void encender_led(){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
}

void apagar_led(){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
}

void parpadear_led(uint8_t repeticiones){
	uint16_t retardo = 1000-(repeticiones * MILLIS_ON)-(repeticiones * MILLIS_OFF);
	for(uint8_t i=repeticiones; i>0; i--){
		encender_led();
		HAL_Delay(MILLIS_ON);
		apagar_led();
		HAL_Delay(MILLIS_OFF);
	}
	HAL_Delay(retardo);
}

void iniciar_cuenta_atras(){
	HAL_Delay(1000);
	parpadear_led(4);
	parpadear_led(3);
	parpadear_led(2);
	parpadear_led(1);
}
