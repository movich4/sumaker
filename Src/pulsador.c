/*
 * pulsador.c
 *
 *  Created on: Sep 22, 2024
 *      Author: movich4
 */
#include "main.h"
#include "pulsador.h"


bool pulsador_presionado(void){
	if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_RESET){
		return true;
	}
	else{
		return false;
	}
}

