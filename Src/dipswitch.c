/*
 * dipswitch.c
 *
 *  Created on: Aug 16, 2024
 *      Author: movic
 */
#include "main.h"
#include "dipswitch.h"
#include "stdio.h"


uint8_t leer_dipswitch(){
	uint8_t valor = 0;
	if(HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin) == 0) valor |= 0x01;
	if(HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin) == 0) valor |= 0x02;
	if(HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin) == 0) valor |= 0x04;
	if(HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin) == 0) valor |= 0x08;
	if(HAL_GPIO_ReadPin(DIP5_GPIO_Port, DIP5_Pin) == 0) valor |= 0x10;
	return valor;
}

orientacion_robot_t obtener_orientacion(){
	uint8_t valor = leer_dipswitch() & 0x07; //recuperamos solo los 3 bits menos significativos
	printf("valor dipswitch = %d\n\r", valor);
	if     (valor == 0x01) return LADO_IZQUIERDO;
	else if(valor == 0x02) return FRENTE;
	else if(valor == 0x04) return LADO_DERECHO;
	else			       return ESPALDAS;
}
