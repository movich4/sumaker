/*
 * motor_control.h
 *
 *  Created on: Aug 12, 2024
 *      Author: movich4
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32f7xx_hal.h"


void delay_us(uint32_t);
void iniciar_motores();
void mover_motores(int16_t izquierdo, int16_t derecho);
void parar_motores();
int calcular_velocidad_motor(int distancia_normalizada, int velocidad_actual);
void parar_suave(int velocidad_actual);


#endif /* INC_MOTOR_CONTROL_H_ */
