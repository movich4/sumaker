/*
 * sensores.h
 *
 *  Created on: Sep 22, 2024
 *      Author: movic
 */
#ifndef INC_SENSORES_H_
#define INC_SENSORES_H_

#include "stm32f7xx_hal.h"
#include "main.h"
#include "tipos.h"
#include "maquina_estados.h"
#include "motor_control.h"

// Definir bits para los sensores de borde
typedef enum {
    BORDE_IZQUIERDO = (1 << 0),  // Bit 0: Sensor de borde izquierdo
    BORDE_DERECHO   = (1 << 1),  // Bit 1: Sensor de borde derecho
} borde_sensor_t;

extern volatile borde_sensor_t primer_borde_detectado;  // Variable para el primer borde detectado
extern const uint16_t UMBRAL_PRESENCIA;



////////////////////////////////////////// Funciones //////////////////////////////////////////
uint32_t leer_sensor_frontal_derecho(void);
uint32_t leer_sensor_frontal_izquierdo(void);
GPIO_PinState leer_sensor_lateral_derecho(void);
GPIO_PinState leer_sensor_lateral_izquierdo(void);
GPIO_PinState leer_sensor_borde_derecho(void);
GPIO_PinState leer_sensor_borde_izquierdo(void);
GPIO_PinState leer_sensor_inclinacion_izquierdo(void);
int calcular_distancia_normalizada(uint16_t lectura_sensor);


#endif /* INC_SENSORES_H_ */
