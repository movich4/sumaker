/*
 * dipswitch.h
 *
 *  Created on: Aug 16, 2024
 *      Author: movic
 */

#ifndef INC_DIPSWITCH_H_
#define INC_DIPSWITCH_H_

#include "stm32f7xx_hal.h"

typedef enum {
    FRENTE,
	LADO_DERECHO,
	LADO_IZQUIERDO,
	ATRAS_RAPIDO,
	ESPALDAS
} orientacion_robot_t;


uint8_t leer_dipswitch(void);
orientacion_robot_t obtener_orientacion(void);

#endif /* INC_DIPSWITCH_H_ */
