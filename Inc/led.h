/*
 * led.h
 *
 *  Created on: Aug 16, 2024
 *      Author: movich4
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32f7xx_hal.h"


void encender_led ();
void apagar_led();
void parpadear_led(uint8_t repeticiones);
void iniciar_cuenta_atras(void);

#endif /* INC_LED_H_ */
