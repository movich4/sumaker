/*
 * infrarrojos.h
 *
 *  Created on: Oct 9, 2024
 *      Author: movic
 */

#ifndef INC_INFRARROJOS_H_
#define INC_INFRARROJOS_H_
#include "tipos.h"
#include "main.h"

extern volatile uint32_t tiempoAnterior;

//funciones del receptor de infrarrojos
void guardarPulsoIR(uint32_t, GPIO_PinState);
void mostrarTiempoPulsosIR();
bool mostrarSiRecepcionIniciada();
uint32_t decodificarCodigo();
bool parar_recibido();

#endif /* INC_INFRARROJOS_H_ */
