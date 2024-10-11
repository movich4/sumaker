/*
 * infrarrojos.c
 *
 *  Created on: Oct 9, 2024
 *      Author: movich4
 */

#include "infrarrojos.h"
#include "stdio.h"
#include "stdint.h"
#include "utiles.h"
#include "main.h"



#define CODIGO_PARAR   0xf3e7c300		//codigo de la tecla flecha hacia adelante de mi mando IR (24 bits)
#define TIEMPO_POR_BIT      550       	//longitud en microsegundos de un bit del codigo de Infrarrojos

extern TIM_HandleTypeDef htim2;
bool iniciadaRecepcionIR;             	//flag que avisa de que se ha iniciado la recepcion de un codigo IR
uint32_t registroDeTiemposIR[32];  //array donde se almacenan los tiempos a nivel alto y bajo recibidos por el pinIR
uint8_t indiceRegistroIR;             	//indice de referencia para almacenar los tiempos en el array registroDeTiemposIR


///////////////////////////////////////// iniciarRegitroIR /////////////////////////////////////////
// borra todo el array que contiene los tiempos en alto y bajo del pin receptor de Infrarrojos
void iniciarRegitroDeTiemosIR() {
  for (int i = 0; i < 16; i++) {
    registroDeTiemposIR[i] = 0;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool mostrarSiRecepcionIniciada() {
  return iniciadaRecepcionIR;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void guardarPulsoIR(uint32_t usTiempo, GPIO_PinState nivelLogico) {
  if (iniciadaRecepcionIR) {                              //Si la recepcion ya estaba iniciada simplemente guardamos el tiempo
    registroDeTiemposIR[indiceRegistroIR] = usTiempo;
    indiceRegistroIR++;
  }
  else if ( usTiempo > 5000 && nivelLogico) {             // si el pulso a durado mas de 5ms y el PIN_RECEPTOR_IR está en alto es porque
    iniciadaRecepcionIR = true;                   		  // es el bitStart por lo que iniciamos el regitro de tiempos.
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mostrarTiempoPulsosIR() {
  uint8_t bitNumero = 31;
  int bits;
  bool nivelLogico = true;
  uint32_t codigoIR = 0;

  for (int i = 0; i < 16; i++) {
    bits = registroDeTiemposIR[i] / TIEMPO_POR_BIT;
    while (bits >= 1) {
      bitWrite(&codigoIR, bitNumero, nivelLogico);
      bitNumero--;
      bits--;
    }
    printf("%d  %lu  ", i + 1, registroDeTiemposIR[i]);
    if (nivelLogico) {
      printf("%s  \n\r", "HIGH");

    }
    else {
    	printf("%s  \n\r", "LOW");
    }
    nivelLogico = !nivelLogico;
//    if (bitNumero == 1) {
//    	bitWrite(&codigoIR, 1, 1);
//    }
  }
  printf("%s %#lx\n\r", "Codigo IR recibido ->", codigoIR);
  iniciarRegitroDeTiemosIR();
  indiceRegistroIR = 0;
  iniciadaRecepcionIR = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t decodificarCodigo() {
  uint8_t bitNumero = 31;                           //almacena el bit sobre el que se va a actuar en codigoIR
  int bits;                                         //numero de bits que hay dentro del periodo de tiempo
  bool nivelLogico = true;                          //despues del bitStart el primer bit siempre es HIGH
  unsigned long codigoIR = 0;                       //variable que almacena el codigo infrarrojo

  for (int i = 0; i < 16; i++) {
    bits = registroDeTiemposIR[i] / TIEMPO_POR_BIT;
    while (bits >= 1) {
      bitWrite(&codigoIR, bitNumero, nivelLogico);
      bitNumero--;
      bits--;
    }
    nivelLogico = !nivelLogico;
//    if (bitNumero == 1) {
//    	bitWrite(&codigoIR, 1, 1);
//    }
  }
  indiceRegistroIR = 0;
  iniciadaRecepcionIR = false;
  return codigoIR;
}

bool parar_recibido(){
	if(HAL_GPIO_ReadPin(STOP_IR_GPIO_Port, STOP_IR_Pin) && (__HAL_TIM_GET_COUNTER(&htim2) > (tiempoAnterior + 2000)) && mostrarSiRecepcionIniciada()){
		uint32_t codigo = decodificarCodigo();
		//printf("%s %#lx\n\r", "Codigo IR recibido ->", codigo);
		if(codigo == CODIGO_PARAR) return true;
	}
	else{
		return false;
	}
}
