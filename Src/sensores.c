/*
 * sensores.c
 *
 *  Created on: Sep 22, 2024
 *      Author: movich4
 */
#include "sensores.h"
#include "utiles.h"

volatile borde_sensor_t primer_borde_detectado = 0;  // Variable para almacenar el primer borde detectado
extern volatile estado_robot_t estado_actual;		 // Variable de la maquina de estados
extern volatile uint32_t media_movil_sensor_derecho;
extern volatile uint32_t media_movil_sensor_izquierdo;
extern TIM_HandleTypeDef htim5;
const uint16_t UMBRAL_PRESENCIA = 950;


uint32_t leer_sensor_frontal_derecho(){ 			//Los valores oscilan entre lejos a 450 y pegados al robot 3700 y
	uint32_t media = media_movil_sensor_derecho;	//los devuelve normalizados entre 0 y 1000
	return calcular_distancia_normalizada(media);
}

uint32_t leer_sensor_frontal_izquierdo(){
	uint32_t media = media_movil_sensor_izquierdo;
	return calcular_distancia_normalizada(media);
}

GPIO_PinState leer_sensor_lateral_derecho(){
	return HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin);
}

GPIO_PinState leer_sensor_lateral_izquierdo(){
	return HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin);
}

GPIO_PinState leer_sensor_borde_derecho(){
	return HAL_GPIO_ReadPin(FINAL_D_GPIO_Port, FINAL_D_Pin);
}
GPIO_PinState leer_sensor_borde_izquierdo(){
	return HAL_GPIO_ReadPin(FINAL_I_GPIO_Port, FINAL_I_Pin);
}

GPIO_PinState leer_sensor_inclinacion_izquierdo(){
	return HAL_GPIO_ReadPin(INCL_1_GPIO_Port, INCL_1_Pin);
}

GPIO_PinState leer_sensor_inclinacion_derecho(){
	return HAL_GPIO_ReadPin(INCL_2_GPIO_Port, INCL_2_Pin);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { //ISR externas

	if (GPIO_Pin == FINAL_D_Pin) {
		parar_motores();
		if(primer_borde_detectado == 0){
			primer_borde_detectado = BORDE_DERECHO;
			estado_actual = EVADIENDO;
		}

	}
	else if (GPIO_Pin == FINAL_I_Pin) {
		parar_motores();
		if(primer_borde_detectado == 0){
			primer_borde_detectado = BORDE_IZQUIERDO;
			estado_actual = EVADIENDO;
		}

	}
	else if (GPIO_Pin == INCL_1_Pin || GPIO_Pin == INCL_2_Pin) {
		__HAL_TIM_SET_COUNTER(&htim5, 0);    // Reinicia el contador del timer5 a 0
		HAL_TIM_Base_Start_IT(&htim5);		 // Ponemos en marcha el timer5
	}
}

// ISR del Timer 5: se llama 50ms después de la primera deteccion de inclinación
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim5) {
    	HAL_TIM_Base_Stop_IT(&htim5);  // Detener el temporizador, solo se necesita una vez
    	// Si todabia permanece detectando damos por hecho que estamos levantados y pasamos a un estado DEFENSIVO
        if(HAL_GPIO_ReadPin(INCL_1_GPIO_Port, INCL_1_Pin) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(INCL_2_GPIO_Port, INCL_2_Pin) == GPIO_PIN_RESET){
        	//estado_actual = DEFENSIVO;
        }
    }
}

//	funcion que devuelve valores entre 0, cuando el adc arroja 3700 porque algo esta muy pegado al robot, y
//1000 cuando la detección se encuentra a la otra punta del dojo y el adc da un valor de 450. Valores mayores deveran considerarse como
//que el oponente no està en frente.
int calcular_distancia_normalizada(uint16_t lectura_sensor) {
    const uint16_t LECTURA_CERCANA = 3800;
    const uint16_t LECTURA_LEJANA = 0;
    const int DISTANCIA_MAX_NORMALIZADA = 1138;


    if (lectura_sensor > LECTURA_CERCANA) {
        lectura_sensor = LECTURA_CERCANA;
    }

    int distancia_normalizada = map(lectura_sensor, LECTURA_CERCANA, LECTURA_LEJANA, 0, DISTANCIA_MAX_NORMALIZADA);

    return distancia_normalizada;
}
