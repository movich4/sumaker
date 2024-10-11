/*
 * motor_control.c
 *
 *  Created on: Aug 12, 2024
 *      Author: movich4
 */
#include "motor_control.h"
#include "utiles.h"
#include "sensores.h"

// Define las constantes para los canales
#define MOTOR_DER TIM_CHANNEL_2
#define MOTOR_IZQ TIM_CHANNEL_1

// Define los rangos para optener 125us, 132.5us y 250us para OneShot125
#define MAX_PWM 27000
#define NEUTRAL_PWM 20250
#define MIN_PWM 13500

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;



// Inicializa los canales del Timer para los motores
void iniciar_motores() {
    HAL_TIM_PWM_Start(&htim3, MOTOR_DER);
    HAL_TIM_PWM_Start(&htim3, MOTOR_IZQ);
}

// mueve los motores en un rango entre -1000 y +1000
void mover_motores(int16_t izquierdo, int16_t derecho){

	uint32_t pwm_izquierdo = NEUTRAL_PWM + (izquierdo * 6.75); 	// combierte -1000/+1000 a -6750/+6750 y
	uint32_t pwm_derecho = NEUTRAL_PWM + (derecho * 6.75);		// luego lo suma al valor central

	constrain(pwm_izquierdo, MIN_PWM, MAX_PWM);
	constrain(pwm_derecho, MIN_PWM, MAX_PWM);


	__HAL_TIM_SET_COMPARE(&htim3, MOTOR_IZQ, pwm_izquierdo);
	__HAL_TIM_SET_COMPARE(&htim3, MOTOR_DER, pwm_derecho);
}


void parar_motores(){
	__HAL_TIM_SET_COMPARE(&htim3, MOTOR_IZQ, NEUTRAL_PWM);
	__HAL_TIM_SET_COMPARE(&htim3, MOTOR_DER, NEUTRAL_PWM);
}

//rutina que genera un retardo en us
void delay_us(uint32_t us) {
    uint32_t limite = __HAL_TIM_GET_COUNTER(&htim2) + us;  //Inicializamos limite con el valor del timer 2 + los micros pasados como argumento
    while (__HAL_TIM_GET_COUNTER(&htim2) < limite);  // Esperar hasta que el contador llegue al limite
}

//	Funcion que devuelve la velocidad que hay que aplicar al motor del lado hacia donde se pretende girar teniendo en cuenta
//tanto la distancia hacia el objetivo como la velocidad actual
int calcular_velocidad_motor(int distancia, int velocidad_actual) {
    const int DISTANCIA_MAXIMA = UMBRAL_PRESENCIA;
    const int DISTANCIA_MINIMA = 0;
    const int VELOCIDAD_MAXIMA = 1000;
    const int VELOCIDAD_MINIMA = 0;

    constrain(distancia, DISTANCIA_MINIMA, DISTANCIA_MAXIMA);
    constrain(velocidad_actual, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA);

    // Mapear la distancia normalizada al rango de velocidades
	// Cuando distancia_normalizada es mínima (0), velocidad = VELOCIDAD_MINIMA_MOTOR
	// Cuando distancia_normalizada es máxima (1000), velocidad = velocidad_actual
	int velocidad_calculada = map(distancia, DISTANCIA_MINIMA, DISTANCIA_MAXIMA, VELOCIDAD_MINIMA, velocidad_actual);

	velocidad_calculada = constrain(velocidad_calculada, VELOCIDAD_MINIMA, velocidad_actual);


    return velocidad_calculada;
}

void parar_suave(int velocidad_actual){
	for(int i= velocidad_actual; i > 0; i--){
		mover_motores(i, i);
		delay_us(500);
	}
}
