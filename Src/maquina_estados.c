/*
 * maquina_estados.c
 *
 *  Created on: Sep 22, 2024
 *      Author: movich4
 */

#include "maquina_estados.h"
#include "sensores.h"
#include "motor_control.h"
#include "pulsador.h"
#include "led.h"
#include "dipswitch.h"
#include "tipos.h"
#include "utiles.h"
#include "stdio.h"

#define DEBUG 1

const int16_t VELOCIDAD_MAXIMA = 1000; // Velocidad máxima permitida (0-1000)
const int16_t INCREMENTO_VELOCIDAD = 3; // Incremento de velocidad en cada ciclo
const int16_t VELOCIDAD_MINIMA_MOTOR = 0; // Velocidad mínima
const uint32_t INTERVALO_ACTUALIZACION = 1; // Intervalo de actualización en ms
uint32_t ultima_actualizacion = 0;
estado_robot_t estado_actual = ESPERANDO_INICIO;
int16_t velocidad_actual = 0;
bool ataque_iniciado = false;
uint32_t inicio_contador = 0;
int tiempo_atacando = 0;



// Función principal de la máquina de estados
void ejecutar_maquina_estados() {
	int frontal_derecho = 1100;
	int frontal_izquierdo = 1100;
    switch (estado_actual) {
        case ESPERANDO_INICIO:
        	frontal_derecho = leer_sensor_frontal_derecho();
			frontal_izquierdo = leer_sensor_frontal_izquierdo();
			if(DEBUG)printf("%s %d %d \n\r", "ESPERANDO_INICIO", frontal_izquierdo, frontal_derecho);
            // Esperar hasta que el pulsador esté presionado para comenzar el combate

            if (pulsador_presionado()) {

            	orientacion_robot_t orientacion = obtener_orientacion();
            	printf("orientacion robot: %d\n\r", orientacion);

            	if     (orientacion == FRENTE) estado_actual = BUSCANDO_OPONENTE;
                else if(orientacion == LADO_DERECHO) estado_actual = ATACANDO_DERECHA;
                else if(orientacion == LADO_IZQUIERDO) estado_actual = ATACANDO_IZQUIERDA;
                else if(orientacion == ESPALDAS) estado_actual = ATACANDO_ESPALDA;
            	printf("estado actual: %d\n\r", estado_actual);
            	while(pulsador_presionado()){ 	//Esperar a que se suelte el pulsador

            	}
                iniciar_cuenta_atras(); 		//Retardo de 5 segundos que se muestran con parpadeos led
            }
            break;

        case BUSCANDO_OPONENTE:
        	frontal_derecho = leer_sensor_frontal_derecho();
        	frontal_izquierdo = leer_sensor_frontal_izquierdo();
        	if(DEBUG)printf("%s %d %d \n\r", "BUSCANDO_OPONENTE", frontal_izquierdo, frontal_derecho);

        	//Si lo tenemos delante pasamos a atacar
        	if (frontal_derecho <= UMBRAL_PRESENCIA || frontal_izquierdo <= UMBRAL_PRESENCIA) {
        		estado_actual = ATACANDO;
        	}

        	//En caso de que ningun sensor este detectando nada empezamos a girar asta que localizamos al adversario
        	else {
        		mover_motores(-150, 150);
        		while(leer_sensor_frontal_izquierdo() > UMBRAL_PRESENCIA){
        			if(estado_actual != BUSCANDO_OPONENTE){ //Si se detecta una situación critica por interrupcion dejamos de girar
        				break; //y pasamos a un estado de EVADIENDO o DEFENSIVO
        			}
        		}
        		parar_motores();
        	}

            break;

        case ATACANDO:

        	frontal_derecho = leer_sensor_frontal_derecho();
        	frontal_izquierdo = leer_sensor_frontal_izquierdo();

			uint32_t tiempo_actual = HAL_GetTick();

			if(ataque_iniciado == false){
				inicio_contador = (int)tiempo_actual;
				ataque_iniciado = true;
			}
			if (tiempo_actual - ultima_actualizacion >= INTERVALO_ACTUALIZACION) {
				ultima_actualizacion = tiempo_actual;

				// Verificar si los sensores detectan al oponente
				bool oponente_detectado_derecha = false;
				bool oponente_detectado_izquierda = false;
				if(frontal_derecho <= UMBRAL_PRESENCIA)oponente_detectado_derecha = 1;
				if(frontal_izquierdo <= UMBRAL_PRESENCIA)oponente_detectado_izquierda = 1;

				tiempo_atacando = tiempo_actual - inicio_contador;
				if(tiempo_atacando < 1000){
					// Aumentar la velocidad progresivamente hasta la velocidad máxima
					if (velocidad_actual < VELOCIDAD_MAXIMA) {
						velocidad_actual += INCREMENTO_VELOCIDAD;
						if (velocidad_actual > VELOCIDAD_MAXIMA) velocidad_actual = VELOCIDAD_MAXIMA;
					}
				}

				else if(tiempo_atacando >= 1000){
					velocidad_actual -= 10;
					if (velocidad_actual < 300) velocidad_actual = 300;
				}

				//Establecemos una velocidad mínima para no tener que realizar 100 bucles x 1ms antes no se empieza
				//a mover el motor
				if(velocidad_actual < 400)velocidad_actual = 400;

				int16_t velocidad_izquierda = velocidad_actual;
				int16_t velocidad_derecha = velocidad_actual;

				// Ajustes basados en las lecturas de los sensores
				if (oponente_detectado_izquierda && oponente_detectado_derecha) {
					// Ambos sensores detectan al oponente, avanzar recto
					// No se requiere ajuste adicional
				}
				else if (oponente_detectado_izquierda == false && oponente_detectado_derecha) {
					// Oponente desplazado a la derecha, reducir velocidad del motor derecho
					// Asegurar que la velocidad no cae por debajo de la mínima
					velocidad_derecha = calcular_velocidad_motor(frontal_derecho, velocidad_actual);
					if (velocidad_derecha < VELOCIDAD_MINIMA_MOTOR) velocidad_derecha = VELOCIDAD_MINIMA_MOTOR;
				}
				else if (oponente_detectado_izquierda && oponente_detectado_derecha == false) {
					// Oponente desplazado a la izquierda, reducir velocidad del motor izquierdo
					velocidad_izquierda = calcular_velocidad_motor(frontal_izquierdo, velocidad_actual);
					if (velocidad_izquierda < VELOCIDAD_MINIMA_MOTOR) velocidad_izquierda = VELOCIDAD_MINIMA_MOTOR;
				}
				else {
					// No detectamos al oponente, movemos motores y cambiar de estado
					parar_motores();
					velocidad_actual = 0;
					estado_actual = BUSCANDO_OPONENTE;
					break;
				}

				// Asegurar que las velocidades no exceden los límites
				velocidad_izquierda = constrain(velocidad_izquierda, VELOCIDAD_MINIMA_MOTOR, VELOCIDAD_MAXIMA);
				velocidad_derecha = constrain(velocidad_derecha, VELOCIDAD_MINIMA_MOTOR, VELOCIDAD_MAXIMA);
				if(DEBUG)printf("%s %d %d %d %d %d\n\r", "ATACANDO", frontal_izquierdo, frontal_derecho, velocidad_izquierda,velocidad_derecha, tiempo_atacando);
				// Establecer las velocidades de los motores
				mover_motores(velocidad_izquierda, velocidad_derecha);
			}


            break;

        case ATACANDO_DERECHA:
        	if(DEBUG)printf("%s\n\r", "ATACANDO_DERECHA");
        	mover_motores(300, -300);
        	velocidad_actual = 300;
			while(leer_sensor_frontal_derecho() > UMBRAL_PRESENCIA){
				if(estado_actual != ATACANDO_DERECHA){ //Si se detecta una situación critica por interrupcion dejamos de girar
					break; //y pasamos a un estado de EVADIENDO o DEFENSIVO
				}
			}
			//parar_motores();
			estado_actual = ATACANDO;
        	break;

        case ATACANDO_IZQUIERDA:
        	if(DEBUG)printf("%s\n\r", "ATACANDO_IZQUIERDA");
        	mover_motores(800, -500);
			velocidad_actual = 500;
			while(leer_sensor_frontal_izquierdo() > UMBRAL_PRESENCIA){
				if(estado_actual != ATACANDO_DERECHA){ //Si se detecta una situación critica por interrupcion dejamos de girar
					break; //y pasamos a un estado de EVADIENDO o DEFENSIVO
				}
			}
			estado_actual = ATACANDO;
        	break;

        case ATACANDO_ESPALDA:
        	if(DEBUG)printf("%s\n\r", "ATACANDO_ESPALDA");

                	break;

        case ATACANDO_BANDERITAS:
        	if(DEBUG)printf("%s\n\r", "ATACANDO_BANDERITAS");

        	break;

        case EVADIENDO:
        	parar_motores();
        	tiempo_atacando = HAL_GetTick() - inicio_contador;
        	ataque_iniciado = false;
        	if(DEBUG)printf("%s\n\r", "EVADIENDO");
			if(primer_borde_detectado == BORDE_DERECHO){
				printf("%s\n\r", "borde derecho detectado");

			}
			else printf("%s\n\r", "borde izquierdo detectado");
			while(1);
        	//printf("Tiempo atacando: %d\n\r", tiempo_atacando);

        	primer_borde_detectado = 0;
        	//while(1);
//        	HAL_Delay(100);
//        	mover_motores(-200, -200);
//        	HAL_Delay(1000);
//        	velocidad_actual = 0;
//        	primer_borde_detectado = 0;
//        	estado_actual = BUSCANDO_OPONENTE;


            break;

        case DEFENSIVO:
        	if(DEBUG)printf("%s\n\r", "DEFENSIVO");
        	parpadear_led(2);
//        	parar_motores();
//        	while(1){
//        		parpadear_led(2);
//        		if(pulsador_presionado()){
//        			estado_actual = BUSCANDO_OPONENTE;
//        			break;
//        		}
//        	}

            break;

        case TEST:

        	break;
    }
}



