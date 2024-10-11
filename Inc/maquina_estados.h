/*
 * maquina_estados.h
 *
 *  Created on: Sep 22, 2024
 *      Author: movich4
 */

#ifndef INC_MAQUINA_ESTADOS_H_
#define INC_MAQUINA_ESTADOS_H_


// Definir los posibles estados del robot
typedef enum {
    ESPERANDO_INICIO,
    BUSCANDO_OPONENTE,
    ATACANDO,
	ATACANDO_DERECHA,
	ATACANDO_IZQUIERDA,
	ATACANDO_ESPALDA,
	ATACANDO_BANDERITAS,
    EVADIENDO,
    DEFENSIVO,
	ESTRATEGIA_MARCHA_ATRAS,
	TEST
} estado_robot_t;

// Función para ejecutar la máquina de estados
void ejecutar_maquina_estados(void);


#endif /* INC_MAQUINA_ESTADOS_H_ */
