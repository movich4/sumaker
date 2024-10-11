/*
 * utiles.c
 *
 *  Created on: Sep 28, 2024
 *      Author: movich4
 */


#include "utiles.h"

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (int)((long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int constrain(int x, int a, int b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

float constrain_float(float x, float a, float b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}


// Implementación de la función bitWrite para STM32
void bitWrite(uint32_t *variable, uint8_t bit, uint8_t value) {
    if (value) {
        // Poner el bit a 1 usando OR
        *variable |= (1 << bit);
    } else {
        // Poner el bit a 0 usando AND con la negación de la máscara
        *variable &= ~(1 << bit);
    }
}
