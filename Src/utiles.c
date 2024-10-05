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
