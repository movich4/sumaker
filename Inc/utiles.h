/*
 * utiles.h
 *
 *  Created on: Sep 28, 2024
 *      Author: movich4
 */

#ifndef INC_UTILES_H_
#define INC_UTILES_H_
#include "stdint.h"

int map(int x, int in_min, int in_max, int out_min, int out_max);
int constrain(int x, int a, int b);
float constrain_float(float x, float a, float b);
void bitWrite(uint32_t *variable, uint8_t bit, uint8_t value);


#endif /* INC_UTILES_H_ */
