/*
 * lab1.math.h
 *
 *  Created on: Sep. 8, 2020
 *      Author: yizhu
 */
#include "main.h"
#ifndef INC_LAB1_MATH_H_
#define INC_LAB1_MATH_H_

extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
extern void asmProduct(float *arraya, float *arrayb, uint32_t size, float *res);
extern void asmStd(float *array, uint32_t size, float *res);

#endif /* INC_LAB1_MATH_H_ */
