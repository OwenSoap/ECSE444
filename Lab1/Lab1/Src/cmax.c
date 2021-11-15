/*
 * cmax.c
 *
 *  Created on: Sep. 8, 2020
 *      Author: yizhu
 */
#include "main.h"

//Function cMax for finding the max in an array
void cMax(float *array, uint32_t size, float *maxC, uint32_t *maxIndexC) {
	(*maxC) = array[0];
	(*maxIndexC) = 0;

	for (uint32_t i = 1; i < size; i++) {
		if (array[i] > (*maxC)) {
			(*maxC) = array[i];
			(*maxIndexC) = i;
		}
	}
}
