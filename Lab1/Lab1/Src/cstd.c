/*
 * cstd.c
 *
 *  Created on: Sep. 13, 2020
 *      Author: yizhu
 */
#include "main.h"

//Function cStd for calculating the standard deviation for an array
float cStd(float *array, uint32_t size, float *res) {
	(*res) = 0;
	float sum = 0;
	float miu = 0;
	float nor = 0;
	float norsum = 0;
	for (uint32_t i = 0; i < size; i++) {
		sum += array[i];
	}
	miu = sum / size;
	for (uint32_t j = 0; j < size; j++) {
		nor = (array[j] - miu) * (array[j] - miu) / (size - 1);
		norsum += nor;
	}
	(*res) = sqrt(norsum);
}

