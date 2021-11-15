/*
 * cproduct.c
 *
 *  Created on: Sep. 13, 2020
 *      Author: yizhu
 */
#include "main.h"

// Function cProduct for calculating the product of two arrays
float cProduct(float *arraya, float *arrayb, uint32_t size, float *res) {
	for (uint32_t i = 0; i < size; i++) {
		res[i] = arraya[i] * arrayb[i];
	}
}
