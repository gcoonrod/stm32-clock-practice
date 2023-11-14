/*
 * utils.c
 *
 *  Created on: Nov 14, 2023
 *      Author: greg.coonrod
 */


#include "utils.h"

uint8_t bcd_to_bin(uint8_t bcd) {
	return ((bcd & 0xF0) >> 4) * 10 + (bcd & 0xF0);
}

uint8_t bin_to_bcd(uint8_t bin) {
	return ((bin / 10) << 4) | (bin % 10);
}
