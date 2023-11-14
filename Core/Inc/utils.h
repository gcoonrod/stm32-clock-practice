/*
 * utils.h
 *
 *  Created on: Nov 14, 2023
 *      Author: greg.coonrod
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

uint8_t bcd_to_bin(uint8_t bcd);
uint8_t bin_to_bcd(uint8_t bin);

#ifdef __cplusplus
}
#endif

#endif /* INC_UTILS_H_ */
