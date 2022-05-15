/**
 * seamfind.c
 * Function: Find the seam and then set the motor，
 *   so that the laser can follow the seam.
 *
 *  Created on: 2022年3月15日
 *      Author: jacalt1008
 *  Ver. 1.0.0
 */
#ifndef BSP_INC_SEAMFIND_H_
#define BSP_INC_SEAMFIND_H_

#include "cmsis_os.h"

void find_seam(const uint16_t *seam_location);

#endif /* BSP_INC_SEAMFIND_H_ */
