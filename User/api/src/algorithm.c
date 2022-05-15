/**
  * Company: Jacalt Laser Corp.
  * Author: J. Xu
  * Date: 20220506
  * Version: V1.2.0
  *
  */
#include "algorithm.h"

/**
 * Function: solve the check sum of the data to send
 *           the check sum add all send data except the sum_check itself.
 */
uint8_t get_add(uint8_t* array, const uint8_t length)
{
	uint8_t sum = 0;
	for(uint8_t i=0; i<length; i++)
	{
		sum += array[i];
	}
	return sum;
}
