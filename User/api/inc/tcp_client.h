/**
  * @brief re-write the laser LwIP communicate modules
  *        using sequential API
  *
  * Company: Jacalt Laser Corp.
  * Author: J. Xu
  * Date: 20220506
  * Version: V1.2.0
  *
  */
#ifndef _TCP_CLIENT_H__
#define _TCP_CLIENT_H__

#include "stm32f4xx_hal.h"
#include <string.h>
#include "lwip.h"
#include "tcp.h"

// define the laser IP address, each laser has different IP address.
#define LASER_IP_ADDR0   (uint8_t) 192
#define LASER_IP_ADDR1   (uint8_t) 168
#define LASER_IP_ADDR2   (uint8_t) 2
#define LASER_IP_ADDR3   (uint8_t) 33

#define LASER_PORT       (uint16_t) 3333

struct netconn *laser_tcp_init(void);
void laser_tcp_comm(const void *array, uint16_t len, struct netconn *new_conn);

#endif  //_TCP_CLIENT_H__
