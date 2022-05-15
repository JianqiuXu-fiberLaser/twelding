/**
  * @brief re-write using sequential API of LwIP
  *
  * Company: Jacalt Laser Corp.
  * Author: J. Xu
  * Date: 20220506
  * Version: V1.2.0
  *
  */
#include "cmsis_os.h"
#include "lwip/api.h"
#include "tcp_client.h"
#include "communicate.h"
#include "cmd.h"

/**
 * @called by main.c in StartLwipTask()
 *
 * Function: establish the connection to the laser.
 *           if connect successful, bind to local IP address
 *           passing the msg to TCP_thread
 * &return the connection pointer to the laser
 */
struct netconn *
laser_tcp_init(void)
{
	struct netconn *laser_conn;
	ip_addr_t laser_ip;

	laser_conn = netconn_new(NETCONN_TCP);
	if (laser_conn != NULL)
	{
		IP4_ADDR(&laser_ip, LASER_IP_ADDR0, LASER_IP_ADDR1, LASER_IP_ADDR2, LASER_IP_ADDR3);
		netconn_bind(laser_conn, &laser_ip, LASER_PORT);
	}

	return laser_conn;
}

/**
 * @called by main.c in StartLwipTask()
 *
 * Function: send data to the laser with the client_tcp
 *
 * @param array: the pointer of data array
 * @param len: the length of data
 * @param new_buff: the pointer to the netbuf of receive message
 */
void
laser_tcp_comm(const void *array, uint16_t len, struct netconn *new_conn)
{
    struct netbuf *laser_buff = NULL;
    void **ptr_laser_data = NULL;
    uint8_t *resp_data;  // the laser response command, usually 8 bytes
    uint16_t resp_cmd_len;

    //-- allocate the memory for LwIP buffer
    laser_buff = netbuf_new();

	//-- send command to the laser
	if (netbuf_take(laser_buff, array, len) == ERR_OK)
	{
		netconn_send(new_conn, laser_buff);
	}

	//-- transfer the received data from the laser to the master
	if (netconn_recv(new_conn, &laser_buff) == ERR_OK)
	{
		if (netbuf_data(laser_buff, ptr_laser_data, &resp_cmd_len) == ERR_OK)
		{
			if (resp_cmd_len <= 10)
			{
			    resp_data = (uint8_t *)ptr_laser_data;
				IPC_UART_SendData(resp_data, resp_cmd_len);
			}
		}
	}

	netbuf_delete(laser_buff);
}
