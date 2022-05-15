/**
 * Developed of the Jacalt laser Corp.
 * Author: J. XU
 * Date: 20220418
 * Version: V1.1.0
 *
 * parse the commands and commands of sending
 */
#include "cmsis_os.h"
#include <string.h>
#include "main.h"
#include "cmd.h"
#include "communicate.h"
#include "algorithm.h"
#include "stepmotor.h"

static uint8_t weld_track_state;
static uint8_t welding_state;

static uint8_t laser_on_cmd[8] = {0xab, 0xcd, 0x05, 0xff,
		0x00, 0x3d, 0xaa, 0x00};
static uint8_t laser_off_cmd[8] = {0xab, 0xcd, 0x05, 0xff,
		0x00, 0x3d, 0x55, 0x00};

/***************************************************************************
 * 发送信息给上位机的函数组，统一格式
 * 输入：data will be sent
 * 调用：send函数，定义在 communicate.h中
 * 返回：void
 ***************************************************************************/
static void Update_Current_X_Degree_To_Upper(const uint8_t degree)
{
	uint8_t senddata[7];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x04;
	senddata[3] = 0x00;
	senddata[4] = Control_Motor_X_run;
	senddata[5] = degree;
	senddata[6] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Current_Y_Degree_To_Upper(const uint8_t degree)
{
	uint8_t senddata[7];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x04;
	senddata[3] = 0x00;
	senddata[4] = Control_Motor_Y_run;
	senddata[5] = degree;
	senddata[6] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Welding_State_To_Upper(const uint8_t state)
{
	uint8_t senddata[8];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x04;
	senddata[3] = 0x00;
	senddata[4] = Welding;
	senddata[5] = state;
	senddata[6] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Current_Warning_To_Upper(void)
{
	uint8_t senddata[6+WARNING_BYTE];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x03 + WARNING_BYTE;
	senddata[3] = 0x00;
	senddata[4] = Alarm_Information;
	memcpy(&senddata[5], warning, sizeof(warning));
	senddata[5+WARNING_BYTE] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Current_Temperature_To_Upper(const float temperature)
{
	uint8_t senddata[8];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x05;
	senddata[3] = 0x00;
	senddata[4] = Current_Temperature;
	senddata[5] = (uint8_t)(temperature*10);
	senddata[6] = (uint8_t)((uint16_t)(temperature*10)>>8);
	senddata[7] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Current_Humidity_To_Upper(const float humidity)
{
	uint8_t senddata[8];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x05;
	senddata[3] = 0x00;
	senddata[4] = Current_Humidity;
	senddata[5] = (uint8_t)(humidity*10);
	senddata[6] = (uint8_t)((uint16_t)(humidity*10)>>8);
	senddata[7] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Current_Welding_Length_To_Upper(const uint32_t length)
{
	uint8_t senddata[10];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x05;
	senddata[3] = 0x00;
	senddata[4] = Current_Welding_Length;
	senddata[5] = (uint8_t)length;
	senddata[6] = (uint8_t)(length>>8);
	senddata[7] = (uint8_t)(length>>16);
	senddata[8] = (uint8_t)(length>>24);
	senddata[9] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Total_Welding_Length_To_Upper(const uint32_t length)
{
	uint8_t senddata[10];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x05;
	senddata[3] = 0x00;
	senddata[4] = Total_Welding_Length;
	senddata[5] = (uint8_t)length;
	senddata[6] = (uint8_t)(length>>8);
	senddata[7] = (uint8_t)(length>>16);
	senddata[8] = (uint8_t)(length>>24);
	senddata[9] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Current_Data_And_Time_To_Upper(RTC_DateTypeDef data, RTC_TimeTypeDef time)
{
	uint8_t senddata[13];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x0a;
	senddata[3] = 0x00;
	senddata[4] = Current_Date_And_Time;
	senddata[5] = (uint8_t)(data.Year+2000);
	senddata[6] = (uint8_t)((data.Year+2000)>>8);
	senddata[7] = (uint8_t)data.Month;
	senddata[8] = (uint8_t)data.Date;
	senddata[9] = (uint8_t)time.Hours;
	senddata[10] = (uint8_t)time.Minutes;
	senddata[11] = (uint8_t)time.Seconds;
	senddata[12] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

static void Update_Weld_Track_State_To_Upper(const uint8_t state)
{
	uint8_t senddata[8];
	senddata[0] = BOARD_RESPONSE_HEAD1;
	senddata[1] = BOARD_RESPONSE_HEAD2;
	senddata[2] = 0x04;
	senddata[3] = 0x00;
	senddata[4] = Weld_Tracking;
	senddata[5] = state;
	senddata[6] = get_add(senddata, sizeof(senddata)-1);
	IPC_UART_SendData(senddata, sizeof(senddata));
}

/**
 * Function: parse the commands from the master
 * Input: the pointer of command array
 *        the length of command array
 * Return: void
 */
void Cmd_Handle(uint8_t* cmd, const uint8_t len)
{
	uint8_t msg_prio = 0;
	typedef_stepmotor_para stepmotor_para;

	switch(cmd[5])
	{
		case Control_Motor_X_run:
			if(cmd[4] == Write)
			{
				stepmotor_para.stepmotor = STEPMOTOR_X;
				stepmotor_para.direction = FORWARD;
				stepmotor_para.degree = cmd[6];
				osMessageQueuePut((osMessageQueueId_t)MotorControlQueueHandle,
						  	  	  (const void *)&stepmotor_para,
								  (uint8_t)msg_prio,
								  (uint32_t)osWaitForever);
				x_degree += stepmotor_para.degree;
			}
			else if(cmd[4] == Read)
			{
				Update_Current_X_Degree_To_Upper(x_degree);
			}
			break;

		case Control_Motor_Y_run:
			if(cmd[4] == Write)
			{
				stepmotor_para.stepmotor = STEPMOTOR_Y;
				stepmotor_para.direction = FORWARD;
				stepmotor_para.degree = cmd[6];
				osMessageQueuePut((osMessageQueueId_t)MotorControlQueueHandle,
						  	  	  (const void *)&stepmotor_para,
								  (uint8_t)msg_prio,
								  (uint32_t)osWaitForever);
				y_degree += stepmotor_para.degree;
			}
			else if(cmd[4] == Read)
			{
				Update_Current_Y_Degree_To_Upper(y_degree);
			}
			break;

		case Welding:
			if(cmd[4] == Write)
			{
				if(cmd[6] == 0x01)  // start welding
				{
					HAL_GPIO_WritePin(LASER_INTERLOCK_GPIO_Port, LASER_INTERLOCK_Pin, GPIO_PIN_RESET);
					laser_on_cmd[7] = get_add(laser_on_cmd, sizeof(laser_on_cmd)-1);

					osMessageQueuePut((osMessageQueueId_t)LaserEthernetQueueHandle,
					  				  (const void *)laser_on_cmd,
					  				  (uint8_t)msg_prio,
					  				  (uint32_t)osWaitForever);

					welding_state = 0x00;
				}
				else if(cmd[6] == 0x00)  // stop welding
				{
					HAL_GPIO_WritePin(LASER_INTERLOCK_GPIO_Port, LASER_INTERLOCK_Pin, GPIO_PIN_SET);
					laser_off_cmd[7] = get_add(laser_off_cmd, sizeof(laser_off_cmd)-1);

					osMessageQueuePut((osMessageQueueId_t)LaserEthernetQueueHandle,
					  				  (const void *)laser_off_cmd,
					  				  (uint8_t)msg_prio,
					  				  (uint32_t)osWaitForever);

					welding_state = 0x01;
				}
			}
			else if(cmd[4] == Read)
			{
				Update_Welding_State_To_Upper(welding_state);
			}
			break;

		case Alarm_Information:
			if(cmd[4] == Write)
			{
				// do nothing
			}
			else if(cmd[4] == Read)
			{
				Update_Current_Warning_To_Upper();
			}
			break;

		case Current_Temperature:
			if(cmd[4] == Write)
			{
				// do nothing
			}
			else if(cmd[4] == Read)
			{
				Update_Current_Temperature_To_Upper(shtc3_temp);
			}
			break;

		case Current_Humidity:
			if(cmd[4] == Write)
			{
				// do nothing
			}
			else if(cmd[4] == Read)
			{
				Update_Current_Humidity_To_Upper(shtc3_hum);
			}
			break;

		case Current_Welding_Length:
			if(cmd[4] == Write)
			{
				// do nothing
			}
			else if(cmd[4] == Read)
			{
				Update_Current_Welding_Length_To_Upper(current_welding_length);
			}
			break;

		case Total_Welding_Length:
			if(cmd[4] == Write)
			{
				// TODO: get total welding length from the master
			}
			else if(cmd[4]==Read)
			{
				Update_Total_Welding_Length_To_Upper(total_welding_length);
			}
			break;

		case Current_Date_And_Time:
			if(cmd[4] == Write)
			{
				// do nothing
			}
			else if(cmd[4] == Read)
			{
				Update_Current_Data_And_Time_To_Upper(DateNow, TimeNow);
			}
			break;

		case Weld_Tracking:
			if(cmd[4] == Write)
			{
				if(cmd[6] == 0x00)  // close
				{
					HAL_GPIO_WritePin(LINE_LASER_GPIO_Port, LINE_LASER_Pin, GPIO_PIN_SET);
					weld_track_state = 0x00;
				}
				else if(cmd[6] == 0x01)  // open
				{
					HAL_GPIO_WritePin(LINE_LASER_GPIO_Port, LINE_LASER_Pin, GPIO_PIN_RESET);
					weld_track_state = 0x01;
				}
			}
			else if(cmd[4] == Read)
			{
				Update_Weld_Track_State_To_Upper(weld_track_state);
			}
			break;

		case Check_Parameter:
			Update_Current_X_Degree_To_Upper(x_degree);
			Update_Current_Y_Degree_To_Upper(y_degree);
			Update_Welding_State_To_Upper(welding_state);
			Update_Current_Warning_To_Upper();
			Update_Current_Temperature_To_Upper(shtc3_temp);
			Update_Current_Humidity_To_Upper(shtc3_hum);
			Update_Total_Welding_Length_To_Upper(total_welding_length);
			Update_Weld_Track_State_To_Upper(weld_track_state);
			break;

		default:
			break;
	}
}
