/**
 * @brief
 * Parse command sent by the master to the board.
 *
 * Company: Jacalt Laser Corp.
 * Author: J. Xu
 * Date: 20220506
 * Version: V1.2.0
 *
 */
#ifndef _CMDHANDLE_H__
#define _CMDHANDLE_H__

#include "stm32f4xx_hal.h"

//---- Heads of command and response to the laser
#define LASER_COMMEND_HEAD1 0xab
#define LASER_COMMEND_HEAD2 0xcd

#define LASER_RESPONSE_HEAD1 0xef
#define LASER_RESPONSE_HEAD2 0xef

//---- Heads of command and response to the board
#define BOARD_COMMEND_HEAD1 0xba
#define BOARD_COMMEND_HEAD2 0xdc

#define BOARD_RESPONSE_HEAD1 0xfe
#define BOARD_RESPONSE_HEAD2 0xfe

//---- device address of the board
#define BOARD_ADDRESS 0x00

//---- warning bit
#define STEPMOTOR_X_WARNING_BIT 0x01
#define STEPMOTOR_Y_WARNING_BIT 0x02

typedef enum {
	Write = 0x00,
	Read = 0x01
} enum_COMMEND_ACCESS;  //读写

typedef enum {
	Control_Motor_X_run = 0x00,
	Control_Motor_Y_run,
	Welding,
	Alarm_Information,
	Current_Temperature,
	Current_Humidity,
	Current_Welding_Length,
	Total_Welding_Length,
	Current_Date_And_Time,
	Weld_Tracking,
	Check_Parameter
} enum_BOARD_COMMEND_LIST; // 主控板控制指令

void Cmd_Handle(uint8_t* cmd, const uint8_t len);

#endif//_CMDHANDLE_H__
