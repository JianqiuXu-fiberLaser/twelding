/*
 * 文件：
 * 说明：初始化应用层文件
 * 项目：焊接跟踪项目
 * 作者：张祺
 * 版本：V1.0.0
 *
 * Company: Jacalt Laser Corp.
 * Author: J. Xu
 * Date: 20220506
 * Version: V1.2.0
 *
 */

#ifndef _SYSINIT_H__
#define _SYSINIT_H__

	#include "stm32f4xx_hal.h"
	#include "stepmotor.h"
	#include "m24512.h"
//	#include "shtc3.h"
	#include "main.h"

    #define OTHER_WARNING_BIT 0X10
    #define SHTC_INIT_ERROR 0x01
    #define M24512_INIT_ERROR 0x02

	void software_init(void);
	void get_eeprom_data(void);
	void set_eeprom_data(void);
	void check_machine(void);

#endif //_SYSINIT_H__
