/*
 * 文件：
 * 说明：初始化应用层文件
 * 项目：焊接跟踪项目
 * 作者：张祺
 * 版本：V1.0.0
 *
 * Modification: 根据实际情况精简，且修改了get_eeprom的程序错误，
 *               补充了set_eeprom的内容
 * Company: Jacalt Laser Corp.
 * Author: J. Xu
 * Date: 20220506
 * Version: V1.2.0
 *
 */
#include "sysinit.h"

/*
 * 功能：软件初始化
 * 参数：void
 * 返回：void
 * 备注：所有用到的模块初始化
 */
void software_init(void)
{
//	SHTC3_Init();      //shtc3初始化
	//TODO 水冷机初始化
}

/*
 * 功能：获取EEPROM数据
 *      total welding length and the stepmotor positions
 * 参数：void
 * 返回：void
 * 备注：
 */
void get_eeprom_data(void)
{
	uint8_t temp[12];
	M24512_ReadBuffer(temp, TOTAL_WELDING_LENGTH_ADDRESS, 12);

	total_welding_length = temp[0] | (temp[1]<<8) | (temp[2]<<16) | (temp[3]<<24);
	x_degree = temp[4] | (temp[5]<<8) | (temp[6]<<16) | (temp[7]<<24);
	y_degree = temp[8] | (temp[9]<<8) | (temp[10]<<16) | (temp[11]<<24);

}

/*
 * 功能：设置EEPROM数据
 * 参数：void
 * 返回：void
 * 备注：
 */
void set_eeprom_data(void)
{
//	TODO write data to EEPROM when shut down
}

/*
 * 功能：自检
 * 参数：void
 * 返回：void
 * 备注：所有用到的模块自检一遍，如果有问题，上传给上位机处理
 */
void check_machine(void)
{
//	if (SHTC3_READ_ID() == _SHTC3_ID)
	{
//		warning[0] = OTHER_WARNING_BIT;
//		waring[1] = SHTC_INIT_ERROR;
	}

//	if (M24512_CHECK() == 0)
	{
//		warning[0] = OTHER_WARNING_BIT;
//		waring[1] = M24512_INIT_ERROR;
	}
	// 自检通过，停止闪烁
//	HAL_TIM_Base_Stop_IT(&htim7);
}
