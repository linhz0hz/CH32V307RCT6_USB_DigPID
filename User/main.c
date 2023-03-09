/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"
#include "periodic_spi_transfer.h"
#include "PID_controller_core.h"
//#include "ch32v30x_gpio.h"
/* Global typedef */

/* Global define */

/* Global Variable */





/*
volatile float mul=0.0;
void mutil_10w(void)
{
	
	float a=1.122,b=0.96345;
	mul=0.0;
	u32 ka=0;

	for(ka=0;ka<100000;ka++)
	{
		mul=(a+mul)*b;
	}
}*/
/*
//test result: 7.5ms for 100k iterations at 120MHz
//Each iteration takes 9 cycles.
// Surprisingly, integer/fixed point is the same speed.
*/


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init();
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);

	printf("This is printf example\r\n");
	Setup_PID_Computation_Indicator();
	Initialize_PID_Core();
	Setup_Periodic_Update(24);
	Start_Sweep();
	while(1) {
	    //PID_Update_Callback_Benchmark();
		//Delay_Ms(10);

	}
}

