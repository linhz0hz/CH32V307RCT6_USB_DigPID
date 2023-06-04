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
#include "ch32v30x_usbhs_device.h"
#include "ch32v30x_dbgmcu.h"
#include "debug.h"
#include "periodic_spi_transfer.h"

//#include "PID_controller_core.h"
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
 * @fn      get_ESIG
 *
 * @brief   Get unique digital signature.
 *
 * @return  uint32_t lowest 32bits of the unique identifier
 */
uint32_t get_ESIG32(void){
    return (*(uint32_t *)0x1FFFF7E8);
}

/*********************************************************************
 * @fn      get_ESIG_CRC
 *
 * @brief   Get unique digital signature, with CRC.
 *
 * @return  uint32_t lowest 32bits of the unique identifier
 */
uint32_t get_ESIG32_CRC(void){
	CRC_ResetDR();
    return CRC_CalcBlockCRC((uint32_t *)0x1FFFF7E8, 3);
}


volatile uint64_t systick_ms=0;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void) {
	//GET_INT_SP();
    SysTick->SR=0;
    systick_ms+=1;
	//FREE_INT_SP();
}

static uint32_t SysTick_Config_MS()
{
    NVIC_SetPriority(SysTicK_IRQn,0xf0);
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR=0;
    SysTick->SR=0;
    SysTick->CNT=0;
    SysTick->CMP=144000-1;
    SysTick->CTLR=0b00001111;
    return 0;
}



/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    SystemCoreClockUpdate( );
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init();
	USART_Printf_Init(115200);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	printf( "SystemClk:%d\r\n",SystemCoreClock);
	printf( "ESIG_CRC:%08X\r\n", get_ESIG32_CRC());
	//printf( "Running On USBHS Controller\r\n" );

	/* USB20 device init */
	generate_USB_Serial_Number();
	USBHS_RCC_Init( );
	USBHS_Device_Init( ENABLE );
	NVIC_EnableIRQ( USBHS_IRQn );

	SysTick_Config_MS();
	//Setup_PID_Computation_Indicator();
	//Initialize_PID_Core();
	Setup_Periodic_Update(24);
	//Start_Sweep();
	while(1) {
	    //PID_Update_Callback_Benchmark();
		//Delay_Ms(10);
		
		//printf( "Systick %03d \r\n",systick_ms ); //This has issues
	}
}

