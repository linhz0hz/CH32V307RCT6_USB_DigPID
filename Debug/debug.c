/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "debug.h"
#include "ch32v30x_usart.h"

static uint8_t  p_us = 0;
static uint16_t p_ms = 0;

/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Initializes Delay Funcation.
 *
 * @return  none
 */
void Delay_Init(void)
{
    p_us = SystemCoreClock / 8000000;
    p_ms = (uint16_t)p_us * 1000;
}

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time.
 *
 * @param   n - Microsecond number.
 *
 * @return  None
 */
void Delay_Us(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * p_us;

    SysTick->CMP = i;
    SysTick->CTLR |= (1 << 4) | (1 << 5) | (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0))
        ;
    SysTick->CTLR &= ~(1 << 0);
}

/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond Delay Time.
 *
 * @param   n - Millisecond number.
 *
 * @return  None
 */
void Delay_Ms(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * p_ms;

    SysTick->CMP = i;
    SysTick->CTLR |= (1 << 4) | (1 << 5) | (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0))
        ;
    SysTick->CTLR &= ~(1 << 0);
}

/*********************************************************************
 * @fn      USART_Printf_Init
 *
 * @brief   Initializes the USARTx peripheral.
 *
 * @param   baudrate - USART communication baud rate.
 *
 * @return  None
 */
#define DEBUG_UART_BUF_SIZE 128
#define DEBUG_UART_BUF_MASK DEBUG_UART_BUF_SIZE-1

uint8_t debug_uart_tx_buffer [DEBUG_UART_BUF_SIZE];
//uint8_t debug_uart_rx_buffer [DEBUG_UART_BUF_SIZE];
volatile size_t debug_uart_tx_head;
volatile size_t debug_uart_tx_tail;
//volatile size_t debug_uart_tx_fill;
typedef enum { UART_DBG_READY = 0,UART_DBG_SENDING = 1} UART_TX_Buf_Status;
volatile UART_TX_Buf_Status  debug_uart_tx_status;

void USART_Printf_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx; //Tx only as we are doing debug

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    //Enable the TX interrupt, so when the data is taken to the shift register of the
    // peripheral, we can put in the next byte.

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    debug_uart_tx_head=0;
    debug_uart_tx_tail=0;
    //debug_uart_tx_fill=0;
    debug_uart_tx_status = UART_DBG_READY;
    
}


void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
    {
        //Interrupt handler for trasmit 
        //USART1_send_char_from_buffer();
        //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

        //debug_uart_tx_head=(debug_uart_tx_head+1)&DEBUG_UART_BUF_MASK;
        //debug_uart_tx_fill--;
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        //if (debug_uart_tx_fill == 0)
        if (debug_uart_tx_head == debug_uart_tx_tail)
        {
            // If there is no pending data, stop transmission.
            //USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            debug_uart_tx_status = UART_DBG_READY; 
        }
        else{
            // If there is still pending data, transmit the next byte.
            USART_SendData(USART1, debug_uart_tx_buffer[debug_uart_tx_head]);
            debug_uart_tx_head=(debug_uart_tx_head+1)&DEBUG_UART_BUF_MASK;
            //debug_uart_tx_fill--;
            USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
        }
    }
}


void USART1_putchar(char c){
    //while (debug_uart_tx_fill >= DEBUG_UART_BUF_SIZE);
    debug_uart_tx_buffer[debug_uart_tx_tail] = c;
    debug_uart_tx_tail=(debug_uart_tx_tail+1)&DEBUG_UART_BUF_MASK;
    //debug_uart_tx_fill++;
    if (debug_uart_tx_status == UART_DBG_READY)
    {
        debug_uart_tx_status = UART_DBG_SENDING;
        USART_SendData(USART1, debug_uart_tx_buffer[debug_uart_tx_head]);
        debug_uart_tx_head=(debug_uart_tx_head+1)&DEBUG_UART_BUF_MASK;
        //debug_uart_tx_fill--;
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }
}
/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size: Data length
 */
__attribute__((used)) int _write(int fd, char *buf, int size)
{
    int i;

    for(i = 0; i < size; i++)
    {
        USART1_putchar(*buf++);
    }
    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
/*
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}
*/


