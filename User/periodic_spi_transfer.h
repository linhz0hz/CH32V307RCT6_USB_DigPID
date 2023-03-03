/*
 * periodic_spi_transfer.h
 *
 *  Created on: Jul 16, 2022
 *      Author: linhz
 */

#ifndef USER_PERIODIC_SPI_TRANSFER_H_
#define USER_PERIODIC_SPI_TRANSFER_H_

#include "ch32v30x.h"


#define SPI_BUF_SIZE 128

/* Global Variable */
extern uint16_t TxData1[SPI_BUF_SIZE];
extern uint16_t TxData2[SPI_BUF_SIZE];
extern int16_t RxData[SPI_BUF_SIZE][2];
//extern int16_t RxData[SPI_BUF_SIZE];
//extern uint16_t TxData1;
//extern int16_t RxData;

void Setup_Periodic_Update(uint16_t period);
void Update_Index_After_SPI_Transfer (void);
int32_t Get_Last_Input (void);
void Set_Next_Output (int32_t output );
#endif /* USER_PERIODIC_SPI_TRANSFER_H_ */
