/*
 * decimation_core.c
 *
 *  Created on: Jan 26, 2023
 *      Author: linhz
 */
#include "decimation_core.h"
#include "periodic_spi_transfer.h"

/***
 * Multilevel buffer structure:
 * Layer 1: RxData1/2 Direct data from SPI 
 *          spi_rx_buf_head,spi_rx_buf_tail,spi_rx_buf_n keeps track
 *          of this cyclic buffer. Handled in interrupts directly.
 *          Most code in periodic_spi_transfer.c/h
 * 
 * Layer 2: Decimation buffer. Keep a cyclic buffer of size N and one 
 *          sum accumulator. With each new data, you delete the oldest
 *          data while adding the new data to the accumulator.
 *          With every M of the sum, dump it out to the next layer. Note
 *          to avoid aliasing, M should be smaller than N/2.
 *          This is not time critical, so should be handled in the main
 *          function when nothing of higher priority exist.
 * 
 * Layer 3: The data after decimation is arranged in a buffer, which is
 *          transfered out through USB periodically. 
 */


#define DECIMATION_FILT_N  32
#define DECIMATION_REDC_M  8

size_t decimation_buf_index;
size_t decimation_red_count;

int32_t decimation_buf1 [DECIMATION_FILT_N];
int32_t decimation_buf2 [DECIMATION_FILT_N];
int32_t decimation_sum1;
int32_t decimation_sum2;


/*********************************************************************
 * @fn      Initialze_Decimation
 *
 * @brief   Initialize data buffer and indexes for the decimation
 *
 * @param   none
 *
 * @return  none
 */
void Initialze_Decimation( void )
{
    for (size_t i = 0; i < DECIMATION_FILT_N; i++)
    {
        decimation_buf1[i] = 0;
        decimation_buf2[i] = 0;
    }
    decimation_buf_index = 0;
    decimation_red_count = 0;
    decimation_sum1 = 0;
    decimation_sum2 = 0;
}

/*********************************************************************
 * @fn      Run_Decimation
 *
 * @brief   Check if there is any new data. If there is, incorporate in
 *          the decimation algorithm.
 * 
 * @param   none
 *
 * @return  none
 */
void Run_Decimation( void )
{
    
    while (spi_rx_buf_n >0)
    {
        int32_t new_data;
        // Incorporate new data for channel 1
        decimation_sum1 -= decimation_buf1[decimation_buf_index];
        new_data = RxData1[spi_rx_buf_head];
        decimation_sum1 += new_data;
        decimation_buf1[decimation_buf_index] = new_data;
        // Incorporate new data for channel 2
        decimation_sum2 -= decimation_buf2[decimation_buf_index];
        new_data = RxData2[spi_rx_buf_head];
        decimation_sum2 += new_data;
        decimation_buf2[decimation_buf_index] = new_data;
        
        //Update indexes as new data is consumed
        // DECIMATION_FILT_N may not be a power of 2.
        // The method with if statement is better for 
        // non-power-of-2. The modulus method is better
        // for power-of-2 as it get compiled to a logic shift.
        decimation_buf_index +=1;
        if (decimation_buf_index == DECIMATION_FILT_N)
        {
            decimation_buf_index = 0;
        }

        decimation_red_count +=1;
        if (decimation_red_count == DECIMATION_REDC_M)
        {
            decimation_buf_index = 0;
            //Output new data here.
        }

        spi_rx_buf_head = (spi_rx_buf_head+1)%SPI_BUF_SIZE;
        spi_rx_buf_n -= 1;
        
    }

}


/*********************************************************************
 * @fn      Save_Decimated_Data
 *
 * @brief   Helper function to store the data after decimation correctly
 * 
 * @param   data: The data to be stored
 *
 * @return  none
 */
void Save_Decimated_Data (int32_t data)
{

}