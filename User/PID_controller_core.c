/*
 * PID_controller_core.c
 *
 *  Created on: Jan 26, 2023
 *      Author: linhz
 */
#include "PID_controller_core.h"
#include "ch32v30x_gpio.h"
#include "periodic_spi_transfer.h"



size_t spi_tx_buf_index = 1;
size_t spi_rx_buf_index = 0;

/*********************************************************************
 * @fn      Update_Index_After_SPI_Transfer
 *
 * @brief   Callback used to update buffer index after each transfer.
 *          Should be called in the appropriate timer interrupt.
 *
 * @return  none
 */

__attribute__((always_inline)) inline void Update_Index_After_SPI_Transfer (void)
{
    spi_tx_buf_index = (spi_tx_buf_index+1)%SPI_BUF_SIZE;
    spi_rx_buf_index = (spi_rx_buf_index+1)%SPI_BUF_SIZE;
}


/*********************************************************************
 * @fn      Get_Last_Input
 *
 * @brief   Getter for last ADC input
 *
 * @return  signed int 32, last ADC input
 */

__attribute__((always_inline)) inline int32_t Get_Last_Input (void)
{
    // The ADS8883/8863 sensor is at most 20bits, the last 12 bits are dropped.
    return RxData[spi_rx_buf_index][0];
}

/*********************************************************************
 * @fn      Set_Next_Output
 *
 * @brief   Setter for next DAC output
 *
 * @param   output signed int 32, next DAC output. 
 *          Assume 20 bits are effective
 */

__attribute__((always_inline)) inline void Set_Next_Output (int32_t output )
{
    //int32_t shifted_output = output + 0x80000;
    // For MAX5717/5719
    TxData1[spi_tx_buf_index] = output;
    TxData2[spi_tx_buf_index] = output;

    // For AD5542 / MS5542
    //TxData1[spi_tx_buf_index] = shifted_output >> 16;
    //TxData2[spi_tx_buf_index] = shifted_output & 0xffff;
}



PID_Core pid_a;
//volatile int32_t output_DAC;
void Initialize_PID_Core(void)
{

    pid_a.Kp = -1.0f;
    //pid_a.KiT_half = 1.0e-3f;
    pid_a.KiT_half = 0.0f;
    //pid_a.Ki2T2_quat = 1.0e-9f;
    pid_a.Ki2T2_quat = 0.0f;
    pid_a.lim_intg_h = 15000.0f;
    pid_a.lim_intg_l = -15000.0f;
    pid_a.lim_intg2_h = 15000.0f;
    pid_a.lim_intg2_l = -15000.0f;
    
    // Output limited to 20 bits
    pid_a.lim_out_h = 0x10000 - 100;
    pid_a.lim_out_l = 100; 


    pid_a.prev_err = 0.0f;
    pid_a.prev_err_trapz = 0.0f;
    //pid_a.prev_inp = 0.0f;

    pid_a.prev_intg = 0.0f;
    pid_a.prev_intg2 = 0.0f;
    pid_a.prev_intg2_diff = 0.0f;

    pid_a.setpoint = 0.0f;
    pid_a.offset = 0.0f;

}

void Set_PID_Param( float Kp, float Ki, float sampling_time)
{
    pid_a.Kp = Kp;
    pid_a.KiT_half = 0.5f*Ki*sampling_time;
}

void Update_PID_Setpoint(float setpoint)
{
    pid_a.setpoint = setpoint;
}

void Update_PID_Offset(float offset)
{
    pid_a.offset = offset;
}

/*********************************************************************
 * @fn      SPI1_IRQHandler
 *
 * @brief   This function handles SPI1 interrupt.
 *
 * @return  none
 */
void PID_Update_Callback(void) __attribute__((always_inline));
inline void PID_Update_Callback(void)
{
    int32_t in_int;
    int32_t out_int;
    const float scaling_down  = 1.0f / (1<<16);
    const float scaling_up  = 1<<16;
    static float input,err,err_trapz,prop,intg,intg2;
    float out_fp;
    
    //GPIO_WriteBit(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 ,1);
    GPIOA->BSHR = 0x03; //Avoid function call

    in_int = Get_Last_Input();
    input = in_int;
    err = pid_a.setpoint - input;

    // Proportional feedback
    prop = err*pid_a.Kp;
    // Integral feedback.
    // Trapezoidial integration for higher accuracy / stability.
    err_trapz = (pid_a.prev_err+err);
    intg = pid_a.prev_intg + pid_a.KiT_half*(pid_a.prev_err+err);
    // Clamp the integral term
    // Note I am not using if-else statement here, using if-only
    // is actually faster, since there are instructions lile fmin/fmax
    if (intg > pid_a.lim_intg_h) {
        intg = pid_a.lim_intg_h;
    }
    if (intg < pid_a.lim_intg_l) {
        intg = pid_a.lim_intg_l;
    }
    pid_a.prev_err = err;
    pid_a.prev_intg = intg;

    // Double Integral feedback.
    // Double Trapezoidial integration for higher accuracy / stability.
    intg2 = pid_a.prev_intg2 + pid_a.prev_intg2_diff + pid_a.Ki2T2_quat*(pid_a.prev_err_trapz+err_trapz);
    
    // Clamp the integral term
    //if (intg2 > pid_a.lim_intg2_h) {
    //    intg2 = pid_a.lim_intg2_h;
    //}
    //if (intg2 < pid_a.lim_intg2_l) {
    //    intg2 = pid_a.lim_intg2_l;
    //}
    pid_a.prev_err_trapz = err_trapz;
    pid_a.prev_intg2_diff = intg2-pid_a.prev_intg2;
    pid_a.prev_intg2 = intg2;
    


    // Now collect all terms, and scale back to integer.
    out_fp = prop+intg+intg2;
    out_int = ((int32_t)out_fp) + 0x8000;


    // Clamp the output to be within the 20bit DAc range.
    if (out_int > pid_a.lim_out_h) {
        out_int = pid_a.lim_out_h;
    }
    if (out_int < pid_a.lim_out_l) {
        out_int = pid_a.lim_out_l;
    }
    Set_Next_Output(out_int);
    //GPIO_WriteBit(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 ,0);
    GPIOA->BCR  = 0x03; //Avoid function call
}


/*void PID_Update_Callback_Benchmark (void)
{
    //-Osize result: 13.666-14ms at 120MHz. 82 clock cycles
    //-Ofast result (but diabled inline, otherwize the loop is unrolled): 11.666 ms at 120MHz 70 clcok cycles
    GPIO_WriteBit(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 ,1);
    for(int ka=-10000;ka<10000;ka++) {
	    PID_Update_Callback(ka);
	}
    GPIO_WriteBit(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 ,0);
}*/
void Setup_PID_Computation_Indicator(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 ,0);

}



void TIM4_IRQHandler(void) {

    // One full ADC/DAC transaction takes two SPI transfer.
    // We only process the data after a complete transaction.
    // We count the number of transfers and only execute when
    // the number is even, right after a complete transfer.
    //spi_rx_tansfer_count++;
    //if(spi_rx_tansfer_count & 0x01){
    //    return;
    //}
    //TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
    // Avoid function call, clear interrupt flag directly.
    TIM4->INTFR = (uint16_t)~TIM_IT_CC3; 
    PID_Update_Callback();
    Update_Index_After_SPI_Transfer();
}
