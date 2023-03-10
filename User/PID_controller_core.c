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


enum Operation_Mode_Enum operation_mode = OP_MODE_IDLE;

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
    return RxData1[spi_rx_buf_index];
}

/*********************************************************************
 * @fn      Set_Next_Output
 *
 * @brief   Setter for next DAC output
 *
 * @param   output signed int 32, next DAC output. 
 *          Assume 20 bits are effective
 */

__attribute__((always_inline)) inline void Set_Next_Output (uint16_t output1, uint16_t output2)
{
    //int32_t shifted_output = output + 0x80000;
    // For MAX5717/5719
    //For MAX5717
    TxData1[spi_tx_buf_index] = output1;
    TxData2[spi_tx_buf_index] = output2;

    // For AD5542 / MS5542
    //TxData1[spi_tx_buf_index] = shifted_output >> 16;
    //TxData2[spi_tx_buf_index] = shifted_output & 0xffff;
}



PID_Core pid_a;




// Here we keep a record of a few points with the most 
// positive and the most negative error signal. 
// Then we will average them, find the intersect with 0
// which should be our best guess for the lock point.
/*#define MINMAX_RECORD_N 3
int32_t max_err_val [MINMAX_RECORD_N];
unt32_t max_err_pzv [MINMAX_RECORD_N];
int32_t min_err_val [MINMAX_RECORD_N];
unt32_t min_err_pzv [MINMAX_RECORD_N];



void Execute_Scan_Callback(void){
    // get the error value
    int32_t err_val = Get_Last_Input();
    //bool max_updated = false, min_updated = false;
    for (size_t i = 0;i<MINMAX_RECORD_N;i++){
        if (err_val>max_err_val[i]){
            max_err_val[i]=err_val;
            max_err_pzv[i]=piezo_HV_current;
            break;
        }
    }
    for (size_t i = 0;i<MINMAX_RECORD_N;i++){
        if (err_val<min_err_val[i]){
            min_err_val[i]=err_val;
            min_err_pzv[i]=piezo_HV_current;
            break;
        }
    }

    if (piezo_HV_current > PIEZO_HV_MAX){
        piezo_HV_inc = - piezo_HV_inc;
    }
    if (piezo_HV_current < PIEZO_HV_MIX){
        piezo_HV_inc = - piezo_HV_inc;
    }
    piezo_HV_current += piezo_HV_inc;
    Set_Next_Output(piezo_HV_current);
}
*/


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
    Set_Next_Output(out_int,out_int);
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

// Two-DAC architecture for piezo scanning:
// Slow path: this DAC is connected to amplifying buffer for high 
// voltage bias (30V) and  contains low-pass filter to reduce noise.  
//  This path can also be used to implement the double integrator.
// Fast path: this DAC is connected to the buffer with small gain. 
// The range is small (5V) but there is no low-pass filter.


#define PIEZO_RAMP_MAX 0xFF80
#define PIEZO_RAMP_MIN 0x0080


// Struct containing piezo scan info. 
// For piezo scan, the LV side is fixed, while the HV side is varied
// to find the lock point automatically.
typedef struct {
    
    // Remembers the condition of last lock point.
    int32_t piezo_HV_memory,piezo_LV_memory;
    // The cuurent HV piezo voltage
    int32_t piezo_HV_current;
    // The increment for piezo scan.
    int32_t piezo_HV_inc;
    int32_t piezo_HV_ramp_step;
} Ramping_Core;

Ramping_Core sweep_core;

void Sweep_Callback(void){
    sweep_core.piezo_HV_current += sweep_core.piezo_HV_ramp_step;

    if ( ((sweep_core.piezo_HV_ramp_step > 0) && (sweep_core.piezo_HV_current > PIEZO_RAMP_MAX)) ||
         ((sweep_core.piezo_HV_ramp_step < 0) && (sweep_core.piezo_HV_current < PIEZO_RAMP_MIN)) ) {
        sweep_core.piezo_HV_ramp_step = -sweep_core.piezo_HV_ramp_step;
    }
    Set_Next_Output(sweep_core.piezo_HV_current,sweep_core.piezo_HV_current);
}

void Start_Sweep(void){
    sweep_core.piezo_HV_ramp_step = 1;
    sweep_core.piezo_HV_current = 0x7fff;
    operation_mode = OP_MODE_SWEEP;
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
    if (operation_mode == OP_MODE_LOCK) {
        PID_Update_Callback();
    } else if (operation_mode == OP_MODE_SWEEP){
        Sweep_Callback();
    }
    
    Update_Index_After_SPI_Transfer();
}
