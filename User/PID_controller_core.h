/*
 * PID_controller_core.h
 *
 *  Created on: Jan 26, 2023
 *      Author: linhz
 */

#ifndef USER_PID_CONTROLLER_CORE_H_
#define USER_PID_CONTROLLER_CORE_H_
#include <stdint.h>

typedef struct {
    //Important constant for calculation:
    //-- Kp: proportional gain
    //-- KiT_half: Ki*T/2 integral gain scaled correctly.
    //-- Ki2T2_quat: Ki*T*T/4 double integral gain scaled correctly.
    
    float Kp,KiT_half,Ki2T2_quat;
    // Limit on the integral term
    float lim_intg_h,lim_intg_l,lim_intg2_h,lim_intg2_l;;
    // Limit on the output
    int32_t lim_out_h,lim_out_l;
    // Previous measurements
    float prev_err,prev_err_trapz;
    // Previous integrator and double integrator values
    float prev_intg,prev_intg2,prev_intg2_diff;

    float setpoint,offset;
} PID_Core;

void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void Initialize_PID_Core(void);
void PID_Update_Callback(void);
void Setup_PID_Computation_Indicator(void);
//void PID_Update_Callback_Benchmark (void);

#endif /* USER_PID_CONTROLLER_CORE_H_ */
