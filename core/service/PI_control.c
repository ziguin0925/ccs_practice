#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

// Reference
float Vref = 2.0f;

// Feedback
float Vout_pi = 0.0f;

// PI
float error = 0.0f;
float error_old = 0.0f;

float Kp = 1.0f;
float Ki = 20.0f;

// Switching Frequency (50kHz)
float fsw_cmd = 50000.0f;

float fsw_max = 200000.0f;
float fsw_min = 30000.0f;

float PI_control_PFM(float Ts){

    float delta_fsw;


    Vout_pi = ((float)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) * 3.0f) / 4095.0f; // ADC Read  ADCaResultsRegs.ADCRESULT0

    error = Vref - Vout_pi; // PI

    // Δu=Kp(e(k)−e(k−1))+KiTse(k)
    delta_fsw = Kp * (error - error_old) + Ki * Ts * error;

    fsw_cmd -= delta_fsw;

    // Saturation
    if(fsw_cmd > fsw_max){
        fsw_cmd = fsw_max;
    } else if(fsw_cmd < fsw_min){
        fsw_cmd = fsw_min;
    }

    error_old = error;

    return fsw_cmd;
}
