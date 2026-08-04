#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

#define PI_TSW 10.0f / 100000.0f

// Reference
float Vref = 2.0f;

// Feedback
float Vout_pi = 0.0f;

// PI
float error = 0.0f;
float error_old = 0.0f;

float Kp = 1.0f;
float Ki = 20.0f;

// Switching Frequency (100kHz)
float fsw = 100000.0f;

float fsw_max = 200000.0f;
float fsw_min = 30000.0f;

float PI_control_PFM(){
    // ADC Read  ADCaResultsRegs.ADCRESULT0
    Vout_pi = ((float)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) * 3.0f) / 4095.0f;

    // PI
    error = Vref - Vout_pi;

    //PI_TSW : 제어 주기
    // Δu=Kp(e(k)−e(k−1))+KiTse(k)
    fsw -= Kp * (error - error_old) + Ki * PI_TSW * error;

    error_old = error;

    // Saturation
    if(fsw > fsw_max){
        fsw = fsw_max;
    } else if(fsw < fsw_min){
        fsw = fsw_min;
    }

    // TBPRD
    const float fsw_nominal = 100000.0f;
    float freq_multiplier = fsw / fsw_nominal;
    return freq_multiplier;
}
