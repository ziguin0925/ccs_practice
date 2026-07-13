#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

#define PI_TSW 1.0f / 100000.0f

// Reference
float Vref = 48.0f;

// Feedback
float Vout = 0.0f;

// PI
float error = 0.0f;
float error_old = 0.0f;

float Kp = 0.2f;
float Ki = 20.0f;

// Switching Frequency (100kHz)
float fsw = 100000.0f;

float fsw_max = 200000.0f;
float fsw_min = 50000.0f;

float PI_control_PFM(){
    // ADC Read
    Vout = ADC_to_Voltage();

    // PI
    error = Vref - Vout;

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
    uint16_t tbprd;

    // return tbprd = (uint16_t)fsw/;
}