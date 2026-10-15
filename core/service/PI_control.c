#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "Fmath.h"
#include "PI_control.h"

// PI
float error = 0.0f;
float error_old = 0.0f;

float Vref = 0.0f;
float Iref = 0.0f;

//5ms정도 응답속도 (105V일 때 작동 안 돼서 15000에서 10000으로 변경)
float Kp_v = 3000.0f; //10000.0f;
float Ki_v = 500.0f; //1000.0f;

//5ms정도 응답속도
float Kp_i = 3000.0f; //10000.0f;
float Ki_i = 500.0f; //1000.0f;

// Switching Frequency (100kHz)
float fsw_cmd = 100000.0f;

float fsw_max = 130000.0f;
float fsw_min = 90000.0f;
float fTemp = 0.0f;
float feedback = 100000.0f;
float AWgain_v = 0.0f;
float AWgain_i = 0.0f;
float integ = 100000.0f;

void PI_Init(void){
    AWgain_v = (FLOAT_ONE / Kp_v);
    AWgain_i = (FLOAT_ONE / Kp_i);
}

float PI_V_control_PFM(float FO_LVV){

//    float delta_fsw;

    error = FO_LVV - Vref; // PI

//    // Δu=Kp(e(k)−e(k−1))+KiTse(k)
//    delta_fsw = Kp * (error - error_old) + Ki * error;
    fTemp = feedback - fsw_cmd;
    fTemp = AWgain_v * fTemp;
    fTemp = error - fTemp;
    integ += (Ki_v * fTemp);
    feedback = integ + (Kp_v * error);
    fTemp = feedback;

//    fsw_cmd -= delta_fsw;

    // Saturation
    if(fTemp > fsw_max){
        fsw_cmd = fsw_max;
    } else if(fTemp < fsw_min){
        fsw_cmd = fsw_min;
    } else{
        fsw_cmd = fTemp;
    }

//    error_old = error;

    return fsw_cmd;
}

float PI_I_control_PFM(float FO_LVI){

//    float delta_fsw;

    error = FO_LVI - Iref; // PI

//    // Δu=Kp(e(k)−e(k−1))+KiTse(k)
//    delta_fsw = Kp * (error - error_old) + Ki * error;
    fTemp = feedback - fsw_cmd;
    fTemp = AWgain_i * fTemp;
    fTemp = error - fTemp;
    integ += (Ki_i * fTemp);
    feedback = integ + (Kp_i * error);
    fTemp = feedback;

//    fsw_cmd -= delta_fsw;

    // Saturation
    if(fTemp > fsw_max){
        fsw_cmd = fsw_max;
    } else if(fTemp < fsw_min){
        fsw_cmd = fsw_min;
    } else{
        fsw_cmd = fTemp;
    }

//    error_old = error;

    return fsw_cmd;
}



