#ifndef PI_CONTROL_H
#define PI_CONTROL_H

// Return: frequency multiplier relative to nominal (1.0 = no change)
extern float PI_V_control_PFM(float FO_LVV);
extern float PI_I_control_PFM(float FO_LVI);
extern void PI_Init(void);

// PI
extern float error;

extern float Vref;
extern float Iref;

extern float Kp_v;
extern float Ki_v;

extern float Kp_i;
extern float Ki_i;
// Switching Frequency (100kHz)
extern float fsw_cmd;

extern float fsw_max;
extern float fsw_min;
extern float fTemp;
extern float feedback;
extern float AWgain_v;
extern float AWgain_i;
extern float integ;

#endif
// PI_CONTROL_H
