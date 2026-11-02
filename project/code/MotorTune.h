#ifndef MOTOR_TUNE_H
#define MOTOR_TUNE_H
#include "zf_common_headfile.h"

// USB 电机调参专用；10ms采样、20ms闭环，主循环负责通信/显示。
void MotorTune_Init(void);
void MotorTune_Keys10ms(void);
void MotorTune_Sample10ms(int left, int right);
void MotorTune_Control20ms(void);
void MotorTune_Poll(void);
void MotorTune_Show(void);
#endif
