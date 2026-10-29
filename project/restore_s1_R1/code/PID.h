#ifndef __PID_H
#define __PID_H

#include "zf_common_headfile.h"

#define SPEED_MAX    4000  //电机速度限幅，正
#define SPEED_MIN   -2000 //电机速度限幅，负

int PD_Camera(float expect_val, float err);//舵机PD调节
int PID_L(int set_speed ,int speed);//pid控制电机转速
int PID_R(int set_speed ,int speed);//pid控制电机转速


#endif
