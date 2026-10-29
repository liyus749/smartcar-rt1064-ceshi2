#include "zf_common_headfile.h"
#include "PID.h"


#define STEER_RIGHT_MAX  630  //舵机右打死,-
#define STEER_MIDDLE    684  //舵机归中
#define STEER_LEFT_MAX   760  //舵机左打死,+
 
#define LEFT_Max     (STEER_LEFT_MAX - STEER_MID)//+
#define RIGHT_Max    (STEER_RIGHT_MAX - STEER_MID)//-
 
#define SPEED_MAX  4000
#define SPEED_MIN -2000

extern volatile uint8 Go;             //发车阶段的标志位
extern volatile int Island_State;     //环岛状态标志
extern volatile int Search_Stop_Line;
extern volatile uint8 Stop_Flag;
float Island_P=1.46;
float Island_D=4.1;
float Global_D=4.56;
float P_L=30;
float I_L=1.6;
float P_R=30;
float I_R=1.6;

/*---------------------  ----------------------------------------------------------------------------------------------
  @brief     PD摄像头控制
  @param     err
  @return    舵机打角值
  Sample     Steer_Angle=PD_Camera(0,Err);//舵机PD调节
  @note      null
-------------------------------------------------------------------------------------------------------------------*/
int PD_Camera(float expect_val,float err)//舵机PD调节
{
	
	float  u;
	float  P=3; //参数需自行整定，这里仅作为参考
	float  D=20;
	volatile static float error_current,error_last;
	float ek,ek1;
	error_current=err-expect_val;
	ek=error_current;
	ek1=error_current-error_last;
	u=P*ek+D*ek1;
	error_last=error_current;
	
 
	if(u>=LEFT_Max)//限幅处理
		u=LEFT_Max;
	else if(u<=RIGHT_Max)//限幅处理
		u=RIGHT_Max;
	return (int)u;
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     PID控制左电机
  @param     int set_speed ,int speed,期望值，实际值
  @return    电机占空比SPEED_MIN~SPEED_MAX
  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid控制电机转速
             pwm_L= PID_L(set_speed_left,left_wheel ); //pid控制电机转速
  @note      调参是门玄学
-------------------------------------------------------------------------------------------------------------------*/
int PID_L(int set_speed ,int speed)//pid控制电机转速
{
    volatile static int out;
    volatile static int out_increment;
    volatile static int ek,ek1;
    float kp=1.56,ki=2.1;
  
		kp = P_L;
		ki = I_L;
//	if(Stop_Flag==1)
//	{
////		kp = 80;
//		ki = 5;
//	}

    ek1 = ek;
    ek = set_speed - speed;
    out_increment= (int)(kp*(ek-ek1) + ki*ek);
    out+= out_increment;
 
    if(out>=SPEED_MAX)//限幅处理
        out=SPEED_MAX;
    else if(out<=SPEED_MIN)
        out=SPEED_MIN;
    return (int) out;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     PID控制右电机
  @param     int set_speed ,int speed,期望值，实际值
  @return    电机占空比
  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid控制电机转速
             pwm_L= PID_L(set_speed_left,left_wheel );//pid控制电机转速
  @note      调参呗
-------------------------------------------------------------------------------------------------------------------*/
int PID_R(int set_speed ,int speed)//pid控制电机转速
{
    volatile static int  out;
    volatile static int  out_increment;
    volatile static int  ek,ek1;
    float kp=1.56,ki=2.1;

		kp = P_R;//一套pi足矣，速度拨动不会太大
		ki = I_R;
//		if(Stop_Flag==1)
//		{
////			kp = 80;
//			ki = 5;
//		}
    ek1 = ek;
    ek = set_speed - speed;

    out_increment= (int)(kp*(ek-ek1) + ki*ek);
    out+= out_increment;

    if(out>=SPEED_MAX)//限幅处理
        out=SPEED_MAX;
    else if(out<=SPEED_MIN)
        out=SPEED_MIN;
    return (int) out;
}
