#include "zf_common_headfile.h"
#include "Control.h"
#include "math.h"
#include "PID.h"

//舵机部分宏定义
#define SERVO_MOTOR1_PWM            (PWM4_MODULE2_CHA_C30)      //定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ            (50)                       //定义主板上舵机频率，请务必注意范围50-300，SD-5经验值为300
#define STEER_RIGHT_MAX  630  //舵机右打死,-
#define STEER_MIDDLE    684  //舵机归中
#define STEER_LEFT_MAX   760  //舵机左打死,+ 

#define LEFT_Max     (STEER_LEFT_MAX - STEER_MID)//+
#define RIGHT_Max    (STEER_RIGHT_MAX - STEER_MID)//-

//电机部分
#define SPEED_MAX  4000
#define SPEED_MIN -2000
#define MOTOR1_PWM1                 (PWM2_MODULE3_CHB_D3)
#define MOTOR1_PWM2                 (PWM2_MODULE3_CHA_D2)
#define MOTOR2_PWM1                 (PWM2_MODULE2_CHB_C11)
#define MOTOR2_PWM2                 (PWM2_MODULE2_CHA_C10)

//电磁，摄像头标志位，默认摄像头
volatile uint8 Electromagnet_Flag=0;//0是摄像头，1是电磁
extern volatile uint8 Stop_Flag;

//常规信息类
extern volatile int Search_Stop_Line;     //搜索截止行
extern volatile int Left_Line[MT9V03X_H];  //左边线数组
extern volatile int Right_Line[MT9V03X_H]; //右边线数组

//发车、停止、方向类
volatile uint8 Go=8;             //发车阶段的标志位，默认不动
volatile uint8 Speed_Mode=0;     //速度模式,0低速,1中等,2高速
volatile uint8 Go_Direction=1;   //0右,1左打角，出库
extern volatile uint8 Straight_Flag;//长直道识别
volatile uint8 start_flag;

//方向变量
int Steer_Angle=0;        //舵机打角值
volatile float Err=0;     //摄像头误差
extern volatile int Mid_Line[MT9V03X_H];

//速度类
int Base_Speed=100;    //基准速度
//int Speed_Left_Set=100;
//int Speed_Right_Set=100;
int Ramp_Speed=150;    //坡道速度0
int Island_Speed=150;   //环岛速度
int Straight_Speed=150; //直道高速
float Island_Diff=0.4;  //环岛差速

float Err_Diff=0.5;     //常规差速系数，差速过大容易导致侧翻

float Shift_Ratio=1.5;  //变速系数
float Island_Ratio=0;   //环岛变速

extern volatile int Zebra_Stripes_Flag;
//环岛控制
extern volatile int Island_State;     //环岛状态标志
extern volatile int Ramp_Flag;        //坡道

//横断控制
extern volatile uint8 Barricade_Flag; //横断
extern uint8 Obstacle_Dir; //0右拐，1左拐 

//加权控制
const uint8 Weight[MT9V03X_H]=
{
		1, 1, 1, 1, 1,1, 1, 1, 1, 1,//图像最远端00 ——09 行权重
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, //图像最远端10 ——19 行权重               
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,//图像最远端20 ——29 行权重              
		1, 1, 1, 3, 3, 5, 5, 7, 7, 9,//图像最远端30 ——39 行权重              
		9, 11, 11,13,13,14,14,15,15,15,//图像最远端40 ——49 行权重            
		15,15,15,15,15, 13, 13, 11, 11, 10,//图像最远端50 ——59 行权重             
		10, 9, 9, 7, 7, 5, 5, 3, 3, 1,//图像最远端60 ——69 行权重              
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1,//图像最远端70 ——79 行权重
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1,//图像最远端80 ——89 行权重
		//图像最远端90 ——99 行权重
};





/*-------------------------------------------------------------------------------------------------------------------
  @brief     摄像头误差获取
  @param     null
  @return    获取到的误差
  Sample     err=Err_Sum();
  @note      加权取平均
-------------------------------------------------------------------------------------------------------------------*/
float Err_Sum(void)
{
    int i;
    float err=0;
    float weight_count=0;
    //常规误差
    for(i=MT9V03X_H-1;i>=Search_Stop_Line-1;i--)//常规误差计算
    {
        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1))*Weight[i];//右移1位，等效除2
        weight_count+=Weight[i];
    }
//	for(i=85;i>=65;i--)
//    {
//        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1));//右移1位，等效除2
//		
//	}
	err=err/weight_count;
    
    return err;
}

/*
  GO==1停车，双手离车
  GO==2打角倒退
  GO==3刹车，方向都是反的
  GO==4微微后退
  GO==5刹车充电
  GO==6无
  GO==7常规寻迹
 */
void Direction_Control(void)
{
//	if(Zebra_Stripes_Flag==1)
//		Steer(80);
//	else
//	{
		Steer_Angle=PD_Camera(0, Err);           
		Steer(Steer_Angle);
//	}
	
	
}

/*
  GO==1停车，双手离车
  GO==2打角倒退
  GO==3刹车，方向都是反的
  GO==4微微后退
  GO==5刹车充电
  GO==6无
  GO==7常规寻迹
 */
void Velocity_Control(int speed_left_real,int speed_right_real ,int Speed_Left_Set ,int Speed_Right_Set)//赛道类型判别，来选定速度
{
	int pwm_R=0,pwm_L=0;
    
//	Speed_Left_Set =100;
//	Speed_Right_Set=100;
	
	
//	if(Ramp_Flag==1)
//	{
//		Speed_Left_Set =Ramp_Speed;
//	  Speed_Right_Set=Ramp_Speed;
//	}
//	Speed_Left_Set =Speed_Left_Set -(MT9V03X_H-Search_Stop_Line)*Shift_Ratio;//变速
//	Speed_Right_Set=Speed_Right_Set-(MT9V03X_H-Search_Stop_Line)*Shift_Ratio;//
// 
	Speed_Left_Set =Speed_Left_Set -Err*Err_Diff;//差速
	Speed_Right_Set=Speed_Right_Set+Err*Err_Diff;
   
	pwm_L= PID_L(Speed_Left_Set ,speed_left_real );//pid控制电机转速
	pwm_R= PID_R(Speed_Right_Set,speed_right_real);//pid控制电机转速
    
	Motor_Left (pwm_R);
	Motor_Right(pwm_L);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     电机输出函数
  @param     pwm_L 电机输入占空比，可正可负
  @return    null
  Sample     Motor_Left(pwm_L);
  @note      电机最终输出函数，由其他函数调用
                                      在给电机前有限幅处理的
-------------------------------------------------------------------------------------------------------------------*/
void Motor_Left(int pwm_L)
{
    if(pwm_L>=SPEED_MAX)//限幅处理
        pwm_L=SPEED_MAX;
    else if(pwm_L<=SPEED_MIN)
             pwm_L=SPEED_MIN;

    if(pwm_L>=0)//正转
    {
			pwm_set_duty(MOTOR1_PWM1, pwm_L);                  // 计算占空比
			pwm_set_duty(MOTOR1_PWM2, 0); 
//        gpio_set_level(MOTOR01_DIR_PIN, 1);//01左电机，D15,0反转，1正转
    }
    else
    {
			pwm_set_duty(MOTOR1_PWM1, 0);                  // 计算占空比
			pwm_set_duty(MOTOR1_PWM2, -pwm_L);
		
//       pwm_set_duty(MOTOR01_SPEED_PIN, -pwm_L);//左电机正转
//       gpio_set_level(MOTOR01_DIR_PIN, 0);//01左电机，D15,0反转，1正转
	}
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     电机输出函数
  @param     pwm_R 电机输入占空比，可正可负
  @return    null
  Sample     Motor_Right(pwm_R);
  @note      电机最终输出函数，由其他函数调用
                                      在给电机前有限幅处理的
-------------------------------------------------------------------------------------------------------------------*/
void Motor_Right(int pwm_R)
{
    if(pwm_R>=SPEED_MAX)//限幅处理
        pwm_R=SPEED_MAX;
    else if(pwm_R<=SPEED_MIN)
            pwm_R=SPEED_MIN;
	
    if(pwm_R>=0)//正转
    {
			pwm_set_duty(MOTOR2_PWM1, pwm_R);                  // 计算占空比
			pwm_set_duty(MOTOR2_PWM2, 0); 
//        gpio_set_level(MOTOR01_DIR_PIN, 1);//01左电机，D15,0反转，1正转
    }
    else
    {
			pwm_set_duty(MOTOR2_PWM1, 0);                  // 计算占空比
			pwm_set_duty(MOTOR2_PWM2, -pwm_R);
	}
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     舵机电机输出函数
  @param     angle 输入舵机方向信号，+-LEFT_MAX，RIGHT_MAX之间
  @return    null
  Sample     Steer(angle)；
  @note      舵机最终输出函数，由其他函数调用
                                           在给舵机前有限幅处理的
#define STEER_RIGHT  705  //舵机右打死,-
#define STEER_MID    782  //舵机归中
#define STEER_LEFT   857  //舵机左打死,+
-------------------------------------------------------------------------------------------------------------------*/
void Steer(int angle)
{
    if(angle>=LEFT_Max)//限幅处理
		angle=LEFT_Max;
   else if(angle<=RIGHT_Max)
        angle=RIGHT_Max;
   
    pwm_set_duty(SERVO_MOTOR1_PWM, STEER_MIDDLE+angle);//舵机调节
}


void Start(void)
{
	Steer(200);
	if(FJ_Angle <= -80 && start_flag == 1)
		start_flag = 0;
		
	
}





