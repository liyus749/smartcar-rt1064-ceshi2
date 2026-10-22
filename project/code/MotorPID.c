/*********************************************************************************************************************
* 文件名称          MotorPID.c
* 功能简介          电机速度闭环实现（增量式 PID，说明见 MotorPID.h）
********************************************************************************************************************/

#include "MotorPID.h"

//===================================================================================================================
// 内部状态
//===================================================================================================================
static volatile float g_target   = MOTORPID_DEFAULT_TARGET;   // 设定目标
static volatile float g_cur_target = 0.0f;                   // 软启动爬坡中的实际目标
static volatile float g_kp       = MOTORPID_DEFAULT_KP;
static volatile float g_ki       = MOTORPID_DEFAULT_KI;
static volatile float g_kd       = MOTORPID_DEFAULT_KD;
static volatile uint8 g_enable   = 0;

static volatile float g_out      = 0.0f;      // 左右输出平均值（屏幕显示）
static volatile float g_measured = 0.0f;      // 左右实测速度平均值
static volatile float g_acc_l    = 0.0f;      // 左轮独立 PID 输出累计值
static volatile float g_acc_r    = 0.0f;      // 右轮独立 PID 输出累计值
static volatile float g_last_error_l = 0.0f;
static volatile float g_prev_error_l = 0.0f;
static volatile float g_last_error_r = 0.0f;
static volatile float g_prev_error_r = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     限幅
//-------------------------------------------------------------------------------------------------------------------
static float limitf(float v, float lo, float hi)
{
    if(v < lo) { return lo; }
    if(v > hi) { return hi; }
    return v;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位 PID（清误差、清输出累计）
//-------------------------------------------------------------------------------------------------------------------
static void pid_reset(void)
{
    g_cur_target  = 0.0f;
    g_out         = 0.0f;
    g_measured    = 0.0f;
    g_acc_l       = 0.0f;
    g_acc_r       = 0.0f;
    g_last_error_l = 0.0f;
    g_prev_error_l = 0.0f;
    g_last_error_r = 0.0f;
    g_prev_error_r = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化
//-------------------------------------------------------------------------------------------------------------------
void MotorPID_Init(void)
{
    g_target = MOTORPID_DEFAULT_TARGET;
    g_kp     = MOTORPID_DEFAULT_KP;
    g_ki     = MOTORPID_DEFAULT_KI;
    g_kd     = MOTORPID_DEFAULT_KD;
    g_enable = 0;
    pid_reset();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标速度（限幅 >= 0）
//-------------------------------------------------------------------------------------------------------------------
void MotorPID_SetTarget(float target)
{
    if(target < 0.0f) { target = 0.0f; }
    g_target = target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 PID 参数
//-------------------------------------------------------------------------------------------------------------------
void MotorPID_SetGains(float kp, float ki, float kd)
{
    g_kp = kp;
    g_ki = ki;
    g_kd = kd;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使能 / 停止
//-------------------------------------------------------------------------------------------------------------------
void MotorPID_SetEnable(uint8 enable)
{
    if(enable)
    {
        if(0 == g_enable)
        {
            /* 从停 -> 跑：清干净，从头软启动 */
            pid_reset();
        }
        g_enable = 1;
    }
    else
    {
        g_enable = 0;
        pid_reset();
        Motor_Stop();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     每个控制周期调一次（20ms 中断里）
//-------------------------------------------------------------------------------------------------------------------
static float pid_update_one(float target, float measured,
                            volatile float *last_error,
                            volatile float *prev_error,
                            volatile float *accumulator)
{
    float error = target - measured;
    float d_error = error - *last_error;
    float inc = g_kp * d_error
              + g_ki * error
              + g_kd * (error - 2.0f * *last_error + *prev_error);

    *accumulator += inc;
    *accumulator = limitf(*accumulator, 0.0f, (float)MOTORPID_MAX_DUTY);
    *prev_error = *last_error;
    *last_error = error;
    return *accumulator;
}

int MotorPID_Update(int enc_l, int enc_r)
{
    float measured_l;
    float measured_r;
    float out_l;
    float out_r;

    if(0 == g_enable)
    {
        Motor_Stop();
        g_out      = 0.0f;
        g_measured = 0.0f;
        return 0;
    }

    /* 左右编码器分别取绝对值。两侧电机负载/磨损不同，必须分别闭环，
       否则较快的一侧会掩盖较慢甚至停转的一侧。 */
    measured_l = (float)enc_l;
    measured_r = (float)enc_r;
    if(measured_l < 0.0f) { measured_l = -measured_l; }
    if(measured_r < 0.0f) { measured_r = -measured_r; }
    g_measured = (measured_l + measured_r) * 0.5f;

    /* 软启动：目标速度从 0 慢慢爬到设定值（RAMP_MS 内到满）*/
    if(g_cur_target < g_target)
    {
        float step = g_target * (20.0f / MOTORPID_RAMP_MS);   /* 每 20ms 加这么多 */
        if(step < 0.01f) { step = 0.01f; }
        g_cur_target += step;
        if(g_cur_target > g_target) { g_cur_target = g_target; }
    }
    else
    {
        g_cur_target = g_target;
    }

    /* 左右轮各自追踪同一个目标速度，独立补偿电机差异和弯道负载。 */
    out_l = pid_update_one(g_cur_target, measured_l,
                           &g_last_error_l, &g_prev_error_l, &g_acc_l);
    out_r = pid_update_one(g_cur_target, measured_r,
                           &g_last_error_r, &g_prev_error_r, &g_acc_r);

    g_out = (out_l + out_r) * 0.5f;
    Motor_Set((int)out_l, (int)out_r);

    return (int)g_out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前目标速度
//-------------------------------------------------------------------------------------------------------------------
float MotorPID_GetTarget(void)
{
    return g_cur_target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前输出占空比
//-------------------------------------------------------------------------------------------------------------------
int MotorPID_GetOutput(void)
{
    return (int)g_out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取实测速度
//-------------------------------------------------------------------------------------------------------------------
float MotorPID_GetMeasured(void)
{
    return g_measured;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取使能状态
//-------------------------------------------------------------------------------------------------------------------
uint8 MotorPID_IsEnabled(void)
{
    return g_enable;
}
