/*********************************************************************************************************************
* 文件名称          MotorSpeed
* 功能简介          左右轮编码器速度闭环（前馈 + PI），详见 MotorSpeed.h
********************************************************************************************************************/

#include "MotorSpeed.h"
#include "Motor.h"

static int   sl_target   = 0;                       // 目标速度
static int   sl_speed_l  = 0;                       // 左轮实测（绝对值）
static int   sl_speed_r  = 0;                       // 右轮实测（绝对值）
static int   sl_duty_l   = 0;                       // 左轮输出占空比
static int   sl_duty_r   = 0;                       // 右轮输出占空比

static float sl_kp       = SPEED_KP_DEFAULT;        // Kp
static float sl_ki       = SPEED_KI_DEFAULT;        // Ki
static float sl_corr_l   = 0.0f;                    // 左轮 PI 修正量（累计）
static float sl_corr_r   = 0.0f;                    // 右轮 PI 修正量（累计）
static float sl_e_last_l = 0.0f;                    // 左轮上一拍误差
static float sl_e_last_r = 0.0f;                    // 右轮上一拍误差

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_Init(void)
{
    sl_kp = SPEED_KP_DEFAULT;
    sl_ki = SPEED_KI_DEFAULT;
    SpeedLoop_Reset();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位：目标清零、PI 状态清零、电机停
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_Reset(void)
{
    sl_target   = 0;
    sl_speed_l  = 0;
    sl_speed_r  = 0;
    sl_duty_l   = 0;
    sl_duty_r   = 0;
    sl_corr_l   = 0.0f;
    sl_corr_r   = 0.0f;
    sl_e_last_l = 0.0f;
    sl_e_last_r = 0.0f;

    Motor_Stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标速度（编码器计数/10ms）
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_SetTarget(int target)
{
    if(target < 0)                  { target = 0; }
    if(target > SPEED_TARGET_MAX)   { target = SPEED_TARGET_MAX; }

    sl_target = target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 PI 参数
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_SetGains(float kp, float ki)
{
    if(kp < 0.0f)             { kp = 0.0f; }
    if(kp > SPEED_KP_MAX)     { kp = SPEED_KP_MAX; }
    if(ki < 0.0f)             { ki = 0.0f; }
    if(ki > SPEED_KI_MAX)     { ki = SPEED_KI_MAX; }

    sl_kp = kp;
    sl_ki = ki;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     每 10ms 调一次：算速度环，输出 PWM
// 参数说明     enc_l / enc_r   这一段时间的编码器计数（带符号）
//
// 步骤：
//   1. 实测速度取绝对值（方向不影响）
//   2. 误差 = 目标 − 实测
//   3. 前馈 = 8.7 × 目标（保证"目标一变，占空比立刻跟着变"）
//   4. 增量式 PI 算出修正量
//   5. 修正量限幅，然后和前馈相加
//   6. 输出限幅 0~4000（只正转），写到左右轮
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_Update(int enc_l, int enc_r)
{
    float e_l;
    float e_r;
    float ff;
    int   d_l;
    int   d_r;

    /* ---------- 1. 实测速度（绝对值）---------- */
    sl_speed_l = (enc_l >= 0) ? enc_l : -enc_l;
    sl_speed_r = (enc_r >= 0) ? enc_r : -enc_r;

    /* ---------- 2. 误差 ---------- */
    e_l = (float)sl_target - (float)sl_speed_l;
    e_r = (float)sl_target - (float)sl_speed_r;

    /* ---------- 3. 前馈 ---------- */
    ff = SPEED_FF * (float)sl_target;

    /* ---------- 4. 增量式 PI（左右各一套）---------- */
    sl_corr_l += sl_kp * (e_l - sl_e_last_l) + sl_ki * e_l;
    sl_corr_r += sl_kp * (e_r - sl_e_last_r) + sl_ki * e_r;

    sl_e_last_l = e_l;
    sl_e_last_r = e_r;

    /* ---------- 5. 修正量限幅 ---------- */
    if(sl_corr_l >  SPEED_CORR_MAX) { sl_corr_l =  SPEED_CORR_MAX; }
    if(sl_corr_l < -SPEED_CORR_MAX) { sl_corr_l = -SPEED_CORR_MAX; }
    if(sl_corr_r >  SPEED_CORR_MAX) { sl_corr_r =  SPEED_CORR_MAX; }
    if(sl_corr_r < -SPEED_CORR_MAX) { sl_corr_r = -SPEED_CORR_MAX; }

    /* ---------- 6. 合成 + 输出限幅（只正转）---------- */
    d_l = (int)(ff + sl_corr_l);
    d_r = (int)(ff + sl_corr_r);

    if(d_l < 0)                 { d_l = 0; }
    if(d_l > SPEED_DUTY_MAX)    { d_l = SPEED_DUTY_MAX; }
    if(d_r < 0)                 { d_r = 0; }
    if(d_r > SPEED_DUTY_MAX)    { d_r = SPEED_DUTY_MAX; }

    sl_duty_l = d_l;
    sl_duty_r = d_r;

    Motor_Set(sl_duty_l, sl_duty_r);
}

//-------------------------------------------------------------------------------------------------------------------
// 下面几个是给屏幕 / 波形 / 调参用的查询接口
//-------------------------------------------------------------------------------------------------------------------
int SpeedLoop_GetTarget(void) { return sl_target;  }
int SpeedLoop_GetSpeedL(void) { return sl_speed_l; }
int SpeedLoop_GetSpeedR(void) { return sl_speed_r; }
int SpeedLoop_GetDutyL(void)  { return sl_duty_l;  }
int SpeedLoop_GetDutyR(void)  { return sl_duty_r;  }

float SpeedLoop_GetKp(void) { return sl_kp; }
float SpeedLoop_GetKi(void) { return sl_ki; }
