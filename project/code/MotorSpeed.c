/*********************************************************************************************************************
* 文件名称          MotorSpeed
* 功能简介          左右轮编码器速度闭环（前馈 + PI），详见 MotorSpeed.h
********************************************************************************************************************/

#include "MotorSpeed.h"
#include "Motor.h"

static int   sl_target_l = 0;                       // 左轮目标速度
static int   sl_target_r = 0;                       // 右轮目标速度
static int   sl_speed_l  = 0;                       // 左轮实测（绝对值）
static int   sl_speed_r  = 0;                       // 右轮实测（绝对值）
static int   sl_duty_l   = 0;                       // 左轮输出占空比
static int   sl_duty_r   = 0;                       // 右轮输出占空比

static float sl_kp_l     = SPEED_KP_DEFAULT;        // 左轮 Kp
static float sl_ki_l     = SPEED_KI_DEFAULT;        // 左轮 Ki
static float sl_kp_r     = SPEED_KP_DEFAULT;        // 右轮 Kp
static float sl_ki_r     = SPEED_KI_DEFAULT;        // 右轮 Ki
static float sl_corr_l   = 0.0f;                    // 左轮 PI 修正量（累计）
static float sl_corr_r   = 0.0f;                    // 右轮 PI 修正量（累计）
static float sl_e_last_l = 0.0f;                    // 左轮上一拍误差
static float sl_e_last_r = 0.0f;                    // 右轮上一拍误差

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_Init(void)
{
    sl_kp_l = SPEED_KP_DEFAULT;
    sl_ki_l = SPEED_KI_DEFAULT;
    sl_kp_r = SPEED_KP_DEFAULT;
    sl_ki_r = SPEED_KI_DEFAULT;
    SpeedLoop_Reset();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位：目标清零、PI 状态清零、电机停
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_Reset(void)
{
    sl_target_l = 0;
    sl_target_r = 0;
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

    sl_target_l = target;
    sl_target_r = target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     左右轮分别设置目标速度
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_SetTargetLR(int target_l, int target_r)
{
    if(target_l < 0)                { target_l = 0; }
    if(target_l > SPEED_TARGET_MAX) { target_l = SPEED_TARGET_MAX; }
    if(target_r < 0)                { target_r = 0; }
    if(target_r > SPEED_TARGET_MAX) { target_r = SPEED_TARGET_MAX; }

    sl_target_l = target_l;
    sl_target_r = target_r;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 PI 参数
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_SetGains(float kp, float ki)
{
    SpeedLoop_SetGainsLR(kp, ki, kp, ki);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     左右轮分别设置 PI 参数
//-------------------------------------------------------------------------------------------------------------------
void SpeedLoop_SetGainsLR(float kp_l, float ki_l, float kp_r, float ki_r)
{
    if(kp_l < 0.0f)         { kp_l = 0.0f; }
    if(kp_l > SPEED_KP_MAX) { kp_l = SPEED_KP_MAX; }
    if(ki_l < 0.0f)         { ki_l = 0.0f; }
    if(ki_l > SPEED_KI_MAX) { ki_l = SPEED_KI_MAX; }
    if(kp_r < 0.0f)         { kp_r = 0.0f; }
    if(kp_r > SPEED_KP_MAX) { kp_r = SPEED_KP_MAX; }
    if(ki_r < 0.0f)         { ki_r = 0.0f; }
    if(ki_r > SPEED_KI_MAX) { ki_r = SPEED_KI_MAX; }

    sl_kp_l = kp_l;
    sl_ki_l = ki_l;
    sl_kp_r = kp_r;
    sl_ki_r = ki_r;
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
    int   d_l;
    int   d_r;

    /* ---------- 1. 实测速度（绝对值）---------- */
    sl_speed_l = (enc_l >= 0) ? enc_l : -enc_l;
    sl_speed_r = (enc_r >= 0) ? enc_r : -enc_r;

    /* ---------- 2. 误差 ---------- */
    e_l = (float)sl_target_l - (float)sl_speed_l;
    e_r = (float)sl_target_r - (float)sl_speed_r;

    /* ---------- 3. 增量式 PI（左右各一套，参数也可以不同）---------- */
    sl_corr_l += sl_kp_l * (e_l - sl_e_last_l) + sl_ki_l * e_l;
    sl_corr_r += sl_kp_r * (e_r - sl_e_last_r) + sl_ki_r * e_r;

    sl_e_last_l = e_l;
    sl_e_last_r = e_r;

    /* ---------- 4. 修正量限幅 ---------- */
    if(sl_corr_l >  SPEED_CORR_MAX) { sl_corr_l =  SPEED_CORR_MAX; }
    if(sl_corr_l < -SPEED_CORR_MAX) { sl_corr_l = -SPEED_CORR_MAX; }
    if(sl_corr_r >  SPEED_CORR_MAX) { sl_corr_r =  SPEED_CORR_MAX; }
    if(sl_corr_r < -SPEED_CORR_MAX) { sl_corr_r = -SPEED_CORR_MAX; }

    /* ---------- 5. 前馈 + PI 修正，再限幅（只正转）---------- */
    d_l = (int)(SPEED_FF * (float)sl_target_l + sl_corr_l);
    d_r = (int)(SPEED_FF * (float)sl_target_r + sl_corr_r);

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
int SpeedLoop_GetTarget(void)  { return sl_target_l; }
int SpeedLoop_GetTargetL(void) { return sl_target_l; }
int SpeedLoop_GetTargetR(void) { return sl_target_r; }
int SpeedLoop_GetSpeedL(void) { return sl_speed_l; }
int SpeedLoop_GetSpeedR(void) { return sl_speed_r; }
int SpeedLoop_GetDutyL(void)  { return sl_duty_l;  }
int SpeedLoop_GetDutyR(void)  { return sl_duty_r;  }

float SpeedLoop_GetKp(void)  { return sl_kp_l; }
float SpeedLoop_GetKi(void)  { return sl_ki_l; }
float SpeedLoop_GetKpL(void) { return sl_kp_l; }
float SpeedLoop_GetKiL(void) { return sl_ki_l; }
float SpeedLoop_GetKpR(void) { return sl_kp_r; }
float SpeedLoop_GetKiR(void) { return sl_ki_r; }
