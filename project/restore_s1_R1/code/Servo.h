/*********************************************************************************************************************
* 文件名称          Servo
* 功能简介          舵机模块（转向舵机）
*
* 用法：
*   1. main() 里调用一次  Servo_Init();            舵机初始化并回中
*   2. 用 Servo_Steer(角度) 设置打角，角度以中值为 0，正数往左、负数往右
*   3. 打角控制建议放在定时中断里（比如 20ms 一次），保证控制周期稳定
*
* 重要说明：
*   PWM 波形是硬件自动产生的，调用 pwm_set_duty 写一次就一直有效，
*   不需要在中断里反复输出。定时中断的作用是「每隔固定时间算一次新的打角」，
*   而不是「产生 PWM」。把打角放中断里，是为了让控制周期不受图像处理/USB 发送影响。
*
* 关于函数名：
*   学长原来的 PWM_Servo.c 里也有一个 Servo_SetAngle()，为了不和它冲突，
*   本模块的打角函数命名为 Servo_Steer()。
********************************************************************************************************************/

#ifndef __SERVO_H
#define __SERVO_H

#include "zf_common_headfile.h"

//===================================================================================================================
// 舵机标定值 —— 这三个值需要在你自己的车上标定
// 以下中位与左右端点均为本次结构调整后的实测值。
//
// 数值换算：PWM 满量程 10000 对应 20ms（50Hz），所以
//          数值 790 = 790/10000 × 20ms = 1.580ms 脉宽
//          数值每 +1 = 脉宽 +2 微秒
//===================================================================================================================
#define SERVO_SETUP_RUN         (0)
#define SERVO_SETUP_CENTER      (1)
#define SERVO_SETUP_MANUAL      (2)
#define SERVO_SETUP_AUTO        (3)
#define SERVO_SETUP_TRIM        (4)                     // 只在旧中位附近微调直行零位
#define SERVO_SETUP_MODE        (SERVO_SETUP_RUN)       // 正常循迹控制
#define SERVO_CENTER_TRIM_RANGE (30)                    // 中位附近最多 +/-30，不扫左右端点
#define SERVO_TEST_USE_WIRELESS (0)                     // 当前无线模块不可用，纯主板按键标定
#define SERVO_DUTY_MIDDLE       (790)                   // 结构调整后确认的直行中位（1580us）
#define SERVO_DUTY_RIGHT        (715)                   // 结构调整后重新实测的右端点
#define SERVO_DUTY_LEFT         (870)                   // 结构调整后重新实测的左端点
#define SERVO_TEST_DUTY_MIN     (460)                   // 信号下界 920us；不代表装车机械限位
#define SERVO_TEST_DUTY_MAX     (1050)                  // 信号上界 2100us；不允许一次跳到端点
#define SERVO_TEST_STEP_MAX     (5)                     // 每条命令最多离当前输出 5 个单位；回中命令例外
#define SERVO_AUTO_START_AMP    (40)                    // 初始测试中位 +/-40，非已确认机械限位
#define SERVO_AUTO_MAX_AMP      (150)                   // 仅现场点按逐级增加，绝不自动扩大
#define SERVO_AUTO_AMP_STEP     (10)
#define SERVO_AUTO_START_TICKS  (150)                   // 上电回中等待 3s（每 tick 20ms）
#define SERVO_AUTO_HOLD_TICKS   (50)                    // 每个位置停留约 1s
#define SERVO_AUTO_CYCLES       (3)                     // 三次往返后自动回中暂停

#define SERVO_RUN_HYSTERESIS   (2)                     // 正常循迹忽略 1 单位的相邻输出抖动；回中例外

// 打角范围（相对中值），由上面三个值算出来，不用改
#define SERVO_ANGLE_LEFT_MAX    ((int16)SERVO_DUTY_LEFT  - (int16)SERVO_DUTY_MIDDLE)   // 正数
#define SERVO_ANGLE_RIGHT_MAX   ((int16)SERVO_DUTY_RIGHT - (int16)SERVO_DUTY_MIDDLE)   // 负数

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化（初始化 PWM 并回中）
// 参数说明     void
// 返回参数     void
// 备注信息     调用前必须先执行 clock_init()
//-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接设置舵机 PWM 数值
// 参数说明     duty            舵机占空比数值（建议范围 SERVO_DUTY_RIGHT ~ SERVO_DUTY_LEFT）
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void Servo_SetDuty(uint16 duty);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机打角（相对中值）
// 参数说明     angle           0 = 居中；正数往左，负数往右
//                              超出范围会自动限制到 SERVO_ANGLE_RIGHT_MAX ~ SERVO_ANGLE_LEFT_MAX
// 返回参数     void
// 使用示例     Servo_Steer(0);      // 回中
//              Servo_Steer(30);     // 往左打 30 个数
//              Servo_Steer(-20);    // 往右打 20 个数
//-------------------------------------------------------------------------------------------------------------------
void Servo_Steer(int angle);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前的打角值
// 参数说明     void
// 返回参数     int16           当前打角（相对中值）
//-------------------------------------------------------------------------------------------------------------------
int16 Servo_GetAngle(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前的 PWM 数值
// 参数说明     void
// 返回参数     uint16          当前舵机占空比数值
//-------------------------------------------------------------------------------------------------------------------
uint16 Servo_GetDuty(void);

// 手动标定专用：范围 460~1050，但每次只能从当前输出移动最多 5；中值可直接回中。
// 非法值、跨步命令及 NaN/Inf 拒绝执行；每 20ms 调用一次 Step，最多移动 1。
uint8  Servo_CalibrationSetTarget(float requested);
void   Servo_CalibrationStep(void);
uint16 Servo_GetCalibrationTarget(void);

// 自动测试：每 20ms 调用 Step；K1/K2 调幅，K3 暂停回中，K4 重新开始。
void   Servo_AutoTestStep(void);
void   Servo_AutoTestAdjust(int delta);
void   Servo_AutoTestPause(uint8 paused);
uint16 Servo_AutoTestAmplitude(void);
uint8  Servo_AutoTestPaused(void);
uint8  Servo_AutoTestCycles(void);

#endif
