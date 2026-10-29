/*********************************************************************************************************************
* 文件名称          Servo
* 功能简介          舵机模块（转向舵机）
********************************************************************************************************************/

#include "Servo.h"

#define SERVO_PWM_PIN           (PWM4_MODULE2_CHA_C30)  // 舵机信号线接在 C30
#define SERVO_PWM_FREQ          (50)                    // 安装回中使用 50Hz，20ms 周期

/* 当前 PWM 数值。
   ★ 初值故意写成 0：和 Servo_Init() 里 pwm_init(..., 0) 的初始占空比一致，
     这样第一次 Servo_Steer(0) 一定会真正写下去（见下面 Servo_Steer 里的"只在变化时写"） */
static uint16 servo_duty  = 0;                          // 上一次真正写进 PWM 的数值
static int16  servo_angle = 0;                          // 当前打角（相对中值）
static uint16 calibration_target = SERVO_DUTY_MIDDLE;
static uint16 auto_amplitude = SERVO_AUTO_START_AMP;
static uint16 auto_wait = SERVO_AUTO_START_TICKS;
static uint8 auto_phase = 0;
static uint8 auto_paused = 0;
static uint8 auto_cycles = 0;

#if SERVO_SETUP_MODE == SERVO_SETUP_AUTO
static void auto_restart(uint8 paused)
{
    calibration_target = SERVO_DUTY_MIDDLE;
    auto_phase = 0;
    auto_cycles = 0;
    auto_wait = SERVO_AUTO_START_TICKS;
    auto_paused = paused;
    Servo_SetDuty(SERVO_DUTY_MIDDLE);
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
//-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void)
{
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, 0);         // 初始化 PWM，占空比先给 0
    calibration_target = SERVO_DUTY_MIDDLE;
#if SERVO_SETUP_MODE == SERVO_SETUP_AUTO
    auto_amplitude = SERVO_AUTO_START_AMP;
    auto_restart(0);
#else
    Servo_SetDuty(SERVO_DUTY_MIDDLE);                    // 初始化后必须真正写入回中脉宽
#endif

    system_delay_ms(500);                               // 等舵机转到位
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接设置舵机 PWM 数值
//-------------------------------------------------------------------------------------------------------------------
void Servo_SetDuty(uint16 duty)
{
#if SERVO_SETUP_MODE == SERVO_SETUP_CENTER
    duty = SERVO_DUTY_MIDDLE;                           // 安装模式下，直接写 PWM 的调用也只能回中
#elif SERVO_SETUP_MODE == SERVO_SETUP_TRIM
    if(duty < SERVO_DUTY_MIDDLE - SERVO_CENTER_TRIM_RANGE)
    {
        duty = SERVO_DUTY_MIDDLE - SERVO_CENTER_TRIM_RANGE;
    }
    if(duty > SERVO_DUTY_MIDDLE + SERVO_CENTER_TRIM_RANGE)
    {
        duty = SERVO_DUTY_MIDDLE + SERVO_CENTER_TRIM_RANGE;
    }
    if(duty < SERVO_DUTY_RIGHT) { duty = SERVO_DUTY_RIGHT; }
    if(duty > SERVO_DUTY_LEFT) { duty = SERVO_DUTY_LEFT; }
#elif SERVO_SETUP_MODE == SERVO_SETUP_MANUAL
    if((duty < SERVO_TEST_DUTY_MIN) || (duty > SERVO_TEST_DUTY_MAX))
    {
        duty = SERVO_DUTY_MIDDLE;
    }
#elif SERVO_SETUP_MODE == SERVO_SETUP_AUTO
    if((duty < SERVO_DUTY_MIDDLE - auto_amplitude) ||
       (duty > SERVO_DUTY_MIDDLE + auto_amplitude))
    {
        duty = SERVO_DUTY_MIDDLE;
    }
#endif
    servo_duty  = duty;
    servo_angle = (int16)duty - (int16)SERVO_DUTY_MIDDLE;

    pwm_set_duty(SERVO_PWM_PIN, duty);                  // 写一次即可，PWM 硬件会一直输出这个脉冲
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机打角（会自动限幅）
//-------------------------------------------------------------------------------------------------------------------
void Servo_Steer(int angle)
{
    uint16 duty;

    if(angle > SERVO_ANGLE_LEFT_MAX)
    {
        angle = SERVO_ANGLE_LEFT_MAX;
    }
    if(angle < SERVO_ANGLE_RIGHT_MAX)
    {
        angle = SERVO_ANGLE_RIGHT_MAX;
    }

    duty = (uint16)((int16)SERVO_DUTY_MIDDLE + angle);

    // 正常循迹时抑制相邻输出抖动；保持原有限位和精确回中行为。
#if SERVO_SETUP_MODE == SERVO_SETUP_RUN
    {
        int delta = (int)duty - (int)servo_duty;
        if((angle != 0) && (angle != SERVO_ANGLE_LEFT_MAX) &&
           (angle != SERVO_ANGLE_RIGHT_MAX) &&
           (delta > -SERVO_RUN_HYSTERESIS) && (delta < SERVO_RUN_HYSTERESIS))
        {
            return;                                     // 仅过滤细小 PWM 跳动，大转向仍可立即执行
        }
    }
#endif
    if(duty == servo_duty)
    {
        return;                                             // 和上次写下去的一样：什么都不做
    }

    Servo_SetDuty(duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前打角
//-------------------------------------------------------------------------------------------------------------------
int16 Servo_GetAngle(void)
{
    return servo_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前 PWM 数值
//-------------------------------------------------------------------------------------------------------------------
uint16 Servo_GetDuty(void)
{
    return servo_duty;
}

uint8 Servo_CalibrationSetTarget(float requested)
{
#if SERVO_SETUP_MODE == SERVO_SETUP_MANUAL
    // 用正向范围比较，避免 NaN 绕过检查后被转换成整数。
    if(!((requested >= SERVO_TEST_DUTY_MIN) && (requested <= SERVO_TEST_DUTY_MAX)))
    {
        return 0;
    }
    if(requested != (float)(uint16)requested) { return 0; }
    if((requested != SERVO_DUTY_MIDDLE) &&
       ((requested > (float)servo_duty + SERVO_TEST_STEP_MAX) ||
        (requested < (float)servo_duty - SERVO_TEST_STEP_MAX)))
    {
        return 0;                                     // 不允许从中位直接命令全行程，避免机构顶死
    }
    calibration_target = (uint16)requested;
    if(calibration_target == SERVO_DUTY_MIDDLE)
    {
        Servo_SetDuty(SERVO_DUTY_MIDDLE);
    }
    return 1;
#else
    (void)requested;
    return 0;
#endif
}

void Servo_CalibrationStep(void)
{
#if SERVO_SETUP_MODE == SERVO_SETUP_MANUAL
    if(servo_duty < calibration_target) { Servo_SetDuty(servo_duty + 1); }
    else if(servo_duty > calibration_target) { Servo_SetDuty(servo_duty - 1); }
#endif
}

uint16 Servo_GetCalibrationTarget(void)
{
    return calibration_target;
}

void Servo_AutoTestStep(void)
{
#if SERVO_SETUP_MODE == SERVO_SETUP_AUTO
    if(auto_paused) { return; }
    if(servo_duty < calibration_target) { Servo_SetDuty(servo_duty + 1); return; }
    if(servo_duty > calibration_target) { Servo_SetDuty(servo_duty - 1); return; }
    if(auto_wait > 0) { auto_wait--; return; }

    auto_wait = SERVO_AUTO_HOLD_TICKS;
    switch(auto_phase)
    {
        case 0:
            if(auto_cycles >= SERVO_AUTO_CYCLES) { auto_paused = 1; break; }
            calibration_target = SERVO_DUTY_MIDDLE + auto_amplitude;
            auto_phase = 1;
            break;
        case 1:
            calibration_target = SERVO_DUTY_MIDDLE;
            auto_phase = 2;
            break;
        case 2:
            calibration_target = SERVO_DUTY_MIDDLE - auto_amplitude;
            auto_phase = 3;
            break;
        default:
            calibration_target = SERVO_DUTY_MIDDLE;
            auto_phase = 0;
            auto_cycles++;
            break;
    }
#endif
}

void Servo_AutoTestAdjust(int delta)
{
#if SERVO_SETUP_MODE == SERVO_SETUP_AUTO
    int next;
    // 一次最多变动 10；即使误传很大的数，也不能一次放开全部行程。
    if(delta > SERVO_AUTO_AMP_STEP) { delta = SERVO_AUTO_AMP_STEP; }
    if(delta < -SERVO_AUTO_AMP_STEP) { delta = -SERVO_AUTO_AMP_STEP; }
    next = (int)auto_amplitude + delta;
    if(next < SERVO_AUTO_AMP_STEP) { next = SERVO_AUTO_AMP_STEP; }
    if(next > SERVO_AUTO_MAX_AMP) { next = SERVO_AUTO_MAX_AMP; }
    auto_amplitude = (uint16)next;
    auto_restart(0);                                      // 改幅度先回中等 3s，再开始下一轮
#else
    (void)delta;
#endif
}

void Servo_AutoTestPause(uint8 paused)
{
#if SERVO_SETUP_MODE == SERVO_SETUP_AUTO
    auto_restart(paused ? 1 : 0);
#else
    (void)paused;
#endif
}

uint16 Servo_AutoTestAmplitude(void) { return auto_amplitude; }
uint8 Servo_AutoTestPaused(void) { return auto_paused; }
uint8 Servo_AutoTestCycles(void) { return auto_cycles; }
