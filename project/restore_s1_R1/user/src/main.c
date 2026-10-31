/*********************************************************************************************************************
* 文件名称          main.c
* 功能简介          弯道、十字与纯图像左右环岛循迹；屏幕显示BIN二值图和边线
*
* 硬件              RT1064 + 逐飞主板 + IPS200 SPI屏；无需无线模块
* 上电停车，C13单独启停；C12循环选择SPD/P/D；C15减、C14加。
* SPD默认180、步进5、范围70~240；P/D默认1.80/2.00、步进0.02。
* 二值图+边线；显示左右编码器实际/目标、十字/环岛、丢图/停车原因与舵机PWM。
* C15/C14/C13/C12 对应 KEY1/KEY2/KEY3/KEY4，按低电平有效，30ms消抖。
* 编码器：左C0/C1，右C2/C24；舵机C30，屏幕SPI引脚保持原定义。
* 20ms速度闭环+舵机PD，10ms编码器/按键；有效图像超时500ms停车，需重新按C13。
********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "Motor.h"
#include "Servo.h"
#include "Screen.h"
#include "GYRO.h"
#include "zf_device_imu660ra.h"
#include "MotorPID.h"
#include "MotorTune.h"
#include "Assistant.h"
#include "Contrast.h"
#include "LineDetect.h"
#include "Cross.h"
#include "Roundabout.h"
#include "Err.h"
#include "TrackRemote.h"


//===================================================================================================================
//  一、编译期配置
//===================================================================================================================

#define CAM_EXPOSURE        (512)      // 摄像头曝光：发白/反光调小(400/350)，远处太暗调大(600+)

// 舵机PD：按键选择P/D再调整
#define SERVO_P_DEFAULT     (1.80f)
#define SERVO_P_KEY_STEP    (0.02f)     // C15/C14 调P时每次增减0.02
#define SERVO_P_MIN         (0.0f)
#define SERVO_P_MAX         (5.0f)
#define RUN_KEY_DEBOUNCE_SCANS (3)     // 每 10ms 扫描，连续 3 次稳定才确认
#define SERVO_D_DEFAULT     (2.0f)      // 20ms偏差差分，需实车调参
#define SERVO_D_KEY_STEP    (0.02f)
#define SERVO_D_MIN         (0.0f)
#define SERVO_D_MAX         (50.0f)

// 电机速度环保留原PID参数；SPD可按键调整
#define SPEED_DEFAULT       (180.0f)    // 默认基础目标180；按键调速，弯道/十字仍自动降速
#define SPEED_KEY_STEP      (5.0f)     // SPD按键每次加减5
#define SPEED_KEY_MIN       (70.0f)
#define SPEED_KEY_MAX       (240.0f)
#define SPEED_KEY_LONG_SCANS (100)     // 仅架空测试：每10ms扫描，长按约1秒
#define SPEED_KP_DEFAULT    (MOTORPID_DEFAULT_KP)
#define SPEED_KI_DEFAULT    (MOTORPID_DEFAULT_KI)
#define SPEED_KD_DEFAULT    (MOTORPID_DEFAULT_KD)

#define RUN_AUTO_START      (0)        // 上电停车，C13发车
#define MOTOR_DIAGNOSTIC_MODE (0)      // 0循迹；1单轮；2差速验证；3核心板Type-C电机PID调参

#define DIFF_TEST_TARGET       (100.0f)  // 验证版上电速度，可停车按 C12 调整
#define DIFF_TEST_SPEED_MIN    (70.0f)
#define DIFF_TEST_SPEED_MAX    (150.0f)  // 60%差速时外轮最多240
#define DIFF_TEST_PHASE_TICKS  (200)   // 每段 4 秒（20ms）
#define DIFF_TEST_SETTLE_TICKS (100)   // 前 2 秒不计入均值
#define DIFF_TEST_STALL_START  (75)    // 启动 1.5 秒后检查编码器
#define DIFF_TEST_STALL_TICKS  (25)    // 一侧连续低于 3 计数达 0.5 秒就停

#define USE_SCREEN          (1)        // 逐飞主板IPS200显示二值图、参数和元素状态
#define USE_RUN_KEYS        (1)        // 逐飞主板 C15/C14/C13/C12
#define USE_REMOTE_CONTROL  (0)        // 不使用无线VOFA
#define USE_ASSISTANT       (0)        // 无线模块完全停用，不初始化/接收/发送
#define USE_ISLAND          (1)        // 新版纯摄像头左右环岛；不调用旧陀螺仪环岛状态机
#define USE_LEGACY_GYRO     (0)        // 当前纯图像环岛不需要IMU；不误初始化IMU660RA驱动
#define ISLAND_SLOW_RATIO   (0.6f)     // 进环岛后目标速度降到 60%
#define CROSS_SLOW_RATIO    (0.7f)     // 过十字时目标速度降到 70%
#define TURN_SLOW_MEDIUM    (0.85f)    // 中等转角时目标速度降到 85%
#define TURN_SLOW_SHARP     (0.70f)    // 大转角时目标速度降到 70%
#define TURN_MEDIUM_LIMIT   (30.0f)    // 舵机输出超过此值开始弯道降速
#define TURN_SHARP_LIMIT    (60.0f)    // 舵机输出超过此值按急弯降速
#define TURN_SPEED_MEDIUM_MAX (80.0f)  // 扩大直道调速范围后，弯道速度不能只按比例放大
#define TURN_SPEED_SHARP_MAX  (70.0f)
// 由连续摄像头帧的图像偏差决定是否辅助；不把照片位置或弯道前后半作为硬编码。
#define ERROR_DIFF_ON         (12.0f)  // 两种偏差同向达到12像素，明显弯道及时确认
#define ERROR_DIFF_OFF        (8.0f)   // 回到8像素以内退出，保留迟滞
#define ERROR_DIFF_FULL       (24.0f)  // 24像素达到最大60%差速
#define ERROR_DIFF_MIN_LEVEL  (0.40f)  // 介入后至少请求满幅的40%，避免几乎看不出的微小差速
#define ERROR_DIFF_CONFIRM    (2)      // 连续2个新摄像头帧，不能用定时器重复计旧帧
#define ERROR_DIFF_MIN_ROWS   (20)
#define ERROR_DIFF_MAX_AGE     (8)      // 20ms控制周期；约160ms无新图像则撤销
#define DIFF_STEER_DEADBAND   (10)     // 小打角不做差速，减少直道/S 弯附加摆动
#define TURN_PREVIEW_START    (0.25f)  // 连续确认的前方弯道开始触发减速
#define TURN_PREVIEW_FULL     (1.50f)  // 预警强度到此值时应用完整入弯速度上限
#define TURN_ENTRY_SPEED_MAX  (100.0f)
#define TURN_RELEASE_HOLD_TICKS (10)  // 20ms 一拍，出弯保持低目标 200ms
#define TURN_RELEASE_STEP    (2.0f)   // 其后每拍最多增加 2，即每秒 100 计数/10ms
#define STEER_LIMIT_HOLD_TICKS (10)    // 每拍 20ms，连续顶限位 200ms 才记录

// 舵机 PD 输出限幅（偏差再大也不会一把打死）
#define SERVO_U_LIMIT       (120.0f)

// 转向死区：|偏差| 小于这个值时直接当 0，防止直道被 ±1 像素噪声推着左右摆
#define STEER_DEADBAND      (1.5f)

// 小偏差不额外补偿；明显偏离中近处赛道后，平滑增加大弯转向。
#define STEER_BOOST_START   (12.0f)
#define STEER_BOOST_K       (0.10f)


//===================================================================================================================
//  二、学长 Camera.c / island.c 里的全局量（本程序只读）
//===================================================================================================================

extern uint8          image_two_value[MT9V03X_H][MT9V03X_W];   // 二值图
extern volatile int   Left_Line[MT9V03X_H];
extern volatile int   Right_Line[MT9V03X_H];
extern int            Left_Lost_Flag[MT9V03X_H];
extern int            Right_Lost_Flag[MT9V03X_H];
extern volatile int   Search_Stop_Line;
extern volatile int   Left_Lost_Time;
extern volatile int   Right_Lost_Time;
extern volatile uint8  Img_Disappear_Flag;
extern volatile int   Island_State;


//===================================================================================================================
//  三、全局状态
//===================================================================================================================

// 可在线调参数（主循环从助手写进来，中断里读）
static volatile float run_speed    = SPEED_DEFAULT; // 按键调基础速度，默认180
static volatile float run_kp       = SPEED_KP_DEFAULT;
static volatile float run_ki       = SPEED_KI_DEFAULT;
static volatile float run_kd       = SPEED_KD_DEFAULT;
static volatile float run_p        = SERVO_P_DEFAULT;
static volatile float run_d        = SERVO_D_DEFAULT;
#if (LINE_SCHEME == LINE_MODE_OTSU)
static volatile float run_threshold = OTSU_BIAS_DEFAULT;       // 大津法：默认偏移量 15
#else
static volatile float run_threshold = CONTRAST_RATIO_DEFAULT;  // 对比度：默认阈值 50
#endif
static volatile float run_switch   = 0.0f;              // 首个有效图像之前不启动执行机构
static volatile float run_enable   = 0.0f;                 // 初始化完成后置 1，允许跑

static volatile int16 enc_l = 0;                           // 左轮编码器（计数/10ms）
static volatile int16 enc_r = 0;                           // 右轮编码器（计数/10ms）
static volatile int   dbg_duty = 0;                        // 实际输出占空比
static uint16         hb_cnt = 0;                          // 心跳灯计数

// 冻结一份已完成的灰度帧，识别、屏幕和无线发送都使用同一份数据。
static uint8          frame_gray[MT9V03X_H][MT9V03X_W];

static uint8          line_left[MT9V03X_H];                // 屏幕画线用
static uint8          line_mid[MT9V03X_H];
static uint8          line_right[MT9V03X_H];
static uint8          asst_left[MT9V03X_H];                // 助手边线叠加用（丢线时保持上一个值）
static uint8          asst_right[MT9V03X_H];

static float          err_last = 0.0f;                     // 上一拍偏差（图像丢失时保持）
static float          steer_prev_error = 0.0f;
static float          steer_d_filter = 0.0f;
static uint8          steer_pd_init = 0;

// BEGIN RUN AUTO START
static volatile uint8 run_auto_pending = RUN_AUTO_START;

static void Run_AutoStartTry(uint8 image_usable)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    if(run_auto_pending && run_enable > 0.0f && image_usable)
    {
        run_auto_pending = 0;
        run_switch = 1.0f;
#if USE_REMOTE_CONTROL
        TrackRemote_Stop(REMOTE_REASON_READY);
#endif
    }
    __set_PRIMASK(mask);
}
// END RUN AUTO START

#if USE_REMOTE_CONTROL
// BEGIN WIRELESS TRACK CONTROL
static void Run_RemoteStop(uint8 reason)
{
    run_auto_pending=0; // 自动起步只执行一次；手动停车或故障后不自行重启
    run_switch=0.0f;MotorPID_SetEnable(0);Servo_Steer(0);dbg_duty=0;
    TrackRemote_Stop(reason);
}
static uint8 Run_RemoteCommand(TrackRemote_Command command,const float *v)
{
    uint8 accepted=1;
    uint32 mask=__get_PRIMASK();__disable_irq();
    switch(command)
    {
        case REMOTE_STOP: Run_RemoteStop(REMOTE_REASON_MANUAL);break;
        case REMOTE_RX_FAULT: Run_RemoteStop(REMOTE_REASON_RX);break;
        case REMOTE_RUN:
            if(run_enable<=0.0f || !TrackRemote_Arm()) { accepted=0;break; }
#if USE_ISLAND
            if(Roundabout_GetFault()) { Roundabout_Retry(); }
#endif
            run_auto_pending=0;run_switch=1.0f;break;
        case REMOTE_P: run_p=v[0];break;
        case REMOTE_D: run_d=v[0];break;
        case REMOTE_MOTOR_P: case REMOTE_MOTOR_I: case REMOTE_MOTOR_D:
            if(command==REMOTE_MOTOR_P) { run_kp=v[0]; }
            else if(command==REMOTE_MOTOR_I) { run_ki=v[0]; }
            else { run_kd=v[0]; }
            MotorPID_SetGains(run_kp,run_ki,run_kd);break;
        case REMOTE_SPEED: run_speed=v[0];break;
        default: accepted=0;break;
    }
    __set_PRIMASK(mask);return accepted;
}
static void Run_RemoteService(void)
{
    float wave[TRACK_REMOTE_CHANNELS];uint32 mask;
#if USE_SCREEN
    int current_run,current_waiting;
#endif
    TrackRemote_Poll(Run_RemoteCommand);
    mask=__get_PRIMASK();__disable_irq();
    wave[0]=ErrUse;wave[1]=(float)enc_l;wave[2]=(float)enc_r;
    wave[3]=run_speed;
#if USE_SCREEN
    current_run=(run_switch>0.5f);current_waiting=(int)run_auto_pending;
#endif
    __set_PRIMASK(mask);
    TrackRemote_Send(wave);
#if USE_SCREEN
    {
        static int shown_run=-1,shown_waiting=-1;
        /* 摄像头不再更新时，仍把异步停车状态写到屏幕。 */
        if(shown_run!=current_run || shown_waiting!=current_waiting)
        {
            Screen_ShowRunState((uint8)current_run,(uint8)current_waiting);
            shown_run=current_run;shown_waiting=current_waiting;
        }
    }
#endif
}
// END WIRELESS TRACK CONTROL
#endif


//===================================================================================================================
//  四、小工具
//===================================================================================================================

// BEGIN RUN KEY CONTROL
static const gpio_pin_enum run_key_pins[4] = {C15, C14, C13, C12};
static uint8 run_key_stable[4];
static uint8 run_key_count[4];
static volatile uint8 run_tune_item; // 0 SPD, 1 P, 2 D
#define RUN_IMAGE_TIMEOUT_TICKS (25) // 500ms at 20ms
static volatile uint8 run_key_image_age = RUN_IMAGE_TIMEOUT_TICKS;
static volatile uint8 run_key_fault; // 0 none/manual, 1 no valid image, 2 roundabout fault

static void Run_KeysInit(void)
{
    uint8 i;
    key_init(10);
    run_tune_item = 0;
    run_key_image_age = RUN_IMAGE_TIMEOUT_TICKS;
    run_key_fault = 0;
    for(i = 0; i < 4; i++)
    {
        // Held at power-on: release before accepting a new press.
        run_key_stable[i] = (GPIO_LOW == gpio_get_level(run_key_pins[i]));
        run_key_count[i] = 0;
    }
}

static float Run_ClampGain(float value, float minimum, float maximum)
{
    if(!(value >= minimum)) { value = minimum; }
    if(value > maximum) { value = maximum; }
    return value;
}

static void Run_AdjustGain(int direction)
{
    volatile float *gain;
    float minimum, maximum, step;
    int cents;
    if(run_tune_item == 0)
    {
        run_speed = Run_ClampGain(run_speed + direction*SPEED_KEY_STEP,
                                 SPEED_KEY_MIN, SPEED_KEY_MAX);
        return;
    }
    gain = (run_tune_item == 2) ? &run_d : &run_p;
    minimum = (run_tune_item == 2) ? SERVO_D_MIN : SERVO_P_MIN;
    maximum = (run_tune_item == 2) ? SERVO_D_MAX : SERVO_P_MAX;
    step = (run_tune_item == 2) ? SERVO_D_KEY_STEP : SERVO_P_KEY_STEP;
    cents = (int)(Run_ClampGain(*gain, minimum, maximum)*100.0f+0.5f);
    cents += direction*(int)(step*100.0f+0.5f);
    *gain = Run_ClampGain((float)cents/100.0f, minimum, maximum);
}

static void Run_KeyStop(uint8 fault)
{
    run_switch = 0.0f;
    run_auto_pending = 0;
    run_key_fault = fault;
    MotorPID_SetEnable(0);
    Servo_Steer(0);
    dbg_duty = 0;
}

// Independent of wireless: camera protection remains active with the module removed.
static void Run_KeyTick20ms(void)
{
    if(run_key_image_age < RUN_IMAGE_TIMEOUT_TICKS) { run_key_image_age++; }
    if(run_switch > 0.5f && run_key_image_age >= RUN_IMAGE_TIMEOUT_TICKS)
    {
        Run_KeyStop(1);
    }
}

static void Run_KeyFrame(uint8 usable)
{
    // A byte store is atomic; invalid frames never refresh the valid-image watchdog.
    if(usable) { run_key_image_age = 0; }
}

static void Run_Keys10ms(void)
{
    uint8 i;
    uint8 pressed_events = 0;
    for(i = 0; i < 4; i++)
    {
        uint8 pressed = (GPIO_LOW == gpio_get_level(run_key_pins[i]));
        if(pressed == run_key_stable[i]) { run_key_count[i] = 0; }
        else if(++run_key_count[i] >= RUN_KEY_DEBOUNCE_SCANS)
        {
            run_key_count[i] = 0;
            run_key_stable[i] = pressed;
            if(pressed) { pressed_events |= (uint8)(1u << i); }
        }
    }
    // C13 has priority over selection and adjustment. No long-press repeat.
    if(pressed_events & 4u)
    {
        if(run_switch > 0.5f) { Run_KeyStop(0); }
        else if(run_enable > 0.0f && run_key_image_age < RUN_IMAGE_TIMEOUT_TICKS)
        {
#if USE_ISLAND
            if(Roundabout_GetFault()) { Roundabout_Retry(); }
#endif
            run_auto_pending = 0;
            run_key_fault = 0;
            run_switch = 1.0f;
        }
        else { Run_KeyStop(1); }
        return;
    }
    if(pressed_events & 8u)
    {
        run_tune_item = (uint8)((run_tune_item + 1u) % 3u);
        return;
    }
    if(((pressed_events & 3u) == 1u) && !run_key_stable[1]) { Run_AdjustGain(-1); }
    else if(((pressed_events & 3u) == 2u) && !run_key_stable[0]) { Run_AdjustGain(1); }
}

static void Run_FormatP(float value, char text[5])
{
    int scaled = (int)(Run_ClampGain(value,SERVO_P_MIN,SERVO_P_MAX)*100.0f+0.5f);
    text[0] = (char)('0'+scaled/100);
    text[1] = '.';
    text[2] = (char)('0'+(scaled/10)%10);
    text[3] = (char)('0'+scaled%10);
    text[4] = 0;
}

static void Run_FormatD(float value, char text[6])
{
    int scaled = (int)(Run_ClampGain(value,SERVO_D_MIN,SERVO_D_MAX)*100.0f+0.5f);
    text[0] = scaled >= 1000 ? (char)('0'+scaled/1000) : ' ';
    text[1] = (char)('0'+(scaled/100)%10);
    text[2] = '.';
    text[3] = (char)('0'+(scaled/10)%10);
    text[4] = (char)('0'+scaled%10);
    text[5] = 0;
}
// END RUN KEY CONTROL

#if USE_RUN_KEYS && USE_SCREEN
static void Run_KeyDisplayService(void)
{
    static int last_run = -1, last_lost = -1, last_fault = -1;
    int running = (run_enable > 0.0f && run_switch > 0.5f);
    int lost = (Img_Disappear_Flag || run_key_image_age >= RUN_IMAGE_TIMEOUT_TICKS);
    int fault = run_key_fault;
    if(running != last_run || lost != last_lost || fault != last_fault)
    {
        Screen_ShowRunState((uint8)running, 0);
        Screen_ShowHealth((uint8)lost, (uint8)fault);
        last_run=running; last_lost=lost; last_fault=fault;
    }
}
#endif


// BEGIN STEERING BOOST
static float Steering_Boost(float error)
{
    float magnitude = (error >= 0.0f) ? error : -error;
    float excess = magnitude - STEER_BOOST_START;
    float boost;
    if(excess <= 0.0f) { return 0.0f; }
    boost = STEER_BOOST_K * magnitude * excess;
    return (error >= 0.0f) ? boost : -boost;
}
// END STEERING BOOST

// BEGIN RUN CURVE CONTROL
// 仅在 20ms 控制中断中写，主循环读；停车保留，上一次发车记录在下次发车时清除。
static volatile int dbg_peak_left = SERVO_DUTY_MIDDLE;
static volatile int dbg_peak_right = SERVO_DUTY_MIDDLE;
static volatile uint8 dbg_limit_flags;
static volatile uint8 dbg_image_lost;
static volatile float dbg_diff_peak;
static uint8 limit_left_ticks;
static uint8 limit_right_ticks;

static float curve_release_target;
static uint8 curve_release_ticks;
static uint8 curve_speed_valid;

static float Run_CurveTarget(float target, float u, float preview)
{
    float magnitude = (u >= 0.0f) ? u : -u;
    float ahead = (preview >= 0.0f) ? preview : -preview;
    float entry_cap = target;
    if((ahead > TURN_PREVIEW_START) && (target > TURN_ENTRY_SPEED_MAX))
    {
        float strength = (ahead - TURN_PREVIEW_START) /
                         (TURN_PREVIEW_FULL - TURN_PREVIEW_START);
        if(strength > 1.0f) { strength = 1.0f; }
        entry_cap = target + strength * (TURN_ENTRY_SPEED_MAX - target);
    }
    if(magnitude > TURN_SHARP_LIMIT)
    {
        target *= TURN_SLOW_SHARP;
        if(target > TURN_SPEED_SHARP_MAX) { target = TURN_SPEED_SHARP_MAX; }
    }
    else if(magnitude > TURN_MEDIUM_LIMIT)
    {
        target *= TURN_SLOW_MEDIUM;
        if(target > TURN_SPEED_MEDIUM_MAX) { target = TURN_SPEED_MEDIUM_MAX; }
    }
    if(target > entry_cap) { target = entry_cap; }
    // 运行减速不低于70；零目标继续作为停车命令传递。
    if(target > 0.0f && target < MOTORPID_MIN_RUN_TARGET) { target = MOTORPID_MIN_RUN_TARGET; }
    return target;
}

static float Run_DifferentialTurn(int angle)
{
    float turn = 0.0f;
    if(angle > DIFF_STEER_DEADBAND)
    {
        turn = (float)(angle - DIFF_STEER_DEADBAND) /
               (float)(SERVO_ANGLE_LEFT_MAX - DIFF_STEER_DEADBAND);
    }
    else if(angle < -DIFF_STEER_DEADBAND)
    {
        turn = (float)(angle + DIFF_STEER_DEADBAND) /
               (float)(-SERVO_ANGLE_RIGHT_MAX - DIFF_STEER_DEADBAND);
    }
    if(turn > 1.0f) { turn = 1.0f; }
    if(turn < -1.0f) { turn = -1.0f; }
    return turn;
}

static volatile float error_diff_request;
static volatile uint8 error_diff_age;
static uint8 error_diff_confirm;
static int error_diff_direction;
static uint8 error_diff_active;

static void Run_ErrorDiffReset(void)
{
    error_diff_request = 0.0f;
    error_diff_age = 0;
    error_diff_confirm = 0;
    error_diff_direction = 0;
    error_diff_active = 0;
}

// 只由 Frame_Process 每个新图像调用一次；raw/filtered 同向并连续确认后才辅助。
static void Run_ErrorDiffFrame(float raw, float filtered, int allowed)
{
    float magnitude = (filtered >= 0.0f) ? filtered : -filtered;
    float raw_abs = (raw >= 0.0f) ? raw : -raw;
    int direction = (filtered >= 0.0f) ? 1 : -1;
    error_diff_age = 0;
    if(!allowed || !(magnitude > ERROR_DIFF_OFF) || !(raw_abs > ERROR_DIFF_OFF) ||
       !(raw*filtered > 0.0f))
    {
        Run_ErrorDiffReset();
        return;
    }
    if(direction != error_diff_direction)
    {
        Run_ErrorDiffReset();
        error_diff_direction = direction;
    }
    if(!error_diff_active)
    {
        if(magnitude >= ERROR_DIFF_ON && raw_abs >= ERROR_DIFF_ON)
        {
            if(error_diff_confirm < ERROR_DIFF_CONFIRM) { error_diff_confirm++; }
            if(error_diff_confirm >= ERROR_DIFF_CONFIRM) { error_diff_active=1; }
        }
        else { error_diff_confirm=0; }
    }
    if(error_diff_active)
    {
        float level = ERROR_DIFF_MIN_LEVEL + (1.0f-ERROR_DIFF_MIN_LEVEL) *
                      (magnitude-ERROR_DIFF_OFF)/(ERROR_DIFF_FULL-ERROR_DIFF_OFF);
        if(level > 1.0f) { level=1.0f; }
        error_diff_request = (float)direction*level;
    }
    else { error_diff_request=0.0f; }
}

static float Run_ErrorDiffCommand(int servo_angle)
{
    float actual_turn = Run_DifferentialTurn(servo_angle);
    // PD可能正在快速回正/换向，不能沿用相反方向的旧图像辅助。
    if(actual_turn*error_diff_request <= 0.0f) { return 0.0f; }
    return error_diff_request;
}

static void Run_SpeedReset(void)
{
    curve_release_target = 0.0f;
    curve_release_ticks = 0;
    curve_speed_valid = 0;
}

static float Run_SpeedTarget(float requested, float u, float preview)
{
    float desired = Run_CurveTarget(requested, u, preview);
    if(!curve_speed_valid || (desired <= curve_release_target))
    {
        curve_speed_valid = 1;
        curve_release_target = desired;
        curve_release_ticks = (desired < requested) ? TURN_RELEASE_HOLD_TICKS : 0;
    }
    else if(curve_release_ticks > 0)
    {
        curve_release_ticks--;
    }
    else
    {
        curve_release_target += TURN_RELEASE_STEP;
        if(curve_release_target > desired) { curve_release_target = desired; }
    }
    return curve_release_target;
}

static void Run_MonitorReset(void)
{
    dbg_peak_left = dbg_peak_right = SERVO_DUTY_MIDDLE;
    dbg_limit_flags = dbg_image_lost = 0;
    dbg_diff_peak = 0.0f;
    limit_left_ticks = limit_right_ticks = 0;
}

static void Run_MonitorStep(int duty, uint8 image_lost)
{
    float diff = MotorPID_GetDiffPercent();
    if(diff > dbg_diff_peak) { dbg_diff_peak = diff; }
    if(duty > dbg_peak_left) { dbg_peak_left = duty; }
    if(duty < dbg_peak_right) { dbg_peak_right = duty; }
    if(image_lost) { dbg_image_lost = 1; }
    if(duty >= SERVO_DUTY_LEFT)
    {
        if(limit_left_ticks < STEER_LIMIT_HOLD_TICKS) { limit_left_ticks++; }
        if(limit_left_ticks >= STEER_LIMIT_HOLD_TICKS) { dbg_limit_flags |= 1u; }
    }
    else { limit_left_ticks = 0; }
    if(duty <= SERVO_DUTY_RIGHT)
    {
        if(limit_right_ticks < STEER_LIMIT_HOLD_TICKS) { limit_right_ticks++; }
        if(limit_right_ticks >= STEER_LIMIT_HOLD_TICKS) { dbg_limit_flags |= 2u; }
    }
    else { limit_right_ticks = 0; }
}
// END RUN CURVE CONTROL


// BEGIN DIFFERENTIAL BENCH TEST
#if MOTOR_DIAGNOSTIC_MODE == 2
// 0待机，1同速，2左弯，3同速，4右弯，5完成，6手动停止，7轮速异常停止。
static volatile uint8 diff_test_state;
static volatile uint16 diff_test_ticks;
static uint16 diff_test_stall;
static volatile uint8 diff_fault_side;
static const gpio_pin_enum diff_key_pins[2] = {C13,C12};
static uint8 diff_key_stable[2], diff_key_count[2];
static uint16 diff_speed_hold;
static uint8 diff_speed_active, diff_speed_long;
static volatile float diff_test_speed = DIFF_TEST_TARGET;
static int32 diff_sum_l, diff_sum_r, diff_sum_tl, diff_sum_tr;
static uint16 diff_samples;
static volatile int16 diff_avg_l[4], diff_avg_r[4], diff_avg_tl[4], diff_avg_tr[4];
static volatile uint8 diff_valid[4], diff_pass[4];

static int DiffTest_Active(void) { return diff_test_state >= 1 && diff_test_state <= 4; }

static void DiffTest_ClearWindow(void)
{
    diff_sum_l = diff_sum_r = diff_sum_tl = diff_sum_tr = 0;
    diff_samples = 0;
    diff_test_stall = 0;
}

static void DiffTest_Stop(uint8 state)
{
    MotorPID_SetEnable(0);
    Servo_Steer(0);
    diff_test_state = state;
}

static void DiffTest_Init(void)
{
    uint8 i;
    key_init(10);
    for(i=0;i<2;i++)
    {
        diff_key_stable[i] = (gpio_get_level(diff_key_pins[i]) == GPIO_LOW);
        diff_key_count[i] = 0;
    }
    diff_speed_hold = 0;
    diff_speed_active = diff_speed_long = 0;
    diff_test_speed = DIFF_TEST_TARGET;
    diff_test_state = 0;
    diff_fault_side = 0;
    diff_test_ticks = 0;
    DiffTest_ClearWindow();
    for(i=0;i<4;i++) { diff_valid[i]=diff_pass[i]=0; }
    MotorPID_SetEnable(0);
}

static void DiffTest_AdjustSpeed(int direction)
{
    uint8 i;
    float next = diff_test_speed + (float)direction*SPEED_KEY_STEP;
    if(next < DIFF_TEST_SPEED_MIN) { next = DIFF_TEST_SPEED_MIN; }
    if(next > DIFF_TEST_SPEED_MAX) { next = DIFF_TEST_SPEED_MAX; }
    if(next != diff_test_speed)
    {
        diff_test_speed = (float)((int)(next+0.5f));
        // 新速度要重新比较四个阶段，清除旧结果，电机保持停止。
        for(i=0;i<4;i++) { diff_valid[i]=diff_pass[i]=0; }
        diff_test_state = 0;
        diff_fault_side = 0;
    }
}

static void DiffTest_Keys10ms(void)
{
    uint8 i;
    uint8 events = 0;
    for(i=0;i<2;i++)
    {
        uint8 pressed = (gpio_get_level(diff_key_pins[i]) == GPIO_LOW);
        if(pressed == diff_key_stable[i]) { diff_key_count[i]=0; }
        else if(++diff_key_count[i] >= RUN_KEY_DEBOUNCE_SCANS)
        {
            diff_key_count[i]=0;
            diff_key_stable[i]=pressed;
            if(pressed) { events |= (uint8)(1u<<i); }
        }
    }
    if(events & 1u)
    {
        // C13 优先；启停时丢弃尚未完成的 C12 调速手势。
        diff_speed_active = 0;
        if(DiffTest_Active()) { DiffTest_Stop(6); return; }
        MotorPID_SetEnable(0);
        Servo_Steer(0);
        diff_test_ticks = 0;
        diff_fault_side = 0;
        DiffTest_ClearWindow();
        for(i=0;i<4;i++) { diff_valid[i]=diff_pass[i]=0; }
        diff_test_state = 1;
        return;
    }
    // 测试中冻结速度，避免四段测量使用不同设定值。C13 停车后再调。
    if(DiffTest_Active()) { diff_speed_active=0; return; }
    if(events & 2u)
    {
        diff_speed_active=1;
        diff_speed_long=0;
        diff_speed_hold=0;
    }
    if(diff_speed_active)
    {
        if(diff_key_stable[1])
        {
            if(diff_speed_hold<SPEED_KEY_LONG_SCANS) { diff_speed_hold++; }
            if(diff_speed_hold>=SPEED_KEY_LONG_SCANS && !diff_speed_long)
            {
                DiffTest_AdjustSpeed(-1);
                diff_speed_long=1;
            }
        }
        else
        {
            if(!diff_speed_long) { DiffTest_AdjustSpeed(1); }
            diff_speed_active=0;
        }
    }
}

// 编码器由 10ms 中断读取；只统计每段后 2 秒的全部采样，不受刷屏耗时影响。
static void DiffTest_Sample10ms(void)
{
    if(DiffTest_Active() && (diff_test_ticks > DIFF_TEST_SETTLE_TICKS))
    {
        diff_sum_l += enc_l;
        diff_sum_r += enc_r;
        diff_sum_tl += (int32)(MotorPID_GetTargetLeft()*10.0f+0.5f);
        diff_sum_tr += (int32)(MotorPID_GetTargetRight()*10.0f+0.5f);
        diff_samples++;
    }
}

static void DiffTest_FinishPhase(void)
{
    int index = diff_test_state-1;
    int e_l, e_r, delta;
    if(diff_samples < 100) { return; }
    diff_avg_l[index] = (int16)((diff_sum_l*10+diff_samples/2)/diff_samples);
    diff_avg_r[index] = (int16)((diff_sum_r*10+diff_samples/2)/diff_samples);
    diff_avg_tl[index] = (int16)((diff_sum_tl+diff_samples/2)/diff_samples);
    diff_avg_tr[index] = (int16)((diff_sum_tr+diff_samples/2)/diff_samples);
    e_l = diff_avg_l[index]-diff_avg_tl[index]; if(e_l<0) { e_l=-e_l; }
    e_r = diff_avg_r[index]-diff_avg_tr[index]; if(e_r<0) { e_r=-e_r; }
    delta = diff_avg_r[index]-diff_avg_l[index];
    diff_pass[index] = (e_l<=30 && e_r<=30);
    if(diff_test_state==2) { diff_pass[index] &= (delta>=30); }
    else if(diff_test_state==4) { diff_pass[index] &= (delta<=-30); }
    else { diff_pass[index] &= (delta>=-30 && delta<=30); }
    diff_valid[index] = 1;
}

static void DiffTest_Control20ms(void)
{
    float turn;
    if(!DiffTest_Active()) { MotorPID_SetEnable(0); return; }
    if(diff_test_ticks >= DIFF_TEST_PHASE_TICKS)
    {
        DiffTest_FinishPhase();
        if(diff_test_state==4) { DiffTest_Stop(5); return; }
        diff_test_state++;
        diff_test_ticks = 0;
        DiffTest_ClearWindow();
    }
    diff_test_ticks++;
    MotorPID_SetEnable(1);
    // 前轮保持回中，模拟左右转向命令；使用与循迹相同的速度环和 30% 差速。
    Servo_Steer(0);
    turn = (diff_test_state==2) ? 1.0f : ((diff_test_state==4) ? -1.0f : 0.0f);
    MotorPID_SetTarget(diff_test_speed);
    MotorPID_SetTurn(turn);
    MotorPID_Update(enc_l,enc_r);
    if(diff_test_ticks > DIFF_TEST_STALL_START)
    {
        if(enc_l<3 || enc_r<3) { diff_test_stall++; }
        else { diff_test_stall=0; }
        if(diff_test_stall>=DIFF_TEST_STALL_TICKS)
        {
            diff_fault_side = (uint8)((enc_l<3 ? 1 : 0) | (enc_r<3 ? 2 : 0));
            DiffTest_Stop(7);
        }
    }
}

static int DiffTest_Passed(void)
{
    uint8 i;
    if(diff_test_state!=5) { return 0; }
    for(i=0;i<4;i++) { if(!diff_valid[i] || !diff_pass[i]) { return 0; } }
    return 1;
}
#endif
// END DIFFERENTIAL BENCH TEST

static void boot_led_self_test(void)
{
    uint8 i;

    gpio_init(B9, GPO, GPIO_HIGH, GPO_PUSH_PULL);           // B9 = 核心板蓝灯（低电平点亮）

    for(i = 0; i < 3; i++)
    {
        gpio_set_level(B9, GPIO_LOW);   system_delay_ms(120);
        gpio_set_level(B9, GPIO_HIGH);  system_delay_ms(120);
    }
}

static void log_msg(const char *str)
{
    printf("%s", str);
}

static void Run_StartupWait(uint32 milliseconds)
{
    while(milliseconds)
    {
        uint32 step=milliseconds>1U ? 1U:milliseconds;
#if USE_REMOTE_CONTROL
        Run_RemoteService();
#endif
        system_delay_ms(step);milliseconds-=step;
    }
}

static void Camera_InitForever(void)
{
    uint32 cnt = 0;

    while(mt9v03x_init())
    {
        gpio_set_level(B9, GPIO_LOW);   Run_StartupWait(60);
        gpio_set_level(B9, GPIO_HIGH);  Run_StartupWait(120);
        gpio_set_level(B9, GPIO_LOW);   Run_StartupWait(60);
        gpio_set_level(B9, GPIO_HIGH);  Run_StartupWait(800);

        cnt++;
        if(0 == (cnt % 3))
        {
            log_msg("[cam] camera init FAILED, retry ... (检查 CSI 排线有没有插紧)\r\n");
        }
    }

    gpio_set_level(B9, GPIO_HIGH);
    mt9v03x_set_exposure_time(CAM_EXPOSURE);
}


//===================================================================================================================
//  五、定时中断（isr.c 调用，函数名不能改）
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 10ms：读陀螺仪 + 读编码器
//-------------------------------------------------------------------------------------------------------------------
void pit_handler1(void)
{
#if MOTOR_DIAGNOSTIC_MODE == 3
    MotorTune_Keys10ms();
#elif MOTOR_DIAGNOSTIC_MODE == 2
    DiffTest_Keys10ms();
#elif USE_RUN_KEYS
    Run_Keys10ms();
#endif
#if USE_LEGACY_GYRO
    imu660ra_get_gyro();
    Get_Gyroscope_Angle();
#endif

    /* 编码器：每 10ms 的计数 = 这一小段时间转了多少。

       ★ 只正转，所以统一取正数：两个轮子的"前进转速"都显示成正数，
         不再搞一个正一个负。这样波形第 3/4 条、屏幕 Act 都直接表示转速大小。
         （编码器硬件 A/B 相序接反也无所谓，绝对值兜底。） */
    {
        int32 c = (int32)encoder_get_count(QTIMER1_ENCODER1);
        if(c < 0) { c = -c; }
        enc_l = (int16)c;
    }
    encoder_clear_count(QTIMER1_ENCODER1);

    {
        int32 c = (int32)encoder_get_count(QTIMER1_ENCODER2);
        if(c < 0) { c = -c; }
        enc_r = (int16)c;
    }
    encoder_clear_count(QTIMER1_ENCODER2);
#if MOTOR_DIAGNOSTIC_MODE == 2
    DiffTest_Sample10ms();
#elif MOTOR_DIAGNOSTIC_MODE == 3
    MotorTune_Sample10ms(enc_l, enc_r);
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 20ms：电机增量式 PID + 舵机 PD + 心跳灯
//-------------------------------------------------------------------------------------------------------------------
void pit_handler0(void)
{
#if SERVO_SETUP_MODE != SERVO_SETUP_RUN
    Motor_Stop();
    Servo_Steer(0);
    return;                                               // 安装/舵机标定模式始终禁用电机
#endif
#if MOTOR_DIAGNOSTIC_MODE == 3
    MotorTune_Control20ms();
    return;
#elif MOTOR_DIAGNOSTIC_MODE == 2
    DiffTest_Control20ms();
    return;
#endif
#if USE_RUN_KEYS && !USE_REMOTE_CONTROL
    Run_KeyTick20ms();
#endif
#if USE_REMOTE_CONTROL
    {
        uint8 reason=TrackRemote_Tick20ms((uint8)(run_enable>0.0f && run_switch>0.5f));
        if(reason) { Run_RemoteStop(reason); }
    }
#endif
#if USE_ISLAND
    Roundabout_Tick20ms((uint8)(run_enable>0.0f && run_switch>0.5f));
    if(Roundabout_GetFault() && !Roundabout_ResetPending())
    {
#if USE_RUN_KEYS
        run_key_fault = 2;
#endif
#if USE_REMOTE_CONTROL
        if(run_switch>0.5f) { Run_RemoteStop(REMOTE_REASON_ROUNDABOUT); }
#endif
        run_switch=0.0f;
    }
    if(Roundabout_ResetPending()) { MotorPID_SetEnable(0);dbg_duty=0;Servo_Steer(0);return; }
#endif
    if((run_enable > 0.0f) && (run_switch > 0.5f))
    {
        float use_err;
        float u;
        float d;
        float target = run_speed;

        if(!MotorPID_IsEnabled()) { Run_MonitorReset(); Run_SpeedReset(); }
        MotorPID_SetEnable(1);

#if USE_ISLAND
        /* 进环岛自动降速 */
        if(Island_State != 0)
        {
            target *= ISLAND_SLOW_RATIO;
            if(target>ROUND_SPEED_MAX) { target=ROUND_SPEED_MAX; }
        }
#endif
        /* 过十字自动降速 */
        if(Cross_Flag)
        {
            target *= CROSS_SLOW_RATIO;
        }
        /* 舵机 PD：u = P*偏差 + D*(20ms 偏差变化量)
           图像丢失时保持上一拍，不乱甩 */
        if(Img_Disappear_Flag)
        {
            use_err = err_last;
        }
        else
        {
            use_err = ErrUse;
        }

        /* 转向死区：直道上的小噪声不要推舵机 */
        if((use_err > -STEER_DEADBAND) && (use_err < STEER_DEADBAND))
        {
            use_err = 0.0f;
            d       = 0.0f;
            steer_d_filter = 0.0f;
        }
        else if(!steer_pd_init || Img_Disappear_Flag)
        {
            d = 0.0f;
            steer_d_filter = 0.0f;
        }
        else
        {
            // 在固定控制周期内算 D；不重复使用上一摄像头帧的同一份 D。
            d = use_err - steer_prev_error;
            if(d > ERR_D_LIMIT) { d = ERR_D_LIMIT; }
            if(d < -ERR_D_LIMIT) { d = -ERR_D_LIMIT; }
            steer_d_filter += 0.5f * (d - steer_d_filter);
            d = steer_d_filter;
        }
        steer_prev_error = use_err;
        steer_pd_init = Img_Disappear_Flag ? 0 : 1;

        u = run_p * use_err + run_d * d;

        /* 只在明显偏差时增加转向，小偏差不加额外内切量。 */
        u += Steering_Boost(use_err);

        if(u >  SERVO_U_LIMIT) { u =  SERVO_U_LIMIT; }
        if(u < -SERVO_U_LIMIT) { u = -SERVO_U_LIMIT; }

        /* 比例降速同时受弯道绝对上限约束；设定速度低时不会被上限反向抬高。 */
        // 丢线时不采用失效的前方预警；弯中限速仍按保持的转向量执行。
        target = Run_SpeedTarget(target, u, Img_Disappear_Flag ? 0.0f : ErrPreview);
        Servo_Steer((int)u);
        // 大图像偏差才辅助；旧图像、十字、丢线或回正换向立即撤销。
        if(error_diff_age < ERROR_DIFF_MAX_AGE) { error_diff_age++; }
        if(Img_Disappear_Flag || Cross_Flag || error_diff_age >= ERROR_DIFF_MAX_AGE
#if USE_ISLAND
           || !Roundabout_AllowDifferential()
#endif
          )
        {
            Run_ErrorDiffReset();
        }
        MotorPID_SetTurn(Run_ErrorDiffCommand(Servo_GetAngle()));
        MotorPID_SetTarget(target);
        dbg_duty = MotorPID_Update(enc_l, enc_r);

        err_last = use_err;
        Run_MonitorStep(Servo_GetDuty(), Img_Disappear_Flag);

        /* 心跳灯：每 500ms 翻一下 B9 */
        if(++hb_cnt >= 25)
        {
            hb_cnt = 0;
            gpio_toggle_level(B9);
        }
    }
    else
    {
        MotorPID_SetEnable(0);
        dbg_duty = 0;
        hb_cnt  = 0;
        err_last = 0.0f;
        steer_prev_error = steer_d_filter = 0.0f;
        steer_pd_init = 0;
        Run_ErrorDiffReset();
        Servo_Steer(0);
    }
}


//===================================================================================================================
//  六、一帧图像处理
//===================================================================================================================

// BEGIN FRAME SNAPSHOT
static uint8 Frame_TakeSnapshot(void)
{
    const uint8 *source;
    uint32 irq_state = __get_PRIMASK();
    // 仅保护指针/完成标志的读取；18800字节复制期间保持原中断状态。
    __disable_irq();
    if(!mt9v03x_finish_flag)
    {
        __set_PRIMASK(irq_state);
        return 0;
    }
    source = (const uint8 *)mt9v03x_image;
    mt9v03x_finish_flag = 0;
    __set_PRIMASK(irq_state);
    if(!source) { return 0; }
    memcpy(frame_gray[0], source, sizeof(frame_gray));
    __DMB();
    // 复制途中发生换帧则舍弃本次快照，下一圈从最新完成帧重取，避免拼帧。
    return mt9v03x_finish_flag ? 0 : 1;
}
// END FRAME SNAPSHOT

static void Frame_Process(const uint8 *gray)
{
    int   i;
    uint8 line_mode;
#if USE_ASSISTANT
    float wave[ASSISTANT_OSC_CH_MAX];
#endif

    /* 1. 巡线：方案由 LineDetect.h 的 LINE_SCHEME 决定（改宏后重新编译）*/
    line_mode = LINE_SCHEME;
    LineDetect_Process(gray, line_mode);

    /* 2. 十字先检查原始边线；已确认环岛时Cross_Check会主动让出处理权。 */
    Cross_Check();
#if USE_ISLAND
    Roundabout_Process((uint8)(run_enable>0.0f && run_switch>0.5f));
#endif

    /* 补线后统一统计；十字仍按补出的中线转向，出口有弯时能及时跟线。 */
    LineDetect_RefreshStats();

    /* 3. 高速前方弯道生成减速预警，转向仍使用基础位置偏差。 */
    Err_SetPreviewSpeed(run_speed);
    if(!Err_Compute())
    {
        Img_Disappear_Flag = 1;
    }
#if USE_REMOTE_CONTROL
    TrackRemote_Frame((uint8)!Img_Disappear_Flag);
#endif
#if USE_RUN_KEYS
    Run_KeyFrame((uint8)!Img_Disappear_Flag);
#endif
    Run_AutoStartTry((uint8)!Img_Disappear_Flag);

    Run_ErrorDiffFrame(Err,ErrUse,
        run_enable>0.0f && run_switch>0.5f && !Img_Disappear_Flag &&
        !Cross_Flag && ErrValidRows>=ERROR_DIFF_MIN_ROWS);

    /* 4. 整理三条线给屏幕 + 助手边线（丢线时保持上一个值，避免线飞到边上）*/
    {
        int last_l = 0;
        int last_r = MT9V03X_W - 1;

        for(i = MT9V03X_H - 1; i >= 0; i--)
        {
            int l = Left_Line[i];
            int r = Right_Line[i];
            int visible = (i >= (MT9V03X_H - Search_Stop_Line)) ? 1 : 0;

            if(l < 0)             { l = 0; }
            if(l > MT9V03X_W - 1) { l = MT9V03X_W - 1; }
            if(r < 0)             { r = 0; }
            if(r > MT9V03X_W - 1) { r = MT9V03X_W - 1; }

            /* 单边丢线时，屏幕仍显示另一侧真实边线，方便检查弯道。 */
            line_left[i] = (visible && !Left_Lost_Flag[i]) ? (uint8)l : 255;
            line_right[i] = (visible && !Right_Lost_Flag[i]) ? (uint8)r : 255;
            line_mid[i] = (visible && !Left_Lost_Flag[i] && !Right_Lost_Flag[i])
                          ? (uint8)((l + r) >> 1) : 255;

            /* 助手边线：有效行更新，丢线/视野外保持上一个值 */
            if(visible && (0 == Left_Lost_Flag[i]))  { last_l = (Left_Line[i]  < 0) ? 0 : ((Left_Line[i]  > MT9V03X_W - 1) ? (MT9V03X_W - 1) : Left_Line[i]); }
            if(visible && (0 == Right_Lost_Flag[i])) { last_r = (Right_Line[i] < 0) ? 0 : ((Right_Line[i] > MT9V03X_W - 1) ? (MT9V03X_W - 1) : Right_Line[i]); }
            asst_left[i]  = (uint8)last_l;
            asst_right[i] = (uint8)last_r;
        }
    }

#if USE_SCREEN
    /* 图像与精简状态同帧刷新。 */
    if(Screen_ShowFrame(gray, image_two_value[0], line_left, line_mid, line_right))
    {
        char p_text[5], d_text[6];
        Screen_ShowRefCol(LineDetect_GetRefCol());
        Run_FormatP(run_p, p_text);
        Run_FormatD(run_d, d_text);
        Screen_ShowDriving(ErrUse, (int)run_speed, p_text, d_text,
                           enc_l, (int)MotorPID_GetTargetLeft(),
                           enc_r, (int)MotorPID_GetTargetRight(),
                           (uint8)(run_enable > 0.0f && run_switch > 0.5f), 0);
        Screen_ShowTrackFlags(Cross_Flag, (int)Cross_GetState(), Island_State,
                             Roundabout_GetDirection(),
                             (uint8)(Img_Disappear_Flag || run_key_image_age >= RUN_IMAGE_TIMEOUT_TICKS),
                             run_key_fault, Servo_GetDuty(), run_tune_item);
    }
#endif

    /* 6. 发波形 + 边线 + 图像给逐飞助手 */
#if USE_ASSISTANT
    wave[0] = ErrUse;
    wave[1] = (float)Servo_GetAngle();
    wave[2] = (float)enc_l;
    wave[3] = (float)enc_r;
    wave[4] = MotorPID_GetTarget();
    wave[5] = MotorPID_GetMeasured();
    wave[6] = (float)dbg_duty;
    wave[7] = (float)Search_Stop_Line;

    Assistant_SendFrame(gray, asst_left, asst_right, wave);
#endif
}


//===================================================================================================================
//  七、主程序
//===================================================================================================================

int main(void)
{
    uint8 i;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    boot_led_self_test();

    /* 执行机构 */
    Motor_Init();
    Motor_Stop();
    Servo_Init();

#if MOTOR_DIAGNOSTIC_MODE == 3
    // 独立电机调试：无需摄像头或无线模块即可连接VOFA。
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
    MotorTune_Init();
#if USE_SCREEN
    Screen_Init(); ips200_clear();
    ips200_show_string(0,8,"USB MOTOR PID / FIREWATER");
    ips200_show_string(0,32,"CONNECT CORE TYPE-C USB1");
#endif
    usb_cdc_init();
    pit_ms_init(PIT_CH1,10);
    pit_ms_init(PIT_CH0,20);
    interrupt_global_enable(0);
    while(1)
    {
        MotorTune_Poll();
#if USE_SCREEN
        MotorTune_Show();
#endif
        system_delay_ms(1);
    }
#elif MOTOR_DIAGNOSTIC_MODE == 2
    // 验证版不初始化摄像头或无线模块，计时及闭环由 20ms/10ms 中断运行。
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
    MotorPID_Init();
    DiffTest_Init();
#if USE_SCREEN
    Screen_Init(); ips200_clear();
    ips200_show_string(0,8,"DIFFERENTIAL TEST");
    ips200_show_string(0,32,"LIFT DRIVE WHEELS");
    ips200_show_string(0,56,"C13: START / STOP");
    ips200_show_string(0,104,"LIVE: ACT / TARGET");
    ips200_show_string(0,160,"AVG: LEFT A/T  RIGHT A/T");
    ips200_show_string(0,304,"SPD:");
    ips200_show_string(88,304,"C12:TAP+ HOLD-");
#endif
    pit_ms_init(PIT_CH0,20);
    pit_ms_init(PIT_CH1,10);
    interrupt_global_enable(0);
    while(1)
    {
#if USE_SCREEN
        uint8 row;
        ips200_show_int(40,304,(int)diff_test_speed,3);
        ips200_show_string(0,80,
            (diff_test_state==1) ? "STEP: STRAIGHT 1        " :
            (diff_test_state==2) ? "STEP: LEFT (RIGHT FAST) " :
            (diff_test_state==3) ? "STEP: STRAIGHT 2        " :
            (diff_test_state==4) ? "STEP: RIGHT (LEFT FAST) " :
            (diff_test_state==5) ? "STEP: DONE - MOTOR OFF  " :
            (diff_test_state==6) ? "STEP: STOPPED           " :
            (diff_test_state==7) ? ((diff_fault_side==1) ? "LEFT WHEEL/ENCODER LOW  " :
                                    ((diff_fault_side==2) ? "RIGHT WHEEL/ENCODER LOW " : "BOTH WHEEL/ENCODER LOW  ")) : "STEP: READY             ");
        ips200_show_string(0,128,"L");
        ips200_show_int(16,128,enc_l,4); ips200_show_string(56,128,"/");
        ips200_show_int(64,128,(int)(MotorPID_GetTargetLeft()+0.5f),3);
        ips200_show_string(120,128,"R");
        ips200_show_int(136,128,enc_r,4); ips200_show_string(176,128,"/");
        ips200_show_int(184,128,(int)(MotorPID_GetTargetRight()+0.5f),3);
        for(row=0;row<4;row++)
        {
            uint16 y = (uint16)(184+24*row);
            ips200_show_string(0,y,(row==0) ? "S1" : ((row==1) ? "LT" : ((row==2) ? "S2" : "RT")));
            if(diff_valid[row])
            {
                ips200_show_int(32,y,(diff_avg_l[row]+5)/10,3); ips200_show_string(64,y,"/");
                ips200_show_int(72,y,(diff_avg_tl[row]+5)/10,3);
                ips200_show_int(136,y,(diff_avg_r[row]+5)/10,3); ips200_show_string(168,y,"/");
                ips200_show_int(176,y,(diff_avg_tr[row]+5)/10,3);
            }
            else { ips200_show_string(32,y," -- / --      -- / -- "); }
        }
        ips200_show_string(0,280,
            DiffTest_Passed() ? "RESULT: PASS            " :
            ((diff_test_state==5 || diff_test_state==7) ? "RESULT: CHECK - PHOTO IT " : "RESULT: WAIT            "));
#endif
        system_delay_ms(100);
    }
#elif MOTOR_DIAGNOSTIC_MODE == 1
    {
        uint16 test_ticks = 0;
        uint8 test_state = 0;
#if USE_SCREEN
        uint8 shown_state = 255;
#endif

#if USE_SCREEN
        Screen_Init();
        ips200_clear();
        ips200_show_string(8, 8,   "MOTOR JOG TEST");
        ips200_show_string(8, 40,  "LIFT DRIVE WHEELS");
        ips200_show_string(8, 72,  "C15: START TEST");
        ips200_show_string(8, 96,  "LEFT 0.6s, PAUSE");
        ips200_show_string(8, 120, "RIGHT 0.6s");
        ips200_show_string(8, 144, "C13: EMERGENCY STOP");
        ips200_show_string(8, 192, "DUTY: 1800 / 4000");
        ips200_show_string(8, 240, "STATUS: READY");
#endif
        key_init(5);
        while(1)
        {
            key_scanner();
            if(key_get_state(KEY_3) != KEY_RELEASE)
            {
                Motor_Stop();
                test_ticks = 0;
                test_state = 0;
            }
            else if((key_get_state(KEY_1) == KEY_SHORT_PRESS) && (0 == test_state))
            {
                Motor_Set(1800, 0);
                test_ticks = 120;
                test_state = 1;
            }
            key_clear_all_state();

            if(test_ticks > 0)
            {
                test_ticks--;
                if(0 == test_ticks)
                {
                    Motor_Stop();
                    if(1 == test_state)
                    {
                        test_state = 2;
                        test_ticks = 80;
                    }
                    else if(2 == test_state)
                    {
                        Motor_Set(0, 1800);
                        test_state = 3;
                        test_ticks = 120;
                    }
                    else
                    {
                        test_state = 0;
                    }
                }
            }
#if USE_SCREEN
            if(test_state != shown_state)
            {
                ips200_show_string(8, 240,
                    (1 == test_state) ? "STATUS: LEFT      " :
                    (2 == test_state) ? "STATUS: PAUSE     " :
                    (3 == test_state) ? "STATUS: RIGHT     " : "STATUS: READY     ");
                shown_state = test_state;
            }
#endif
            system_delay_ms(5);
        }
    }
#elif SERVO_SETUP_MODE == SERVO_SETUP_AUTO
    {
        uint8 servo_tick = 0;
        uint8 refresh = 0;
#if USE_SCREEN
        Screen_Init();
        ips200_clear();
        ips200_show_string(8, 8,   "S-U400 AUTO SWEEP");
        ips200_show_string(8, 32,  "MOTOR DISABLED");
        ips200_show_string(8, 56,  "PWM 50Hz CENTER:");
        ips200_show_int(144, 56, SERVO_DUTY_MIDDLE, 3);
        ips200_show_string(8, 80,  "K1:-10 K2:+10 AMP");
        ips200_show_string(8, 104, "K3:STOP K4:RESTART");
        ips200_show_string(8, 136, "DUTY:");
        ips200_show_string(8, 160, "TARGET:");
        ips200_show_string(8, 184, "AMP:");
        ips200_show_string(8, 208, "LOW:");
        ips200_show_string(120, 208, "HIGH:");
        ips200_show_string(120, 240, "CYCLE:");
        ips200_show_string(8, 272, "No wireless / camera");
        ips200_show_string(8, 296, "Observe linkage clearance");
#endif
        key_init(5);
        while(1)
        {
            Motor_Stop();
            key_scanner();
            if(key_get_state(KEY_3) != KEY_RELEASE) { Servo_AutoTestPause(1); }
            else if(key_get_state(KEY_4) == KEY_SHORT_PRESS) { Servo_AutoTestPause(0); }
            else if((key_get_state(KEY_1) == KEY_SHORT_PRESS) &&
                    (key_get_state(KEY_2) == KEY_RELEASE))
            {
                Servo_AutoTestAdjust(-SERVO_AUTO_AMP_STEP);
            }
            else if((key_get_state(KEY_2) == KEY_SHORT_PRESS) &&
                    (key_get_state(KEY_1) == KEY_RELEASE))
            {
                Servo_AutoTestAdjust(SERVO_AUTO_AMP_STEP);
            }
            key_clear_all_state();
            if(++servo_tick >= 4)
            {
                servo_tick = 0;
                Servo_AutoTestStep();
            }
#if USE_SCREEN
            if(++refresh >= 20)
            {
                uint16 amplitude = Servo_AutoTestAmplitude();
                refresh = 0;
                ips200_show_int(80, 136, Servo_GetDuty(), 4);
                ips200_show_int(80, 160, Servo_GetCalibrationTarget(), 4);
                ips200_show_int(80, 184, amplitude, 4);
                ips200_show_int(48, 208, SERVO_DUTY_MIDDLE - amplitude, 4);
                ips200_show_int(168, 208, SERVO_DUTY_MIDDLE + amplitude, 4);
                ips200_show_string(8, 240, Servo_AutoTestPaused() ? "PAUSED " : "RUNNING");
                ips200_show_int(176, 240, Servo_AutoTestCycles(), 1);
            }
#endif
            system_delay_ms(5);
        }
    }
#elif SERVO_SETUP_MODE == SERVO_SETUP_MANUAL
    {
        uint8 wireless_ok;
        uint8 refresh = 0;
        uint8 servo_tick = 0;
        uint8 command_status = 0;
        uint16 low_mark = 0;
        uint16 high_mark = 0;

#if USE_SCREEN
        Screen_Init();
        ips200_clear();
        ips200_show_string(8, 8,   "SU400 KEY LIMIT TEST");
        ips200_show_string(8, 32,  "MOTOR DISABLED");
        ips200_show_string(8, 56,  "C15:-5 C14:+5");
        ips200_show_string(8, 80,  "C13:CENTER C12:RECORD");
        ips200_show_string(8, 104, "DUTY:");
        ips200_show_string(8, 128, "TARGET:");
        ips200_show_string(8, 152, "LOW:");
        ips200_show_string(120, 152, "HIGH:");
#if SERVO_TEST_USE_WIRELESS
        ips200_show_string(8, 176, "RX:");
        ips200_show_string(120, 176, "CH:");
        ips200_show_string(8, 200, "VALUE:");
#else
        ips200_show_string(8, 176, "PULSE(us):");
        ips200_show_string(8, 200, "STEP: 5 = 10 us");
#endif
        ips200_show_string(8, 272, "460..1050; STEP<=5");
        ips200_show_string(8, 296, "0 = not recorded");
#endif
        // 库默认 KEY1~4 对应 C15/C14/C13/C12；只点按，不做长按连发。
        key_init(5);
#if SERVO_TEST_USE_WIRELESS
        interrupt_global_enable(0);
        wireless_ok = (Assistant_Init() == 0);
#else
        wireless_ok = 0;                                 // 无线模块不可用时，不初始化、不等待握手
#endif
#if USE_SCREEN
        ips200_show_string(8, 224, "CENTER:");
        ips200_show_int(80, 224, SERVO_DUTY_MIDDLE, 4);
#endif
        while(1)
        {
            Motor_Stop();
            if(wireless_ok)
            {
                Assistant_Poll();
                if(Assistant_GetUpdated(ASST_PARAM_SPEED))
                {
                    command_status = Servo_CalibrationSetTarget(Assistant_GetParam(ASST_PARAM_SPEED)) ? 1 : 2;
                }
                else
                {
                    for(i = 1; i < 8; i++)
                    {
                        if(Assistant_GetUpdated((assistant_param_enum)i)) { command_status = 4; }
                    }
                }
                for(i = 0; i < 8; i++) { Assistant_ClearUpdated((assistant_param_enum)i); }
            }

            key_scanner();
            // 回中/记录优先，两个方向键同时按下时不动作。
            if(key_get_state(KEY_3) != KEY_RELEASE)
            {
                Servo_CalibrationSetTarget(SERVO_DUTY_MIDDLE);
                command_status = 3;
            }
            else if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                uint16 duty = Servo_GetDuty();
                if(duty < SERVO_DUTY_MIDDLE) { low_mark = duty; }
                else if(duty > SERVO_DUTY_MIDDLE) { high_mark = duty; }
                Servo_CalibrationSetTarget(SERVO_DUTY_MIDDLE);
                command_status = 3;
            }
            else if((key_get_state(KEY_1) == KEY_SHORT_PRESS) &&
                    (key_get_state(KEY_2) == KEY_RELEASE))
            {
                command_status = Servo_CalibrationSetTarget((float)Servo_GetDuty() - 5.0f) ? 3 : 2;
            }
            else if((key_get_state(KEY_2) == KEY_SHORT_PRESS) &&
                    (key_get_state(KEY_1) == KEY_RELEASE))
            {
                command_status = Servo_CalibrationSetTarget((float)Servo_GetDuty() + 5.0f) ? 3 : 2;
            }
            key_clear_all_state();

            if(++servo_tick >= 4)
            {
                servo_tick = 0;
                Servo_CalibrationStep();                 // 每 20ms 最多移动 1 个 PWM 单位
            }
#if USE_SCREEN
            if(++refresh >= 20)
            {
#if SERVO_TEST_USE_WIRELESS
                float value = wireless_ok ? Assistant_GetLastValue() : 0.0f;
#endif
                refresh = 0;
                ips200_show_int(80, 104, Servo_GetDuty(), 4);
                ips200_show_int(80, 128, Servo_GetCalibrationTarget(), 4);
                ips200_show_int(48, 152, low_mark, 4);
                ips200_show_int(168, 152, high_mark, 4);
#if SERVO_TEST_USE_WIRELESS
                ips200_show_int(40, 176, wireless_ok ? Assistant_GetRxCount() : 0, 6);
                ips200_show_int(152, 176, wireless_ok ? Assistant_GetLastChannel() : 0, 1);
                if((value >= -9999.0f) && (value <= 9999.0f))
                {
                    ips200_show_float(72, 200, value, 4, 1);
                }
                else { ips200_show_string(72, 200, "INVALID"); }
#else
                ips200_show_int(104, 176, Servo_GetDuty() * 2, 4);
#endif
                ips200_show_string(8, 248,
                    (command_status == 1) ? "UART CMD OK       " :
                    (command_status == 2) ? "BAD STEP OR RANGE " :
                    (command_status == 3) ? "KEY CMD OK        " :
                    (command_status == 4) ? "WRONG CH: USE CH0 " : "WAIT KEY / CH0    ");
            }
#endif
            system_delay_ms(5);
        }
    }
#elif SERVO_SETUP_MODE == SERVO_SETUP_TRIM
    {
        uint16 trim_duty = SERVO_DUTY_MIDDLE;
#if USE_SCREEN
        uint16 shown_duty = 0;
        Screen_Init();
        ips200_clear();
        ips200_show_string(8, 16,  "S-U400 CENTER TRIM");
        ips200_show_string(8, 48,  "MOTOR DISABLED");
        ips200_show_string(8, 80,  "C15: -1 (RIGHT)");
        ips200_show_string(8, 104, "C14: +1 (LEFT)");
        ips200_show_string(8, 128, "C13: RESET TO");
        ips200_show_int(120, 128, SERVO_DUTY_MIDDLE, 3);
        ips200_show_string(8, 168, "DUTY:");
        ips200_show_string(8, 200, "PULSE(us):");
        ips200_show_string(8, 248, "TRIM ONLY:");
        ips200_show_int(96, 248, SERVO_DUTY_MIDDLE - SERVO_CENTER_TRIM_RANGE, 3);
        ips200_show_string(136, 248, "..");
        ips200_show_int(160, 248, SERVO_DUTY_MIDDLE + SERVO_CENTER_TRIM_RANGE, 3);
        ips200_show_string(8, 280, "Report straight DUTY");
#endif
        key_init(5);
        while(1)
        {
            Motor_Stop();
            key_scanner();
            if(key_get_state(KEY_3) != KEY_RELEASE)
            {
                trim_duty = SERVO_DUTY_MIDDLE;
            }
            else if((key_get_state(KEY_1) == KEY_SHORT_PRESS) &&
                    (key_get_state(KEY_2) == KEY_RELEASE))
            {
                if(trim_duty > SERVO_DUTY_MIDDLE - SERVO_CENTER_TRIM_RANGE) { trim_duty--; }
            }
            else if((key_get_state(KEY_2) == KEY_SHORT_PRESS) &&
                    (key_get_state(KEY_1) == KEY_RELEASE))
            {
                if(trim_duty < SERVO_DUTY_MIDDLE + SERVO_CENTER_TRIM_RANGE) { trim_duty++; }
            }
            key_clear_all_state();
            if(trim_duty != Servo_GetDuty()) { Servo_SetDuty(trim_duty); }
            trim_duty = Servo_GetDuty();
#if USE_SCREEN
            if(trim_duty != shown_duty)
            {
                ips200_show_int(96, 168, trim_duty, 4);
                ips200_show_int(104, 200, trim_duty * 2, 4);
                shown_duty = trim_duty;
            }
#endif
            system_delay_ms(5);
        }
    }
#elif SERVO_SETUP_MODE == SERVO_SETUP_CENTER
    /* 不初始化无线串口、摄像头、编码器或控制定时器；不依赖摄像头连线。 */
#if USE_SCREEN
    Screen_Init();
    ips200_clear();
    ips200_show_string(8, 16,  "S-U400 CENTER MODE");
    ips200_show_string(8, 48,  "MOTOR DISABLED");
    ips200_show_string(8, 80,  "PWM: 50 Hz");
    ips200_show_string(8, 112, "PULSE(us):");
    ips200_show_int(104, 112, SERVO_DUTY_MIDDLE * 2, 4);
    ips200_show_string(8, 144, "DUTY:");
    ips200_show_int(80, 144, SERVO_DUTY_MIDDLE, 4);
    ips200_show_string(8, 192, "Power OFF before assembly");
#endif
    while(1)
    {
        Motor_Stop();
        Servo_Steer(0);
        system_delay_ms(100);
    }
#endif

    /* 编码器（开中断前初始化）*/
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0,  QTIMER1_ENCODER1_CH2_C1);    // 左轮
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2,  QTIMER1_ENCODER2_CH2_C24);   // 右轮

    /* 控制/巡线/偏差模块 */
    MotorPID_Init();
    Contrast_Init();
    LineDetect_Init();
    Err_Init();
    Cross_Init();
    Roundabout_Init();

#if USE_SCREEN
    Screen_Init();
#endif

    /* 不使用无线时，两个初始化分支均跳过。 */
#if USE_REMOTE_CONTROL || USE_ASSISTANT
#if USE_REMOTE_CONTROL
    if(0 != TrackRemote_Init())
#elif USE_ASSISTANT
    if(0 != Assistant_Init())
#endif
    {
        for(i = 0; i < 10; i++)                             /* B9 快闪 10 下 = 无线串口初始化失败 */
        {
            gpio_set_level(B9, GPIO_LOW);   system_delay_ms(80);
            gpio_set_level(B9, GPIO_HIGH);  system_delay_ms(80);
        }
    }
    else
    {
        gpio_set_level(B9, GPIO_LOW);                       /* 常亮 0.5 秒 = 无线串口 OK */
        system_delay_ms(500);
        gpio_set_level(B9, GPIO_HIGH);
    }

#endif // USE_REMOTE_CONTROL || USE_ASSISTANT

#if USE_REMOTE_CONTROL
    /* 摄像头初始化失败时仍有波形和收命令；run_enable=0禁止发车。 */
#if USE_RUN_KEYS
    Run_KeysInit();
#endif
    pit_ms_init(PIT_CH0,20);pit_ms_init(PIT_CH1,10);
    interrupt_global_enable(0);
#endif

    /* 摄像头（失败会一直重试，B9 双闪报警）*/
    Camera_InitForever();

#if USE_LEGACY_GYRO
    {
        uint8 gtry;

        for(gtry = 0; gtry < 5; gtry++)
        {
            if(0 == imu660ra_init()) { break; }
            system_delay_ms(200);
        }

        if(gtry >= 5)
        {
            log_msg("[gyro] imu660ra FAILED -- 环岛不可用，巡线照常\r\n");
        }
        else
        {
            log_msg("[gyro] imu660ra OK, 零漂校准中，车别动 ...\r\n");
            Zero_Point_Detect();
            Clear_Gyroscope_Angle();
        }
    }
#endif

    /* 上电提示 */
    for(i = 0; i < 5; i++)
    {
        gpio_toggle_level(B9);   Run_StartupWait(100);
        gpio_toggle_level(B9);   Run_StartupWait(100);
    }

    /* 开中断 */
#if !USE_REMOTE_CONTROL
#if USE_RUN_KEYS
    Run_KeysInit();
#endif
    pit_ms_init(PIT_CH0, 20);                               // 20ms：电机 + 舵机 PD
    pit_ms_init(PIT_CH1, 10);                               // 10ms：陀螺仪 + 编码器
    interrupt_global_enable(0);
#endif

    run_enable = 1.0f;

    /* 自动起步由新帧中的 Run_AutoStartTry 执行，避免尚无图像时立即触发丢图停车。 */

    err_last = ErrUse;
    gpio_set_level(B9, GPIO_HIGH);

    while(1)
    {
#if USE_RUN_KEYS && USE_SCREEN
        Run_KeyDisplayService();
#endif
        /* 1. 可选的无线参数；当前按键版均关闭。 */
#if USE_REMOTE_CONTROL
        Run_RemoteService();
#elif USE_ASSISTANT
        Assistant_Poll();

        /* 2. 把新参数搬进全局变量（中断里读的就是这些）*/
        // 用户固定SPD=170，无线速度通道只清更新标志，不改变基础巡航值。
        if(Assistant_GetUpdated(ASST_PARAM_SPEED))   { Assistant_ClearUpdated(ASST_PARAM_SPEED); }
        if(Assistant_GetUpdated(ASST_PARAM_MOTOR_KP)){ Assistant_ClearUpdated(ASST_PARAM_MOTOR_KP);run_kp     = Assistant_GetParam(ASST_PARAM_MOTOR_KP); }
        if(Assistant_GetUpdated(ASST_PARAM_MOTOR_KI)){ Assistant_ClearUpdated(ASST_PARAM_MOTOR_KI);run_ki     = Assistant_GetParam(ASST_PARAM_MOTOR_KI); }
        if(Assistant_GetUpdated(ASST_PARAM_MOTOR_KD)){ Assistant_ClearUpdated(ASST_PARAM_MOTOR_KD);run_kd     = Assistant_GetParam(ASST_PARAM_MOTOR_KD); }
        if(Assistant_GetUpdated(ASST_PARAM_SERVO_P)) { Assistant_ClearUpdated(ASST_PARAM_SERVO_P); run_p      = Assistant_GetParam(ASST_PARAM_SERVO_P); }
        if(Assistant_GetUpdated(ASST_PARAM_SERVO_D)) { Assistant_ClearUpdated(ASST_PARAM_SERVO_D); run_d      = Assistant_GetParam(ASST_PARAM_SERVO_D); }
        if(Assistant_GetUpdated(ASST_PARAM_CONTRAST)){ Assistant_ClearUpdated(ASST_PARAM_CONTRAST);run_threshold = Assistant_GetParam(ASST_PARAM_CONTRAST); }
        if(Assistant_GetUpdated(ASST_PARAM_SWITCH))
        {
            Assistant_ClearUpdated(ASST_PARAM_SWITCH);
            run_switch = (Assistant_GetParam(ASST_PARAM_SWITCH) >= 0.5f) ? 1.0f : 0.0f;
        }
#endif

        /* 3. 限幅 */
        if(run_kp     <   0.0f) { run_kp     =   0.0f; }
        if(run_kp     >  50.0f) { run_kp     =  50.0f; }
        if(run_ki     <   0.0f) { run_ki     =   0.0f; }
        if(run_ki     >   2.0f) { run_ki     =   2.0f; }
        if(run_kd     <   0.0f) { run_kd     =   0.0f; }
        if(run_kd     >  20.0f) { run_kd     =  20.0f; }
        // 正常值不回写，避免主循环用旧值覆盖10ms按键中断刚完成的调节。
        if(!(run_p >= SERVO_P_MIN)) { run_p = SERVO_P_MIN; }
        else if(run_p > SERVO_P_MAX) { run_p = SERVO_P_MAX; }
        if(!(run_d >= SERVO_D_MIN)) { run_d = SERVO_D_MIN; }
        else if(run_d > SERVO_D_MAX) { run_d = SERVO_D_MAX; }
        if(run_threshold < -50.0f) { run_threshold = -50.0f; }
        if(run_threshold > 200.0f) { run_threshold = 200.0f; }
        // run_switch 在按键/助手命令入口归一化，避免覆盖中断刚切换的启停状态。

        MotorPID_SetGains(run_kp, run_ki, run_kd);

        /* 通道 6 的含义随巡线方案变化 */
        if(LINE_SCHEME == LINE_MODE_OTSU)
        {
            LineDetect_SetOtsuBias((int)run_threshold);     /* 大津法：阈值偏移量 */
        }
        else
        {
            Contrast_Ratio = run_threshold;                 /* 对比度：对比度阈值 */
        }

        /* 4. 摄像头出一帧 -> 处理 */
        if(Frame_TakeSnapshot())
        {
            Frame_Process(frame_gray[0]);
        }
    }
}
