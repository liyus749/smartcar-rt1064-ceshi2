/*********************************************************************************************************************
* 文件名称          main.c
* 功能简介          智能车主程序；弯道与十字循迹，屏幕显示摄像头原图
*
* 硬件              逐飞 RT1064 核心板 + 逐飞 RT1064 V3.1 主板 + 并口 IPS 屏 + 无线串口模块
*
* ================================ 整体流程 ================================
*   上电 -> 初始化外设 -> 摄像头 -> 陀螺仪 -> 开 20ms/10ms 中断 -> 主循环
*
*   主循环里只做两件事：
*     1. 收逐飞助手下发的参数（速度 / PID / 舵机 PD / 对比度阈值 / 开关）
*     2. 摄像头出一帧 -> 巡线与十字识别 -> 算偏差 -> 屏幕 -> 发波形/图像
*
*   两个定时中断：
*     pit_handler1（10ms）读陀螺仪 + 读编码器
*     pit_handler0（20ms）电机增量式 PID + 舵机 PD + 心跳灯
*
* ================================ 逐飞助手 V2.0 怎么配 ================================
*   连接方式：无线转串口（选无线模块那个 COM 口）
*   图像窗口：能看到 188x100 灰度图（约 2~3 帧/秒，无线串口带宽有限）
*   波形 8 条：ErrUse / 舵机打角 / 左轮 / 右轮 / 目标速度 / 实测速度 / 输出占空比 / 视野长度
*   参数 8 个（助手 UI 通道 0~7）：
*      0 目标速度  1 电机P  2 电机I  3 电机D  4 舵机P  5 舵机D  6 阈值  7 电机开关
*      通道6：对比度方案 = 对比度阈值；大津法方案 = 阈值偏移量
*   巡线方案在 LineDetect.h 的 LINE_SCHEME 里改，重新编译即可
*
* ================================ B9 蓝灯灯语 ================================
*   快闪 3 下        上电自检：程序跑起来了
*   常亮 0.5 秒      无线串口初始化成功
*   快闪 10 下       无线串口初始化失败（检查无线模块插好没有）
*   双闪             摄像头初始化失败，正在重试（检查 CSI 排线）
*   一秒闪一下       正在跑（20ms 控制中断的心跳）
*   灭               没在跑 / 已经停了
*
* ================================ 接线（和主板丝印一致）================================
*   舵机         -> 舵机接口 1        信号 C30
*   编码器 左/右 -> 编码器 1/2        C0+C1 / C2+C24
*   电机         -> 左轮 D3(正)+D2(反)；右轮 C11(正)+C10(反)（按单轮实测映射）
*   摄像头       -> 摄像头 CSI        PCLK B20 / VSY B22 / D0-D7 B31~B24 / SCCB C29+C28
*   屏幕         -> 屏幕接口          SCL B0 / SDA B1 / RST B2 / DC C19 / CS B3 / BL C18
*   姿态传感器   -> 姿态传感器接口    SCK C23 / MOSI C22 / MISO C21 / CS C20
*   无线串口模块 -> 无线模块接口      TX D16 / RX D17 / RTS D26（UART8）
********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "Motor.h"
#include "Servo.h"
#include "Screen.h"
#include "GYRO.h"
#include "zf_device_imu660ra.h"
#include "MotorPID.h"
#include "Assistant.h"
#include "Contrast.h"
#include "LineDetect.h"
#include "Cross.h"
#include "Err.h"


//===================================================================================================================
//  一、编译期配置
//===================================================================================================================

#define CAM_EXPOSURE        (512)      // 摄像头曝光：发白/反光调小(400/350)，远处太暗调大(600+)

// 舵机 PD 默认值（可在逐飞助手通道 4/5 在线改）
#define SERVO_P_DEFAULT     (2.5f)
#define SERVO_D_DEFAULT     (2.0f)      // 20ms 偏差差分的保守阻尼起点，仍需实车确认

// 电机速度环默认值（可在逐飞助手通道 0/1/2/3 在线改）
#define SPEED_DEFAULT       (MOTORPID_DEFAULT_TARGET)
#define SPEED_KP_DEFAULT    (MOTORPID_DEFAULT_KP)
#define SPEED_KI_DEFAULT    (MOTORPID_DEFAULT_KI)
#define SPEED_KD_DEFAULT    (MOTORPID_DEFAULT_KD)

#define RUN_AUTO_START      (0)        // 上电先看图像；C15 发车，C13 停车
#define MOTOR_DIAGNOSTIC_MODE (0)      // 0 = 正常循迹；1 = 单轮开环测试

#define USE_SCREEN          (1)        // 1 = 用屏幕显示图像+边线；0 = 不用
#define USE_ISLAND          (0)        // 当前只跑弯道和十字，关闭环岛识别及陀螺仪采集
#define ISLAND_SLOW_RATIO   (0.6f)     // 进环岛后目标速度降到 60%
#define CROSS_SLOW_RATIO    (0.7f)     // 过十字时目标速度降到 70%
#define TURN_SLOW_MEDIUM    (0.85f)    // 中等转角时目标速度降到 85%
#define TURN_SLOW_SHARP     (0.70f)    // 大转角时目标速度降到 70%
#define TURN_MEDIUM_LIMIT   (30.0f)    // 舵机输出超过此值开始弯道降速
#define TURN_SHARP_LIMIT    (60.0f)    // 舵机输出超过此值按急弯降速

// 舵机 PD 输出限幅（偏差再大也不会一把打死）
#define SERVO_U_LIMIT       (120.0f)

// 转向死区：|偏差| 小于这个值时直接当 0，防止直道被 ±1 像素噪声推着左右摆
#define STEER_DEADBAND      (1.5f)

// 大弯补偿：u += K * Err * |Err|。小误差几乎不变，大误差额外加一点，改善大弯转不过来
#define STEER_QUAD_K        (0.03f)


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
extern void           Island_Detect(void);


//===================================================================================================================
//  三、全局状态
//===================================================================================================================

// 可在线调参数（主循环从助手写进来，中断里读）
static volatile float run_speed    = SPEED_DEFAULT;
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
static volatile float run_switch   = (float)RUN_AUTO_START;
static volatile float run_enable   = 0.0f;                 // 初始化完成后置 1，允许跑

static volatile int16 enc_l = 0;                           // 左轮编码器（计数/10ms）
static volatile int16 enc_r = 0;                           // 右轮编码器（计数/10ms）
static volatile int   dbg_duty = 0;                        // 实际输出占空比
static uint16         hb_cnt = 0;                          // 心跳灯计数

static uint8          line_left[MT9V03X_H];                // 屏幕画线用
static uint8          line_mid[MT9V03X_H];
static uint8          line_right[MT9V03X_H];
static uint8          asst_left[MT9V03X_H];                // 助手边线叠加用（丢线时保持上一个值）
static uint8          asst_right[MT9V03X_H];

static float          err_last = 0.0f;                     // 上一拍偏差（图像丢失时保持）
static float          steer_prev_error = 0.0f;
static float          steer_d_filter = 0.0f;
static uint8          steer_pd_init = 0;


//===================================================================================================================
//  四、小工具
//===================================================================================================================

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

static void Camera_InitForever(void)
{
    uint32 cnt = 0;

    while(mt9v03x_init())
    {
        gpio_set_level(B9, GPIO_LOW);   system_delay_ms(60);
        gpio_set_level(B9, GPIO_HIGH);  system_delay_ms(120);
        gpio_set_level(B9, GPIO_LOW);   system_delay_ms(60);
        gpio_set_level(B9, GPIO_HIGH);  system_delay_ms(800);

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
    /* 正常模式每 10ms 扫描按键；停车按下即生效，发车在短按松开后生效。 */
    key_scanner();
    if(GPIO_LOW == gpio_get_level(C13))
    {
        run_switch = 0.0f;
        MotorPID_SetEnable(0);
    }
    else if(KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        run_switch = 1.0f;
    }
    key_clear_all_state();
#if USE_ISLAND
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
}

//-------------------------------------------------------------------------------------------------------------------
// 20ms：电机增量式 PID + 舵机 PD + 心跳灯
//-------------------------------------------------------------------------------------------------------------------
void pit_handler0(void)
{
#if SERVO_SETUP_MODE != SERVO_SETUP_RUN
    Motor_Stop();
    Servo_Steer(0);
    return;                                               // 即使误调用控制中断，也不接受发车/转向命令
#endif
    if((run_enable > 0.0f) && (run_switch > 0.5f))
    {
        float use_err;
        float u;
        float d;
        float target = run_speed;

        MotorPID_SetEnable(1);

#if USE_ISLAND
        /* 进环岛自动降速 */
        if(Island_State != 0)
        {
            target *= ISLAND_SLOW_RATIO;
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

        /* 大弯补偿：小误差基本不变，大误差额外加一点 */
        u += STEER_QUAD_K * use_err * ((use_err >= 0.0f) ? use_err : -use_err);

        if(u >  SERVO_U_LIMIT) { u =  SERVO_U_LIMIT; }
        if(u < -SERVO_U_LIMIT) { u = -SERVO_U_LIMIT; }

        /* 转角越大越降速，降低弯中轮胎和电机负载。 */
        {
            float abs_u = (u >= 0.0f) ? u : -u;
            if(abs_u > TURN_SHARP_LIMIT)      { target *= TURN_SLOW_SHARP; }
            else if(abs_u > TURN_MEDIUM_LIMIT){ target *= TURN_SLOW_MEDIUM; }
        }
        MotorPID_SetTarget(target);
        dbg_duty = MotorPID_Update(enc_l, enc_r);

        err_last = use_err;
        Servo_Steer((int)u);

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
        Servo_Steer(0);
    }
}


//===================================================================================================================
//  六、一帧图像处理
//===================================================================================================================

static void Frame_Process(void)
{
    int   i;
    uint8 line_mode;
    float wave[ASSISTANT_OSC_CH_MAX];

    /* 1. 巡线：方案由 LineDetect.h 的 LINE_SCHEME 决定（改宏后重新编译）*/
    line_mode = LINE_SCHEME;
    LineDetect_Process(mt9v03x_image[0], line_mode);

    /* 2. 环岛识别（会读/补边界，放在巡线之后、算偏差之前）*/
#if USE_ISLAND
    Island_Detect();
#endif

    /* 2.5 十字识别（环岛之后、算偏差之前）*/
    Cross_Check();

    /* 补线后统一统计；十字仍按补出的中线转向，出口有弯时能及时跟线。 */
    LineDetect_RefreshStats();

    /* 3. 算偏差（内部做滤波和 D 项）；失败时保持上一有效方向。 */
    if(!Err_Compute())
    {
        Img_Disappear_Flag = 1;
    }

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
    /* 5. 屏幕 */
    Screen_ShowFrame(mt9v03x_image[0], image_two_value[0], line_left, line_mid, line_right);
    Screen_ShowRefCol(LineDetect_GetRefCol());
    Screen_ShowLineMode((int)line_mode);
    Screen_ShowCross((int)Cross_Flag);
    Screen_ShowCrossDebug(Cross_GetRunLen());
    Screen_ShowStatus(Err, Island_State, (int)MotorPID_GetTarget());
    Screen_ShowServoAngle((int)Servo_GetAngle());
    Screen_ShowParam((int)run_switch, dbg_duty, (int)MotorPID_GetTarget(), (int)MotorPID_GetMeasured());
    Screen_ShowParamRx(Assistant_GetRxCount(), Assistant_GetLastChannel(), Assistant_GetLastValue());
    ips200_show_string(0, 224, "L:");
    ips200_show_int(24, 224, enc_l, 4);
    ips200_show_string(120, 224, "R:");
    ips200_show_int(144, 224, enc_r, 4);
    ips200_show_string(0, 272, "C15:RUN C13:STOP");
#endif

    /* 6. 发波形 + 边线 + 图像给逐飞助手 */
    wave[0] = ErrUse;
    wave[1] = (float)Servo_GetAngle();
    wave[2] = (float)enc_l;
    wave[3] = (float)enc_r;
    wave[4] = MotorPID_GetTarget();
    wave[5] = MotorPID_GetMeasured();
    wave[6] = (float)dbg_duty;
    wave[7] = (float)Search_Stop_Line;

    Assistant_SendFrame(mt9v03x_image[0], asst_left, asst_right, wave);
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

#if MOTOR_DIAGNOSTIC_MODE
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
        ips200_show_string(8, 56,  "PWM 50Hz CENTER 760");
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
        ips200_show_string(8, 8,   "SU400 LIMIT TEST V2");
        ips200_show_string(8, 32,  "MOTOR DISABLED");
        ips200_show_string(8, 56,  "K1:-5 K2:+5 K3:CENTER");
        ips200_show_string(8, 80,  "K4:RECORD + CENTER");
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
        ips200_show_string(8, 224, wireless_ok ? "WIRELESS OK" : "KEY CONTROL ONLY");
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
#elif SERVO_SETUP_MODE == SERVO_SETUP_CENTER
    /* 不初始化无线串口、摄像头、编码器或控制定时器；不依赖摄像头连线。 */
#if USE_SCREEN
    Screen_Init();
    ips200_clear();
    ips200_show_string(8, 16,  "S-U400 CENTER MODE");
    ips200_show_string(8, 48,  "MOTOR DISABLED");
    ips200_show_string(8, 80,  "PWM: 50 Hz");
    ips200_show_string(8, 112, "PULSE: 1520 us");
    ips200_show_string(8, 144, "DUTY: 760");
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

#if USE_SCREEN
    Screen_Init();
#endif

    /* 逐飞助手（无线串口）*/
    if(0 != Assistant_Init())
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

    /* 摄像头（失败会一直重试，B9 双闪报警）*/
    Camera_InitForever();

#if USE_ISLAND
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
        gpio_toggle_level(B9);   system_delay_ms(100);
        gpio_toggle_level(B9);   system_delay_ms(100);
    }

    /* 开中断 */
    key_init(10);
    pit_ms_init(PIT_CH0, 20);                               // 20ms：电机 + 舵机 PD
    pit_ms_init(PIT_CH1, 10);                               // 10ms：陀螺仪 + 编码器
    interrupt_global_enable(0);

    run_enable = 1.0f;

    if(1 == RUN_AUTO_START)
    {
        run_switch = 1.0f;
    }

    err_last = ErrUse;
    gpio_set_level(B9, GPIO_HIGH);

    while(1)
    {
        /* 1. 收逐飞助手参数 */
        Assistant_Poll();

        /* 2. 把新参数搬进全局变量（中断里读的就是这些）*/
        if(Assistant_GetUpdated(ASST_PARAM_SPEED))   { Assistant_ClearUpdated(ASST_PARAM_SPEED);   run_speed  = Assistant_GetParam(ASST_PARAM_SPEED); }
        if(Assistant_GetUpdated(ASST_PARAM_MOTOR_KP)){ Assistant_ClearUpdated(ASST_PARAM_MOTOR_KP);run_kp     = Assistant_GetParam(ASST_PARAM_MOTOR_KP); }
        if(Assistant_GetUpdated(ASST_PARAM_MOTOR_KI)){ Assistant_ClearUpdated(ASST_PARAM_MOTOR_KI);run_ki     = Assistant_GetParam(ASST_PARAM_MOTOR_KI); }
        if(Assistant_GetUpdated(ASST_PARAM_MOTOR_KD)){ Assistant_ClearUpdated(ASST_PARAM_MOTOR_KD);run_kd     = Assistant_GetParam(ASST_PARAM_MOTOR_KD); }
        if(Assistant_GetUpdated(ASST_PARAM_SERVO_P)) { Assistant_ClearUpdated(ASST_PARAM_SERVO_P); run_p      = Assistant_GetParam(ASST_PARAM_SERVO_P); }
        if(Assistant_GetUpdated(ASST_PARAM_SERVO_D)) { Assistant_ClearUpdated(ASST_PARAM_SERVO_D); run_d      = Assistant_GetParam(ASST_PARAM_SERVO_D); }
        if(Assistant_GetUpdated(ASST_PARAM_CONTRAST)){ Assistant_ClearUpdated(ASST_PARAM_CONTRAST);run_threshold = Assistant_GetParam(ASST_PARAM_CONTRAST); }
        if(Assistant_GetUpdated(ASST_PARAM_SWITCH))  { Assistant_ClearUpdated(ASST_PARAM_SWITCH);  run_switch = Assistant_GetParam(ASST_PARAM_SWITCH); }

        /* 3. 限幅 */
        if(run_speed  <   0.0f) { run_speed  =   0.0f; }
        if(run_speed  > 500.0f) { run_speed  = 500.0f; }
        if(run_kp     <   0.0f) { run_kp     =   0.0f; }
        if(run_kp     >  50.0f) { run_kp     =  50.0f; }
        if(run_ki     <   0.0f) { run_ki     =   0.0f; }
        if(run_ki     >   2.0f) { run_ki     =   2.0f; }
        if(run_kd     <   0.0f) { run_kd     =   0.0f; }
        if(run_kd     >  20.0f) { run_kd     =  20.0f; }
        if(run_p      <   0.0f) { run_p      =   0.0f; }
        if(run_p      >  20.0f) { run_p      =  20.0f; }
        if(run_d      <   0.0f) { run_d      =   0.0f; }
        if(run_d      >  50.0f) { run_d      =  50.0f; }
        if(run_threshold < -50.0f) { run_threshold = -50.0f; }
        if(run_threshold > 200.0f) { run_threshold = 200.0f; }
        if(run_switch <   0.5f) { run_switch =   0.0f; }
        else                    { run_switch =   1.0f; }

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
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            Frame_Process();
        }
    }
}
