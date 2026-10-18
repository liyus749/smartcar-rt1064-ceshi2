/*********************************************************************************************************************
* 文件名称          main.c
* 功能简介          智能车主程序：MT9V03X 对比度巡线 + 舵机 PD + 电机增量式 PID + 环岛 + 逐飞助手 V2.0
*
* 硬件              逐飞 RT1064 核心板 + 逐飞 RT1064 V3.1 主板 + 并口 IPS 屏 + 无线串口模块
*
* ================================ 整体流程 ================================
*   上电 -> 初始化外设 -> 摄像头 -> 陀螺仪 -> 开 20ms/10ms 中断 -> 主循环
*
*   主循环里只做两件事：
*     1. 收逐飞助手下发的参数（速度 / PID / 舵机 PD / 对比度阈值 / 开关）
*     2. 摄像头出一帧 -> Contrast 巡线 -> 环岛识别 -> 算偏差 -> 屏幕 -> 发波形/图像
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
*   电机         -> 电机驱动接口 2    左 C11(正)+C10(反)  右 D3(正)+D2(反)
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
#include "Err.h"


//===================================================================================================================
//  一、编译期配置
//===================================================================================================================

#define CAM_EXPOSURE        (512)      // 摄像头曝光：发白/反光调小(400/350)，远处太暗调大(600+)

// 舵机 PD 默认值（可在逐飞助手通道 4/5 在线改）
#define SERVO_P_DEFAULT     (3.0f)
#define SERVO_D_DEFAULT     (0.0f)

// 电机速度环默认值（可在逐飞助手通道 0/1/2/3 在线改）
#define SPEED_DEFAULT       (MOTORPID_DEFAULT_TARGET)
#define SPEED_KP_DEFAULT    (MOTORPID_DEFAULT_KP)
#define SPEED_KI_DEFAULT    (MOTORPID_DEFAULT_KI)
#define SPEED_KD_DEFAULT    (MOTORPID_DEFAULT_KD)

#define RUN_AUTO_START      (0)        // 1 = 上电就自动跑；0 = 上电停着，在助手通道 7 打开开关再发车

#define USE_SCREEN          (1)        // 1 = 用屏幕显示图像+边线；0 = 不用
#define USE_ISLAND          (1)        // 1 = 启用环岛识别；0 = 只巡线
#define ISLAND_SLOW_RATIO   (0.6f)     // 进环岛后目标速度降到 60%

// 舵机 PD 输出限幅（偏差再大也不会一把打死）
#define SERVO_U_LIMIT       (120.0f)


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
static volatile float run_threshold = CONTRAST_RATIO_DEFAULT;
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
    if((run_enable > 0.0f) && (run_switch > 0.5f))
    {
        float use_err;
        float u;
        float target = run_speed;

        MotorPID_SetEnable(1);

#if USE_ISLAND
        /* 进环岛自动降速 */
        if(Island_State != 0)
        {
            target *= ISLAND_SLOW_RATIO;
        }
#endif
        MotorPID_SetTarget(target);
        dbg_duty = MotorPID_Update(enc_l, enc_r);

        /* 舵机 PD：u = P*偏差 + D*(偏差变化量)
           图像丢失时保持上一拍，不乱甩 */
        if(Img_Disappear_Flag)
        {
            use_err = err_last;
        }
        else
        {
            use_err = ErrUse;
        }

        u = run_p * use_err + run_d * ErrDFrame;

        if(u >  SERVO_U_LIMIT) { u =  SERVO_U_LIMIT; }
        if(u < -SERVO_U_LIMIT) { u = -SERVO_U_LIMIT; }

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

    /* 1. 巡线：助手通道 7 选择方案（1 = 对比度，2 = 分段大津法）*/
    line_mode = LINE_SCHEME;
    LineDetect_Process(mt9v03x_image[0], line_mode);

    /* 2. 环岛识别（会读/补边界，放在巡线之后、算偏差之前）*/
#if USE_ISLAND
    Island_Detect();
#endif

    /* 3. 算偏差（内部做滤波和 D 项）*/
    Err_Compute();

    /* 4. 整理三条线给屏幕 + 助手边线（丢线时保持上一个值，避免线飞到边上）*/
    {
        int last_l = 0;
        int last_r = MT9V03X_W - 1;

        for(i = MT9V03X_H - 1; i >= 0; i--)
        {
            int l = Left_Line[i];
            int r = Right_Line[i];
            int visible = (i >= (MT9V03X_H - Search_Stop_Line)) ? 1 : 0;

            if((!visible) || (1 == Left_Lost_Flag[i]) || (1 == Right_Lost_Flag[i]))
            {
                line_left[i]  = 255;
                line_mid[i]   = 255;
                line_right[i] = 255;
            }
            else
            {
                if(l < 0)             { l = 0; }
                if(l > MT9V03X_W - 1) { l = MT9V03X_W - 1; }
                if(r < 0)             { r = 0; }
                if(r > MT9V03X_W - 1) { r = MT9V03X_W - 1; }

                line_left[i]  = (uint8)l;
                line_right[i] = (uint8)r;
                line_mid[i]   = (uint8)((l + r) >> 1);
            }

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
    Screen_ShowStatus(Err, Island_State, (int)MotorPID_GetTarget());
    Screen_ShowParam((int)run_switch, dbg_duty, (int)MotorPID_GetTarget(), (int)MotorPID_GetMeasured());
    Screen_ShowParamRx(Assistant_GetRxCount(), Assistant_GetLastChannel(), Assistant_GetLastValue());
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

    /* 编码器（开中断前初始化）*/
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0,  QTIMER1_ENCODER1_CH2_C1);    // 左轮
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2,  QTIMER1_ENCODER2_CH2_C24);   // 右轮

    /* 控制/巡线/偏差模块 */
    MotorPID_Init();
    Contrast_Init();
    LineDetect_Init();
    Err_Init();

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
