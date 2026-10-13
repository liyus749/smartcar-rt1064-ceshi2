/*********************************************************************************************************************
* 文件名称          main.c
* 功能简介          逐飞 RT1064 主板 V3.1 + 核心板 —— 测试程序（4 个模式，一次只编译一个）
*
* ================================ 怎么用 ================================
*  1. 只改下面那个 TEST_MODE（一个数字），选一个模式
*  2. 编译 → 下载 → 按一下核心板上的 RST 键（想再跑一遍就再按 RST）
*  3. 程序跑到哪一步，看【核心板上的 B9 蓝灯】（灯语见下）
*
* ================================ B9 灯语 ================================
*   快闪 3 下            上电自检 —— 程序跑起来了
*   慢闪（0.4 秒）        正在连手机热点（只有模式 1 会）
*   中速闪（0.15 秒）     热点连上了，正在连电脑上的助手（只有模式 1 会）
*   双闪（快2下→停0.8s）  摄像头初始化失败，在重试（检查 CSI 排线）
*   一秒闪一下           正在驱动电机（模式 1 跑起来之后）
*   灭                   没在跑 / 已经停了
*
* ================================ 4 个模式 ================================
*   1 = 跑车（★ 平时就用这个）
*         图像巡线 + 舵机 PD + 电机 + 环岛
*         · 图像：直接看【屏幕】（本地显示，流畅，不占无线带宽）
*         · 波形：无线串口 -> VOFA+，8 个通道
*         · 调参：VOFA+ 滑动条绑定命令，发下来就能改
*              P1=%f!  -> 舵机 P
*              D1=%f!  -> 舵机 D
*              T1=%f!  -> 阈值偏置（可负）
*              V1=%f!  -> 电机速度
*              S1=%f!  -> 电机开关（0=停，1=跑）
*         · 8 个波形通道：Err / 舵机打角 / 左轮速 / 右轮速 /
*                          速度设定 / 视野 / 左丢线 / 右丢线
*
*   2 = 电机单项测试
*         不依赖图像 / 参数 / 网络，用来判断"电机 + 编码器"这条链路通不通
*
*   3 = 屏幕自检
*         验证 IPS 屏幕能不能显示（不依赖摄像头 / 网络）
*         ★ 屏幕没反应先看屏幕背面丝印，改 code/Screen.h 里的 IPS_SCREEN_TYPE
*
*   4 = VOFA+ 自检
*         验证"无线串口模块 + VOFA+ 上位机"这条线通不通
*         ★ 走无线串口模块（UART8，D16/D17，115200）
*
* ================================ 引脚对照（和主板丝印一致）================================
*   舵机           -> 舵机接口 1            信号 C30
*   编码器 左/右   -> 编码器 1 / 编码器 2    C0+C1 / C2+C24
*   电机           -> 电机驱动信号接口 2     左 C11(正)+C10(反)   右 D3(正)+D2(反)
*   姿态传感器     -> 姿态传感器接口         SCK C23 / MOSI C22 / MISO C21 / CS C20
*   摄像头         -> 摄像头 CSI 接口        PCLK B20 / VSY B22 / D0-D7 B31~B24 / SCCB C29+C28
*   屏幕           -> 屏幕接口               SCL B0 / SDA B1 / RST B2 / DC C19 / CS B3 / BL C18
*   无线串口模块   -> 无线模块接口           TX D16 / RX D17 / RTS D26   （UART8，115200）
*   WiFi SPI 模块  -> SPI 无线模块接口       SCK D12 / CS D13 / MOSI D14 / MISO D15 / RST B16 / INT B17
*
*   ⚠ 屏幕（并口屏）和 WiFi SPI 模块会抢引脚，只能二选一；
*     无线串口模块和它们都不冲突。
*
* ================================ 工程里 code/ 下的各文件 ================================
*   Camera.c     图像处理（大津法、二值化、找左右边线）    —— 学长写的，在用
*   Control.c    算图像偏差 Err_Sum()                     —— 学长写的，在用
*   Servo.c      舵机（Servo_Steer 打角）                 —— 在用
*   Motor.c      电机（Motor_Set 左/右）                  —— 在用
*   Screen.c     屏幕显示（图像 + 边线 + 状态）            —— 在用
*   Vofa.c       VOFA+ 通信（无线串口发波形 + 收参数调参） —— 在用
*   GYRO.c       陀螺仪                                   —— 环岛用
*   island.c     环岛识别（学长写的 8 状态机）             —— 在用
*   PID.c        学长的 PD_Camera / PID_L / PID_R         —— 目前没被调用（以后上速度环用）
*
* 【注意】本程序不使用蜂鸣器。
********************************************************************************************************************/

#include "zf_common_headfile.h"

//===================================================================================================================
//  选模式：一次只编译一个（详细说明见上面的文件头）
//
//  1=WiFi图传   2=舵机+调参   3=电机单项   4=陀螺仪+编码器   5=推车看巡线
//  6=测WiFi模块 7=低速跑车    8=舵机静态   9=参数回显
//===================================================================================================================
#define TEST_MODE               (1)


//-------------------------------------------------------------------------------------------------------------------
// 开机自检：B9 快闪 3 下
//
//   每个模式一启动都会先闪 3 下 —— 这是"程序真的跑起来了"最直接的证据，
//   不用串口、不用电脑，上电就能看见。
//   如果上电后 B9 一下都不闪，那就是：没供电 / 程序没下载进去 / 单片机没复位成功。
//-------------------------------------------------------------------------------------------------------------------
static void boot_led_self_test(void)
{
    uint8 i;

    gpio_init(B9, GPO, GPIO_HIGH, GPO_PUSH_PULL);       // B9 = 核心板蓝色 LED（低电平点亮）

    for(i = 0; i < 3; i++)
    {
        gpio_set_level(B9, GPIO_LOW);   system_delay_ms(120);
        gpio_set_level(B9, GPIO_HIGH);  system_delay_ms(120);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 摄像头曝光时间 —— 这是调"画面亮不亮、黑线清不清楚"的唯一入口
//
//   驱动默认值是 512。值越大画面越亮，越小越暗。
//     画面发白、赛道和背景分不开  -> 调小（400 / 300 / 200）
//     画面太暗、看不到远处        -> 调大（600 / 700 / 800）
//   注意：曝光时间太大还会降低帧率（官方驱动注释里明确写了），所以别一味调大。
//   改完重新编译下载即可，模式 1 和模式 5 都会用这个值。
//-------------------------------------------------------------------------------------------------------------------
#define CAM_EXPOSURE            (512)


//===================================================================================================================
//  公共部分：WiFi 连接（模式 1、4 用得到）
//
//  下面三个参数改成你自己的：
//    WIFI_SSID / WIFI_PASSWORD ：模块要连的那个 WiFi 热点（路由器或电脑/手机开的热点）
//    WIFI_TARGET_IP            ：电脑在这个热点里的 IP
//                                电脑开"移动热点"时，一般是 192.168.137.1
//                                用路由器的话，看电脑的无线网卡 IP（ipconfig 查）
//    WIFI_TARGET_PORT          ：逐飞助手里的端口，默认 8080，两边要一样
//===================================================================================================================
#if (1 == TEST_MODE)

#include "zf_device_wifi_spi.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"

#define WIFI_SSID               "longy-phone"           // 手机热点名字（已按你电脑实连的热点填好）
#define WIFI_PASSWORD           "1122334455"            // 手机热点密码
#define WIFI_TARGET_IP          "172.20.10.3"           // 电脑在热点里的 IP（已按你电脑当前 IP 填好）
#define WIFI_TARGET_PORT        "8086"                  // <- 和逐飞助手里填的端口一致（助手默认就是 8086）
#define WIFI_LOCAL_PORT         "6666"                  // <- 模块自己用的端口，和官方例程一致，不用改

//===================================================================================================================
//  逐飞助手 V2 适配层
//
//  为什么要这一层：逐飞助手 V1 和 V2 的通信协议不一样，单片机这边的库也换了。
//  V2 的写法是"结构体传参"（图像、边线、波形各自一个结构体），比较啰嗦；
//  这里包一层，让下面各个模式还是按简单写法调用。
//
//  V2 的边线格式和 V1 不同：每个点要同时给 x 和 y，所以这里先把 (横坐标, 行号) 组装成点对。
//===================================================================================================================
seekfree_assistant_camera_struct                    v2_camera;              // 图像通道
seekfree_assistant_camera_boundary_struct           v2_bound_left;          // 左边界通道
seekfree_assistant_camera_boundary_struct           v2_bound_mid;           // 中线通道
seekfree_assistant_camera_boundary_struct           v2_bound_right;         // 右边界通道
seekfree_assistant_camera_boundary_uint8_struct     v2_pt_left[MT9V03X_H];
seekfree_assistant_camera_boundary_uint8_struct     v2_pt_mid[MT9V03X_H];
seekfree_assistant_camera_boundary_uint8_struct     v2_pt_right[MT9V03X_H];
seekfree_assistant_oscilloscope_struct              v2_osc;                 // 示波器通道
seekfree_assistant_debug_param_struct               v2_param;               // 参数调试

const uint8 *sa_line_left  = 0;                     // 记住用户传进来的三个边线数组
const uint8 *sa_line_mid   = 0;
const uint8 *sa_line_right = 0;
uint8        sa_has_boundary = 0;                   // 1 = 配置过边线
uint8        sa_has_camera   = 0;                   // 1 = 配置过图像

// 参数数组照旧用老名字，下面的模式代码不用改
#define seekfree_assistant_parameter                (v2_param.data)
#define seekfree_assistant_parameter_update_flag    (v2_param.update_flag)

//-------------------------------------------------------------------------------------------------------------------
// 配置"图像"通道：img 传 NULL 表示只发边线、不发图像
//-------------------------------------------------------------------------------------------------------------------
void sa_camera_config(uint8 *img)
{
    seekfree_assistant_camera_config(&v2_camera, SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                     MT9V03X_W, MT9V03X_H, img);
    sa_has_camera = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 配置"三条边线"通道：把三个数组的地址记下来（每帧发之前会自动取最新值）
//-------------------------------------------------------------------------------------------------------------------
void sa_boundary_config(const uint8 *l, const uint8 *m, const uint8 *r)
{
    sa_line_left  = l;
    sa_line_mid   = m;
    sa_line_right = r;

    seekfree_assistant_camera_boundary_config(&v2_bound_left,  SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
                                              RGB565_BLUE,  MT9V03X_H, v2_pt_left);
    seekfree_assistant_camera_boundary_config(&v2_bound_mid,   SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
                                              RGB565_GREEN, MT9V03X_H, v2_pt_mid);
    seekfree_assistant_camera_boundary_config(&v2_bound_right, SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
                                              RGB565_RED,   MT9V03X_H, v2_pt_right);
    sa_has_boundary = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 发【三条边线】—— 每帧都可以发，数据量很小（3 x 120 个点，约 750 字节）
//-------------------------------------------------------------------------------------------------------------------
void sa_send_boundary(void)
{
    uint16 i;

    if(sa_has_boundary)
    {
        for(i = 0; i < MT9V03X_H; i++)
        {
            v2_pt_left[i].x  = sa_line_left  ? sa_line_left[i]  : 0;   v2_pt_left[i].y  = (uint8)i;
            v2_pt_mid[i].x   = sa_line_mid   ? sa_line_mid[i]   : 0;   v2_pt_mid[i].y   = (uint8)i;
            v2_pt_right[i].x = sa_line_right ? sa_line_right[i] : 0;   v2_pt_right[i].y = (uint8)i;
        }
        seekfree_assistant_camera_boundary_send(&v2_bound_left);
        seekfree_assistant_camera_boundary_send(&v2_bound_mid);
        seekfree_assistant_camera_boundary_send(&v2_bound_right);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 发【一整幅摄像头图像】—— 很重！一帧 188x120 = 22560 字节
//   千万不要每帧都调用！WiFi 发不完会把数据丢掉，助手收到的流就错位了，
//   表现就是图传页黑屏 / 卡死 / FPS 掉成 0。要分频发（见模式 1 里的 IMG_EVERY_N）。
//-------------------------------------------------------------------------------------------------------------------
void sa_send_image(void)
{
    if(sa_has_camera)
    {
        seekfree_assistant_camera_send(&v2_camera);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 发一帧（边线 + 图像）。数据量小的时候（只发边线）可以每帧调用。
//-------------------------------------------------------------------------------------------------------------------
void sa_send(void)
{
    sa_send_boundary();
    sa_send_image();
}

//-------------------------------------------------------------------------------------------------------------------
// 读"参数调试"下发的参数（结果放在 v2_param.data[] / update_flag[] 里）
//-------------------------------------------------------------------------------------------------------------------
void sa_param_read(void)
{
    seekfree_assistant_debug_param_analysis(&v2_param);
}

//-------------------------------------------------------------------------------------------------------------------
// 发一包波形：vals 是 float 数组，n 是通道数
//-------------------------------------------------------------------------------------------------------------------
void sa_osc_send(float *vals, uint8 n)
{
    seekfree_assistant_oscilloscope_config(&v2_osc, n, vals);
    seekfree_assistant_oscilloscope_send(&v2_osc);
}

//-------------------------------------------------------------------------------------------------------------------
// 所有模式通用：让测试信息能被你看见
//
//   你核心板没焊串口跳线，printf 看不到 —— 所以这里让日志走两条路：
//     printf               -> 调试串口（焊了跳线才看得到）
//     wifi_spi_send_string -> WiFi（逐飞助手【串口助手】页，通信方式选"网络"，不用焊跳线）
//-------------------------------------------------------------------------------------------------------------------
static uint8 net_ok = 0;                                // 1 = WiFi 和电脑都连上了

static void log_msg(const char *str)
{
    printf("%s", str);                                  // 走串口
    if(net_ok) { wifi_spi_send_string(str); }           // 走 WiFi
}



#if (1 == TEST_MODE)
//-------------------------------------------------------------------------------------------------------------------
// 摄像头初始化（一直重试到成功）—— 用摄像头的模式都调这个
//
//   失败时的报警节奏是【双闪】：快闪 2 下 -> 停 0.8 秒 -> 快闪 2 下 ...
//   为什么要用双闪：以前用的是"每 60ms 翻一下"，但 mt9v03x_init() 失败一次本身要花约 1 秒，
//   结果看起来和"连网络失败的慢闪"几乎一样，根本分不清是摄像头还是网络的问题。
//   现在双闪是独一无二的节奏，一眼就能认出是摄像头没起来。
//
//   网络已经连上的话，还会通过 WiFi 发一条文字到助手的串口助手页。
//-------------------------------------------------------------------------------------------------------------------
static void camera_init_forever(void)
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
#endif

#if (1 == TEST_MODE)
static void wifi_try_link(void)
{
    uint8 i;

    // 连手机热点：最多试 8 次
    for(i = 0; i < 8; i++)
    {
        if(0 == wifi_spi_init(WIFI_SSID, WIFI_PASSWORD)) { break; }
        gpio_toggle_level(B9);
        system_delay_ms(500);
    }
    if(8 <= i) { return; }

    // 连电脑上的 TCP Server：最多试 8 次
    for(i = 0; i < 8; i++)
    {
        if(0 == wifi_spi_socket_connect("TCP", WIFI_TARGET_IP, WIFI_TARGET_PORT, WIFI_LOCAL_PORT)) { break; }
        gpio_toggle_level(B9);
        system_delay_ms(500);
    }
    if(8 <= i) { return; }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    net_ok = 1;

    gpio_set_level(B9, GPIO_LOW);                       // 常亮一下 = 联网成功
    system_delay_ms(400);
    gpio_set_level(B9, GPIO_HIGH);
}

//-------------------------------------------------------------------------------------------------------------------
// 试着连一次网络（热点 + 电脑上的 TCP 服务），成功返回 1
//   给主循环做"断线自动重连"用：助手什么时候准备好，模块都能自己连上
//-------------------------------------------------------------------------------------------------------------------
static uint8 net_try_once(void)
{
    if(0 != wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        return 0;                                       // 热点都没连上
    }
    if(0 != wifi_spi_socket_connect("TCP", WIFI_TARGET_IP, WIFI_TARGET_PORT, WIFI_LOCAL_PORT))
    {
        return 0;                                       // 热点连上了，但电脑上的 TCP 服务没连上
    }
    return 1;
}
#endif      // 模式1（跑车）：用"尽力连+自动重连"的网络方式

#endif


//===================================================================================================================
//  模式 1：低速闭环跑车（图像巡线 + 舵机 PD + 电机）
//
//  做什么：
//    图像：大津法(+偏置) -> 二值化 -> 找左右边界 -> 算偏差 Err
//    舵机：用 Err 做 PD 打角（打角范围用你标定好的 79 / 61）
//    电机：速度由"第 1 个滑动条"给，跑 / 停由"第 5 个滑动条（电机开关）"控制，【没有自动停车】
//    图传：发【摄像头灰度原图 + 左边界 + 中线 + 右边界】，可在图传页边看边调
//
//  流程：上电 -> 连网络 -> 摄像头 -> B9 快闪 3 秒（把车摆到起点、车头摆正）
//        -> 【车不会自己跑】，先在助手【图传】页确认能看到图像、边线正常
//        -> 再把助手【参数调试】页"第 5 个滑动条（电机开关）"拖到 1 = 发车
//        停 / 跑：把第 5 个滑动条拖回 0 / 再拖到 1（车失控时就拖回 0）
//        速度：第 1 个滑动条，跑的时候也能随时改
//        想从头再来：按一下核心板 RST
//
//  ⚠ 第一次跑一定要：车放在直道起点、朝向正前方，人站在旁边随时能按住车
//===================================================================================================================
#if (1 == TEST_MODE)

#include "Motor.h"
#include "Servo.h"
#include "GYRO.h"                       // 陀螺仪（环岛状态机靠它的偏航角 FJ_Angle 判断进出）
#include "zf_device_imu660ra.h"
#include "Screen.h"                     // IPS 屏幕显示（图像 + 边线）
#include "Vofa.h"                       // VOFA+ 通信（无线串口：发波形 + 收参数）

extern volatile int   Left_Line[MT9V03X_H];
extern volatile int   Right_Line[MT9V03X_H];
extern uint8          image_two_value[MT9V03X_H][MT9V03X_W];
extern volatile float Err;
extern volatile uint8 Img_Disappear_Flag;

//---------------- 环岛（学长 island.c 的那套 8 状态机，直接用，不改他的文件） ----------------
extern void           Island_Detect(void);
extern volatile int   Island_State;         // 0 = 没进环岛；1~5 = 环岛里走到第几步
extern volatile int   Left_Island_Flag;     // 1 = 正在左环岛；2 = 刚出左环岛
extern volatile int   Right_Island_Flag;    // 1 = 正在右环岛；2 = 刚出右环岛
extern volatile float FJ_Angle;             // 陀螺仪偏航角（向左为正）
extern volatile int   Left_Lost_Time;       // 左边界丢线行数（入环判据之一）
extern volatile int   Right_Lost_Time;      // 右边界丢线行数
extern volatile int   Search_Stop_Line;     // 视野长度（最长白列）—— 右环岛入环判据之一

//---------------- 可调参数（只改这一段就行） ----------------
#define RUN_DUTY            (800)       // 电机速度【上电默认值】：800 = 占空比 800 = 8%
                                        //   这个值可以在助手【参数调试】第 1 条滑动条里随时改（0 ~ 2000）
                                        //   800 就是你觉得刚好能跑的那个速度，不用改代码
#define RAMP_MS             (2000)      // 起步软启动：2 秒慢慢加速，不会一下冲出去
#define RUN_AUTO_START      (0)         // 0 = 上电【不跑】，要你在助手里把"第 5 个滑动条"拖到 1 才跑 ←【现在用这个】
                                        // 1 = 上电倒计时结束后自动开始跑（不安全，别用）
                                        // 【没有自动停车】：车会一直跑，直到你把开关拖回 0、或者按 RST
#define RUN_P               (3.0f)      // 舵机 PD 的比例项（学长 PD_Camera 用的就是 3）
#define RUN_D               (20.0f)     // 舵机 PD 的微分项（学长 PD_Camera 用的就是 20）
                                        // 【PD 怎么调】
                                        // 车一直往一边打死 / 转圈 -> P 太大了，往下调
                                        // 车不跟着线走、切内道、压线跑出去 -> P 太小，往上加
                                        // 车左右摆（画龙） -> D 往上加，或者 P 往下降
                                        // 起始值就是学长的 3 / 20，一次只改一个数
#define RUN_OSTU_BIAS       (15)        // 阈值偏置（背景误判成白色就加大）
#define SEND_LINES_ONLY     (0)         // 0 = 发摄像头原图 + 边线【现在用这个】  1 = 只发边线（省带宽，但看不到图）

// 【图传分频】一整幅灰度图是 188x120 = 22560 字节，如果每帧都发，WiFi 根本发不完，
//   模块会把没发完的数据丢掉 -> 助手收到的数据流错位 -> 图传页黑屏/卡死。
//   所以：边线每帧都发（很轻），图像每 IMG_EVERY_N 帧才发一幅。
//   摄像头大概 50 帧/秒，所以 4 就是约 12 幅/秒；嫌卡就改成 6（约 8 幅/秒），想更流畅改成 3。
//   【注意】这个数越小图传越流畅的"假象"其实是丢数据，别改成 1 或 2。
#define IMG_EVERY_N         (4)

//===================================================================================================================
// 【环岛】—— 用的是学长 island.c 里的那套"8 状态机"，我不改他的文件，只是把它接进来
//
//   USE_ISLAND = 0 -> 关掉环岛，回到纯巡线（★ 现在用这个：先把巡线跑稳，再回来调环岛）
//   USE_ISLAND = 1 -> 启用环岛识别（等巡线稳了再打开）
//
//   学长这套是怎么工作的（看懂这个你才好调）：
//     入环判据（看图像特征）：一边的边界严重丢线 + 出现拐点 + 单调性/连续性变化
//     环内推进（看陀螺仪角度）：进环后清零角度，然后 FJ_Angle 转到 30° / 240° / 330° /
//                               358° 这几个坎，状态就一步一步往前推
//     出环条件：转够 360° 左右且两边都不丢线
//     ★ 所以【陀螺仪是这套状态机的命根子】——陀螺仪没起来，环岛会永远卡在状态 1
//
//   ⚠ 上电后有 0.3 秒陀螺仪零漂校准，这段时间车必须完全静止，否则角度会漂
//===================================================================================================================
#define USE_ISLAND          (1)
#define ISLAND_SLOW_RATIO   (0.6f)      // 进环岛自动降速的比例：0.6 = 降到 60%（学长是 105 -> 90）

//===================================================================================================================
// 【显示方式】用屏幕 还是 用 WiFi？—— 这两个可能抢引脚，先确认清楚
//
//   USE_SCREEN = 1 -> 用 IPS 屏幕显示图像+边线（库在 code/Screen.c）
//   USE_WIFI   = 1 -> 用 WiFi + 逐飞助手（图传 + 在线调参）
//
//   ★ 能不能同时开？看你的屏幕是哪种（型号在 code/Screen.h 的 IPS_SCREEN_TYPE 里改）：
//       SPI 屏 ：只占 B0/B1/B2/B3/C18/C19
//                -> 和 WiFi 模块(D12~D15/B16/B17) 不冲突，两个可以都设成 1
//       并口屏 ：占满屏幕接口 16 个脚，其中 D0~D7 就是 WiFi 模块的引脚
//                -> 【必须二选一】，同时开会互相干扰，两个都用不了
//
//   ⚠ 并口屏 + WiFi 同时插着用，不是"不能烧录"，而是两个外设抢同一批 IO，
//     结果就是屏幕花屏、WiFi 也连不上。用哪个就把另一个的开关设 0。
//===================================================================================================================
#define USE_SCREEN          (1)
#define USE_WIFI            (0)
//-------------------------------------------------------------------------------------------------------------------
// 显示模式：逐飞助手的三个页面【同一时间只能用一种】，所以"发边线"和"发波形"必须二选一！
//   0 = 发【边线】-> 在【图像传输】页看     （跑车、看巡线时用这个）
//   1 = 发【波形】-> 在【虚拟示波器】页看   （调参数时用这个）
//   ★ 现在可以在助手里用"第 6 条滑动条"当场切换，不用重新烧录了！
//     0 = 图传（看边线+补线）   1 = 示波器（看环岛状态）
//-------------------------------------------------------------------------------------------------------------------
#define RUN_VIEW_MODE       (0)
//-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 在线调参：助手里【参数调试】页现在有 6 条滑动条，顺序是固定的 —— 不要改顺序！
//
//   第 1 条 = 速度       量程 0 ~ 2000      默认 800   （直接就是占空比，800 = 8%）
//   第 2 条 = P          量程 0 ~ 20        默认 3
//   第 3 条 = D          量程 0 ~ 50        默认 20
//   第 4 条 = 阈值       量程 -50 ~ +50     默认 15
//            （这条可以直接填负数：负数 = 阈值调低，正数 = 阈值调高）
//   第 5 条 = 电机开关   0 = 停，1 = 跑     默认 0（上电是停的，等你在助手里发车）
//   第 6 条 = 视图       0 = 图传，1 = 示波器   默认 0
//            （调环岛时：0 看边线和补线，1 看 Island_State / 丢线数 / 陀螺仪角度）
//
//   起步软启动固定在 RAMP_MS = 2000（2 秒），不在助手里调。
//
//   注意：这些值存在内存里，一断电 / 按 RST 就回到上面写的默认值。
//         调好之后把满意的数值告诉我，我写进代码固定下来。
//-------------------------------------------------------------------------------------------------------------------
static volatile float run_duty   = (float)RUN_DUTY;         // 第 1 条滑动条：电机速度
static volatile float run_p      = RUN_P;                   // 第 2 条滑动条：舵机 P
static volatile float run_d      = RUN_D;                   // 第 3 条滑动条：舵机 D
static volatile float run_bias   = (float)RUN_OSTU_BIAS;    // 第 4 条滑动条：阈值偏置（可负）
static volatile float run_ramp   = (float)RAMP_MS;          // 起步软启动（固定值，不在助手调）
static volatile float run_switch = (float)RUN_AUTO_START;   // 第 5 条滑动条：电机开关（0=停，1=跑）
static volatile float run_view   = (float)RUN_VIEW_MODE;    // 第 6 条滑动条：视图（0=图传，1=示波器）
static volatile float run_enable = 0.0f;                    // 1 = 允许跑（上电由 RUN_AUTO_START 决定）
static uint8          run_was_on = 0;                       // 记录上一拍开关的状态
static uint16         net_retry  = 0;                       // 断线重连计数
static volatile int16 enc_l      = 0;                       // 左轮编码器：每 10ms 的计数（就是转速）
static volatile int16 enc_r      = 0;                       // 右轮编码器
static uint16         img_div    = 0;                       // 图传分频计数（每 IMG_EVERY_N 帧发一幅图）

static uint8 line_left[MT9V03X_H];      // 要发给助手的左边界（X 坐标）
static uint8 line_mid[MT9V03X_H];       // 中线
static uint8 line_right[MT9V03X_H];     // 右边界
static uint8 image_copy[MT9V03X_H][MT9V03X_W];

static volatile uint32 sys_ms    = 0;   // 系统时间，10ms 中断累加
static volatile uint32 isr_ms    = 0;   // 中断自己的计时（每次中断 +20ms），软启动用它
static float           err_last  = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// 10ms 中断：累加系统时间
//-------------------------------------------------------------------------------------------------------------------
void pit_handler1(void)
{
    sys_ms += 10;

    /* 陀螺仪：必须每 10ms 读一次、积分一次
       （GYRO.c 里的 DT = 0.01 就是按 10ms 写的，周期不对角度就会算错）
       环岛状态机靠 FJ_Angle 判断"转进环岛 / 转出环岛"——漏了这一步，环岛永远走不到状态 2 */
#if USE_ISLAND
    imu660ra_get_gyro();
    Get_Gyroscope_Angle();
#endif

    /* 读两个轮子的编码器：每 10ms 的计数就代表"这一小段时间转了多少"，也就是转速 */
    enc_l = (int16)(-(int32)encoder_get_count(QTIMER1_ENCODER1));   // 左轮
    encoder_clear_count(QTIMER1_ENCODER1);
    enc_r = (int16)( (int32)encoder_get_count(QTIMER1_ENCODER2));   // 右轮
    encoder_clear_count(QTIMER1_ENCODER2);
}

//-------------------------------------------------------------------------------------------------------------------
// 20ms 中断：舵机 PD + 电机输出（放在中断里，不受 USB/WiFi 发图快慢影响）
//-------------------------------------------------------------------------------------------------------------------
void pit_handler0(void)
{
    if((run_enable > 0.0f) && (run_switch > 0.5f))
    {
        int    duty = (int)run_duty;                        // 800 -> 占空比 800（8%）
        float  use_err;                                     // 这一拍实际拿来算 PD 的偏差
        float  d;
        float  u;

        isr_ms += 20;                                   // 中断自己计时（不依赖别的地方）

        /* 环岛里自动降速：进环以后（Island_State != 0）速度按比例压下来，出环自动恢复 */
#if USE_ISLAND
        if(Island_State != 0)
        {
            duty = (int)((float)duty * ISLAND_SLOW_RATIO);
        }
#endif

        /* 电机：软启动到 run_duty，然后一直保持（速度可以边跑边改） */
        if(isr_ms < (uint32)run_ramp)
        {
            duty = (int)((float)duty * (float)isr_ms / run_ramp);
        }
        Motor_Set(duty, duty);

        /* 舵机：PD 打角（Servo_Steer 内部会限幅到你标定的 79 / 61）
           u = P * 偏差 + D * (偏差 - 上一拍偏差)
           偏差 Err 来自图像中线（正 = 车偏了，负 = 车偏了另一边） */
        if(Img_Disappear_Flag)                          // 这帧没找到线：别乱甩，保持上一拍的偏差
        {
            use_err = err_last;
        }
        else
        {
            use_err = Err;
        }

        d        = use_err - err_last;                  // 偏差的变化量（微分项用）
        u        = run_p * use_err + run_d * d;

        if(u >  120.0f) { u =  120.0f; }                // 限幅：一次偏差再大也不会把舵机一把打死
        if(u < -120.0f) { u = -120.0f; }

        err_last = use_err;
        Servo_Steer((int)u);

        /* 心跳灯：每 500ms 翻转一次 B9 —— 灯在闪，就说明"驱动中断"确实在跑 */
        if(0 == (isr_ms % 500))
        {
            gpio_toggle_level(B9);
        }
    }
    else
    {
        isr_ms = 0;                                     // 停下时清零，下次启动重新计时
        Motor_Stop();
        Servo_Steer(0);
    }
}

int main(void)
{
    int i;
    int threshold;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    boot_led_self_test();                               // 上电先闪 3 下

    Motor_Init();
    Motor_Stop();
    Servo_Init();

    /* ---------- 编码器初始化（左轮=编码器1，右轮=编码器2）----------
       要在开中断之前初始化好，否则示波器里读不到两个轮子的转速 */
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0,  QTIMER1_ENCODER1_CH2_C1);    // 左轮
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2,  QTIMER1_ENCODER2_CH2_C24);   // 右轮

#if USE_WIFI
    // ---------- 连网络：尽力而为（最多试几次）----------
    //   连不上也照样能跑，只是看不到图 —— 这样网络有问题时你也能测车
    wifi_try_link();
#endif

#if USE_SCREEN
    // ---------- 屏幕 ----------
    //   ★ 放在摄像头【前面】：万一摄像头初始化失败，屏幕也已经是亮的，
    //     你至少能看到屏幕亮起、能判断"到底是屏幕问题还是摄像头问题"
    Screen_Init();
#endif

    // ---------- VOFA+ 无线串口（发波形 + 收参数）----------
    //   走无线串口模块（UART8，D16/D17），自动波特率，不用配模块
    //   上电看 B9 灯：常亮 0.5 秒 = 初始化成功；快闪 10 下 = 失败
    Vofa_Init();

    // ---------- 摄像头 ----------
    camera_init_forever();

#if USE_ISLAND
    // ---------- 陀螺仪（环岛状态机要用）----------
    //   1) 初始化 IMU660RA，最多试 5 次（失败了也继续跑，只是环岛不能识别，巡线不受影响）
    //   2) 零漂校准 0.3 秒 —— ★ 这段时间车必须完全静止 ★
    {
        uint8 gtry;
        for(gtry = 0; gtry < 5; gtry++)
        {
            if(0 == imu660ra_init()) { break; }
            system_delay_ms(200);
        }

        if(gtry >= 5)
        {
            log_msg("[gyro] imu660ra FAILED -- 环岛识别不可用，巡线照常\r\n");
        }
        else
        {
            log_msg("[gyro] imu660ra OK, 零漂校准中，车别动 ...\r\n");
            Zero_Point_Detect();                        // 100 次 x 3ms = 0.3 秒
            log_msg("[gyro] zero drift done\r\n");
            Clear_Gyroscope_Angle();                    // 从 0 度开始
        }
    }
#endif

    // ---------- 先把图传通道配好（这样网络一通就能直接发）----------
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
#if SEND_LINES_ONLY
    sa_camera_config(NULL);
#else
    sa_camera_config(image_copy[0]);
#endif
    sa_boundary_config(line_left, line_mid, line_right);

    // ---------- 上电提示：B9 快闪 1 秒（只是告诉你"程序起来了"，车不会跑）----------
    //   以前这里是 3 秒倒计时，因为那时是自动发车；现在发车由你在助手控制，不用等了
    for(i = 0; i < 5; i++)
    {
        gpio_toggle_level(B9);   system_delay_ms(100);
        gpio_toggle_level(B9);   system_delay_ms(100);
    }

    // ---------- 起步 ----------
    pit_ms_init(PIT_CH0, 20);                           // 20ms：舵机 PD + 电机
    pit_ms_init(PIT_CH1, 10);                           // 10ms：系统时间
    interrupt_global_enable(0);

    run_enable = 1.0f;                                  // 允许跑

    /* 什么时候自动把"电机开关"打开？
         RUN_AUTO_START=1 -> 上电倒计时结束就自动开（不推荐）
         RUN_AUTO_START=0 -> 上电一直是"停"，必须你在助手【参数调试】里
                             把"第 5 个滑动条（电机开关）"拖到 1 才发车   ← 现在就是这个
        【重要】不再"网络没连上就自动开"了 —— 那样车会在你还没看到图像时自己冲出去 */
    if(1 == RUN_AUTO_START)
    {
        run_switch = 1.0f;
    }

    err_last = Err;
    gpio_set_level(B9, GPIO_HIGH);                      // 熄灭；跑起来后由中断里的"心跳灯"接管

    while(1)
    {
#if USE_WIFI
        /* ---------- 网络没连上？每 10 秒自动重连一次（助手什么时候准备好都能连上）---------- */
        if(0 == net_ok)
        {
            net_retry++;
            if(net_retry >= 500)                        // 500 × 20ms = 10 秒（间隔长一点，少卡顿）
            {
                net_retry = 0;
                if(net_try_once())
                {
                    net_ok = 1;
                    log_msg("[net] tcp reconnected\r\n");
                }
            }
        }
#endif

        /* ---------- 在线调参：收电脑发来的参数（逐飞助手【参数调试】页）---------- */
        /* ---------- 在线调参：收 VOFA+ 滑动条发来的参数 ----------
           ★ 每圈都要调一次 Vofa_Poll()，不然无线串口的 64 字节缓冲会满、参数会丢 */
        Vofa_Poll();

        /* VOFA+ 里滑动条绑定的命令格式：【字母】【数字】=【数值】!
              P1=%f!   -> 舵机 P
              D1=%f!   -> 舵机 D
              T1=%f!   -> 阈值偏置（可以是负数）
              V1=%f!   -> 电机速度
              S1=%f!   -> 电机开关（0 = 停，1 = 跑）
           ★ 每收到一次参数 B9 灯闪一下 —— 这是"参数到底有没有传进来"的直接证据 */
        if(Vofa_ParamFlag['P'-'A'][1]) { Vofa_ParamFlag['P'-'A'][1] = 0; run_p      = Vofa_Param['P'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['D'-'A'][1]) { Vofa_ParamFlag['D'-'A'][1] = 0; run_d      = Vofa_Param['D'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['T'-'A'][1]) { Vofa_ParamFlag['T'-'A'][1] = 0; run_bias   = Vofa_Param['T'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['V'-'A'][1]) { Vofa_ParamFlag['V'-'A'][1] = 0; run_duty   = Vofa_Param['V'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['S'-'A'][1]) { Vofa_ParamFlag['S'-'A'][1] = 0; run_switch = Vofa_Param['S'-'A'][1]; gpio_toggle_level(B9); }

        /* 安全限幅：防止滑动条拖过头把车弄失控 */
        if(run_duty <    0.0f) { run_duty =    0.0f; }
        if(run_duty > 4000.0f) { run_duty = 4000.0f; }
        if(run_p    <    0.0f) { run_p    =    0.0f; }
        if(run_p    >   20.0f) { run_p    =   20.0f; }
        if(run_d    <    0.0f) { run_d    =    0.0f; }
        if(run_d    >   50.0f) { run_d    =   50.0f; }
        if(run_bias <  -50.0f) { run_bias =  -50.0f; }
        if(run_bias >   50.0f) { run_bias =   50.0f; }
        if(run_switch <  0.5f) { run_switch = 0.0f; }
        if(run_switch >= 0.5f) { run_switch = 1.0f; }

        /* ---------- 第 5 个滑动条（电机开关）：从"停"拨到"跑"的瞬间，清一下微分项（避免第一拍猛打）---------- */
        if((run_enable > 0.0f) && (run_switch > 0.5f) && (0 == run_was_on))
        {
            err_last = Err;
        }
        run_was_on = ((run_enable > 0.0f) && (run_switch > 0.5f)) ? 1 : 0;

        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            /* 1. 大津法 + 偏置 */
            threshold = (int)Camera_GetOSTU(mt9v03x_image) + (int)run_bias;
            if(threshold > 255) { threshold = 255; }
            if(threshold < 1)   { threshold = 1;   }

            /* 2. 二值化 */
            Image_Binarization(threshold);

            /* 3. 最长白列巡线：找左右边界 */
            Longest_White_Column();

            /* 3.5 环岛识别（学长的 8 状态机）
                   它可能会调用 Left_Add_Line/Right_Add_Line 修改左右边界来"补线"，
                   所以必须在 Longest_White_Column() 之后、Err_Sum() 之前调用 —— 这个
                   顺序和学长主程序里是一模一样的，顺序错了补线就不生效。 */
#if USE_ISLAND
            Island_Detect();
#endif

            /* 4. 算偏差（给舵机 PD 用）*/
            if(Img_Disappear_Flag == 0)
            {
                Err = Err_Sum();
            }

            /* 5. 把边界整理成要发送的三个数组（助手会给它们画出来）*/
            for(i = 0; i < MT9V03X_H; i++)
            {
                int l = Left_Line[i];
                int r = Right_Line[i];

                if(l < 0)             { l = 0; }
                if(l > MT9V03X_W - 1) { l = MT9V03X_W - 1; }
                if(r < 0)             { r = 0; }
                if(r > MT9V03X_W - 1) { r = MT9V03X_W - 1; }

                line_left[i]  = (uint8)l;
                line_right[i] = (uint8)r;
                line_mid[i]   = (uint8)((l + r) >> 1);
            }

#if USE_SCREEN
            /* 5.6 【屏幕】显示这一帧：图像 + 左/中/右三条边线 + 中心参考虚线
                   必须放在这里 —— 因为 line_left/line_mid/line_right 刚刚才填好 */
            Screen_ShowFrame(mt9v03x_image[0], image_two_value[0],
                             line_left, line_mid, line_right);
            Screen_ShowStatus(Err, Island_State, (int)run_duty);
#endif

#if !SEND_LINES_ONLY
            memcpy(image_copy, mt9v03x_image, MT9V03X_IMAGE_SIZE);
#endif

            /* 6. 发一包波形给 VOFA+（8 通道）
                  ★ 在 VOFA+ 里把这 8 条都勾上，重点是前两条：
                     Err 和 舵机实际打角 —— 调 PD 就看这两条 */
            {
                float osc[8];
                osc[0] = Err;                                   /* 图像偏差（调 PD 主要看这条）*/
                osc[1] = (float)Servo_GetAngle();               /* 舵机实际打角 */
                osc[2] = (float)enc_l;                          /* 左轮转速 */
                osc[3] = (float)enc_r;                          /* 右轮转速 */
                osc[4] = run_duty;                              /* 速度设定值 */
                osc[5] = (float)Search_Stop_Line;               /* 视野长度（图像健康）*/
                osc[6] = (float)Left_Lost_Time;                 /* 左丢线行数 */
                osc[7] = (float)Right_Lost_Time;                /* 右丢线行数 */
                Vofa_SendWaves(osc, 8);
            }
        }
    }
}


//===================================================================================================================
//  模式 2：电机测试
//
//  ⚠ 先把车架起来，让两个轮子悬空！下面每一步都会让轮子转。
//
//  接线：电机驱动模块的信号线插主板"电机驱动信号接口 2"
//        左电机 正转 C11 / 反转 C10，右电机 正转 D3 / 反转 D2
//        驱动模块的电源从主板"电机驱动电源接口"取（和电池电压一样，受主开关控制）
//
//  测试流程（一轮接一轮循环，按 RST 重新开始）：
//    左轮正转 -> 左轮反转 -> 右轮正转 -> 右轮反转 -> 双轮正转 -> 双轮反转
//    最后是从小到大加速，用来看"多大占空比车才动得起来"
//
//  每一步都会打印两路编码器的计数，怎么判断：
//    轮子转了  -> 编码器数字应该是几百上千
//    数字是负的 -> 转向反了，把 Motor.c 里的 MOTOR_LEFT_SIGN / MOTOR_RIGHT_SIGN 改成 -1
//    数字一直是 0 -> 编码器线没插好（或者插反了），电机没转也要先查驱动电源
//===================================================================================================================
#elif (2 == TEST_MODE)

#include "Motor.h"

#include "zf_device_wireless_uart.h"                // 无线串口模块：测试结果从这儿发到电脑

#define TEST_DUTY               (2000)              // 每个方向测试用的速度（10000 = 100%）
#define RUN_MS                  (1500)              // 每个方向转多久
#define IDLE_MS                 (2500)              // 每步之间停多久（这段时间可以手转轮子）

static volatile int32 enc_l_acc = 0;                // 左编码器累加值
static volatile int32 enc_r_acc = 0;                // 右编码器累加值

//-------------------------------------------------------------------------------------------------------------------
// 10ms 定时中断：只负责读编码器
//-------------------------------------------------------------------------------------------------------------------
void pit_handler1(void)
{
    enc_l_acc += (int32)(-(int32)encoder_get_count(QTIMER1_ENCODER1));
    encoder_clear_count(QTIMER1_ENCODER1);

    enc_r_acc += (int32)( (int32)encoder_get_count(QTIMER1_ENCODER2));
    encoder_clear_count(QTIMER1_ENCODER2);
}

void pit_handler0(void) { }

//-------------------------------------------------------------------------------------------------------------------
// 走一步：设占空比 -> 等一会儿 -> 停车 -> 打印这一步的编码器读数
//-------------------------------------------------------------------------------------------------------------------
static void motor_step(const char *name, int duty_l, int duty_r, uint32 ms)
{
    uint32 i;
    char   buf[96];

    enc_l_acc = 0;
    enc_r_acc = 0;

    Motor_Set(duty_l, duty_r);
    for(i = 0; i < (ms / 10); i++)
    {
        system_delay_ms(10);
    }
    Motor_Stop();

    sprintf(buf, "[test3] %s  dutyL=%d dutyR=%d -> encL=%d encR=%d\r\n",
            name, duty_l, duty_r, (int)enc_l_acc, (int)enc_r_acc);
    wireless_uart_send_string(buf);
}

int main(void)
{
    int duty;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    boot_led_self_test();                               // 上电先闪 3 下 = 程序跑起来了

    // ---------- 编码器 ----------
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);    // 编码器 1 = 左
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);   // 编码器 2 = 右

    // ---------- 电机 ----------
    Motor_Init();

    // ---------- 开定时中断 ----------
    pit_ms_init(PIT_CH1, 10);
    interrupt_global_enable(0);

    // ---------- 无线串口模块：结果发到电脑（用 VOFA+ 或串口助手看）----------
    //   ★ 这个模式【完全不依赖 WiFi】，用的是无线串口模块（UART8，D16/D17，115200）
    wireless_uart_init();

    wireless_uart_send_string("\r\n==================================================\r\n");
    wireless_uart_send_string("[test3] 电机 + 编码器 单项测试\r\n");
    wireless_uart_send_string("[test3] !! 先把车架起来，轮子悬空 !!\r\n");
    wireless_uart_send_string("[test3] 下面的结果会发到这里（电脑端看这个窗口）\r\n");

    system_delay_ms(1000);

    while(1)
    {
        wireless_uart_send_string("\r\n---- new cycle ----\r\n");

        motor_step("LEFT  forward",  TEST_DUTY,  0,          RUN_MS);
        system_delay_ms(IDLE_MS);

        motor_step("LEFT  reverse", -TEST_DUTY,  0,          RUN_MS);
        system_delay_ms(IDLE_MS);

        motor_step("RIGHT forward",  0,          TEST_DUTY,  RUN_MS);
        system_delay_ms(IDLE_MS);

        motor_step("RIGHT reverse",  0,         -TEST_DUTY,  RUN_MS);
        system_delay_ms(IDLE_MS);

        motor_step("BOTH  forward",  TEST_DUTY,  TEST_DUTY,  RUN_MS);
        system_delay_ms(IDLE_MS);

        motor_step("BOTH  reverse", -TEST_DUTY, -TEST_DUTY,  RUN_MS);
        system_delay_ms(IDLE_MS);

        // ---------- 从小到大加速：看看多大占空比车才动得起来 ----------
        wireless_uart_send_string("[test3] ramp up: find the minimum duty that moves the car\r\n");
        for(duty = 500; duty <= 4000; duty += 250)
        {
            motor_step("BOTH  ramp", duty, duty, 600);
        }

        // ---------- 休息一会儿，再从头来 ----------
        wireless_uart_send_string("[test3] rest 5s, then repeat\r\n");
        Motor_Stop();
        system_delay_ms(5000);
    }
}


//===================================================================================================================
//  模式 3：屏幕自检 —— 专门验证"屏幕到底能不能用"
//
//  ★ 不依赖摄像头 / WiFi / 陀螺仪 / 电机，摄像头坏了也能跑
//  流程（每步停 0.7 秒，循环）：
//    ① 全屏刷 红 -> 绿 -> 蓝 -> 白 -> 黑    （能刷出颜色 = 屏幕接线驱动全通）
//    ② 显示字符串/整数/浮点 + 当前屏幕类型  （验字库）
//    ③ 显示一张"模拟摄像头画面"（渐变 + 黑线 + 左中右三条彩线）
//       —— 位置和真正显示摄像头图像时一样，就是你以后调车看到的样子
//
//  屏幕没反应怎么办：
//    1) 看屏幕【背面丝印】：带 SPI -> Screen.h 用 IPS200_TYPE_SPI
//                           没 SPI -> 用 IPS200_TYPE_PARALLEL8   （改完重新编译）
//    2) 检查 3V3 / GND 有没有插对位置
//    3) 背光都不亮 = 供电问题；背光亮但没内容 = 屏幕类型选错了
//
//  详细实现都在 code/Screen.c 的 Screen_SelfTest() 里
//===================================================================================================================
#elif (3 == TEST_MODE)

#include "Screen.h"

void pit_handler0(void) { }
void pit_handler1(void) { }

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    boot_led_self_test();               // B9 快闪 3 下 = 程序跑起来了

    Screen_Init();                      // 屏幕初始化（内部会显示 "screen init OK"）
    system_delay_ms(1000);

    Screen_SelfTest();                  // 死循环，画面全在 Screen.c 里

    while(1) { }
}




//===================================================================================================================
//  模式 4：VOFA+ 自检 —— 专门验证"和 VOFA+ 上位机的这条线通不通"
//
//  ★ 不依赖摄像头 / 电机 / WiFi / 屏幕
//
//  硬件接线（这一步必须做，否则没数据）：
//      主板 "UART1 接口"（5V / TXD / RXD）  <->  USB-TTL 模块  <->  电脑
//      注意 TX/RX 要交叉：  主板 TXD -> USB-TTL 的 RXD
//                            主板 RXD <- USB-TTL 的 TXD
//                            GND 一定要接（共地）
//      （核心板 Type-C 口看不到 printf，串口跳线没焊，所以必须走主板的 UART1 接口）
//
//  VOFA+ 那边怎么配：
//      1. 左下角选【串口】-> 选 USB-TTL 的 COM 口 -> 115200 -> 连接
//      2. 协议引擎选【JustFloat】
//      3. 拖一条滑动条验证"电脑->单片机"：
//         左栏【命令】新建一条，模式 Str，内容  P=%f
//         右边加【滑动条】控件，右键 -> 绑定命令 -> 选 P=%f，把 from/to 设成 -100 / 100
//
//  ★ 怎么判断成功：
//      ① 看到 3 条波形在动（正弦/二倍频/锯齿）  -> 单片机 -> 电脑 通了
//      ② 拖滑动条，第 4 条线跟着动               -> 电脑 -> 单片机 也通了
//
//  详细实现都在 code/Vofa.c 的 Vofa_SelfTest() 里
//===================================================================================================================
#elif (4 == TEST_MODE)

#include "Vofa.h"

void pit_handler0(void) { }
void pit_handler1(void) { }

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();                       // ★ 必须：初始化 UART1(B12/B13) 115200

    boot_led_self_test();               // B9 快闪 3 下 = 程序跑起来了

    Vofa_SelfTest();                    // 死循环：发波形 + 收参数，全在 Vofa.c 里

    while(1) { }
}




#endif
