/*********************************************************************************************************************
* 文件名称          main.c
* 功能简介          智能车主程序：MT9V03X 摄像头巡线 + 前瞻加权偏差 + 舵机 PD + 电机 + 环岛
* 硬件              逐飞 RT1064 核心板 + 逐飞 RT1064 V3.1 主板 + 并口 IPS 屏 + 无线串口模块
*
* ================================ 怎么用 ================================
*   1. 编译下载，按一下核心板上的 RST 键就开跑（想再跑一遍就再按 RST）
*   2. 【上电就自动开始跑】，速度默认 1000（占空比 10%），2 秒软启动
*         想停     -> 在 VOFA+ 里把 S1 拖到 0
*         想继续   -> 把 S1 拖回 1
*         想改速度 -> 拖 V1（跑的时候也能改，立刻生效）
*   3. 看图像   -> 看【屏幕】（本地显示，流畅，不占无线带宽）
*      看波形/调参 -> 无线串口模块 -> 电脑上的 VOFA+
*
* ================================ B9 蓝灯灯语 ================================
*   快闪 3 下                 上电自检：程序跑起来了
*   常亮 0.5 秒               无线串口初始化成功（能和 VOFA 通信）
*   快闪 10 下                无线串口初始化失败（检查模块插好没有）
*   双闪（快 2 下 -> 停 0.8 秒）  摄像头初始化失败，正在重试（检查 CSI 排线）
*   一秒闪一下                正在跑（控制中断的心跳）
*   灭                        没在跑 / 已经停了
*
* ================================ VOFA+ 怎么配 ================================
*   串口：选无线串口模块那个 COM 口，波特率用你之前能出波形的那套设置
*   数据引擎：JustFloat
*
*   波形 15 个通道（从下往上数，第 1 条在最下面）：
*      1 = Err             图像偏差【前瞻加权后的值】
*      2 = 舵机实际打角    Servo_GetAngle()
*      3 = 左轮转速        编码器（计数/10ms，只观察）
*      4 = 右轮转速        编码器（计数/10ms，只观察）
*      5 = 占空比设定(V1)  开环占空比 0~4000
*      6 = 视野长度        突然变小 = 看不到路了
*      7 = 左侧丢线行数
*      8 = 右侧丢线行数
*      9 = 电机开关(S1)    0 = 停，1 = 跑
*     10 = 中断里实际输出的占空比
*     11 = 电脑发来的字节数（只涨不跌）    ★ 拖滑动条它不动 = 电脑没发东西
*     12 = 成功收到的参数条数（只涨不跌）  ★ 在涨 = 参数确实进到单片机了
*     13 = 心跳（锯齿 0~999）              ★ 在爬 = 单片机和上位机都正常
*     14 = 滤波后的偏差（= 真正送进 PD 的那个值）
*     15 = 这一帧参与算偏差的有效行数（太小 = 边线找得不好）
*
*   调参命令（滑动条绑定，格式：字母 + 数字 + "=" + 数值 + "!"）：
*      P1=%f!  -> 舵机 P           0 ~ 20
*      D1=%f!  -> 舵机 D           0 ~ 50
*      T1=%f!  -> 阈值偏置（可负）  -50 ~ +50
*      F1=%f!  -> Err 滤波强度      0 ~ 0.9（0 = 关滤波）
*      R1=%f!  -> 远块权重          0 ~ 5（0 = 完全不看远处）
*      R2=%f!  -> 中块权重          0 ~ 5
*      R3=%f!  -> 近块权重          0 ~ 5（★默认 1 / 2 / 2）
*      C1=%f!  -> Err 偏置          -50 ~ 50（把直道零点归零：显示 -10 就把 C1 设成 -10，即 C1 = 你看到的 Err）
*      V1=%f!  -> 电机占空比        0 ~ 4000（800 = 8%，★平时只调这一个）
*      S1=%f!  -> 电机开关          0 = 停，1 = 跑
*
* ================================ 接线（和主板丝印一致）================================
*   舵机           -> 舵机接口 1              信号 C30
*   编码器 左/右   -> 编码器 1 / 编码器 2     C0+C1 / C2+C24
*   电机           -> 电机驱动信号接口 2       左 C11(正)+C10(反)   右 D3(正)+D2(反)
*   摄像头         -> 摄像头 CSI 接口          PCLK B20 / VSY B22 / D0-D7 B31~B24 / SCCB C29+C28
*   屏幕           -> 屏幕接口                 SCL B0 / SDA B1 / RST B2 / DC C19 / CS B3 / BL C18
*   姿态传感器     -> 姿态传感器接口           SCK C23 / MOSI C22 / MISO C21 / CS C20
*   无线串口模块   -> 无线模块接口             TX D16 / RX D17 / RTS D26（UART8）
*
* ================================ 工程里 code/ 下的文件 ================================
*   Camera.c     图像处理（大津法、二值化、找左右边界）—— 学长写的
*   island.c     环岛识别（学长写的 8 状态机）
*   Control.c / PID.c   学长的偏差、舵机、电机、PID 函数（本程序没调用，保留未删）
*   Servo.c      舵机（Servo_Steer）
*   Motor.c      电机（Motor_Set）
*   Screen.c     屏幕显示（图像 + 边线 + 状态）
*   Vofa.c       VOFA+ 通信（无线串口：发波形 + 收参数）
*   GYRO.c       陀螺仪角度积分（环岛用）
*
* 【注意】本程序不使用蜂鸣器。
********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "Motor.h"
#include "Servo.h"
#include "Screen.h"                     // IPS 屏幕显示（图像 + 边线 + 状态）
#include "Vofa.h"                       // VOFA+ 通信（无线串口：发波形 + 收参数）
#include "GYRO.h"                       // 陀螺仪角度（环岛状态机用）
#include "zf_device_imu660ra.h"


//===================================================================================================================
//  一、可调参数（改这里就行）—— 标 ★ 的可以在 VOFA+ 里在线改
//===================================================================================================================

#define CAM_EXPOSURE        (512)       // 摄像头曝光：画面发白/反光就调小(400/350)，远处太暗就调大(600+)

//-------------------------------------------------------------------------------------------------------------------
// 【电机】—— 回到原来的方案：V1 直接就是占空比，不用编码器闭环
//
//   V1 = 0 ~ 4000（10000 = 100%，所以 800 = 8%，1000 = 10%）
//   S1 = 1 开跑，= 0 立刻停
//   起步有 2 秒软启动；进环岛自动降到 60%
//   屏幕第 3 行 Spd 显示的就是 V1，Out 显示实际输出的占空比
//-------------------------------------------------------------------------------------------------------------------
#define RUN_SPEED           (800)       // ★V1  上电默认占空比：800 = 8%

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
#define RAMP_MS             (2000)      // 起步软启动：2 秒慢慢加速，不会一下冲出去
#define RUN_AUTO_START      (1)         // 1 = 上电就自动跑（★现在用这个）；0 = 上电停着，等你在 VOFA 里按 S1

#define RUN_P               (3.0f)      // ★P1  舵机 PD 的比例项（值越大打得越狠）
                                        //   ⚠ 换成前瞻偏差以后 Err 的大小变了，P 要重新标：
                                        //     推车进弯看 Err 峰值，P ≈ 40 / Err峰值（比如峰值 25 -> P 从 1.6 开始）
#define RUN_D               (0.0f)      // ★D1  舵机 PD 的微分项（值越大越"踩刹车"，压摆动）
                                        //   ⚠ 建议从 0 开始，每次只加一点点；D 太大反而会"打过头再反打"
#define RUN_OSTU_BIAS       (15)        // ★T1  阈值偏置（-50 ~ +50；背景被误判成白色就加大）

#define ERR_FILTER          (0.35f)     // ★F1  Err 低通滤波强度：0 = 不滤波，0.3~0.5 低速好用
#define TH_SMOOTH_K         (0.35f)     // 大津法阈值的平滑强度（固定值，不用在线调）

//-------------------------------------------------------------------------------------------------------------------
// 【阈值】分带大津法 —— 解决"有些地方背景被误识别成赛道"
//
//   为什么要分带：大津法（全局）假设"整幅图里只有背景和赛道两种东西"。
//     但你的画面上部一大片是背景（视野外），它会主导直方图，把阈值算偏；
//     而且远处暗、近处亮，一个大津法阈值根本照顾不过来。
//
//   现在的做法：把 100 行切成 4 段（每段 25 行），每段自己算一次大津法、自己做平滑，
//     再统一加 T1 偏置。这样"远处暗"由远处的段自己适应，"上面的背景"也不会污染下面的段。
//     代价：几乎没有（统计的像素总数不变，只是分成 4 个直方图）。
//
//   USE_BAND_OTSU = 1 -> 分带大津法（★ 现在用这个）
//   USE_BAND_OTSU = 0 -> 回到学长的全局大津法（想对比效果时用）
//-------------------------------------------------------------------------------------------------------------------
#define USE_BAND_OTSU       (1)
#define OTSU_BANDS          (4)         // 分几段（100 行 / 4 = 每段 25 行）

//-------------------------------------------------------------------------------------------------------------------
// 【偏差：远 / 中 / 近 三段分块加权】
//
//   做法：
//     1. 先从画面里找出"看得见赛道的那一段"：从最远一行（100 − 视野长度）到最下面一行；
//     2. 把这一段【等分成三块】：远块（上 1/3）、中块（中间 1/3）、近块（下 1/3）；
//     3. 每一块里，把"两边边线都有效"的行的中线取平均，得到这一块的偏差；
//     4. 三块的偏差按 R1 / R2 / R3 加权平均，就是最终 Err。
//
//   所以 R1/R2/R3 就是你要的"远处占多少、中间占多少、近处占多少"：
//       R1 = 1, R2 = 1, R3 = 1  -> 三段一样重
//       R1 = 0                   -> 完全不看远处（只看中+近）
//       R1 = 1, R2 = 2, R3 = 2   -> 中近为主（★默认，适合小弯多、低速）
//       R1 = 2, R2 = 2, R3 = 1   -> 重视远景（提速、缓弯用）
//     ★ 权重是相对比例，不是百分比：1/2/2 和 0.5/1/1 效果完全一样。
//
//   丢线处理：某一行只要有一侧边线是"丢线"状态，这一行就不参与；整块有效行太少也整块跳过。
//   连续性：从近往远扫，某一行中线如果和"上一行有效的"差太多（> ERR_SPIKE_LIMIT），
//           判定为假点，跳过（防止背景里误匹配出来的边线把偏差带飞）。
//-------------------------------------------------------------------------------------------------------------------
#define ZONE_FAR_DEFAULT    (1.0f)      // ★R1  远块权重
#define ZONE_MID_DEFAULT    (2.0f)      // ★R2  中块权重
#define ZONE_NEAR_DEFAULT   (2.0f)      // ★R3  近块权重
#define ZONE_W_MAX          (5.0f)      // 单个权重上限
#define ZONE_MIN_ROWS       (3)         // 一块里至少要有几行有效数据才算数

#define ERR_D_LIMIT         (20.0f)     // D 项用的"偏差变化量"限幅（像素/帧），防止某一帧抖动把舵机甩飞

#define ERR_SPIKE_LIMIT     (15)        // 一行中线和"左右相邻两行平均"差超过这么多像素 -> 当坏行丢掉

#define USE_SCREEN          (1)         // 1 = 用并口屏显示图像+边线；0 = 不用屏幕（省时间）
#define USE_ISLAND          (1)         // 1 = 启用环岛识别（学长的 island.c）；0 = 关掉，只巡线
#define ISLAND_SLOW_RATIO   (0.6f)      // 进环岛后自动降速的比例：0.6 = 降到 60%


//===================================================================================================================
//  二、学长写的模块里的全局变量（Camera.c / island.c / Control.c）
//===================================================================================================================

extern volatile int   Left_Line[MT9V03X_H];     // 左边界（每行一个 x 坐标）
extern volatile int   Right_Line[MT9V03X_H];    // 右边界
extern int            Left_Lost_Flag[MT9V03X_H];    // 1 = 这一行左边没找到边界（丢线）
extern int            Right_Lost_Flag[MT9V03X_H];   // 1 = 这一行右边没找到边界（丢线）
extern uint8          image_two_value[MT9V03X_H][MT9V03X_W];    // 二值化后的图（0x00 黑 / 0xFF 白）
volatile float        Err = 0.0f;               // 图像偏差（我们自己算，放在这里；island.c 也会读它）
extern volatile uint8 Img_Disappear_Flag;       // 1 = 这一帧"图像丢了"

extern volatile int   Search_Stop_Line;         // 视野长度（最长白列有多少行）
extern volatile int   Left_Lost_Time;           // 左边丢线行数
extern volatile int   Right_Lost_Time;          // 右边丢线行数

extern void           Island_Detect(void);      // 环岛识别（学长写的状态机）
extern volatile int   Island_State;             // 0 = 没进环岛；1~5 = 环岛里走到第几步


//===================================================================================================================
//  三、全局状态
//===================================================================================================================

static volatile float run_speed    = (float)RUN_SPEED;       // V1：占空比（0~4000）
static volatile float run_p       = RUN_P;                  // P1：舵机 P
static volatile float run_d       = RUN_D;                  // D1：舵机 D
static volatile float run_bias    = (float)RUN_OSTU_BIAS;   // T1：阈值偏置（可负）
static volatile float run_errfilt = (float)ERR_FILTER;      // F1：Err 低通滤波强度（0 = 关）
static volatile float run_w_far    = ZONE_FAR_DEFAULT;       // R1：远块权重
static volatile float run_w_mid    = ZONE_MID_DEFAULT;       // R2：中块权重
static volatile float run_w_near   = ZONE_NEAR_DEFAULT;      // R3：近块权重
static volatile float run_err_bias = 0.0f;                   // C1：Err 偏置（把直道零点归零用）
static volatile float run_switch  = (float)RUN_AUTO_START;  // S1：电机开关（0 = 停，1 = 跑）

static volatile float run_enable  = 0.0f;                   // 1 = 允许跑（上电后由 main 置 1）
static const    float run_ramp    = (float)RAMP_MS;         // 起步软启动时长（固定）

static volatile int16 enc_l       = 0;                      // 左轮转速（每 10ms 的编码器计数）
static volatile int16 enc_r       = 0;                      // 右轮转速
static volatile int   dbg_duty    = 0;                      // 中断里实际输出的占空比（波形第 10 条 / 屏幕 Out）
static uint16         hb_cnt      = 0;                      // 心跳灯计数（25 x 20ms = 500ms）
static volatile int   dbg_valid_rows = 0;                    // 这一帧参与算偏差的有效行数（波形第 15 条）
static volatile int   dbg_zone_far  = 0;                     // 远块起始行（给屏幕画分界线用）
static volatile int   dbg_zone_mid  = 0;                     // 中块起始行
static volatile float err_use     = 0.0f;                   // 真正送给舵机 PD 的偏差（= 滤波后的 Err）
static volatile float err_d_frame = 0.0f;                   // 这一帧的偏差变化量（D 项直接用这个，见 Frame_Process）
static volatile uint16 wave_cnt   = 0;                      // 心跳计数：每发一包波形 +1
static volatile uint32 isr_ms     = 0;                      // 控制中断自己的计时（软启动用）

static float          err_last    = 0.0f;                   // 上一拍的偏差（微分项用）
static float          err_use_prev = 0.0f;                  // 上一帧滤波后的偏差（算帧间变化量用）
static uint8          run_was_on  = 0;                      // 上一拍电机开关的状态（用来抓"刚发车"那一拍）

/* ---- 阈值：每一段一个"平滑后的阈值" ---- */
static float          th_smooth[OTSU_BANDS];                // 每个分段平滑后的阈值
static uint8          th_band[OTSU_BANDS];                  // 每个分段最终用的阈值（平滑值 + T1 偏置）
static uint32         otsu_hist[256];                       // 大津法用的灰度直方图（放全局，省栈）

/* ---- 压"偏差抖动"用的状态 ---- */
static float          err_filt_val = 0.0f;                  // 滤波后的偏差（内部）
static uint8          err_filt_init = 0;                    // 第一次拿到有效 Err 时直接赋值，别慢慢爬

/* ---- 给屏幕画的三个数组 ---- */
static uint8          line_left[MT9V03X_H];
static uint8          line_mid[MT9V03X_H];
static uint8          line_right[MT9V03X_H];


//===================================================================================================================
//  四、小工具
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开机自检：B9 快闪 3 下
// 备注信息     每个程序一启动都会闪 3 下 —— 这是"程序真的跑起来了"最直接的证据，
//              不用串口、不用电脑，上电就能看见。
//              如果上电后 B9 一下都不闪，那就是：没供电 / 程序没下载进去 / 复位没成功。
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
// 函数简介     打一行日志（走调试串口，核心板焊了串口跳线才看得到；没有也不影响运行）
//-------------------------------------------------------------------------------------------------------------------
static void log_msg(const char *str)
{
    printf("%s", str);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头初始化（失败就重试，直到成功才返回）
// 备注信息     失败时的报警节奏是【双闪】：快闪 2 下 -> 停 0.8 秒 -> 快闪 2 下 ...
//              为什么要用双闪：mt9v03x_init() 失败一次本身要花约 1 秒，
//              用"慢闪"会和"无线串口失败的快闪 10 下"混在一起，分不清是哪个问题。
//-------------------------------------------------------------------------------------------------------------------
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
//  五、偏差：远 / 中 / 近 三段分块加权（说明见文件头"一、可调参数"）
//
//  流程：
//    1. 找出看得见赛道的行：[100 − 视野长度, 99]
//    2. 这一段等分成三块：远块（上 1/3）、中块、近块（下 1/3）
//    3. 每一行算中线；丢线的行不用；和上下相邻两行平均差太多的"孤立假点"也不用
//    4. 每块内部取平均偏差，三块再按 R1/R2/R3 加权合成 -> Err
//===================================================================================================================
static int   zone_mid[MT9V03X_H];                  /* 每一行的中线 */
static uint8 zone_ok[MT9V03X_H];                   /* 这一行能不能用 */

static uint8 Err_Zone(float *err_out)
{
    int   farthest;
    int   span;
    int   far_end;
    int   mid_end;
    int   row;
    int   z;
    int   v;
    int   cnt[3];
    float sum[3];
    float w[3];
    float total_e = 0.0f;
    float total_w = 0.0f;

    /* ---------- 1. 看得见赛道的那一段 ---------- */
    farthest = MT9V03X_H - Search_Stop_Line;
    if(farthest < 0)             { farthest = 0; }
    if(farthest > MT9V03X_H - 2) { farthest = MT9V03X_H - 2; }

    span = MT9V03X_H - farthest;
    if(span < 9)                 { return 0; }          /* 看得太少，这帧不算 */

    far_end = farthest + span / 3;                      /* 远块：[farthest, far_end) */
    mid_end = farthest + (span * 2) / 3;                /* 中块：[far_end, mid_end) 近块：[mid_end, 99] */

    dbg_zone_far = far_end;
    dbg_zone_mid = mid_end;

    /* ---------- 2. 取每一行的中线和有效性 ---------- */
    for(row = farthest; row < MT9V03X_H; row++)
    {
        zone_mid[row] = (Left_Line[row] + Right_Line[row]) >> 1;
        zone_ok[row]  = ((0 == Left_Lost_Flag[row]) && (0 == Right_Lost_Flag[row])) ? 1 : 0;
    }

    /* ---------- 3. 去掉"孤立的假点"----------
       一行中线如果和"上下相邻两行的平均"差太多（> ERR_SPIKE_LIMIT），就是假点
       （典型情况：右边界在背景里匹配到假线，一下跑到画面边上）。
       和邻居平均比而不是和某一行比：正常的弯道里中线是平滑变化的，不会误伤。 */
    for(row = farthest + 1; row < MT9V03X_H - 1; row++)
    {
        int avg;
        int d;

        if(0 == zone_ok[row])                        { continue; }
        if((0 == zone_ok[row-1]) || (0 == zone_ok[row+1])) { continue; }

        avg = (zone_mid[row-1] + zone_mid[row+1]) >> 1;
        d   = zone_mid[row] - avg;

        if((d > ERR_SPIKE_LIMIT) || (d < -ERR_SPIKE_LIMIT)) { zone_ok[row] = 0; }
    }

    /* ---------- 4. 分块累加 ---------- */
    cnt[0] = cnt[1] = cnt[2] = 0;
    sum[0] = sum[1] = sum[2] = 0.0f;
    dbg_valid_rows = 0;

    for(row = farthest; row < MT9V03X_H; row++)
    {
        if(0 == zone_ok[row]) { continue; }

        if     (row < far_end) { z = 0; }               /* 远块 */
        else if(row < mid_end) { z = 1; }               /* 中块 */
        else                   { z = 2; }               /* 近块 */

        v = zone_mid[row];

        sum[z] += (float)((MT9V03X_W / 2) - v);
        cnt[z]++;
        dbg_valid_rows++;
    }

    /* ---------- 5. 每块取平均，再按权重合成 ---------- */
    w[0] = run_w_far;
    w[1] = run_w_mid;
    w[2] = run_w_near;

    for(z = 0; z < 3; z++)
    {
        if(cnt[z] < ZONE_MIN_ROWS) { continue; }        /* 这块有效行太少，跳过 */
        if(w[z] <= 0.001f)         { continue; }        /* 权重为 0 = 不看这块 */

        total_e += w[z] * (sum[z] / (float)cnt[z]);
        total_w += w[z];
    }

    if(total_w < 0.001f) { return 0; }                  /* 三块都没用上：这帧不算 */

    *err_out = (total_e / total_w) - run_err_bias;      /* C1：零点偏置 */
    return 1;
}


//===================================================================================================================
//  六、定时中断（isr.c 会调用 pit_handler0 / pit_handler1，函数名不能改）
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 10ms 中断：读陀螺仪 + 读编码器（编码器只用来观察，不参与控制）
//-------------------------------------------------------------------------------------------------------------------
void pit_handler1(void)
{
    /* 陀螺仪：必须每 10ms 读一次、积分一次
       （GYRO.c 里的 DT = 0.01 就是按 10ms 写的，周期不对角度就会算错）
       环岛状态机靠角度判断"转进环岛 / 转出环岛"，漏了这一步环岛永远走不到下一步 */
#if USE_ISLAND
    imu660ra_get_gyro();
    Get_Gyroscope_Angle();
#endif

    /* 编码器：每 10ms 的计数就代表"这一小段时间转了多少"，只给屏幕/波形看 */
    enc_l = (int16)(-(int32)encoder_get_count(QTIMER1_ENCODER1));   // 左轮
    encoder_clear_count(QTIMER1_ENCODER1);
    enc_r = (int16)( (int32)encoder_get_count(QTIMER1_ENCODER2));   // 右轮
    encoder_clear_count(QTIMER1_ENCODER2);
}

//-------------------------------------------------------------------------------------------------------------------
// 20ms 中断：电机输出 + 舵机 PD
//
//   电机：V1 = 占空比（0~4000），左右轮给同一个值 → Motor_Set(duty, duty)
//         （编码器只用来在屏幕/波形上观察转速，不参与控制）
//   舵机：P1/D1 的 PD，用"滤波后偏差"和"每帧算好的偏差变化量"
//
//   放在中断里算，控制周期就不受图像处理、屏幕刷新的快慢影响
//-------------------------------------------------------------------------------------------------------------------
void pit_handler0(void)
{
    if((run_enable > 0.0f) && (run_switch > 0.5f))      /* 开关打开着 = 跑 */
    {
        int   duty = (int)run_speed;                    /* ★ V1 就是占空比（0~4000） */
        float use_err;
        float d;
        float u;

        isr_ms += 20;

#if USE_ISLAND
        /* 进环岛自动降速 */
        if(Island_State != 0)
        {
            duty = (int)((float)duty * ISLAND_SLOW_RATIO);
        }
#endif

        /* 起步软启动：2 秒内占空比从 0 线性加到 V1 */
        if(isr_ms < (uint32)run_ramp)
        {
            duty = (int)((float)duty * (float)isr_ms / run_ramp);
        }

        dbg_duty = duty;
        Motor_Set(duty, duty);                          /* 左右同占空比（原来的方案）*/

        /* 舵机 PD： u = P * 偏差 + D * (偏差 - 上一拍偏差)
           偏差来自图像（正 = 车偏了，负 = 车偏了另一边），Servo_Steer 内部会限幅 */
        if(Img_Disappear_Flag)                          // 这帧没找到线：别乱甩，保持上一拍的偏差
        {
            use_err = err_last;
        }
        else
        {
            use_err = err_use;                          // ★ 用【滤波后】的偏差，不是原始 Err
        }

        d = err_d_frame;                                // ★ 用【每帧算好】的偏差变化量（见 Frame_Process 第 6.5 步）
        u = run_p * use_err + run_d * d;

        if(u >  120.0f) { u =  120.0f; }                // 限幅：偏差再大也不会把舵机一把打死
        if(u < -120.0f) { u = -120.0f; }

        err_last = use_err;
        Servo_Steer((int)u);

        /* 心跳灯：每 500ms 翻一下 B9（25 x 20ms）—— 灯在闪，就说明控制中断确实在跑 */
        if(++hb_cnt >= 25)
        {
            hb_cnt = 0;
            gpio_toggle_level(B9);
        }
    }
    else                                                /* 开关关着 = 停 */
    {
        isr_ms   = 0;
        dbg_duty = 0;
        hb_cnt   = 0;
        Motor_Stop();
        Servo_Steer(0);
    }
}


//===================================================================================================================
//  六点五、阈值与二值化（分带大津法）
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对图像的某一段行做一次大津法，返回阈值
// 参数说明     row_begin / row_end   要统计的行范围（含两端）
// 返回参数     uint8                 这一段的阈值
// 备注信息     算法和大津法标准做法一样：遍历所有可能的阈值，找"类间方差最大"的那个。
//              差别只是统计范围限定在指定的行里（这样各段互不干扰）。
//-------------------------------------------------------------------------------------------------------------------
static uint8 Otsu_Rows(uint8 row_begin, uint8 row_end)
{
    uint16 i;
    uint16 j;
    uint32 amount        = 0;
    uint32 pix_back      = 0;
    uint32 integral      = 0;
    uint32 integral_back = 0;
    int32  pix_fore;
    int32  integral_fore;
    float  omega_back;
    float  omega_fore;
    float  micro_back;
    float  micro_fore;
    float  sigma;
    float  sigma_max = -1.0f;
    uint8  th        = 0;
    uint8  min_v;
    uint8  max_v;

    for(j = 0; j < 256; j++) { otsu_hist[j] = 0; }

    for(i = row_begin; i <= row_end; i++)
    {
        for(j = 0; j < MT9V03X_W; j++)
        {
            otsu_hist[mt9v03x_image[i][j]]++;
        }
    }

    for(min_v = 0;    (min_v < 255) && (0 == otsu_hist[min_v]); min_v++) { }
    for(max_v = 255;  (max_v > min_v) && (0 == otsu_hist[max_v]); max_v--) { }

    if(max_v <= (uint8)(min_v + 1)) { return max_v; }   /* 这一段的灰度几乎只有一两种 */

    for(j = min_v; j <= max_v; j++)
    {
        amount   += otsu_hist[j];
        integral += otsu_hist[j] * j;
    }

    for(j = min_v; j < max_v; j++)
    {
        pix_back += otsu_hist[j];
        if(0 == pix_back) { continue; }

        pix_fore = (int32)amount - (int32)pix_back;
        if(pix_fore <= 0) { break; }

        integral_back += otsu_hist[j] * j;
        integral_fore  = (int32)integral - (int32)integral_back;

        omega_back = (float)pix_back / (float)amount;
        omega_fore = (float)pix_fore / (float)amount;
        micro_back = (float)integral_back / (float)pix_back;
        micro_fore = (float)integral_fore / (float)pix_fore;

        sigma = omega_back * omega_fore * (micro_back - micro_fore) * (micro_back - micro_fore);
        if(sigma > sigma_max)
        {
            sigma_max = sigma;
            th        = (uint8)j;
        }
    }

    return th;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     算阈值 + 二值化（一次搞定）
// 备注信息     分带模式下：每段算自己的大津法 -> 时间上平滑 -> 加 T1 偏置 -> 按行用对应段的阈值二值化。
//              结果写进 image_two_value[][]:  IMG_WHITE = 赛道，IMG_BLACK = 背景/黑线
//-------------------------------------------------------------------------------------------------------------------
static void Threshold_And_Binarize(void)
{
    uint16 row;
    uint16 col;
    uint8  b;
    uint8  row_begin;
    uint8  row_end;
    int    th;

#if USE_BAND_OTSU
    for(b = 0; b < OTSU_BANDS; b++)
    {
        row_begin = (uint8)(b * (MT9V03X_H / OTSU_BANDS));
        row_end   = (uint8)(row_begin + (MT9V03X_H / OTSU_BANDS) - 1);

        {
            float otsu = (float)Otsu_Rows(row_begin, row_end);

            if(th_smooth[b] < 1.0f)
            {
                th_smooth[b] = otsu;                        /* 第一帧直接赋值，别慢慢爬 */
            }
            th_smooth[b] += TH_SMOOTH_K * (otsu - th_smooth[b]);

            th = (int)th_smooth[b] + (int)run_bias;         /* ★ T1 偏置照样起作用 */
            if(th > 255) { th = 255; }
            if(th < 1)   { th = 1;   }
            th_band[b] = (uint8)th;
        }
    }

    /* 按行用"它所在那一段"的阈值二值化 */
    for(row = 0; row < MT9V03X_H; row++)
    {
        th = th_band[row / (MT9V03X_H / OTSU_BANDS)];

        for(col = 0; col < MT9V03X_W; col++)
        {
            image_two_value[row][col] = (mt9v03x_image[row][col] >= th) ? IMG_WHITE : IMG_BLACK;
        }
    }
#else
    /* 对比用：学长的全局大津法 */
    {
        float otsu = (float)Camera_GetOSTU(mt9v03x_image);

        if(th_smooth[0] < 1.0f)
        {
            th_smooth[0] = otsu;
        }
        th_smooth[0] += TH_SMOOTH_K * (otsu - th_smooth[0]);

        th = (int)th_smooth[0] + (int)run_bias;
        if(th > 255) { th = 255; }
        if(th < 1)   { th = 1;   }
    }
    Image_Binarization(th);
#endif
}


//===================================================================================================================
//  七、一帧图像处理（摄像头出一帧就调一次）
//     流程：阈值+二值化 -> 巡线找边界 -> 环岛识别 -> 算分块偏差 -> 滤波 -> 屏幕 -> 波形
//===================================================================================================================
static void Frame_Process(void)
{
    int i;

    /* ---------- 1. 算阈值 + 二值化（分带大津法，详见上面 Threshold_And_Binarize）----------
       每 30 行一段、各算各的大津法、各自做时间平滑，再统一加 T1 偏置。 */
    Threshold_And_Binarize();

    /* ---------- 2. 最长白列巡线：找左右边界 ---------- */
    Longest_White_Column();

    /* ---------- 3. 环岛识别（学长的 8 状态机）----------
       它可能会调用 Left_Add_Line/Right_Add_Line 来"补线"，
       所以必须放在巡线之后、算偏差之前 —— 顺序和学长主程序里一样，顺序错了补线不生效。 */
#if USE_ISLAND
    Island_Detect();
#endif

    /* ---------- 4. 算偏差（远/中/近 三段分块加权）---------- */
    if(Img_Disappear_Flag == 0)
    {
        float e;

        if(Err_Zone(&e))                                /* 返回 0 = 这帧有效数据太少 */
        {
            Err = e;
        }
    }
    if(Err != Err) { Err = err_last; }                  /* 兜底：万一是 NaN，退回上一拍 */

    /* ---------- 5. Err 低通滤波（PD 用的是滤波后的 err_use）----------
        - F1 = 0    ：不滤波，原样送给 PD（提速以后用这个）
        - F1 = 0.35 ：默认，压掉约一半的帧间抖动，转向稍微迟钝一点点 */
    if(run_errfilt > 0.01f)
    {
        if(0 == err_filt_init)
        {
            err_filt_val  = Err;                        /* 第一次直接赋值 */
            err_filt_init = 1;
        }
        err_filt_val += run_errfilt * (Err - err_filt_val);
        err_use       = err_filt_val;
    }
    else
    {
        err_filt_init = 0;
        err_filt_val  = Err;
        err_use       = Err;
    }

    /* ---------- 6. D 项用的"偏差变化量"：每帧算一次 ----------
       为什么要每帧算：偏差是【每帧】才更新一次的，而舵机 PD 是在【20ms 中断】里跑的，
       两者不同步。如果直接在中断里做"这一拍减上一拍"：
         有时两拍之间是同一帧 -> 差值是 0（D 项白给）
         有时一拍之间跨了两帧 -> 差值翻倍（D 项猛一下）
       表现出来就是"一进弯猛打一下、然后反打"。所以统一改成【每帧】算一次差值，
       中断里直接取用；再限幅，防止某一帧的抖动把舵机一把甩出去。 */
    err_d_frame  = err_use - err_use_prev;
    err_use_prev = err_use;
    if(err_d_frame >  ERR_D_LIMIT) { err_d_frame =  ERR_D_LIMIT; }
    if(err_d_frame < -ERR_D_LIMIT) { err_d_frame = -ERR_D_LIMIT; }

    /* ---------- 7. 整理成三条线（给屏幕画）---------- */
    for(i = 0; i < MT9V03X_H; i++)
    {
        int l = Left_Line[i];
        int r = Right_Line[i];

        /* ★ 两种行不画线（画 255 = 屏幕那边会跳过，留空）：
             ① 视野以上的行（i < 100 - 视野长度）：这些行根本没扫过，
                Left_Line/Right_Line 还是初始化时的 0 和 187 ——
                以前会把线画到屏幕最左边/最右边，看起来就像"右边线飞到一边去了"，其实那是假的。
             ② 这一行有边线丢了（Left_Lost_Flag/Right_Lost_Flag = 1）：丢线时边界会被顶到
                屏幕边缘（2 或 185），画出来同样是"飞到边上"。留空反而能一眼看出"这里丢线了"。 */
        if((i < (MT9V03X_H - Search_Stop_Line)) ||
           (1 == Left_Lost_Flag[i]) || (1 == Right_Lost_Flag[i]))
        {
            line_left[i]  = 255;
            line_mid[i]   = 255;
            line_right[i] = 255;
            continue;
        }

        if(l < 0)             { l = 0; }
        if(l > MT9V03X_W - 1) { l = MT9V03X_W - 1; }
        if(r < 0)             { r = 0; }
        if(r > MT9V03X_W - 1) { r = MT9V03X_W - 1; }

        line_left[i]  = (uint8)l;
        line_right[i] = (uint8)r;
        line_mid[i]   = (uint8)((l + r) >> 1);
    }

#if USE_SCREEN
    /* ---------- 8. 屏幕：图像 + 三条边线 + 状态 ---------- */
    Screen_ShowFrame(mt9v03x_image[0], image_two_value[0], line_left, line_mid, line_right);
    Screen_ShowStatus(Err, Island_State, (int)run_speed);           /* 屏幕第 3 行：占空比设定 */
    Screen_ShowParam((int)run_switch, dbg_duty, Vofa_RxBytes, Vofa_CmdOk);
    /* 两条黄线 = 分块边界：上面是远块/中块的分界，下面是中块/近块的分界 */
    Screen_ShowProspect((uint8)dbg_zone_far, (uint8)dbg_zone_mid);
#endif

    /* ---------- 9. 发一包波形给 VOFA+（15 通道，顺序见文件头）---------- */
    {
        float osc[15];

        osc[0]  = Err;                                  /* 1  图像偏差（远中近分块加权后的值）*/
        osc[1]  = (float)Servo_GetAngle();              /* 2  舵机实际打角 */
        osc[2]  = (float)enc_l;                         /* 3  左轮实测（编码器计数/10ms）*/
        osc[3]  = (float)enc_r;                         /* 4  右轮实测（编码器计数/10ms）*/
        osc[4]  = run_speed;                            /* 5  占空比设定（V1）*/
        osc[5]  = (float)Search_Stop_Line;              /* 6  视野长度 */
        osc[6]  = (float)Left_Lost_Time;                /* 7  左侧丢线行数 */
        osc[7]  = (float)Right_Lost_Time;               /* 8  右侧丢线行数 */
        osc[8]  = run_switch;                           /* 9  电机开关状态 */
        osc[9]  = (float)dbg_duty;                      /* 10 中断里实际输出的占空比 */
        osc[10] = (float)Vofa_RxBytes;                  /* 11 电脑发来的字节数（只涨不跌）*/
        osc[11] = (float)Vofa_CmdOk;                    /* 12 成功收到的参数条数（只涨不跌）*/
        osc[12] = (float)(wave_cnt % 1000);             /* 13 心跳（锯齿）*/
        osc[13] = err_use;                              /* 14 滤波后的偏差（PD 用的就是它）*/
        osc[14] = (float)dbg_valid_rows;                /* 15 这一帧参与算偏差的有效行数 */

        Vofa_SendWaves(osc, 15);
        wave_cnt++;
    }
}


//===================================================================================================================
//  八、主程序
//===================================================================================================================
int main(void)
{
    int i;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    boot_led_self_test();                               // B9 快闪 3 下 = 程序跑起来了

    /* ---------- 执行机构 ---------- */
    Motor_Init();
    Motor_Stop();
    Servo_Init();                                       // 内部会回中，并等 0.5 秒

    /* ---------- 编码器（左轮 = 编码器 1，右轮 = 编码器 2）----------
       要在开中断之前初始化好，否则读不到两个轮子的转速 */
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0,  QTIMER1_ENCODER1_CH2_C1);    // 左轮
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2,  QTIMER1_ENCODER2_CH2_C24);   // 右轮

#if USE_SCREEN
    /* ---------- 屏幕（放在摄像头前面）----------
       万一摄像头初始化失败，屏幕也已经是亮的，你至少能判断"是屏幕问题还是摄像头问题" */
    Screen_Init();
#endif

    /* ---------- 无线串口（VOFA+ 发波形 + 收参数）----------
       上电看 B9：常亮 0.5 秒 = 成功；快闪 10 下 = 失败 */
    Vofa_Init();

    /* ---------- 摄像头（失败会一直重试，B9 双闪报警）---------- */
    Camera_InitForever();

#if USE_ISLAND
    /* ---------- 陀螺仪（环岛状态机要用）----------
       1) 初始化 IMU660RA，最多试 5 次（失败了也继续跑，只是环岛不能识别，巡线不受影响）
       2) 零漂校准 0.3 秒 —— ★ 这段时间车必须完全静止 ★ */
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

    /* ---------- 上电提示：B9 快闪 1 秒（车这时还不会动）---------- */
    for(i = 0; i < 5; i++)
    {
        gpio_toggle_level(B9);   system_delay_ms(100);
        gpio_toggle_level(B9);   system_delay_ms(100);
    }

    /* ---------- 开中断 ---------- */
    pit_ms_init(PIT_CH0, 20);                           // 20ms：电机 + 舵机 PD
    pit_ms_init(PIT_CH1, 10);                           // 10ms：陀螺仪 + 编码器
    interrupt_global_enable(0);

    run_enable = 1.0f;                                  // 允许跑

    if(1 == RUN_AUTO_START)
    {
        run_switch = 1.0f;                              // 上电就开跑（★ 想改成"手动发车"就把 RUN_AUTO_START 设 0）
    }

    err_last = Err;
    gpio_set_level(B9, GPIO_HIGH);                      // 熄灭；跑起来之后由中断里的心跳灯接管

    while(1)
    {
        /* ---------- 收电脑发来的参数（每圈都要调，不然无线串口的 64 字节缓冲会满）---------- */
        Vofa_Poll();

        /* 命令格式：【字母】【数字】=【数值】!
              P1 舵机P    D1 舵机D    T1 阈值偏置（可负）
              F1 Err滤波  R1/R2/R3 远中近三块权重  C1 Err偏置
              V1 电机占空比
              S1 电机开关（0 = 停，1 = 跑）
           ★ 每收到一次参数 B9 闪一下 —— 这是"参数到底有没有传进来"最直接的证据 */
        if(Vofa_ParamFlag['P'-'A'][1]) { Vofa_ParamFlag['P'-'A'][1] = 0; run_p       = Vofa_Param['P'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['D'-'A'][1]) { Vofa_ParamFlag['D'-'A'][1] = 0; run_d       = Vofa_Param['D'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['T'-'A'][1]) { Vofa_ParamFlag['T'-'A'][1] = 0; run_bias    = Vofa_Param['T'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['F'-'A'][1]) { Vofa_ParamFlag['F'-'A'][1] = 0; run_errfilt = Vofa_Param['F'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['R'-'A'][1]) { Vofa_ParamFlag['R'-'A'][1] = 0; run_w_far   = Vofa_Param['R'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['R'-'A'][2]) { Vofa_ParamFlag['R'-'A'][2] = 0; run_w_mid   = Vofa_Param['R'-'A'][2]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['R'-'A'][3]) { Vofa_ParamFlag['R'-'A'][3] = 0; run_w_near  = Vofa_Param['R'-'A'][3]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['C'-'A'][1]) { Vofa_ParamFlag['C'-'A'][1] = 0; run_err_bias= Vofa_Param['C'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['V'-'A'][1]) { Vofa_ParamFlag['V'-'A'][1] = 0; run_speed   = Vofa_Param['V'-'A'][1]; gpio_toggle_level(B9); }
        if(Vofa_ParamFlag['S'-'A'][1]) { Vofa_ParamFlag['S'-'A'][1] = 0; run_switch  = Vofa_Param['S'-'A'][1]; gpio_toggle_level(B9); }

        /* ---------- 安全限幅：防止滑动条拖过头把车弄失控 ---------- */
        if(run_speed    <    0.0f) { run_speed    =    0.0f; }   // V1：占空比（0~4000）
        if(run_speed    > 4000.0f) { run_speed    = 4000.0f; }

        if(run_p        <    0.0f) { run_p        =    0.0f; }
        if(run_p        >   20.0f) { run_p        =   20.0f; }
        if(run_d        <    0.0f) { run_d        =    0.0f; }
        if(run_d        >   50.0f) { run_d        =   50.0f; }
        if(run_bias     <  -50.0f) { run_bias     =  -50.0f; }
        if(run_bias     >   50.0f) { run_bias     =   50.0f; }
        if(run_errfilt  <    0.0f) { run_errfilt  =    0.0f; }
        if(run_errfilt  >    0.9f) { run_errfilt  =    0.9f; }
        if(run_w_far    <    0.0f) { run_w_far    =    0.0f; }   // R1/R2/R3：三块权重 0~5
        if(run_w_far    > ZONE_W_MAX) { run_w_far = ZONE_W_MAX; }
        if(run_w_mid    <    0.0f) { run_w_mid    =    0.0f; }
        if(run_w_mid    > ZONE_W_MAX) { run_w_mid = ZONE_W_MAX; }
        if(run_w_near   <    0.0f) { run_w_near   =    0.0f; }
        if(run_w_near   > ZONE_W_MAX) { run_w_near = ZONE_W_MAX; }
        if(run_err_bias <  -50.0f) { run_err_bias =  -50.0f; }   // C1：Err 偏置（±50 像素够用）
        if(run_err_bias >   50.0f) { run_err_bias =   50.0f; }
        if(run_switch   <    0.5f) { run_switch   =    0.0f; }
        else                       { run_switch   =    1.0f; }

        /* ---------- 从"停"拨到"跑"的那一瞬间，清一下微分项（避免第一拍猛打）---------- */
        if((run_enable > 0.0f) && (run_switch > 0.5f) && (0 == run_was_on))
        {
            err_last     = Err;
            err_use_prev = err_use;                     // 刚发车时把 D 项的历史清干净，避免第一拍猛打
            err_d_frame  = 0.0f;
        }
        run_was_on = ((run_enable > 0.0f) && (run_switch > 0.5f)) ? 1 : 0;

        /* ---------- 摄像头出了一帧：处理这一帧 ---------- */
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            Frame_Process();
        }
    }
}
