/*********************************************************************************************************************
* 文件名称          Assistant.c
* 功能简介          逐飞助手 V2.0 通信实现（说明见 Assistant.h）
********************************************************************************************************************/

#include "Assistant.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wireless_uart.h"

//===================================================================================================================
// 逐飞助手协议对象
//===================================================================================================================
static seekfree_assistant_camera_struct            g_camera;
static seekfree_assistant_camera_boundary_struct   g_boundary_left;
static seekfree_assistant_camera_boundary_struct   g_boundary_right;
static seekfree_assistant_oscilloscope_struct      g_osc;
static seekfree_assistant_debug_param_struct       g_param;

//===================================================================================================================
// 缓冲
//===================================================================================================================
static float  g_osc_buf[ASSISTANT_OSC_CH_MAX];                              // 波形缓冲
static seekfree_assistant_camera_boundary_uint8_struct
              g_left_buf[MT9V03X_H];                                        // 左边界点
static seekfree_assistant_camera_boundary_uint8_struct
              g_right_buf[MT9V03X_H];                                       // 右边界点
static uint8  g_image_copy[MT9V03X_W * MT9V03X_H];                          // 图像拷贝（发送时摄像头会换缓冲，必须拷贝）

static uint8  g_image_div = 0;                                              // 图像分频计数
static uint8  g_wave_div  = 0;                                              // 波形分频计数

static uint32 g_rx_count  = 0;                                              // 收到的参数总条数
static uint8  g_last_ch   = 0;                                              // 最近一条参数的通道
static float  g_last_val  = 0.0f;                                           // 最近一条参数的值

// 边线颜色（RGB565）
#define ASST_COLOR_LEFT     (0x001F)   // 蓝
#define ASST_COLOR_RIGHT    (0xF800)   // 红

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 Assistant_Init(void)
{
    uint8 i;
    uint8 ret;

    /* 初始化无线串口（UART8，D16/D17/D26，自动波特率）*/
    ret = wireless_uart_init();
    if(0 != ret)
    {
        return 1;
    }

    /* 让逐飞助手库走"无线转串口"这条路 */
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);

    /* 摄像头图像：MT9V03X 灰度 188x100 */
    seekfree_assistant_camera_config(&g_camera,
                                     SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                     MT9V03X_W, MT9V03X_H,
                                     g_image_copy);

    /* 左右边线：uint8 坐标，100 个点 */
    seekfree_assistant_camera_boundary_config(&g_boundary_left,
                                              SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
                                              ASST_COLOR_LEFT,
                                              MT9V03X_H,
                                              g_left_buf);
    seekfree_assistant_camera_boundary_config(&g_boundary_right,
                                              SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
                                              ASST_COLOR_RIGHT,
                                              MT9V03X_H,
                                              g_right_buf);

    /* 波形：8 个 float 通道 */
    seekfree_assistant_oscilloscope_config(&g_osc, ASSISTANT_OSC_CH_MAX, g_osc_buf);

    /* 参数结构清空 */
    for(i = 0; i < SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX; i++)
    {
        g_param.data[i]        = 0.0f;
        g_param.update_flag[i] = 0;
    }

    g_image_div = 0;
    g_rx_count  = 0;
    g_last_ch   = 0;
    g_last_val  = 0.0f;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收参数
//-------------------------------------------------------------------------------------------------------------------
void Assistant_Poll(void)
{
    uint8 i;

    seekfree_assistant_debug_param_analysis(&g_param);

    /* 记下这一次收到了哪些通道（主循环随后会清 update_flag）*/
    for(i = 0; i < SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX; i++)
    {
        if(g_param.update_flag[i])
        {
            g_rx_count++;
            g_last_ch  = i;
            g_last_val = g_param.data[i];
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发一帧
//-------------------------------------------------------------------------------------------------------------------
void Assistant_SendFrame(const uint8 *gray, const uint8 *left, const uint8 *right, const float *wave)
{
    uint8 i;

    (void)gray;
    (void)left;
    (void)right;
    (void)wave;
    (void)i;

#if ASSISTANT_SEND_WAVE
    /* 1. 波形（小包，先发）*/
    if(NULL != wave)
    {
        g_wave_div++;
        if(g_wave_div >= ASSISTANT_WAVE_DIV)
        {
            g_wave_div = 0;

            for(i = 0; i < ASSISTANT_OSC_CH_MAX; i++)
            {
                float v = wave[i];
                if((v != v) || (v > 1.0e30f) || (v < -1.0e30f)) { v = 0.0f; }   // 消毒 NaN/Inf
                g_osc_buf[i] = v;
            }

#if ASSISTANT_OSC_PROTOCOL
            /* Printf 文本协议：一行 8 个通道，逗号分隔，换行结尾 */
            {
                char line[160];

                snprintf(line, sizeof(line),
                         "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                         g_osc_buf[0], g_osc_buf[1], g_osc_buf[2], g_osc_buf[3],
                         g_osc_buf[4], g_osc_buf[5], g_osc_buf[6], g_osc_buf[7]);

                wireless_uart_send_string(line);
            }
#else
            seekfree_assistant_oscilloscope_send(&g_osc);
#endif

            if(ASSISTANT_OSC_GAP_MS > 0)
            {
                system_delay_ms(ASSISTANT_OSC_GAP_MS);
            }
        }
    }
#endif

    /* 2. 左右边线（叠加在图像上）*/
#if ASSISTANT_SEND_BOUNDARY
    if((NULL != left) && (NULL != right))
    {
        for(i = 0; i < MT9V03X_H; i++)
        {
            g_left_buf[i].x  = (left[i]  >= MT9V03X_W) ? (uint8)(MT9V03X_W - 1) : left[i];
            g_left_buf[i].y  = (uint8)i;
            g_right_buf[i].x = (right[i] >= MT9V03X_W) ? (uint8)(MT9V03X_W - 1) : right[i];
            g_right_buf[i].y = (uint8)i;
        }
        seekfree_assistant_camera_boundary_send(&g_boundary_left);
        seekfree_assistant_camera_boundary_send(&g_boundary_right);
    }
#endif

    /* 3. 图像（大包，分频 + RTS 空闲才发）*/
#if ASSISTANT_SEND_IMAGE
    if(NULL != gray)
    {
        g_image_div++;
        if(g_image_div >= ASSISTANT_IMAGE_DIV)
        {
            g_image_div = 0;

            /* 模块忙（RTS 高）就跳过这一帧，避免死等 */
            if(!gpio_get_level(WIRELESS_UART_RTS_PIN))
            {
                memcpy(g_image_copy, gray, (uint32)(MT9V03X_W * MT9V03X_H));
                seekfree_assistant_camera_send(&g_camera);
            }
        }
    }
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读参数
//-------------------------------------------------------------------------------------------------------------------
float Assistant_GetParam(assistant_param_enum ch)
{
    if(ch >= SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX) { return 0.0f; }
    return g_param.data[ch];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读"有没有新值"
//-------------------------------------------------------------------------------------------------------------------
uint8 Assistant_GetUpdated(assistant_param_enum ch)
{
    if(ch >= SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX) { return 0; }
    return g_param.update_flag[ch];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清"有新值"标志
//-------------------------------------------------------------------------------------------------------------------
void Assistant_ClearUpdated(assistant_param_enum ch)
{
    if(ch < SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX)
    {
        g_param.update_flag[ch] = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取收到的参数统计
//-------------------------------------------------------------------------------------------------------------------
uint32 Assistant_GetRxCount(void)
{
    return g_rx_count;
}

uint8 Assistant_GetLastChannel(void)
{
    return g_last_ch;
}

float Assistant_GetLastValue(void)
{
    return g_last_val;
}
