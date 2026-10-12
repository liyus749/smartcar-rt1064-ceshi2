/*********************************************************************************************************************
* 文件名称          Vofa
* 功能简介          VOFA+ 上位机通信（无线串口发波形 + 收参数在线调参）
*
* 硬件             逐飞【无线串口模块】，插主板"无线模块"接口
*                  单片机 D16(TX) / D17(RX) / D26(RTS)，走 UART8，115200
*                  接收通路逐飞库已经接好（isr.c 的 LPUART8_IRQHandler 会调回调）
*
* 协议参考         https://www.vofa.plus/docs/learning/dataengines/justfloat
*                  https://www.vofa.plus/docs/learning/start/data_cmd_parameter
********************************************************************************************************************/

#include "Vofa.h"
#include "zf_device_wireless_uart.h"        /* 无线串口模块（UART8）*/
#include <math.h>                           /* sinf() */

volatile float  Vofa_Param[26][10];
volatile uint8  Vofa_ParamFlag[26][10];

static const uint8 vofa_tail[4] = { 0x00, 0x00, 0x80, 0x7f };       /* JustFloat 帧尾 */

static char  vofa_line[48];                                         /* 收命令用的行缓冲 */
static uint8 vofa_len = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化（内部会初始化 UART8 无线串口）
//-------------------------------------------------------------------------------------------------------------------
void Vofa_Init(void)
{
    uint8 i, j;

    for(i = 0; i < 26; i++)
    {
        for(j = 0; j < 10; j++)
        {
            Vofa_Param[i][j]     = 0.0f;
            Vofa_ParamFlag[i][j] = 0;
        }
    }
    vofa_len = 0;

    wireless_uart_init();                       /* 初始化无线串口（UART8，D16/D17）*/

    wireless_uart_send_string("\r\n[VOFA] ready, UART8 115200\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发一包波形（JustFloat：小端 float 数组 + 4 字节帧尾 00 00 80 7F）
// 参数说明     vals        浮点数组
// 参数说明     n           通道数（1 ~ VOFA_MAX_CH）
// 使用示例     float w[8] = {Err, angle, enc_l, enc_r, duty, stopline, lost_l, lost_r};
//              Vofa_SendWaves(w, 8);
//-------------------------------------------------------------------------------------------------------------------
void Vofa_SendWaves(const float *vals, uint8 n)
{
    if((NULL == vals) || (0 == n))
    {
        return;
    }
    if(n > VOFA_MAX_CH)
    {
        n = VOFA_MAX_CH;
    }

    wireless_uart_send_buffer((const uint8 *)vals, (uint32)n * sizeof(float));
    wireless_uart_send_buffer(vofa_tail, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析一条命令，格式：【字母】【数字】=【数值】!
// 举例         "P1=2.5!"   ->  Vofa_Param['P'-'A'][1] = 2.5
//              "M=1!"      ->  Vofa_Param['M'-'A'][0] = 1.0   （不带数字相当于 0）
//              "D1=-5.5!"  ->  支持负数
// 备注信息     手写解析，不用 sscanf（省浮点扫描库的开销），小数用累乘法（不用 powf）
//-------------------------------------------------------------------------------------------------------------------
static void vofa_parse_line(const char *s, uint8 len)
{
    uint8 i;
    uint8 eq = 0xFF;                            /* '=' 的位置 */
    uint8 ex = 0xFF;                            /* '!' 的位置 */
    uint8 letter;
    uint8 digit = 0;
    uint8 neg = 0;
    uint8 dot = 0;
    uint8 seen = 0;
    float v = 0.0f;
    float frac = 0.1f;
    char  c;

    /* ---------- 1. 找定界符 '=' 和 '!' ---------- */
    for(i = 0; i < len; i++)
    {
        if(('=' == s[i]) && (0xFF == eq)) { eq = i; }
        if('!' == s[i])                   { ex = i; break; }
    }
    if((0xFF == eq) || (0xFF == ex))      { return; }   /* 缺定界符，丢弃 */
    if(ex <= (eq + 1))                    { return; }   /* 等号和感叹号之间没数值 */
    if(0 == eq)                           { return; }   /* 等号前面没名字 */

    /* ---------- 2. 解析名字：等号前面的字母（可选跟一个数字）---------- */
    letter = (uint8)s[0];
    if((letter >= 'a') && (letter <= 'z')) { letter -= 32; }    /* 小写转大写 */
    if((letter < 'A') || (letter > 'Z'))   { return; }

    if(eq >= 2)                                                  /* 有第二个字符 */
    {
        if((s[1] >= '0') && (s[1] <= '9'))
        {
            digit = (uint8)(s[1] - '0');
        }
    }

    /* ---------- 3. 解析数值：等号和感叹号之间 ---------- */
    i = (uint8)(eq + 1);
    if(('-' == s[i]) || ('+' == s[i]))
    {
        neg = ('-' == s[i]) ? 1 : 0;
        i++;
    }

    for(; i < ex; i++)
    {
        c = s[i];

        if((c >= '0') && (c <= '9'))
        {
            if(0 == dot) { v = v * 10.0f + (float)(c - '0'); }
            else         { v = v + (float)(c - '0') * frac;  frac *= 0.1f; }
            seen = 1;
        }
        else if('.' == c)
        {
            dot = 1;
        }
        else
        {
            return;                                             /* 中间有非法字符，整条丢弃 */
        }
    }

    if(0 == seen) { return; }                                   /* 一个数字都没有 */
    if(neg)       { v = -v; }

    Vofa_Param[letter - 'A'][digit]     = v;
    Vofa_ParamFlag[letter - 'A'][digit] = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     收参数（每圈都要调！无线串口接收缓冲只有 64 字节）
// 备注信息     用 '!' 当一条命令的结束标志，所以不需要换行
//-------------------------------------------------------------------------------------------------------------------
void Vofa_Poll(void)
{
    uint8  buf[32];
    uint32 n;
    uint32 i;
    char   c;

    n = wireless_uart_read_buffer(buf, sizeof(buf));

    for(i = 0; i < n; i++)
    {
        c = (char)buf[i];

        if('!' == c)                                            /* 遇到 '!' = 一条命令结束 */
        {
            if(vofa_len > 0)
            {
                if(vofa_len < (sizeof(vofa_line) - 1))
                {
                    vofa_line[vofa_len++] = '!';                /* 把 '!' 也放进去一起解析 */
                }
                vofa_parse_line(vofa_line, vofa_len);
                vofa_len = 0;
            }
        }
        else if(vofa_len < (sizeof(vofa_line) - 2))
        {
            vofa_line[vofa_len++] = c;
        }
        else
        {
            vofa_len = 0;                                       /* 一条太长（比如收到乱码），丢掉重来 */
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读一个参数：Vofa_Get('P', 1)
//-------------------------------------------------------------------------------------------------------------------
float Vofa_Get(char letter, uint8 digit)
{
    if((letter >= 'a') && (letter <= 'z')) { letter -= 32; }
    if((letter < 'A') || (letter > 'Z'))   { return 0.0f; }
    if(digit > 9)                          { return 0.0f; }

    return Vofa_Param[letter - 'A'][digit];
}


//===================================================================================================================
//  VOFA+ 自检（模式 11 用）—— 死循环，不返回
//
//  发 4 条波形：
//      通道1 = sin(t)                 标准正弦，确认波形能显示
//      通道2 = sin(2t)                二倍频，确认多通道
//      通道3 = 锯齿                    确认数据在持续刷新
//      通道4 = Vofa_Get('P', 1)       ★ 你在 VOFA+ 里拖 P1 滑动条，这条线就跟着动
//
//  ★ 怎么判断成功：
//      前 3 条波形在动              -> 单片机 -> 电脑 通了
//      拖 P1 滑动条，第 4 条跟着动   -> 电脑 -> 单片机 也通了
//      两条都通，就可以拿它调参数了
//===================================================================================================================
void Vofa_SelfTest(void)
{
    float  t = 0.0f;
    float  w[4];
    uint16 tick = 0;

    Vofa_Init();

    while(1)
    {
        /* ---------- 1. 先收参数（每圈都要收，不然 64 字节缓冲会满）---------- */
        Vofa_Poll();

        /* ---------- 2. 造 4 个通道的数据 ---------- */
        t += 0.05f;
        if(t > 6.28f) { t = 0.0f; }

        w[0] = sinf(t) * 100.0f;                        /* 通道1：正弦（放大 100 倍好观察）*/
        w[1] = sinf(t * 2.0f) * 100.0f;                 /* 通道2：二倍频 */
        w[2] = (float)(tick % 200) - 100.0f;            /* 通道3：锯齿 */
        w[3] = Vofa_Get('P', 1);                        /* 通道4：★ 收到的参数 P1 */

        /* ---------- 3. 发出去 ---------- */
        Vofa_SendWaves(w, 4);

        /* ---------- 4. B9 灯闪一下，表示程序活着 ---------- */
        if(0 == (tick % 25))
        {
            gpio_toggle_level(B9);
        }

        tick++;
        system_delay_ms(20);                            /* 50Hz 发送 */
    }
}
