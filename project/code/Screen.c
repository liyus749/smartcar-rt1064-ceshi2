/*********************************************************************************************************************
* 文件名称          Screen
* 功能简介          IPS200 液晶屏封装 —— 显示【摄像头图像 + 左/中/右边界 + 中心参考线 + 状态文字】
*
* 参考              逐飞官方例程 E06_04_ips200_display_demo
*
* 说明：
*   屏幕 240 x 320（竖屏），摄像头图像 188 x 100，1:1 显示在左上角，
*   图像下面是状态文字（Err / 环岛状态 / 速度）。
*
*   画面中心那条竖直虚线 = 图像正中间（x = 94），
*   用它来对照中线偏了多少 —— 这也是 Err 的来源。
********************************************************************************************************************/

#include "Screen.h"

static uint16 scr_div = 0;                                  // 刷屏分频计数

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕初始化
// 备注信息     官方例程开头先等 300ms，这里照做，避免主板其他外设还没上电完成
//-------------------------------------------------------------------------------------------------------------------
void Screen_Init(void)
{
    system_delay_ms(300);

    ips200_set_dir(IPS200_PORTAIT);                         // 竖屏 240 x 320
    ips200_set_font(IPS200_8X16_FONT);                      // 8x16 字体
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);           // 白字黑底
    ips200_init(IPS_SCREEN_TYPE);                           // ★ 屏幕型号在这里生效
    ips200_clear();

    // 开机先写一行字，方便判断"屏幕到底有没有起来"
    ips200_show_string(0, SCR_TXT_Y, "screen init OK");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示一帧：图像 + 三条边线 + 中心参考虚线
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowFrame(const uint8 *gray, const uint8 *bin,
                      const uint8 *line_l, const uint8 *line_m, const uint8 *line_r)
{
    uint16 i;
    uint16 x;

    // ---------- 分频：SCREEN_EVERY_N = 1 时每帧都刷 ----------
    scr_div ++;
    if(scr_div < SCREEN_EVERY_N)
    {
        return;
    }
    scr_div = 0;

    // ---------- 1. 画图像 ----------
#if (1 == SCREEN_SHOW_BINARY)
    // 二值图：输入是 0x00(黑) / 0xFF(白)，阈值传 128 正好把两者分开
    if(NULL != bin)
    {
        ips200_show_gray_image(SCR_IMG_X, SCR_IMG_Y, bin,
                               MT9V03X_W, MT9V03X_H, SCR_IMG_W, SCR_IMG_H, 128);
    }
#else
    // 灰度原图：阈值传 0 = 按灰度显示（每个像素转成对应亮度的 RGB565）
    if(NULL != gray)
    {
        ips200_show_gray_image(SCR_IMG_X, SCR_IMG_Y, gray,
                               MT9V03X_W, MT9V03X_H, SCR_IMG_W, SCR_IMG_H, 0);
    }
#endif

    // ---------- 2. 画面中心参考虚线（每 4 行点一个点）----------
    for(i = 0; i < SCR_IMG_H; i += 4)
    {
        ips200_draw_point(SCR_IMG_X + (SCR_IMG_W >> 1), SCR_IMG_Y + i, SCR_COLOR_REF);
    }

    // ---------- 3. 三条边线（画在图像上面）----------
    for(i = 0; i < SCR_IMG_H; i++)
    {
        if(NULL != line_l)
        {
            x = line_l[i];
            if(x < SCR_IMG_W) { ips200_draw_point(SCR_IMG_X + x, SCR_IMG_Y + i, SCR_COLOR_LINE_L); }
        }
        if(NULL != line_m)
        {
            x = line_m[i];
            if(x < SCR_IMG_W) { ips200_draw_point(SCR_IMG_X + x, SCR_IMG_Y + i, SCR_COLOR_LINE_M); }
        }
        if(NULL != line_r)
        {
            x = line_r[i];
            if(x < SCR_IMG_W) { ips200_draw_point(SCR_IMG_X + x, SCR_IMG_Y + i, SCR_COLOR_LINE_R); }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示状态文字（Err / 环岛状态 / 十字标志 / 占空比）
// 备注信息     文字写在图像下方，和图像区域不重叠，所以每帧调用也不会闪
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowStatus(float err, int state, int duty, int cross)
{
    /* ★ Err 的整数位数给 3 位（原来是 2 位）。
       逐飞这个 show_float 的实现是"整数部分按 num 位取模"，
       也就是说 num=2 时，Err = -123.4 会被显示成 -23.4（少了个百位），
       和 VOFA 上看到的 -123.4 就会"对不上"。给 3 位就不会再截断。 */
    ips200_show_string(0,  SCR_TXT_Y,       "Err ");
    ips200_show_float (40, SCR_TXT_Y,       err, 3, 1);

    ips200_show_string(0,  SCR_TXT_Y + 16,  "Isl ");
    ips200_show_int   (40, SCR_TXT_Y + 16,  state, 2);

    ips200_show_string(80, SCR_TXT_Y + 16,  "Cr ");
    ips200_show_int   (112, SCR_TXT_Y + 16, cross, 1);

    ips200_show_string(0,  SCR_TXT_Y + 32,  "Spd ");    /* 电机占空比设定值（V1）*/
    ips200_show_int   (40, SCR_TXT_Y + 32,  duty, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示"参数下行"的诊断信息（第 4、5 行）
//
//   屏幕布局（图像下面）：
//       Err  x.x        （Screen_ShowStatus 画的第 1 行）
//       Isl  x          （第 2 行）
//       Duty 1000       （第 3 行）—— 你拖 V1 滑动条，这个数必须跟着变
//       Sw 1   Out 1000 （第 4 行）★ Sw = 开关状态，Out = 中断里真正写进电机的占空比
//       rx 1234 ok 5    （第 5 行）★ rx = 电脑发来的字节数，ok = 成功收到的参数条数
//
//   ★ 为什么要看 Out：屏幕上原来的 Duty 只是"设定值"，Out 才是"真的用到电机上的值"。
//     两个都对上，才叫参数真的控制到电机了。
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowParam(int sw, int out_duty, uint32 rx_bytes, uint32 cmd_ok)
{
    ips200_show_string(0,   SCR_TXT_Y + 48, "Sw ");
    ips200_show_int   (24,  SCR_TXT_Y + 48, (int32)sw, 1);

    ips200_show_string(64,  SCR_TXT_Y + 48, "Out");
    ips200_show_int   (104, SCR_TXT_Y + 48, (int32)out_duty, 4);

    ips200_show_string(0,   SCR_TXT_Y + 64, "rx ");
    ips200_show_int   (24,  SCR_TXT_Y + 64, (int32)rx_bytes, 6);

    ips200_show_string(120, SCR_TXT_Y + 64, "ok");
    ips200_show_int   (144, SCR_TXT_Y + 64, (int32)cmd_ok, 5);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在图像上画两条黄色横虚线，标出"远/中/近"三个分块的分界
//
//   画法：每隔一个像素点一个点（虚线），这样不会把下面的赛道完全挡住。
//   坐标：图像是 1:1 画在 (SCR_IMG_X, SCR_IMG_Y) 的，所以"图像第 row 行"就是"屏幕第 SCR_IMG_Y+row 行"。
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowProspect(uint8 row_top, uint8 row_bottom)
{
    uint16 x;

    if(row_top    >= SCR_IMG_H) { row_top    = (uint8)(SCR_IMG_H - 1); }
    if(row_bottom >= SCR_IMG_H) { row_bottom = (uint8)(SCR_IMG_H - 1); }

    for(x = 0; x < SCR_IMG_W; x += 2)
    {
        ips200_draw_point(SCR_IMG_X + x, SCR_IMG_Y + row_top,    RGB565_YELLOW);
        ips200_draw_point(SCR_IMG_X + x, SCR_IMG_Y + row_bottom, RGB565_YELLOW);
    }
}

