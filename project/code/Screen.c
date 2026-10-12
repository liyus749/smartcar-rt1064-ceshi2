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
// 函数简介     清屏
//-------------------------------------------------------------------------------------------------------------------
void Screen_Clear(void)
{
    ips200_clear();
    scr_div = 0;
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
// 函数简介     显示状态文字（Err / 环岛状态 / 速度）
// 备注信息     文字写在图像下方，和图像区域不重叠，所以每帧调用也不会闪
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowStatus(float err, int state, int duty)
{
    ips200_show_string(0,  SCR_TXT_Y,       "Err ");
    ips200_show_float (40, SCR_TXT_Y,       err, 2, 1);

    ips200_show_string(0,  SCR_TXT_Y + 16,  "Isl ");
    ips200_show_int   (40, SCR_TXT_Y + 16,  state, 2);

    ips200_show_string(0,  SCR_TXT_Y + 32,  "Duty");
    ips200_show_int   (40, SCR_TXT_Y + 32,  duty, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示一行字符串
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowText(uint16 x, uint16 y, const char *str)
{
    ips200_show_string(x, y, str);
}


//===================================================================================================================
//  屏幕自检（模式 10 用）—— 死循环，不会返回
//
//  它会循环做这些事，每一步停 0.7 秒：
//    ① 全屏刷 红 -> 绿 -> 蓝 -> 白 -> 黑
//       能刷出颜色 = 屏幕、接线、驱动全都通了
//    ② 显示字符串 / 整数 / 浮点数 + 当前用的屏幕类型
//    ③ 显示一张"模拟的摄像头画面"：灰度渐变 + 中间黑线 + 左中右三条彩色线
//       位置和真正显示摄像头图像时完全一样 —— 这就是你以后调车看到的样子
//===================================================================================================================
static uint8 fake_img[MT9V03X_H][MT9V03X_W];            // 模拟用的"摄像头图像"

void Screen_SelfTest(void)
{
    uint8  phase = 0;
    uint16 i, j;
    uint16 x;

    // ---------- 先造一张假图：左亮右暗的渐变 ----------
    for(i = 0; i < MT9V03X_H; i++)
    {
        for(j = 0; j < MT9V03X_W; j++)
        {
            fake_img[i][j] = (uint8)(255 - (j * 200) / (MT9V03X_W - 1));
        }
    }

    while(1)
    {
        switch(phase)
        {
            // ---------- ① 刷纯色：验"屏幕和接线通不通" ----------
            case 0:  ips200_full(RGB565_RED);       break;
            case 1:  ips200_full(RGB565_GREEN);     break;
            case 2:  ips200_full(RGB565_BLUE);      break;
            case 3:  ips200_full(RGB565_WHITE);     break;
            case 4:  ips200_full(RGB565_BLACK);     break;

            // ---------- ② 显示文字和数据：验"字库" ----------
            case 5:
                ips200_clear();
                ips200_show_string(0,   0, "SCREEN  TEST  OK");
                ips200_show_string(0,  20, "type 0=SPI 1=PAR8 =");
                ips200_show_int   (170, 20, (int32)IPS_SCREEN_TYPE, 1);
                ips200_show_string(0,  40, "int     =");
                ips200_show_int   (80,  40, 12345, 5);
                ips200_show_string(0,  60, "float   =");
                ips200_show_float (80,  60, -12.34, 3, 2);
                ips200_show_string(0,  80, "if you see this");
                ips200_show_string(0, 100, "screen works !!");
                break;

            // ---------- ③ 模拟摄像头画面 + 边线（以后调车看到的就是这样）----------
            case 6:
                // 灰度图：一次性画完（和以后显示摄像头图像用的是同一个函数）
                ips200_show_gray_image(SCR_IMG_X, SCR_IMG_Y, fake_img[0],
                                       MT9V03X_W, MT9V03X_H, SCR_IMG_W, SCR_IMG_H, 0);

                // 中间刷一条黑带，模拟赛道黑线
                for(i = 35; i < 75; i++)
                {
                    for(j = 70; j < 100; j++)
                    {
                        ips200_draw_point(SCR_IMG_X + j, SCR_IMG_Y + i, RGB565_BLACK);
                    }
                }

                // 三条彩色线：蓝=左边界 绿=中线 红=右边界（和真正调车时的颜色一致）
                for(i = 0; i < SCR_IMG_H; i++)
                {
                    x = (uint16)(60 + (i * 30) / SCR_IMG_H);
                    ips200_draw_point(SCR_IMG_X + x,  SCR_IMG_Y + i, SCR_COLOR_LINE_L);
                    ips200_draw_point(SCR_IMG_X + 94, SCR_IMG_Y + i, SCR_COLOR_LINE_M);
                    ips200_draw_point(SCR_IMG_X + 102 + (uint16)((SCR_IMG_H - i) / 3),
                                      SCR_IMG_Y + i, SCR_COLOR_LINE_R);
                }

                ips200_show_string(0, SCR_TXT_Y,      "fake image 188x100");
                ips200_show_string(0, SCR_TXT_Y + 16, "B=left G=mid R=right");
                break;

            default:
                phase = 0;
                break;
        }

        system_delay_ms(700);

        phase ++;
        if(phase > 6) { phase = 0; }
    }
}
