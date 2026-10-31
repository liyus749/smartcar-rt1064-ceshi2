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
#include "Cross.h"
#include "Roundabout.h"

static uint8 status_first_frame = 1;
static uint16 scr_div = 0;                                  // 刷屏分频计数

// 补线只在本次图像实际刷新时叠加，避免分频刷屏期间留下旧角点。
static void Screen_CrossOverlay(const uint8 *line_l, const uint8 *line_r)
{
    int row, which, x, y, d;
    for(row = 2; row < SCR_IMG_H; row += 2)
    {
        uint8 mask = Cross_GetFillMask(row);
        if((mask & 1) && line_l && line_l[row] < SCR_IMG_W)
        { ips200_draw_point(SCR_IMG_X+line_l[row], SCR_IMG_Y+row, RGB565_YELLOW); }
        if((mask & 2) && line_r && line_r[row] < SCR_IMG_W)
        { ips200_draw_point(SCR_IMG_X+line_r[row], SCR_IMG_Y+row, RGB565_YELLOW); }
    }
    for(which = 0; which < CROSS_CORNER_COUNT; which++)
    {
        uint16 color = (which < CROSS_LOWER_LEFT) ? 0xF81F : 0x07FF; // 上品红，下青色
        if(!Cross_GetCorner(which, &x, &y)) { continue; }
        for(d = -2; d <= 2; d++)
        {
            if(x+d >= 0 && x+d < SCR_IMG_W && y >= 0 && y < SCR_IMG_H)
            { ips200_draw_point(SCR_IMG_X+x+d, SCR_IMG_Y+y, color); }
            if(y+d >= 0 && y+d < SCR_IMG_H && x >= 0 && x < SCR_IMG_W)
            { ips200_draw_point(SCR_IMG_X+x, SCR_IMG_Y+y+d, color); }
        }
    }
}

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
    status_first_frame = 1;

    // 开机先写一行字，方便判断"屏幕到底有没有起来"
    ips200_show_string(0, SCR_TXT_Y, "KEY RUN R3");
}

// BEGIN SIMPLE RUN DISPLAY
void Screen_ShowRunState(uint8 running, uint8 waiting)
{
    ips200_show_string(176, SCR_TXT_Y, running ? "RUN " : (waiting ? "WAIT" : "STOP"));
}

void Screen_ShowDriving(float err, int speed, const char *p, const char *d,
                        int left, int left_target, int right, int right_target,
                        uint8 running, uint8 waiting)
{
    ips200_show_string(0, SCR_TXT_Y, "KEY RUN R3");
    Screen_ShowRunState(running, waiting);
    ips200_show_string(0, SCR_TXT_Y + 24, "SPD");
    ips200_show_int(40, SCR_TXT_Y + 24, speed, 3);
    ips200_show_string(112, SCR_TXT_Y + 24, "ERR");
    ips200_show_float(144, SCR_TXT_Y + 24, err, 3, 1);
    ips200_show_string(0, SCR_TXT_Y + 48, "P");
    ips200_show_string(24, SCR_TXT_Y + 48, p);
    ips200_show_string(112, SCR_TXT_Y + 48, "D");
    ips200_show_string(136, SCR_TXT_Y + 48, d);
    ips200_show_string(0, SCR_TXT_Y + 72, "L:");
    ips200_show_int(24, SCR_TXT_Y + 72, left, 4);
    ips200_show_string(72, SCR_TXT_Y + 72, "/");
    ips200_show_int(88, SCR_TXT_Y + 72, left_target, 3);
    ips200_show_string(0, SCR_TXT_Y + 96, "R:");
    ips200_show_int(24, SCR_TXT_Y + 96, right, 4);
    ips200_show_string(72, SCR_TXT_Y + 96, "/");
    ips200_show_int(88, SCR_TXT_Y + 96, right_target, 3);
    ips200_show_string(0, SCR_TXT_Y + 184, "L/R: ACT/TGT  15-/14+");
    ips200_show_string(0, SCR_TXT_Y + 200, "C12:SEL  C13:RUN/STOP");
}

void Screen_ShowHealth(uint8 lost, uint8 fault)
{
    ips200_show_string(0, SCR_TXT_Y + 168, lost ? "CAM:LOST" : "CAM:OK  ");
    ips200_show_string(88, SCR_TXT_Y + 168,
                       fault == 1 ? "STOP:CAMERA     " :
                       fault == 2 ? "STOP:RING       " : "STOP:--         ");
}

void Screen_ShowTrackFlags(int cross, int cross_state, int island, int direction,
                          uint8 lost, uint8 fault, int pwm, uint8 selected)
{
    ips200_show_string(0, SCR_TXT_Y + 24, selected == 0 ? ">SPD" : " SPD");
    ips200_show_string(0, SCR_TXT_Y + 48, selected == 1 ? ">P" : " P");
    ips200_show_string(112, SCR_TXT_Y + 48, selected == 2 ? ">D" : " D");
    ips200_show_string(128, SCR_TXT_Y + 72, "PWM");
    ips200_show_int(160, SCR_TXT_Y + 72, pwm, 4);
    ips200_show_string(0, SCR_TXT_Y + 120, "Cr:");
    ips200_show_int(24, SCR_TXT_Y + 120, cross ? 1 : 0, 1);
    ips200_show_string(88, SCR_TXT_Y + 120, "Cs:");
    ips200_show_int(112, SCR_TXT_Y + 120, cross_state, 1);
    ips200_show_string(0, SCR_TXT_Y + 144, "Is:");
    ips200_show_int(24, SCR_TXT_Y + 144, island, 1);
    ips200_show_string(88, SCR_TXT_Y + 144, "Dir:");
    ips200_show_string(128, SCR_TXT_Y + 144, direction > 0 ? "LEFT " : direction < 0 ? "RIGHT" : "--   ");
    Screen_ShowHealth(lost, fault);
}
// END SIMPLE RUN DISPLAY

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示一帧：图像 + 三条边线 + 中心参考虚线
//-------------------------------------------------------------------------------------------------------------------
uint8 Screen_ShowFrame(const uint8 *gray, const uint8 *bin,
                      const uint8 *line_l, const uint8 *line_m, const uint8 *line_r)
{
    uint16 i;
    uint16 x;

    // ---------- 分频：SCREEN_EVERY_N = 1 时每帧都刷 ----------
    scr_div ++;
    if(scr_div < SCREEN_EVERY_N)
    {
        return 0;
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
    Screen_CrossOverlay(line_l, line_r);
    // 环岛补线：青色；真实边线保持原配色。角点只标当前帧观测。
    for(i=2;i<SCR_IMG_H;i+=2)
    {
        uint8 mask=Roundabout_GetFillMask(i);
        if((mask&1) && line_l && line_l[i]<SCR_IMG_W)
        { ips200_draw_point(SCR_IMG_X+line_l[i],SCR_IMG_Y+i,0x07FF); }
        if((mask&2) && line_r && line_r[i]<SCR_IMG_W)
        { ips200_draw_point(SCR_IMG_X+line_r[i],SCR_IMG_Y+i,0x07FF); }
    }
    for(i=0;i<ROUND_POINT_COUNT;i++)
    {
        int cx,cy,d;
        if(!Roundabout_GetPoint(i,&cx,&cy)) { continue; }
        for(d=-2;d<=2;d++)
        {
            if(cx+d>=0 && cx+d<SCR_IMG_W && cy>=0 && cy<SCR_IMG_H)
            { ips200_draw_point(SCR_IMG_X+cx+d,SCR_IMG_Y+cy,0xF81F); }
            if(cy+d>=0 && cy+d<SCR_IMG_H && cx>=0 && cx<SCR_IMG_W)
            { ips200_draw_point(SCR_IMG_X+cx,SCR_IMG_Y+cy+d,0xF81F); }
        }
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画参考列竖虚线
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowRefCol(uint8 col)
{
    uint16 i;

    if(col >= SCR_IMG_W)
    {
        col = (uint8)(SCR_IMG_W - 1);
    }

    for(i = 0; i < SCR_IMG_H; i += 4)
    {
        ips200_draw_point(SCR_IMG_X + col, SCR_IMG_Y + i, RGB565_YELLOW);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示当前巡线方案
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowLineMode(int mode)
{
    ips200_show_string(80, SCR_TXT_Y + 16, "Md ");
    ips200_show_string(112, SCR_TXT_Y + 16, (1 == mode) ? "O" : "C");
#if SCREEN_SHOW_BINARY
    ips200_show_string(120, SCR_TXT_Y + 16, "BIN");
#else
    ips200_show_string(120, SCR_TXT_Y + 16, "GRY");
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示十字路口标志
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowCross(int cross)
{
    ips200_show_string(152, SCR_TXT_Y + 16, "Cr");
    ips200_show_int   (176, SCR_TXT_Y + 16, (int32)cross, 1);
    ips200_show_string(192, SCR_TXT_Y + 16, "Cs");
    ips200_show_int(216, SCR_TXT_Y + 16, (int32)Cross_GetState(), 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示十字调试信息：连续丢线行数
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowCrossDebug(int run_len)
{
    ips200_show_string(0,  SCR_TXT_Y + 96, "Ln ");
    ips200_show_int   (40, SCR_TXT_Y + 96, (int32)run_len, 3);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示舵机打角值（和 Tgt 同一行）
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowServoAngle(int angle)
{
    ips200_show_string(96,  SCR_TXT_Y + 32, "Ang");
    ips200_show_int   (136, SCR_TXT_Y + 32, (int32)angle, 3);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示状态文字（Err / 环岛状态 / 电机目标速度）
// 备注信息     文字写在图像下方，和图像区域不重叠，所以每帧调用也不会闪
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowStatus(float err, int state, int target)
{
    if(status_first_frame)
    {
        // 清除启动提示，避免 Err 后残留 e/OK 被误认为运行状态。
        ips200_show_string(0, SCR_TXT_Y, "                              ");
        status_first_frame = 0;
    }
    /* ★ Err 的整数位数给 3 位（原来是 2 位）。
       逐飞这个 show_float 的实现是"整数部分按 num 位取模"，
       也就是说 num=2 时，Err = -123.4 会被显示成 -23.4（少了个百位），
       和 VOFA 上看到的 -123.4 就会"对不上"。给 3 位就不会再截断。 */
    ips200_show_string(0,  SCR_TXT_Y,       "Err ");
    ips200_show_float (40, SCR_TXT_Y,       err, 3, 1);

    ips200_show_string(0,  SCR_TXT_Y + 16,  "Isl ");
    ips200_show_int   (40, SCR_TXT_Y + 16,  state, 2);

    ips200_show_string(0,  SCR_TXT_Y + 32,  "Tgt ");    /* 电机目标速度（编码器计数/10ms）*/
    ips200_show_int   (40, SCR_TXT_Y + 32,  target, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示电机状态（第 4、5 行）
//
//   屏幕布局（图像下面）：
//       Err  x.x        （Screen_ShowStatus 画的第 1 行）
//       Isl  x          （第 2 行）
//       Tgt 150         （第 3 行）—— 目标速度，拖滑动条这个数必须跟着变
//       Sw 1   Out 1000 （第 4 行）★ Sw = 开关状态，Out = 真正写进电机的占空比
//       Act 143         （第 5 行）★ 实测速度，Tgt/Act 对上说明速度环工作正常
//
//   ★ Tgt 变、Act 不动 -> 参数没进来，或电机/编码器有问题。
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowParam(int sw, int out_duty, int target, int actual)
{
    ips200_show_string(0,   SCR_TXT_Y + 48, "Sw ");
    ips200_show_int   (24,  SCR_TXT_Y + 48, (int32)sw, 1);

    ips200_show_string(64,  SCR_TXT_Y + 48, "Out");
    ips200_show_int   (104, SCR_TXT_Y + 48, (int32)out_duty, 4);

    ips200_show_string(0,   SCR_TXT_Y + 64, "Act");
    ips200_show_int   (24,  SCR_TXT_Y + 64, (int32)actual, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示单片机最近收到的参数
//-------------------------------------------------------------------------------------------------------------------
void Screen_ShowParamRx(uint32 count, uint8 ch, float val)
{
    ips200_show_string(0,   SCR_TXT_Y + 80, "Rx ");
    ips200_show_int   (24,  SCR_TXT_Y + 80, (int32)count, 5);

    ips200_show_string(88,  SCR_TXT_Y + 80, "ch");
    ips200_show_int   (112, SCR_TXT_Y + 80, (int32)ch, 1);

    ips200_show_string(128, SCR_TXT_Y + 80, "=");
    ips200_show_float (144, SCR_TXT_Y + 80, val, 3, 1);
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

