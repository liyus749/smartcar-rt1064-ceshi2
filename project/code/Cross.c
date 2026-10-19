/*********************************************************************************************************************
* 文件名称          Cross.c
* 功能简介          十字路口识别实现
*
* 识别原理：
*   巡线时每一行都有一个"丢线"标志：
*     Left_Lost_Flag[i]  = 1  表示这一行左边没找到边界
*     Right_Lost_Flag[i] = 1  表示这一行右边没找到边界
*
*   十字在图像上的典型样子（车正对十字往前开）：
*     图像下方：近处赛道正常，左右边线都在；
*     图像中间：十字路口横向两条路把白区域连成一片，左右边线"同时消失"；
*     图像上方：过了路口，远处赛道又重新出现，左右边线又能找到。
*
*   所以判据是：
*     1. 从下往上找到第一段"左右两边都丢线"的连续行；
*     2. 这段长度 >= CROSS_LOST_MIN；
*     3. 这段不是从图像最底行就开始（近处还得有赛道）；
*     4. 这段的顶部没有顶到图像最上面（排除"赛道看到头了"）；
*     5. 这段的【上方】还能重新找到几行有效赛道（排除"冲出赛道/图像丢失"）；
*     6. 不在环岛状态里（环岛和十字有些特征像，环岛优先）。
*
*   满足以上条件连续 CROSS_CONFIRM_FRAMES 帧，就置 Cross_Flag=1，
*   并保持 CROSS_HOLD_FRAMES 帧（路口过去后还继续直行一小段，防止舵机突然甩）。
********************************************************************************************************************/

#include "Cross.h"

extern int          Left_Lost_Flag[MT9V03X_H];
extern int          Right_Lost_Flag[MT9V03X_H];
extern volatile int Left_Line[MT9V03X_H];
extern volatile int Right_Line[MT9V03X_H];
extern volatile int Mid_Line[MT9V03X_H];
extern volatile int Road_Wide[MT9V03X_H];
extern volatile int Island_State;
extern volatile int Left_Island_Flag;
extern volatile int Right_Island_Flag;

static uint8   cross_confirm = 0;      // 连续满足条件的帧数
static uint16  cross_hold    = 0;      // 确认后的保持帧数
static volatile int cross_run_len = 0; // 当前两边都丢线的连续行数

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     十字补线：把 top~bottom 这几行缺失的左右边线，用下方和上方的有效边线线性插值补出来
// 参数说明     above_row   上方最近的有效行（远）
// 参数说明     below_row   下方最近的有效行（近）
// 参数说明     top/bottom  要补的连续行范围
//-------------------------------------------------------------------------------------------------------------------
static void Cross_FillLines(int above_row, int below_row, int top, int bottom)
{
    int i;
    int lb = Left_Line[below_row];
    int rb = Right_Line[below_row];
    int la = Left_Line[above_row];
    int ra = Right_Line[above_row];
    float den = (float)(below_row - above_row);

    if(den < 1.0f) { return; }

    for(i = top; i <= bottom; i++)
    {
        float t = ((float)(below_row - i)) / den;
        int l = (int)((float)lb + ((float)(la - lb)) * t);
        int r = (int)((float)rb + ((float)(ra - rb)) * t);

        if(l < 0)             { l = 0; }
        if(l > MT9V03X_W - 1) { l = MT9V03X_W - 1; }
        if(r < 0)             { r = 0; }
        if(r > MT9V03X_W - 1) { r = MT9V03X_W - 1; }

        Left_Line[i]       = l;
        Right_Line[i]      = r;
        Left_Lost_Flag[i]  = 0;
        Right_Lost_Flag[i] = 0;
        Mid_Line[i]        = (l + r) >> 1;
        Road_Wide[i]       = r - l;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     上方找不到有效赛道时，用下方最近的有效边线竖直延伸，把缺的行补出来
//-------------------------------------------------------------------------------------------------------------------
static void Cross_ExtendLines(int below_row, int top, int bottom)
{
    int i;
    int l = Left_Line[below_row];
    int r = Right_Line[below_row];

    for(i = top; i <= bottom; i++)
    {
        Left_Line[i]       = l;
        Right_Line[i]      = r;
        Left_Lost_Flag[i]  = 0;
        Right_Lost_Flag[i] = 0;
        Mid_Line[i]        = (l + r) >> 1;
        Road_Wide[i]       = r - l;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化
//-------------------------------------------------------------------------------------------------------------------
void Cross_Init(void)
{
    Cross_Flag    = 0;
    cross_confirm = 0;
    cross_hold    = 0;
    cross_run_len = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     每帧检测
//-------------------------------------------------------------------------------------------------------------------
void Cross_Check(void)
{
    int i;
    int run_bottom = -1;
    int run_top    = -1;
    int valid_above = 0;
    int valid_below = 0;
    int candidate   = 0;

    /* 1. 从最底行往上，找第一段"左右两边都丢线"的连续行 */
    for(i = MT9V03X_H - 1; i >= 0; i--)
    {
        if(Left_Lost_Flag[i] && Right_Lost_Flag[i])
        {
            if(run_bottom < 0) { run_bottom = i; }
            run_top = i;
        }
        else
        {
            if(run_bottom >= 0) { break; }   /* 这段到顶了，停下 */
        }
    }

    cross_run_len = (run_bottom >= 0) ? (run_bottom - run_top + 1) : 0;

    /* 2. 近处还得有赛道：丢线段不能从最底两行就开始 */
    valid_below = (run_bottom >= 0 && run_bottom < MT9V03X_H - 2) ? 1 : 0;

    /* 3. 丢线段上方还能重新看到赛道 */
    valid_above = 0;
    if(run_top >= CROSS_TOP_MARGIN)
    {
        int cnt = 0;
        for(i = run_top - 1; i >= 0; i--)
        {
            if(!Left_Lost_Flag[i] && !Right_Lost_Flag[i])
            {
                cnt++;
                if(cnt >= CROSS_VALID_ABOVE)
                {
                    valid_above = 1;
                    break;
                }
            }
        }
    }

    /* 3.5 只要丢线段足够长，就先补线并画出来（方便测试；十字标志仍按严格判据） */
    if(cross_run_len >= CROSS_LOST_MIN)
    {
        int below_row = run_bottom + 1;
        int above_row = -1;

        while((below_row < MT9V03X_H) &&
              (Left_Lost_Flag[below_row] || Right_Lost_Flag[below_row]))
        {
            below_row++;
        }

        for(i = run_top - 1; i >= 0; i--)
        {
            if(!Left_Lost_Flag[i] && !Right_Lost_Flag[i])
            {
                above_row = i;
                break;
            }
        }

        if(below_row < MT9V03X_H)
        {
            if(above_row >= 0)
            {
                Cross_FillLines(above_row, below_row, run_top, run_bottom);
            }
            else
            {
                Cross_ExtendLines(below_row, run_top, run_bottom);
            }
        }
    }

    /* 4. 综合判据 */
    candidate = (cross_run_len >= CROSS_LOST_MIN)
                && valid_below
                && valid_above
                && (run_top >= CROSS_TOP_MARGIN)
                && (Island_State == 0)
                && (Left_Island_Flag == 0)
                && (Right_Island_Flag == 0);

    if(candidate)
    {
        if(cross_confirm < CROSS_CONFIRM_FRAMES)
        {
            cross_confirm++;
        }

        if(cross_confirm >= CROSS_CONFIRM_FRAMES)
        {
            Cross_Flag = 1;
            cross_hold = CROSS_HOLD_FRAMES;
        }
    }
    else
    {
        cross_confirm = 0;
    }

    /* 5. 保持：过了路口再继续直行 CROSS_HOLD_FRAMES 帧 */
    if(Cross_Flag)
    {
        if(cross_hold > 0)
        {
            cross_hold--;
        }
        else
        {
            Cross_Flag = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前两边都丢线的连续行数
//-------------------------------------------------------------------------------------------------------------------
int Cross_GetRunLen(void)
{
    return cross_run_len;
}
