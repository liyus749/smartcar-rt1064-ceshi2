/*********************************************************************************************************************
* 十字识别：近处双边有效 + 中间连续双丢且向左右展开为白区 + 远处双边重新出现。
* 连续两帧确认后补线，舵机仍按补出的中线控制；赛道恢复后尽快退出十字减速状态。
********************************************************************************************************************/
#include "Cross.h"

extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];
extern int Left_Lost_Flag[MT9V03X_H];
extern int Right_Lost_Flag[MT9V03X_H];
extern volatile int Left_Line[MT9V03X_H];
extern volatile int Right_Line[MT9V03X_H];
extern volatile int Mid_Line[MT9V03X_H];
extern volatile int Road_Wide[MT9V03X_H];
extern volatile int Island_State;

static uint8 cross_confirm;
static uint16 cross_hold;
static uint8 cross_recovered;
static volatile int cross_run_len;
static int cross_near_left;
static int cross_near_right;
static uint8 cross_near_valid;

static int Pair_Valid(int row)
{
    return !Left_Lost_Flag[row] && !Right_Lost_Flag[row] &&
           (Right_Line[row] > Left_Line[row]);
}

// 黑色遮挡也会双丢：必须确认缺口同时向原赛道左右两侧展开为白区。
static int Gap_IsWhite(int top, int bottom, int l, int r)
{
    int row;
    int count = 0;
    int mid = (l + r) / 2;
    int outside_l = l - CROSS_SIDE_OFFSET;
    int outside_r = r + CROSS_SIDE_OFFSET;

    if((outside_l <= 2) || (outside_r >= MT9V03X_W - 3)) { return 0; }
    for(row = top; row <= bottom; row++)
    {
        if((image_two_value[row][outside_l] == IMG_WHITE) &&
           (image_two_value[row][l] == IMG_WHITE) &&
           (image_two_value[row][mid] == IMG_WHITE) &&
           (image_two_value[row][r] == IMG_WHITE) &&
           (image_two_value[row][outside_r] == IMG_WHITE))
        {
            count++;
        }
    }
    return count * 3 >= (bottom - top + 1) * 2;
}

static void Cross_FillLines(int above, int below, int top, int bottom)
{
    int row;
    int lb = Left_Line[below];
    int rb = Right_Line[below];
    int la = Left_Line[above];
    int ra = Right_Line[above];
    int den = below - above;

    for(row = top; row <= bottom; row++)
    {
        int l = lb + (la - lb) * (below - row) / den;
        int r = rb + (ra - rb) * (below - row) / den;
        Left_Line[row] = l;
        Right_Line[row] = r;
        Left_Lost_Flag[row] = Right_Lost_Flag[row] = 0;
        Mid_Line[row] = (l + r) / 2;
        Road_Wide[row] = r - l;
    }
}

// 已确认的十字进入近处后，用真实远端出口和最近的近端锚点延续引导。
// 必须仍看见连续双边出口和横向白区；黑色遮挡/出口消失时不补线。
static int Cross_FillFromExit(void)
{
    int above;
    if(!cross_near_valid) { return 0; }
    for(above = MT9V03X_H - 2 - CROSS_LOST_MIN; above >= CROSS_TOP_MARGIN; above--)
    {
        int count = 0;
        int row;
        for(row = above; (row >= 2) && Pair_Valid(row); row--) { count++; }
        if(count < CROSS_VALID_ABOVE) { continue; }
        if((Right_Line[above] - Left_Line[above]) >
           (cross_near_right - cross_near_left + 2 * CROSS_SIDE_OFFSET)) { continue; }
        if(!Gap_IsWhite(above + 1, MT9V03X_H - 2, cross_near_left, cross_near_right)) { continue; }

        Left_Line[MT9V03X_H - 1] = cross_near_left;
        Right_Line[MT9V03X_H - 1] = cross_near_right;
        Cross_FillLines(above, MT9V03X_H - 1, above + 1, MT9V03X_H - 1);
        return 1;
    }
    return 0;
}

void Cross_Init(void)
{
    Cross_Flag = 0;
    cross_confirm = cross_hold = cross_recovered = 0;
    cross_run_len = 0;
    cross_near_valid = 0;
}

void Cross_Check(void)
{
    int row = MT9V03X_H - 2; // 底行可能受车头遮挡，不让单独一行丢线挡住中部十字。
    int selected_top = -1;
    int selected_bottom = -1;
    int selected_above = -1;
    int selected_below = -1;
    int selected_len = 0;
    int candidate;

    cross_run_len = 0;
    if(Island_State != 0)
    {
        Cross_Init();
        return;
    }

    while(row >= CROSS_TOP_MARGIN)
    {
        int bottom;
        int top;
        int length;
        int above_count = 0;
        int below_count = 0;
        int i;

        if(!Left_Lost_Flag[row] || !Right_Lost_Flag[row])
        {
            row--;
            continue;
        }
        bottom = row;
        while((row >= 2) && Left_Lost_Flag[row] && Right_Lost_Flag[row]) { row--; }
        top = row + 1;
        length = bottom - top + 1;
        if(length > cross_run_len) { cross_run_len = length; }
        if((length < CROSS_LOST_MIN) || (top < CROSS_TOP_MARGIN)) { continue; }

        // 两侧锚点必须来自紧邻缺口的连续实测赛道，不能拼接零散噪点。
        for(i = bottom + 1; (i < MT9V03X_H) && Pair_Valid(i); i++) { below_count++; }
        for(i = top - 1; (i >= 2) && Pair_Valid(i); i--) { above_count++; }
        if((below_count < CROSS_VALID_BELOW) || (above_count < CROSS_VALID_ABOVE)) { continue; }
        if(!Gap_IsWhite(top, bottom, Left_Line[bottom + 1], Right_Line[bottom + 1])) { continue; }
        if(length > selected_len)
        {
            selected_top = top;
            selected_bottom = bottom;
            selected_above = top - 1;
            selected_below = bottom + 1;
            selected_len = length;
        }
    }

    candidate = (selected_top >= 0);
    if(candidate)
    {
        if(cross_confirm < CROSS_CONFIRM_FRAMES) { cross_confirm++; }
        if(cross_confirm >= CROSS_CONFIRM_FRAMES)
        {
            Cross_Flag = 1;
            cross_hold = CROSS_HOLD_FRAMES;
        }
        cross_recovered = 0;
        // 第一帧候选不改丢线标志，未确认的弯道/遮挡不会被擅自补成十字。
        if(Cross_Flag)
        {
            int near = selected_below;
            while((near + 1 < MT9V03X_H) && Pair_Valid(near + 1)) { near++; }
            cross_near_left = Left_Line[near];
            cross_near_right = Right_Line[near];
            cross_near_valid = 1;
            Cross_FillLines(selected_above, selected_below, selected_top, selected_bottom);
        }
    }
    else
    {
        int near_valid = 0;
        cross_confirm = 0;
        for(row = MT9V03X_H - 2; row >= MT9V03X_H - 1 - CROSS_EXIT_ROWS; row--)
        {
            if(Pair_Valid(row)) { near_valid++; }
        }
        if(near_valid == CROSS_EXIT_ROWS)
        {
            if(cross_recovered < CROSS_EXIT_FRAMES) { cross_recovered++; }
        }
        else
        {
            cross_recovered = 0;
        }
        if(Cross_Flag)
        {
            if(cross_hold > 0) { cross_hold--; }
            if((near_valid < CROSS_EXIT_ROWS) && (cross_hold > 0))
            {
                Cross_FillFromExit();
            }
            if((cross_hold == 0) || (cross_recovered >= CROSS_EXIT_FRAMES))
            {
                Cross_Flag = 0;
                cross_hold = 0;
                cross_near_valid = 0;
            }
        }
    }
}

int Cross_GetRunLen(void)
{
    return cross_run_len;
}
