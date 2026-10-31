#include "zf_common_headfile.h"
#include "island.h"
#include <string.h>
#include <stdlib.h>

//数字图像数组
const uint8 Image_Flags[][9][8]={
        {                                //0
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //1
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //2
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //3
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //4
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00},
},
{                                        //5
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //6
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //7
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //8
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //9
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
};

//常用基本变量
extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];//二值化后的原数组
extern volatile int Left_Line[MT9V03X_H]; //左边线数组
extern volatile int Right_Line[MT9V03X_H];//右边线数组
extern volatile int Search_Stop_Line;     //搜索截止行
extern volatile int Boundry_Start_Left;   //左右边界起始点
extern volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
extern volatile int Left_Lost_Time;       //边界丢线数
extern volatile int Right_Lost_Time; 
extern volatile int Both_Lost_Time;//两边同时丢线数
extern int Left_Lost_Flag[MT9V03X_H];   /* per-row lost flags (LineDetect.c) */
extern int Right_Lost_Flag[MT9V03X_H];
extern volatile float Err;//摄像头误差

//十字变量
extern volatile int Cross_Flag; 

//环岛变量
volatile int Island_State=0;     //环岛状态标志
volatile int Left_Island_Flag=0; //左右环岛标志
volatile int Right_Island_Flag=0;//左右环岛标志
volatile int Left_Up_Guai[2];    //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
volatile int Right_Up_Guai[2];   //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列

extern volatile int Mid_Line[MT9V03X_H];
volatile int monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列


volatile int monotonicity_change_left_flag=0;//不转折是0
volatile int monotonicity_change_right_flag=0;//不转折是0
volatile int continuity_change_right_flag=0; //连续是0
volatile int continuity_change_left_flag=0;  //连续是0

volatile int left_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
volatile int right_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列

volatile float kk;

//斑马线
extern volatile int Zebra_State;

//坡道标志位
extern volatile int Ramp_Flag;//坡道标志




/*-------------------------------------------------------------------------------------------------------------------
  @brief     环岛检测
  @param     null
  @return    null
  Sample     Island_Detect(void);
  @note      利用四个拐点判别函数，单调性改变函，连续性数撕裂点，分为8步
-------------------------------------------------------------------------------------------------------------------*/
/* [20261021] "island arc" feature: count rows whose horizontal scan shows >=3 black runs.
   Normal track row = 2 runs (left border + right border). The inner/outer arcs of an
   island add runs. Measured on 94x48 downsampled images (20261021 20:29, hand-push
   through the right island): island in view = 18~36 rows / max 6~7 runs;
   normal track = 3~10 rows / max 3~4 runs. mean_x tells which side the arc sits on.
   English comments only - this file is GBK. */
volatile int island_arc_rows   = 0;
volatile int island_arc_max    = 0;
volatile int island_arc_mean_x = MT9V03X_W / 2;
volatile int island_arc_peak   = 0;   /* max arc rows seen since last start (reset by main.c) */
volatile int island_arc_enable = 0;   /* set by main.c: 0=disabled, 1=seven-stage visual control */
/* [20261026] which rule set the island state - recorded so the blackbox can prove it */
volatile int island_trigger_reason = 0;
static void Island_Arc_Scan(void)
{
    int y, x, s, runs, w;
    int mx = 0, rows = 0;
    int xsum = 0, n = 0;              /* all black runs (fallback) */
    int xsum_mid = 0, n_mid = 0;      /* middle runs only = the island arc */

    for(y = 12; y < 88; y++)
    {
        int cx[10];
        runs = 0; s = -1;
        for(x = 0; x < MT9V03X_W; x++)
        {
            if(image_two_value[y][x] == IMG_BLACK)
            {
                if(s < 0) s = x;
            }
            else if(s >= 0)
            {
                w = x - s;
                if(w >= 4)
                {
                    if(runs < 10) { cx[runs] = (s + x) / 2; }
                    runs++; xsum += (s + x) / 2; n++;
                }
                s = -1;
            }
        }
        if(s >= 0 && MT9V03X_W - s >= 4)
        {
            if(runs < 10) { cx[runs] = (s + MT9V03X_W) / 2; }
            runs++; xsum += (s + MT9V03X_W) / 2; n++;
        }
        if(runs >= 3)
        {
            int k;
            rows++;
            /* skip the outermost two runs (track borders); what is left is the ring arc */
            for(k = 1; k < (runs - 1) && k < 9; k++) { xsum_mid += cx[k]; n_mid++; }
        }
        if(runs > mx) mx = runs;
    }
    island_arc_rows = rows;
    island_arc_max  = mx;
    if(rows > island_arc_peak) { island_arc_peak = rows; }
    if(n_mid > 0)      { island_arc_mean_x = xsum_mid / n_mid; }
    else if(n > 0)     { island_arc_mean_x = xsum / n; }
    else               { island_arc_mean_x = MT9V03X_W / 2; }
}

/* Vision-only seven-stage ring controller. All coordinates are image coordinates:
   y=0 far, y=99 near. A missing point has y=-1. No angle/time-based success. */
IslandDebug island_debug;
uint8 island_patch_left[MT9V03X_H], island_patch_right[MT9V03X_H];
volatile int island_fault = 0;
static int confirm_count, candidate_dir, clear_count, state_age, previous_state;
static int exit_clear_count, exit_ready, recovery_count;
static int entry_guided, exit_guided;
/* A separate scan follows the white corridor connected to the near road. This
   prevents a tracker stuck on the outside shoulder from hiding a detached ring. */
static int contour[2][MT9V03X_H];
static uint8 contour_ok[2][MT9V03X_H], arc_confirmed[2];
static uint8 island_moving = 1;
void Island_SetMoving(uint8 moving) { island_moving = moving; }

static int ClampX(int x)
{
    if(x < 0) return 0;
    if(x >= MT9V03X_W) return MT9V03X_W - 1;
    return x;
}

static IslandPoint MissingPoint(void)
{
    IslandPoint p; p.x = -1; p.y = -1; return p;
}

static int SupportedEdge(int y, int x, int side)
{
    int k, outside = side ? 1 : -1;
    if(x < 5 || x >= MT9V03X_W-5) return 0;
    for(k=1;k<=3;k++)
        if(image_two_value[y][x+outside*k] != IMG_BLACK) return 0;
    for(k=0;k<3;k++)
        if(image_two_value[y][x-outside*k] != IMG_WHITE) return 0;
    return 1;
}

static void ScanVisibleContour(void)
{
    int y, center=MT9V03X_W/2, connected=1, start=-1;
    memset(contour_ok,0,sizeof(contour_ok));
    /* Prefer a current near-road pair; do not seed from arbitrary far white blobs. */
    for(y=MT9V03X_H-2;y>=75;y--)
        if(!Left_Lost_Flag[y] && !Right_Lost_Flag[y] &&
           Right_Line[y]-Left_Line[y]>=15)
        { center=(Left_Line[y]+Right_Line[y])/2; break; }
    center=ClampX(center);
    /* The lowest row can be hidden by the chassis. Start at the nearest visible
       white road row, not at an assumed valid row99. Once started, a black
       barrier ABOVE that seed still disconnects all rows beyond the barrier. */
    for(y=MT9V03X_H-1;y>=75;y--)
        if(image_two_value[y][center]==IMG_WHITE) { start=y;break; }
    if(start<0) return;
    for(y=start;y>=2;y--)
    {
        int l,r,d,found=connected && image_two_value[y][center]==IMG_WHITE;
        for(d=1;connected && !found && d<=24;d++)
        {
            if(center-d>=3 && image_two_value[y][center-d]==IMG_WHITE)
            { center-=d; found=1; }
            else if(center+d<MT9V03X_W-3 && image_two_value[y][center+d]==IMG_WHITE)
            { center+=d; found=1; }
        }
        if(!found) { connected=0;continue; }
        l=r=center;
        while(l>2 && image_two_value[y][l-1]==IMG_WHITE) l--;
        while(r<MT9V03X_W-3 && image_two_value[y][r+1]==IMG_WHITE) r++;
        contour[0][y]=l; contour[1][y]=r;
        contour_ok[0][y]=(uint8)SupportedEdge(y,l,0);
        contour_ok[1][y]=(uint8)SupportedEdge(y,r,1);
        if(r-l>=15 && r-l<=150) center=(l+r)/2;
    }
}

static int RealPoint(int side, int y)
{
    return y>=0 && y<MT9V03X_H && contour_ok[side][y];
}

static IslandPoint PointAt(int side, int y)
{
    IslandPoint p = MissingPoint();
    if(RealPoint(side,y)) { p.y=y;p.x=contour[side][y]; }
    return p;
}

static void ApplyVisibleContour(void)
{
    int y;
    for(y=2;y<MT9V03X_H;y++)
    {
        Left_Lost_Flag[y]=!contour_ok[0][y];
        Right_Lost_Flag[y]=!contour_ok[1][y];
        if(contour_ok[0][y]) Left_Line[y]=contour[0][y];
        if(contour_ok[1][y]) Right_Line[y]=contour[1][y];
    }
}

static int Stable(int side, int y, int step)
{
    int i, last;
    if(!RealPoint(side, y)) return 0;
    last = contour[side][y];
    for(i = 1; i <= 3; i++)
    {
        int yy = y + i * step, x;
        if(!RealPoint(side, yy)) return 0;
        x = contour[side][yy];
        if(abs(x - last) > 5) return 0;
        last = x;
    }
    return 1;
}

/* The gap endpoints are supported by real segments on both sides. A smooth
   inward protrusion supplies the relaxed lower corner used to find an exit. */
static IslandSide ExtractSide(int side)
{
    IslandSide f;
    int y, best_mid = 0, best_down = 0;
    arc_confirmed[side]=0;
    f.up = f.mid = f.down = MissingPoint();
    f.gap = f.near_valid = f.bottom_lost = f.valid = 0;
    for(y = 8; y < MT9V03X_H; y++)
    {
        if(RealPoint(side, y))
        {
            f.valid++;
            if(y >= 75) f.near_valid++;
        }
        else if(y >= 90) f.bottom_lost++;
    }
    for(y = 9; y < 94; y++)
    {
        if(!RealPoint(side, y))
        {
            int top = y, end;
            while(y < 95 && !RealPoint(side, y)) y++;
            end = y - 1;
            if(end - top + 1 >= ISLAND_GAP_MIN && end - top + 1 > f.gap &&
               Stable(side, top - 1, -1) && Stable(side, end + 1, 1))
            {
                f.gap = end - top + 1;
                f.up = PointAt(side, top - 1);
                f.down = PointAt(side, end + 1);
            }
        }
    }
    for(y = 12; y <= 90; y++)
    {
        int x, a, b, inward;
        if(!Stable(side, y, -1) || !Stable(side, y, 1) ||
           !RealPoint(side, y-5) || !RealPoint(side, y+5)) continue;
        x = contour[side][y];
        a = contour[side][y-5];
        b = contour[side][y+5];
        inward = side ? ((a-x < b-x) ? a-x : b-x) : ((x-a < x-b) ? x-a : x-b);
        if(inward >= ISLAND_CORNER_PROM && inward > best_mid)
        { best_mid = inward; f.mid = PointAt(side, y); }
        if(f.gap == 0 && inward >= ISLAND_CORNER_PROM && inward > best_down)
        { best_down = inward; f.down = PointAt(side, y); }
    }
    /* Upper corner: stable upper branch followed by an outward jump below it.
       Unlike legacy corner tests, invalid/sentinel rows cannot be corners. */
    if(f.up.y < 0)
    {
        for(y = 12; y < 89; y++)
        {
            int x, lower, jump;
            if(!Stable(side, y, -1) || !Stable(side, y+3, 1)) continue;
            x = contour[side][y];
            lower = contour[side][y+3];
            jump = side ? x-lower : lower-x;
            if(jump >= ISLAND_JUMP_MIN) { f.up = PointAt(side, y+3); break; }
        }
    }
    /* Prefer all THREE corners on one continuous road-facing arc. In physical
       image coordinates, a left island protrudes RIGHT into the road (x maximum).
       A right island is mirrored. Do not combine unrelated global extrema. */
    {
        int bottom, best_height=0;
        for(bottom=94;bottom>=25;bottom--)
        {
            int top,mid=-1,peak=-1000,score,yy,trend,shape=1;
            int n=side ? -contour[side][bottom]:contour[side][bottom];
            int lower=RealPoint(side,bottom+1) ? (side ? -contour[side][bottom+1]:contour[side][bottom+1]):n-ISLAND_JUMP_MIN;
            if(!RealPoint(side,bottom) || n-lower<ISLAND_JUMP_MIN) continue;
            top=bottom;
            while(top>8 && RealPoint(side,top-1) &&
                  abs(contour[side][top]-contour[side][top-1])<ISLAND_JUMP_MIN) top--;
            if(top<=8 || bottom-top<ISLAND_ARC_MIN_HEIGHT) continue;
            n=side ? -contour[side][top]:contour[side][top];
            if(RealPoint(side,top-1) && n-(side ? -contour[side][top-1]:contour[side][top-1])<ISLAND_JUMP_MIN) continue;
            for(yy=top;yy<=bottom;yy++)
            {
                int v=side ? -contour[side][yy]:contour[side][yy];
                if(v>peak) { peak=v;mid=yy; }
            }
            if(mid-top<6 || bottom-mid<6 ||
               peak-(side ? -contour[side][top]:contour[side][top])<ISLAND_CORNER_PROM ||
               peak-(side ? -contour[side][bottom]:contour[side][bottom])<ISLAND_CORNER_PROM) continue;
            /* Flat small peaks are allowed; isolated spikes and double humps are not. */
            for(yy=mid-1;yy<=mid+1;yy++)
                if((side ? -contour[side][yy]:contour[side][yy])<peak-2) shape=0;
            trend=side ? -contour[side][top]:contour[side][top];
            for(yy=top;yy<=mid;yy++)
            {
                int v=side ? -contour[side][yy]:contour[side][yy];
                if(v<trend-2) shape=0;
                if(v>trend) trend=v;
            }
            trend=peak;
            for(yy=mid;yy<=bottom;yy++)
            {
                int v=side ? -contour[side][yy]:contour[side][yy];
                if(v>trend+2) shape=0;
                if(v<trend) trend=v;
            }
            score=bottom-top;
            if(shape && score>best_height)
            {
                best_height=score;arc_confirmed[side]=1;
                f.up=PointAt(side,top);f.mid=PointAt(side,mid);f.down=PointAt(side,bottom);
            }
        }
    }
    return f;
}

static int CornerPair(IslandSide *f)
{
    return f->mid.y >= 0 && f->up.y >= 0 && f->up.y < f->mid.y &&
           abs(f->up.x - f->mid.x) <= ISLAND_PAIR_X_MAX;
}

static int RingShape(IslandSide *f, int side)
{
    int chord, bulge;
    if(!arc_confirmed[side] || !CornerPair(f)) return 0;
    if(f->down.y < 0)
        return f->bottom_lost >= 3 && (side ? f->up.x-f->mid.x : f->mid.x-f->up.x) >= ISLAND_CORNER_PROM;
    if(f->mid.y-f->up.y < 6 || f->down.y-f->mid.y < 6) return 0;
    chord = f->up.x + (f->down.x-f->up.x)*(f->mid.y-f->up.y)/(f->down.y-f->up.y);
    bulge = side ? chord-f->mid.x : f->mid.x-chord;
    return bulge >= ISLAND_CORNER_PROM &&
           (side ? f->down.x-f->mid.x : f->mid.x-f->down.x) >= ISLAND_CORNER_PROM;
}

/* One uninterrupted white-to-black edge, scanned from the ring side. Used only
   after the mouth is confirmed, when a tracked left/right label may have swapped. */
static int TurnEdge(int dir, int y)
{
    int x, k;
    if(y < 0 || y >= MT9V03X_H) return -1;
    for(k = 3; k < MT9V03X_W - 3; k++)
    {
        x = dir > 0 ? k : MT9V03X_W - 1 - k;
        if(dir > 0)
        {
            if(image_two_value[y][x-3] == IMG_WHITE && image_two_value[y][x-2] == IMG_WHITE &&
               image_two_value[y][x-1] == IMG_WHITE && image_two_value[y][x] == IMG_BLACK &&
               image_two_value[y][x+1] == IMG_BLACK && image_two_value[y][x+2] == IMG_BLACK) return x;
        }
        else if(image_two_value[y][x+3] == IMG_WHITE && image_two_value[y][x+2] == IMG_WHITE &&
                image_two_value[y][x+1] == IMG_WHITE && image_two_value[y][x] == IMG_BLACK &&
                image_two_value[y][x-1] == IMG_BLACK && image_two_value[y][x-2] == IMG_BLACK) return x;
    }
    return -1;
}

static IslandPoint RescanUpper(int dir)
{
    IslandPoint p = MissingPoint();
    int y;
    for(y = 12; y < 88; y++)
    {
        int a = TurnEdge(dir, y-3), b = TurnEdge(dir, y), c = TurnEdge(dir, y+3);
        if(a >= 0 && b >= 0 && c >= 0 && abs(a-b) <= 5 && abs(c-b) >= ISLAND_JUMP_MIN)
        { p.x = b; p.y = y; break; }
    }
    return p;
}

/* Synthetic provenance is separate from raw features. Invalid/identical anchors
   generate no line. A line never crosses the opposite visible boundary. */
static int Patch(int side, IslandPoint a, IslandPoint b)
{
    int y,count=0;
    if(a.y < 0 || b.y < 0 || a.y == b.y) return 0;
    if(a.y > b.y) { IslandPoint t = a; a = b; b = t; }
    if(a.y < 0 || b.y >= MT9V03X_H) return 0;
    for(y = a.y; y <= b.y; y++)
    {
        int x = ClampX(a.x + (b.x-a.x)*(y-a.y)/(b.y-a.y));
        int other_ok = side ? !Left_Lost_Flag[y] : !Right_Lost_Flag[y];
        int other = side ? Left_Line[y] : Right_Line[y];
        if(other_ok && (side ? x-other : other-x) < 15) continue;
        if(other_ok && image_two_value[y][(x+other)/2] != IMG_WHITE) continue;
        if(side) { Right_Line[y] = x; Right_Lost_Flag[y] = 0; island_patch_right[y] = 1; }
        else     { Left_Line[y] = x; Left_Lost_Flag[y] = 0; island_patch_left[y] = 1; }
        count++;
    }
    return count;
}

static int RealInsideCurve(int dir)
{
    int side=dir>0 ? 1:0, y,near=-1,far=-1,valid=0,taper=0;
    for(y=97;y>=60;y--) if(RealPoint(side,y))
    {
        valid++;
        if(near<0) near=y;
        if(y<=66 && far<0) far=y;
    }
    if(valid<20 || near<90 || far<0) return 0;
    if(RealPoint(!side,near) && RealPoint(!side,far))
    {
        int nw=contour[1][near]-contour[0][near];
        int fw=contour[1][far]-contour[0][far];
        if(nw>fw) taper=(nw-fw)/2;
    }
    return dir*(contour[side][near]-contour[side][far])>taper+ISLAND_CURVE_SHIFT;
}

static int Confirm(int condition, int frames)
{
    if(!condition) { confirm_count = 0; return 0; }
    if(confirm_count < frames) confirm_count++;
    return confirm_count >= frames;
}

static void ChangeState(int state, int reason)
{
    Island_State = state;
    island_debug.reason = reason;
    island_debug.transitions++;
    confirm_count = state_age = 0;
}

void Island_Reset(void)
{
    memset(&island_debug, 0, sizeof(island_debug));
    memset(island_patch_left, 0, sizeof(island_patch_left));
    memset(island_patch_right, 0, sizeof(island_patch_right));
    Island_State = Left_Island_Flag = Right_Island_Flag = island_fault = 0;
    confirm_count = candidate_dir = clear_count = state_age = previous_state = 0;
    exit_clear_count = exit_ready = recovery_count = 0;
    entry_guided = exit_guided = 0;
    island_arc_peak = island_trigger_reason = 0;
}

void Island_Detect(void)
{
    IslandSide *inner, *outer;
    IslandPoint upper, bottom;
    int dir, side, candidate = 0, gate, y, pair_near = 0;
    memset(island_patch_left, 0, sizeof(island_patch_left));
    memset(island_patch_right, 0, sizeof(island_patch_right));
    Island_Arc_Scan();
    ScanVisibleContour();
    island_debug.left = ExtractSide(0); island_debug.right = ExtractSide(1);
    island_debug.lost_l = Left_Lost_Time; island_debug.lost_r = Right_Lost_Time;
    island_debug.span = Search_Stop_Line;
    island_debug.scan = island_debug.patch_l = island_debug.patch_r = 0;
    for(y = 60; y < MT9V03X_H; y++)
        if(RealPoint(0,y) && RealPoint(1,y) && contour[1][y]-contour[0][y] >= 15 &&
           contour[1][y]-contour[0][y] <= 180) pair_near++;
    island_debug.pairs = pair_near;
    /* Only an ordered upper/middle/lower pattern (or curved approach with
       lower loss) can claim a ring; arcs/loss alone never start a manoeuvre. */
    if(RingShape(&island_debug.left,0) && island_debug.right.valid >= 30 &&
       ((island_debug.left.down.y > island_debug.left.mid.y) || island_debug.left.bottom_lost >= 3) &&
       island_debug.left.mid.y >= 0) candidate = 1;
    if(RingShape(&island_debug.right,1) && island_debug.left.valid >= 30 &&
       ((island_debug.right.down.y > island_debug.right.mid.y) || island_debug.right.bottom_lost >= 3) &&
       island_debug.right.mid.y >= 0)
        candidate = candidate == 1 ? 0 : -1;
    island_debug.candidate = candidate;
    if(!island_arc_enable) { Island_Reset(); return; }
    if(Island_State == 0)
    {
        /* Rearm only after real corridor recovery, not after elapsed time. */
        if(recovery_count > 0)
        {
            if(pair_near >= 30 && island_arc_rows <= 10) recovery_count--;
            island_debug.wait = 8;
            return;
        }
        if(Cross_Flag) candidate = 0;
        if(candidate != candidate_dir) { candidate_dir = candidate; confirm_count = 0; }
        if(Confirm(candidate != 0, ISLAND_CONFIRM))
        {
            island_debug.dir = candidate;
            Left_Island_Flag = candidate > 0; Right_Island_Flag = candidate < 0;
            inner = candidate > 0 ? &island_debug.left : &island_debug.right;
            ChangeState(inner->bottom_lost >= 3 ? 2 : 1, 1);
            island_trigger_reason = 1; island_debug.entries++;
        }
        island_debug.wait = candidate ? 1 : 0;
        island_debug.confirm = confirm_count;
        return;
    }
    dir = island_debug.dir; side = dir < 0;
    inner = dir > 0 ? &island_debug.left : &island_debug.right;
    outer = dir > 0 ? &island_debug.right : &island_debug.left;
    upper = inner->up;
    ApplyVisibleContour();
    /* Freeze one state per image. Sensors are read before any patch is applied. */
    switch(Island_State)
    {
    case 1: /* far ring: continue straight along lower/middle/upper anchors */
        Patch(side, inner->down, inner->mid); Patch(side, inner->mid, inner->up);
        gate = inner->down.y >= ISLAND_MOUTH_Y || inner->bottom_lost >= 3;
        if(Confirm(gate, ISLAND_CONFIRM)) ChangeState(2, 2);
        if(candidate == 0 && pair_near >= 30 && island_arc_rows <= 5) clear_count++;
        else clear_count = 0;
        if(clear_count >= 5) { Island_Reset(); return; }
        island_debug.wait = 2; break;
    case 2: /* mouth: retain straight guidance until the upper corner is near */
        Patch(side, inner->mid, inner->up);
        if(upper.y < 0) { upper = RescanUpper(dir); island_debug.scan = 1; }
        gate = upper.y >= ISLAND_ENTRY_Y && (inner->bottom_lost >= 3 || inner->down.y >= ISLAND_MOUTH_Y);
        if(Confirm(gate, ISLAND_CONFIRM)) { ChangeState(3, 3); entry_guided=0; }
        island_debug.wait = 3; break;
    case 3: /* entry: opposite boundary goes toward the ring-side upper corner */
        if(upper.y < 0) { upper = RescanUpper(dir); island_debug.scan = 1; }
        bottom = MissingPoint();
        for(y = 99; y >= 75; y--) if(RealPoint(!side,y)) { bottom = PointAt(!side,y); break; }
        if(upper.y >= 0)
        {
            upper.x = ClampX(upper.x + dir*15);
            if(Patch(!side, upper, bottom)>=ISLAND_GUIDE_MIN_ROWS && entry_guided<ISLAND_CONFIRM) entry_guided++;
        }
        gate = entry_guided>=ISLAND_CONFIRM && RealInsideCurve(dir) &&
               inner->near_valid >= 20 && outer->near_valid >= 12 && inner->up.y < 25;
        if(Confirm(gate, ISLAND_CONFIRM)) { ChangeState(4,4); exit_clear_count = exit_ready = 0; }
        island_debug.wait = 4; break;
    case 4: /* normal bend; only a NEW opposite-side lower corner can be exit */
        if(outer->down.y < 0 && outer->valid>=30 && outer->near_valid>=12)
        { if(exit_clear_count < ISLAND_CONFIRM) exit_clear_count++; }
        else if(!exit_ready) exit_clear_count = 0;
        if(exit_clear_count >= ISLAND_CONFIRM) exit_ready = 1;
        gate = exit_ready && outer->down.y >= ISLAND_EXIT_Y && inner->valid >= 25;
        if(Confirm(gate, ISLAND_CONFIRM)) { ChangeState(5,5); exit_guided=0; }
        island_debug.wait = exit_ready ? 5 : 9; break;
    case 5: /* exit: connect real outside lower corner to an inside boundary point */
        upper = MissingPoint();
        if(outer->down.y >= 0)
            for(y = outer->down.y-20; y >= 10; y--)
                if(RealPoint(side,y)) { upper = PointAt(side,y); upper.x = ClampX(upper.x+dir*15); break; }
        if(Patch(!side, upper, outer->down)>=ISLAND_GUIDE_MIN_ROWS) exit_guided=1;
        gate = exit_guided && (outer->down.y < 0 || outer->down.y >= ISLAND_LOW_Y);
        if(Confirm(gate, ISLAND_CONFIRM)) ChangeState(6,6);
        island_debug.wait = 6; break;
    case 6: /* align after exit; only current real anchors may generate guidance */
        upper = MissingPoint(); bottom = MissingPoint();
        for(y = 35; y < 80; y++) if(RealPoint(side,y)) { upper = PointAt(side,y); upper.x = ClampX(upper.x+dir*15); break; }
        for(y = 99; y >= 80; y--) if(RealPoint(!side,y)) { bottom = PointAt(!side,y); break; }
        Patch(!side, upper, bottom);
        gate = (inner->up.y >= 0 && inner->valid >= 15) || (pair_near >= 30 && island_arc_rows <= 10);
        if(Confirm(gate, ISLAND_CONFIRM)) ChangeState(7,7);
        island_debug.wait = 7; break;
    case 7: /* straight recovery; completion requires measured corridor recovery */
        Patch(side, inner->up, inner->down);
        gate = pair_near >= 30 && island_arc_rows <= 10 &&
               (inner->up.y < 0 || inner->up.y >= ISLAND_LOW_Y);
        if(Confirm(gate, ISLAND_CONFIRM))
        {
            ChangeState(0,8); Left_Island_Flag = Right_Island_Flag = 0;
            recovery_count = ISLAND_CONFIRM; island_debug.exits++;
        }
        island_debug.wait = 8; break;
    default: island_fault = 2; break;
    }
    if(previous_state != Island_State) { state_age = 0; previous_state = Island_State; }
    else if(island_moving && state_age < ISLAND_STALL_FRAMES + 1) state_age++;
    /* A watchdog is an explicit fault/stop, NEVER an invented successful exit. */
    if(Island_State >= 3 && state_age > ISLAND_STALL_FRAMES) island_fault = 1;
    island_debug.confirm = confirm_count; island_debug.exit_ready = exit_ready;
    for(y = 0; y < MT9V03X_H; y++)
    { island_debug.patch_l += island_patch_left[y]; island_debug.patch_r += island_patch_right[y]; }
    Left_Up_Guai[0] = island_debug.left.up.y; Left_Up_Guai[1] = island_debug.left.up.x;
    Right_Up_Guai[0] = island_debug.right.up.y; Right_Up_Guai[1] = island_debug.right.up.x;
    left_down_guai[0] = island_debug.left.down.y; right_down_guai[0] = island_debug.right.down.y;
}

int Continuity_Change_Left(int start,int end)//连续性阈值设置为5
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    
    for(i=start;i>=end;i--)
    {
       if(abs(Left_Line[i]-Left_Line[i-1])>=5)//连续判断阈值是5,可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=5&&
              (Left_Line[i]-Left_Line[i-3])>=10&&
              (Left_Line[i]-Left_Line[i-4])>=10)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(left_up_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&  
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=5&& 
              (Left_Line[i]-Left_Line[i+3])>=10&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            left_up_line=i;//获取行数即可
            break;
        }
    }
    return left_up_line;//如果是MT9V03X_H-1，说明没有这么个拐点
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
//        if(right_down_line==0&&//只找第一个符合条件的点
//           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
//           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&  
//           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
//              (Right_Line[i]-Right_Line[i-2])<=-5&&
//              (Right_Line[i]-Right_Line[i-3])<=-10&&
//              (Right_Line[i]-Right_Line[i-4])<=-10)
			
			if(right_down_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=3&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=3&&  
           abs(Right_Line[i+2]-Right_Line[i+3])<=3&&
              (Right_Line[i]-Right_Line[i-2])<=-3&&
              (Right_Line[i]-Right_Line[i-3])<=-5&&
              (Right_Line[i]-Right_Line[i-4])<=-5)
        {
            right_down_line=i;//获取行数即可
            break;
        }
    }
    return right_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_up_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }    
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
//        if(right_up_line==0&&//只找第一个符合条件的点
//           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
//           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&  
//           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
//              (Right_Line[i]-Right_Line[i+2])<=-8&&
//              (Right_Line[i]-Right_Line[i+3])<=-15&&
//              (Right_Line[i]-Right_Line[i+4])<=-15)
			
			if(right_up_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&  
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
           abs(Right_Line[i]-Right_Line[i+2])>=5&&
           abs(Right_Line[i]-Right_Line[i+3])>=10&&
           abs(Right_Line[i]-Right_Line[i+4])>=15)
        {
            right_up_line=i;//获取行数即可
            break;
        }
    }
    return right_up_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
       return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护，在判断第i个点时
       start=MT9V03X_H-1-5; //要访问它前后5个点，数组两头的点要不能作为起点终点
    if(end<=5)
        end=5;
    if(start<=end)//递减计算，入口反了，直接返回0
      return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
        Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
        Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
        Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
        Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Left_Line[i]>=Left_Line[i+5]&&Left_Line[i]>=Left_Line[i-5]&&
        Left_Line[i]>=Left_Line[i+4]&&Left_Line[i]>=Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;

    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Right_Line[i]<=Right_Line[i+5]&&Right_Line[i]<=Right_Line[i-5]&&
        Right_Line[i]<=Right_Line[i+4]&&Right_Line[i]<=Right_Line[i-4]&&
        Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
        Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
        Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     元素标志位显示
  @param     二值化图片数组
  @return    null
  Sample     Image_Flag_Show(MT9V03X_W,image_two_value,Island_State);
  @note      要在图片显示前使用，传入标志位可更改
-------------------------------------------------------------------------------------------------------------------*/
void Image_Flag_Show(uint8 MT9V03XW,uint8(*InImg)[MT9V03XW],uint8 image_flag)
{
    for(uint8 H=1;H<8;H++)
    {
        for(uint8 W=1;W<7;W++)
        {
            switch (image_flag)
            {
               // case 0:     *(*(InImg+H)+W)=Image_Flags[0][H][W];    break;//环岛0~8
                case 1:     *(*(InImg+H)+W)=Image_Flags[1][H][W];    break;
                case 2:     *(*(InImg+H)+W)=Image_Flags[2][H][W];    break;
                case 3:     *(*(InImg+H)+W)=Image_Flags[3][H][W];    break;
                case 4:     *(*(InImg+H)+W)=Image_Flags[4][H][W];    break;
                case 5:     *(*(InImg+H)+W)=Image_Flags[5][H][W];    break;
                case 6:     *(*(InImg+H)+W)=Image_Flags[6][H][W];    break;
                case 7:     *(*(InImg+H)+W)=Image_Flags[7][H][W];    break;
                case 8:     *(*(InImg+H)+W)=Image_Flags[8][H][W];    break;
                case 9:     *(*(InImg+H)+W)=Image_Flags[9][H][W];    break;
                default:    break;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线--
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null
  Sample     K_Add_Boundry_Left(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)//--操作，start需要大
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Left_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Left_Line[i]>=MT9V03X_W-1)
        {
            Left_Line[i]=MT9V03X_W-1;
        }
        else if(Left_Line[i]<=0)
        {
            Left_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null    直接补边线
  Sample     K_Add_Boundry_Right(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Right_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Right_Line[i]>=MT9V03X_W-1)
        {
            Right_Line[i]=MT9V03X_W-1;
        }
        else if(Right_Line[i]<=0)
        {
            Right_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     根据斜率划线
  @param     输入斜率，定点，画一条黑线
  @return    null
  Sample     K_Draw_Line(k, 20,MT9V03X_H-1 ,0)
  @note      补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void K_Draw_Line(float k, int startX, int startY,int endY)
{
    int endX=0;

    if(startX>=MT9V03X_W-1)//限幅处理
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    endX=(int)((endY-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    Draw_Line(startX,startY,endX,endY);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取平均赛宽
  @param     int start_line,int end_line，起始行，中止行
  @return    这几行赛宽平均值
  Sample     road_wide=Get_Road_Wide(68,69);
  @note      ++运算，向下寻找，算出平均赛宽
-------------------------------------------------------------------------------------------------------------------*/
int Get_Road_Wide(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    int i=0,t=0;
    int road_wide=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    for(i=start_line;i<=end_line;i++)
    {
        road_wide+=Right_Line[i]-Left_Line[i];
    }
    road_wide=road_wide/(end_line-start_line+1);//平均赛宽
    return road_wide;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取左赛道边界斜率
  @param     int start_line,int end_line，起始行，中止行
  @return    两点之间的斜率
  Sample     k=Get_Left_K(68,69);
  @note      两点之间得出斜率，默认第一个参数小，第二个参数大
-------------------------------------------------------------------------------------------------------------------*/
float Get_Left_K(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Left_Line[start_line]-(float)Left_Line[end_line])/(end_line-start_line+1));
    return k;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取右赛道边界斜率
  @param     int start_line,int end_line，起始行，中止行
  @return    两点之间的斜率
  Sample     k=Get_Right_K(68,69);
  @note      两点之间得出斜率，默认第一个参数小，第二个参数大
-------------------------------------------------------------------------------------------------------------------*/
float Get_Right_K(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Right_Line[start_line]-(float)Right_Line[end_line])/(end_line-start_line+1));
    return k;
}
