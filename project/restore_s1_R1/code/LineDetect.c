/*********************************************************************************************************************
* 文件名称          LineDetect.c
* 功能简介          巡线方案切换实现（说明见 LineDetect.h）
********************************************************************************************************************/

#include "LineDetect.h"
#include "Contrast.h"

//===================================================================================================================
// Camera.c 里的全局量（只写，不重复定义）
//===================================================================================================================
extern uint8          image_two_value[MT9V03X_H][MT9V03X_W];
extern volatile int   Left_Line[MT9V03X_H];
extern volatile int   Right_Line[MT9V03X_H];
extern volatile int   Mid_Line[MT9V03X_H];
extern volatile int   Road_Wide[MT9V03X_H];
extern int            Left_Lost_Flag[MT9V03X_H];
extern int            Right_Lost_Flag[MT9V03X_H];
extern volatile int   Search_Stop_Line;
extern volatile int   Left_Lost_Time;
extern volatile int   Right_Lost_Time;
extern volatile int   Both_Lost_Time;
extern volatile int   Boundry_Start_Left;
extern volatile int   Boundry_Start_Right;
extern volatile uint8  Img_Disappear_Flag;

//===================================================================================================================
// 内部状态
//===================================================================================================================
static uint32 otsu_hist[256];
static float  th_smooth[OTSU_BANDS];
static uint8  th_band[OTSU_BANDS];

static uint8  g_mode    = LINE_MODE_CONTRAST;
static uint8  g_ref_col = MT9V03X_W / 2;
static volatile int g_otsu_bias = OTSU_BIAS_DEFAULT;
static int seed_left;
static int seed_right;
static uint8 seed_valid;

//===================================================================================================================
//  分段大津法
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对图像的某一段行做一次大津法，返回阈值
//-------------------------------------------------------------------------------------------------------------------
static int Otsu_Rows(const uint8 *image, uint8 row_begin, uint8 row_end)
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
    float  best_class_gap = 0.0f;
    uint32 best_back = 0, best_fore = 0;
    uint8  th        = 0;
    uint8  min_v;
    uint8  max_v;

    for(j = 0; j < 256; j++) { otsu_hist[j] = 0; }

    for(i = row_begin; i <= row_end; i++)
    {
        for(j = 0; j < MT9V03X_W; j++)
        {
            otsu_hist[image[i * MT9V03X_W + j]]++;
        }
    }

    for(min_v = 0;    (min_v < 255) && (0 == otsu_hist[min_v]); min_v++) { }
    for(max_v = 255;  (max_v > min_v) && (0 == otsu_hist[max_v]); max_v--) { }

    // 一整段都是十字白区时，没有两个灰度类别可分，不能把白灰度当成新阈值。
    if((int)max_v - (int)min_v < OTSU_MIN_CONTRAST) { return -1; }

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
            best_class_gap = micro_fore - micro_back;
            best_back = pix_back;
            best_fore = (uint32)pix_fore;
        }
    }

    // 十字整段白区也会有几十级纹理/传感器噪声。仅检查灰度跨度会把白与白重新分成两类，
    // 随时间平滑把阈值逐步推到白色内部，造成静止时边线忽有忽无。
    if(best_class_gap < OTSU_MIN_CLASS_GAP ||
       best_back*100 < amount*OTSU_MIN_CLASS_PERCENT ||
       best_fore*100 < amount*OTSU_MIN_CLASS_PERCENT) { return -1; }
    return th;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     分带大津法：每段自己的阈值，时间平滑，加偏置，再按行二值化
//-------------------------------------------------------------------------------------------------------------------
static void Threshold_And_Binarize(const uint8 *image)
{
    uint16 row;
    uint16 col;
    uint8  b;
    uint8  row_begin;
    uint8  row_end;
    int    th;

    for(b = 0; b < OTSU_BANDS; b++)
    {
        row_begin = (uint8)(b * (MT9V03X_H / OTSU_BANDS));
        row_end   = (uint8)(row_begin + (MT9V03X_H / OTSU_BANDS) - 1);

        {
            float otsu = (float)Otsu_Rows(image, row_begin, row_end);

            if(otsu >= 0.0f)
            {
                if(th_smooth[b] < 0.0f) { th_smooth[b] = otsu; }
                th_smooth[b] += OTSU_SMOOTH_K * (otsu - th_smooth[b]);
            }
            else if(th_smooth[b] < 0.0f)
            {
                th_smooth[b] = 128.0f; // 尚无历史值时，用中灰阈值区分全白/全黑。
            }

            th = (int)th_smooth[b] + g_otsu_bias;
            if(th > 255) { th = 255; }
            if(th < 1)   { th = 1;   }
            th_band[b] = (uint8)th;
        }
    }

    for(row = 0; row < MT9V03X_H; row++)
    {
        th = th_band[row / (MT9V03X_H / OTSU_BANDS)];

        for(col = 0; col < MT9V03X_W; col++)
        {
            image_two_value[row][col] = (image[row * MT9V03X_W + col] >= th) ? IMG_WHITE : IMG_BLACK;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     3x3 邻域滤波（去孤立噪点、填小黑洞）
//-------------------------------------------------------------------------------------------------------------------
static void Image_Filter(void)
{
    uint16 row;
    uint16 col;
    uint16 cnt;

    for(row = 1; row < MT9V03X_H - 1; row++)
    {
        for(col = 1; col < MT9V03X_W - 1; col++)
        {
            cnt = 0;

            if(IMG_WHITE == image_two_value[row-1][col-1]) { cnt++; }
            if(IMG_WHITE == image_two_value[row-1][col  ]) { cnt++; }
            if(IMG_WHITE == image_two_value[row-1][col+1]) { cnt++; }
            if(IMG_WHITE == image_two_value[row  ][col-1]) { cnt++; }
            if(IMG_WHITE == image_two_value[row  ][col+1]) { cnt++; }
            if(IMG_WHITE == image_two_value[row+1][col-1]) { cnt++; }
            if(IMG_WHITE == image_two_value[row+1][col  ]) { cnt++; }
            if(IMG_WHITE == image_two_value[row+1][col+1]) { cnt++; }

            if((cnt >= IMG_FLT_MAX_WHITE) && (IMG_BLACK == image_two_value[row][col]))
            {
                image_two_value[row][col] = IMG_WHITE;
            }
            else if((cnt <= IMG_FLT_MIN_WHITE) && (IMG_WHITE == image_two_value[row][col]))
            {
                image_two_value[row][col] = IMG_BLACK;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     给二值图加黑边，保证搜索不出界
//-------------------------------------------------------------------------------------------------------------------
static void Image_DrawBlackBorder(void)
{
    uint16 row;
    uint16 col;

    for(row = 0; row < MT9V03X_H; row++)
    {
        image_two_value[row][0]           = IMG_BLACK;
        image_two_value[row][1]           = IMG_BLACK;
        image_two_value[row][MT9V03X_W-1] = IMG_BLACK;
        image_two_value[row][MT9V03X_W-2] = IMG_BLACK;
    }

    for(col = 0; col < MT9V03X_W; col++)
    {
        image_two_value[0][col] = IMG_BLACK;
        image_two_value[1][col] = IMG_BLACK;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在上一行边界附近寻找最近的跳变；人工黑边不算真实边线。
//-------------------------------------------------------------------------------------------------------------------
static int Edge_HasSupport(int row, int col, int left)
{
    int k;
    int outside = left ? -1 : 1;
    for(k = 1; k <= EDGE_BLACK_SUPPORT; k++)
    {
        int c = col + outside * k;
        // 人工加上的黑边不作为黑色赛道证据。
        if((c < 2) || (c >= MT9V03X_W - 2) ||
           (image_two_value[row][c] != IMG_BLACK)) { return 0; }
    }
    for(k = 0; k < EDGE_WHITE_SUPPORT; k++)
    {
        int c = col - outside * k;
        if((c < 2) || (c >= MT9V03X_W - 2) ||
           (image_two_value[row][c] != IMG_WHITE)) { return 0; }
    }
    return 1;
}

static int Find_Edge(int row, int previous, int left)
{
    int d;
    int side;

    for(d = 0; d <= EDGE_WIN_WIDE; d++)
    {
        for(side = 0; side < ((d == 0) ? 1 : 2); side++)
        {
            int c = previous + ((side == 0) ? -d : d);
            if((c <= 2) || (c >= MT9V03X_W - 3)) { continue; }
            if(image_two_value[row][c] != IMG_WHITE) { continue; }
            if(Edge_HasSupport(row, c, left))
            {
                return c;
            }
        }
    }
    return -1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     左右独立跟踪；弯道单边出画面时保留另一侧，双丢后仍尝试重捕获。
//-------------------------------------------------------------------------------------------------------------------
static void Track_Lines(void)
{
    int row;
    int c;
    int run_start = -1;
    int best_l = 0;
    int best_r = 0;
    int best_w = 0;
    int center_run = 0;
    int l;
    int r;

    for(row = 0; row < MT9V03X_H; row++)
    {
        Left_Line[row] = 0;
        Right_Line[row] = MT9V03X_W - 1;
        Left_Lost_Flag[row] = Right_Lost_Flag[row] = 1;
        Mid_Line[row] = MT9V03X_W / 2;
        Road_Wide[row] = 0;
    }

    for(c = 2; c < MT9V03X_W - 1; c++)
    {
        if(image_two_value[MT9V03X_H - 2][c] == IMG_WHITE)
        {
            if(run_start < 0) { run_start = c; }
        }
        else if(run_start >= 0)
        {
            int width = c - 1 - run_start;
            int centered = (run_start <= MT9V03X_W / 2) && (c > MT9V03X_W / 2);
            if((width >= EDGE_MIN_W) && (width <= EDGE_MAX_W) &&
               ((centered && !center_run) || ((centered == center_run) && (width > best_w))))
            {
                best_l = run_start;
                best_r = c - 1;
                best_w = width;
                center_run = centered;
            }
            run_start = -1;
        }
    }

    if(best_w >= EDGE_MIN_W)
    {
        seed_left = best_l;
        seed_right = best_r;
        seed_valid = 1;
    }
    // 十字横向白区进入近处后，沿用上一帧种子，仍能搜索横向白区上方的出口。
    if(!seed_valid) { return; }

    l = seed_left;
    r = seed_right;
    g_ref_col = (uint8)((l + r) / 2);
    for(row = MT9V03X_H - 1; row >= 2; row--)
    {
        int nl = Find_Edge(row, l, 1);
        int nr = Find_Edge(row, r, 0);
        if((nl >= 0) && (nr >= 0) &&
           (((nr - nl) < EDGE_MIN_W) || ((nr - nl) > EDGE_MAX_W)))
        {
            continue;
        }
        if(nl >= 0)
        {
            l = nl;
            Left_Line[row] = l;
            Left_Lost_Flag[row] = 0;
        }
        if(nr >= 0)
        {
            r = nr;
            Right_Line[row] = r;
            Right_Lost_Flag[row] = 0;
        }
    }
}

// 丢弃孤立短段；保留正常弯道和十字两端的长边线。
static void Remove_ShortEdgeRuns(void)
{
    int side;
    for(side = 0; side < 2; side++)
    {
        int *lost = side ? Right_Lost_Flag : Left_Lost_Flag;
        int row = 2;
        while(row < MT9V03X_H)
        {
            int begin;
            int i;
            if(lost[row]) { row++; continue; }
            begin = row;
            while((row < MT9V03X_H) && !lost[row]) { row++; }
            if(row - begin < EDGE_MIN_RUN)
            {
                for(i = begin; i < row; i++) { lost[i] = 1; }
            }
        }
    }
}

//===================================================================================================================
//  对外接口
//===================================================================================================================

void LineDetect_Init(void)
{
    uint8 i;

    for(i = 0; i < OTSU_BANDS; i++)
    {
        th_smooth[i] = -1.0f;
        th_band[i]   = 0;
    }

    g_mode    = LINE_MODE_CONTRAST;
    g_ref_col = MT9V03X_W / 2;
    g_otsu_bias = OTSU_BIAS_DEFAULT;
    seed_left = MT9V03X_W / 4;
    seed_right = MT9V03X_W * 3 / 4;
    seed_valid = 0;
}

void LineDetect_Process(const uint8 *image, uint8 mode)
{
    g_mode = (mode == LINE_MODE_OTSU) ? LINE_MODE_OTSU : LINE_MODE_CONTRAST;

    if(g_mode == LINE_MODE_OTSU)
    {
        /* 分带大津法 -> 滤波 -> 黑边 -> 左右独立跟踪 */
        Threshold_And_Binarize(image);

#if USE_IMG_FILTER
        Image_Filter();
#endif
        Image_DrawBlackBorder();
        Track_Lines();
        Remove_ShortEdgeRuns();
    }
    else
    {
        /* 当前这版：对比度巡线 */
        Contrast_Process(image);
        g_ref_col = Contrast_GetReferenceCol();
    }

    LineDetect_RefreshStats();
}

void LineDetect_RefreshStats(void)
{
    int row;
    int farthest = MT9V03X_H;

    Left_Lost_Time = Right_Lost_Time = Both_Lost_Time = 0;
    Boundry_Start_Left = Boundry_Start_Right = 0;
    for(row = MT9V03X_H - 1; row >= 0; row--)
    {
        int left_ok = !Left_Lost_Flag[row];
        int right_ok = !Right_Lost_Flag[row];
        if(!left_ok) { Left_Lost_Time++; }
        if(!right_ok) { Right_Lost_Time++; }
        if(!left_ok && !right_ok) { Both_Lost_Time++; }
        if(left_ok && !Boundry_Start_Left) { Boundry_Start_Left = row; }
        if(right_ok && !Boundry_Start_Right) { Boundry_Start_Right = row; }
        if(left_ok || right_ok) { farthest = row; }
        Road_Wide[row] = (left_ok && right_ok) ? Right_Line[row] - Left_Line[row] : 0;
        Mid_Line[row] = (Left_Line[row] + Right_Line[row]) / 2;
    }
    Search_Stop_Line = MT9V03X_H - farthest;
    Img_Disappear_Flag = (Search_Stop_Line < 20) ? 1 : 0;
}

uint8 LineDetect_GetRefCol(void)
{
    return g_ref_col;
}

uint8 LineDetect_GetMode(void)
{
    return g_mode;
}

void LineDetect_SetOtsuBias(int bias)
{
    if(bias < -100) { bias = -100; }
    if(bias >  100) { bias =  100; }
    g_otsu_bias = bias;
}

int LineDetect_GetOtsuBias(void)
{
    return g_otsu_bias;
}
