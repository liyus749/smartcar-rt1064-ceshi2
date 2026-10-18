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
extern volatile int   Longest_White_Column_Left[2];
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

//===================================================================================================================
//  分段大津法
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对图像的某一段行做一次大津法，返回阈值
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

    if(max_v <= (uint8)(min_v + 1)) { return max_v; }

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
// 函数简介     分带大津法：每段自己的阈值，时间平滑，加偏置，再按行二值化
//-------------------------------------------------------------------------------------------------------------------
static void Threshold_And_Binarize(void)
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
            float otsu = (float)Otsu_Rows(row_begin, row_end);

            if(th_smooth[b] < 1.0f)
            {
                th_smooth[b] = otsu;
            }
            th_smooth[b] += OTSU_SMOOTH_K * (otsu - th_smooth[b]);

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
            image_two_value[row][col] = (mt9v03x_image[row][col] >= th) ? IMG_WHITE : IMG_BLACK;
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
// 函数简介     八邻域边线跟踪（从底行白段出发，逐行往上跟着走）
//-------------------------------------------------------------------------------------------------------------------
static void Track_Lines(void)
{
    int row;
    int c;
    int l;
    int r;
    int run_start;
    int run_w;
    int best_l = 0;
    int best_r = 0;
    int best_w = 0;
    int has_center_run = 0;
    int lost;
    int found;

    run_start = -1;

    for(c = 1; c < MT9V03X_W - 1; c++)
    {
        if(IMG_WHITE == image_two_value[MT9V03X_H - 2][c])
        {
            if(run_start < 0) { run_start = c; }
        }
        else
        {
            if(run_start >= 0)
            {
                run_w = c - run_start;

                if((run_start <= (MT9V03X_W / 2)) && (c > (MT9V03X_W / 2)))
                {
                    best_l = run_start;
                    best_r = c - 1;
                    has_center_run = 1;
                    break;
                }

                if(run_w > best_w)
                {
                    best_w = run_w;
                    best_l = run_start;
                    best_r = c - 1;
                }
                run_start = -1;
            }
        }
    }

    if(!has_center_run && (run_start >= 0))
    {
        run_w = (MT9V03X_W - 1) - run_start;
        if(run_w > best_w) { best_w = run_w;  best_l = run_start;  best_r = MT9V03X_W - 2; }
    }

    for(row = 0; row < MT9V03X_H; row++)
    {
        Left_Lost_Flag[row]  = 1;
        Right_Lost_Flag[row] = 1;
    }

    if((best_r - best_l) < EDGE_MIN_W) { return; }

    l = best_l;
    r = best_r;

    Left_Line[MT9V03X_H - 2]       = l;
    Right_Line[MT9V03X_H - 2]      = r;
    Left_Lost_Flag[MT9V03X_H - 2]  = 0;
    Right_Lost_Flag[MT9V03X_H - 2] = 0;
    Road_Wide[MT9V03X_H - 2]       = r - l;
    Mid_Line[MT9V03X_H - 2]        = (l + r) >> 1;

    lost = 0;

    for(row = MT9V03X_H - 3; row >= 0; row--)
    {
        found = 0;

        for(c = l - EDGE_WIN; c <= l + EDGE_WIN; c++)
        {
            if((c <= 0) || (c >= MT9V03X_W - 1)) { continue; }

            if((IMG_WHITE == image_two_value[row][c]) &&
               (IMG_BLACK == image_two_value[row][c - 1]))
            {
                l = c;
                found |= 1;
                break;
            }
        }

        if(0 == (found & 1))
        {
            for(c = l - EDGE_WIN_WIDE; c <= l + EDGE_WIN_WIDE; c++)
            {
                if((c <= 0) || (c >= MT9V03X_W - 1)) { continue; }

                if((IMG_WHITE == image_two_value[row][c]) &&
                   (IMG_BLACK == image_two_value[row][c - 1]))
                {
                    l = c;
                    found |= 1;
                    break;
                }
            }
        }

        for(c = r + EDGE_WIN; c >= r - EDGE_WIN; c--)
        {
            if((c <= 0) || (c >= MT9V03X_W - 1)) { continue; }

            if((IMG_WHITE == image_two_value[row][c]) &&
               (IMG_BLACK == image_two_value[row][c + 1]))
            {
                r = c;
                found |= 2;
                break;
            }
        }

        if(0 == (found & 2))
        {
            for(c = r + EDGE_WIN_WIDE; c >= r - EDGE_WIN_WIDE; c--)
            {
                if((c <= 0) || (c >= MT9V03X_W - 1)) { continue; }

                if((IMG_WHITE == image_two_value[row][c]) &&
                   (IMG_BLACK == image_two_value[row][c + 1]))
                {
                    r = c;
                    found |= 2;
                    break;
                }
            }
        }

        if((3 == found) && ((r - l) >= EDGE_MIN_W) && ((r - l) <= EDGE_MAX_W))
        {
            Left_Line[row]       = l;
            Right_Line[row]      = r;
            Left_Lost_Flag[row]  = 0;
            Right_Lost_Flag[row] = 0;
            Road_Wide[row]       = r - l;
            Mid_Line[row]        = (l + r) >> 1;
            lost = 0;
        }
        else
        {
            lost++;
            if(lost > EDGE_LOST_MAX) { break; }
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
        th_smooth[i] = 0.0f;
        th_band[i]   = 0;
    }

    g_mode    = LINE_MODE_CONTRAST;
    g_ref_col = MT9V03X_W / 2;
    g_otsu_bias = OTSU_BIAS_DEFAULT;
}

void LineDetect_Process(const uint8 *image, uint8 mode)
{
    g_mode = (mode == LINE_MODE_OTSU) ? LINE_MODE_OTSU : LINE_MODE_CONTRAST;

    if(g_mode == LINE_MODE_OTSU)
    {
        /* 以前那版：分带大津法 -> 滤波 -> 黑边 -> 最长白列 -> 八邻域跟踪 */
        Threshold_And_Binarize();

#if USE_IMG_FILTER
        Image_Filter();
#endif
        Image_DrawBlackBorder();
        Longest_White_Column();
        Track_Lines();

        /* 参考列 = 最长白列所在列 */
        g_ref_col = (uint8)Longest_White_Column_Left[1];
        if(g_ref_col >= MT9V03X_W) { g_ref_col = MT9V03X_W - 1; }
    }
    else
    {
        /* 当前这版：对比度巡线 */
        Contrast_Process(image);
        g_ref_col = Contrast_GetReferenceCol();
    }

    /* 视野太短 = 基本没看到赛道 */
    if(Search_Stop_Line < 20)
    {
        Img_Disappear_Flag = 1;
    }
    else
    {
        Img_Disappear_Flag = 0;
    }
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
