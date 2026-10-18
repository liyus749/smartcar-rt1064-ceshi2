/*********************************************************************************************************************
* 文件名称          Err.c
* 功能简介          图像偏差计算实现（说明见 Err.h）
********************************************************************************************************************/

#include "Err.h"

extern volatile int   Left_Line[MT9V03X_H];
extern volatile int   Right_Line[MT9V03X_H];
extern volatile int   Road_Wide[MT9V03X_H];
extern int            Left_Lost_Flag[MT9V03X_H];
extern int            Right_Lost_Flag[MT9V03X_H];
extern volatile int   Search_Stop_Line;

//===================================================================================================================
// 对外输出
//===================================================================================================================
volatile float Err       = 0.0f;      // 原始图像偏差
volatile float ErrUse    = 0.0f;      // 滤波后偏差（PD 用）
volatile float ErrDFrame = 0.0f;      // 每帧变化量（D 项用）
volatile int   ErrValidRows = 0;      // 有效行数

//===================================================================================================================
// 内部状态
//===================================================================================================================
static int   zone_mid[MT9V03X_H];     // 每行中线
static uint8 zone_ok[MT9V03X_H];      // 每行能不能用

static int   ln_farthest = 0;         // 看得见的最远行
static int   ln_span     = 0;         // 可见行数

// 5 段权重：远 -> 近
const float Err_ZoneWeight[ERR_SEGMENTS] = { 1.0f, 1.5f, 2.0f, 2.5f, 2.5f };

static float err_filt_val  = 0.0f;    // 滤波值
static uint8 err_filt_init = 0;       // 第一次直接赋值
static float err_use_prev  = 0.0f;    // 上一帧滤波值（算 D 项）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏差模块初始化
//-------------------------------------------------------------------------------------------------------------------
void Err_Init(void)
{
    int i;

    Err       = 0.0f;
    ErrUse    = 0.0f;
    ErrDFrame = 0.0f;
    ErrValidRows = 0;

    err_filt_val  = 0.0f;
    err_filt_init = 0;
    err_use_prev  = 0.0f;

    for(i = 0; i < MT9V03X_H; i++)
    {
        zone_mid[i] = 0;
        zone_ok[i]  = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     算每行中线 + 有效性（两边都找到、不是孤立假点）
// 返回参数     uint8    1 = 数据够用；0 = 看得太少
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_line_prepare(void)
{
    int farthest;
    int span;
    int row;

    farthest = MT9V03X_H - Search_Stop_Line;
    if(farthest < 0)             { farthest = 0; }
    if(farthest > MT9V03X_H - 2) { farthest = MT9V03X_H - 2; }

    span = MT9V03X_H - farthest;
    if(span < ERR_MIN_SPAN)      { return 0; }

    ln_farthest = farthest;
    ln_span     = span;

    for(row = farthest; row < MT9V03X_H; row++)
    {
        zone_mid[row] = (Left_Line[row] + Right_Line[row]) >> 1;
        zone_ok[row]  = ((0 == Left_Lost_Flag[row]) && (0 == Right_Lost_Flag[row])) ? 1 : 0;
    }

    /* 去掉孤立假点：和上下相邻两行平均差太多 */
    for(row = farthest + 1; row < MT9V03X_H - 1; row++)
    {
        int avg;
        int d;

        if(0 == zone_ok[row])                              { continue; }
        if((0 == zone_ok[row - 1]) || (0 == zone_ok[row + 1])) { continue; }

        avg = (zone_mid[row - 1] + zone_mid[row + 1]) >> 1;
        d   = zone_mid[row] - avg;

        if((d > ERR_SPIKE_LIMIT) || (d < -ERR_SPIKE_LIMIT))
        {
            zone_ok[row] = 0;
        }
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     远 / 中 / 近 三段分块加权，得到位置偏差
// 返回参数     uint8    1 = 算出来了；0 = 有效数据太少
//-------------------------------------------------------------------------------------------------------------------
static uint8 err_zone(float *err_out)
{
    int   row;
    int   r2;
    int   z;
    int   v;
    int   cnt[ERR_SEGMENTS];
    float sum[ERR_SEGMENTS];
    float wsum[ERR_SEGMENTS];
    float total_e = 0.0f;
    float total_w = 0.0f;
    float lateral;

    for(z = 0; z < ERR_SEGMENTS; z++)
    {
        cnt[z]  = 0;
        sum[z]  = 0.0f;
        wsum[z] = 0.0f;
    }
    ErrValidRows = 0;

    for(row = ln_farthest; row < MT9V03X_H; row++)
    {
        float pos;
        float kernel;
        float conf;
        float rw;
        float e;

        if(0 == zone_ok[row]) { continue; }

        /* ---------- 这一段属于哪一块（远 -> 近，共 ERR_SEGMENTS 块）---------- */
        z = ((row - ln_farthest) * ERR_SEGMENTS) / ln_span;
        if(z >= ERR_SEGMENTS) { z = ERR_SEGMENTS - 1; }

        /* ---------- 段内三角核：段中心权重 1.0，段边界 0.5 ---------- */
        pos = ((float)((row - ln_farthest) * ERR_SEGMENTS) / (float)ln_span) - (float)z;
        kernel = 1.0f;
#if ERR_USE_ROW_KERNEL
        {
            float d = pos - 0.5f;
            if(d < 0.0f) { d = -d; }
            kernel = 1.0f - d;
        }
#endif

        /* ---------- 宽度一致性：和附近 5 行平均宽度差太多的行降权 ---------- */
        conf = 1.0f;
#if ERR_USE_WIDTH_CONF
        {
            int sw = 0;
            int cw = 0;
            int r0 = row - 2;
            int r1 = row + 2;

            if(r0 < ln_farthest) { r0 = ln_farthest; }
            if(r1 >= MT9V03X_H)  { r1 = MT9V03X_H - 1; }

            for(r2 = r0; r2 <= r1; r2++)
            {
                if(0 == zone_ok[r2]) { continue; }
                sw += Road_Wide[r2];
                cw++;
            }

            if(cw > 0)
            {
                float avg = (float)sw / (float)cw;
                float d   = (float)Road_Wide[row] - avg;

                if(d < 0.0f) { d = -d; }
                conf = 1.0f / (1.0f + d / 20.0f);
                if(conf < 0.25f) { conf = 0.25f; }
            }
        }
#endif

        v = zone_mid[row];
        e  = (float)((MT9V03X_W / 2) - v);
        rw = kernel * conf;

        sum[z]  += rw * e;
        wsum[z] += rw;
        cnt[z]++;
        ErrValidRows++;
    }

    /* ---------- 每块取加权平均，再按段权重合成 ---------- */
    for(z = 0; z < ERR_SEGMENTS; z++)
    {
        if(cnt[z] < ZONE_MIN_ROWS)      { continue; }
        if(wsum[z] < 0.001f)            { continue; }
        if(Err_ZoneWeight[z] <= 0.001f) { continue; }

        total_e += Err_ZoneWeight[z] * (sum[z] / wsum[z]);
        total_w += Err_ZoneWeight[z];
    }

    if(total_w < 0.001f) { return 0; }

    lateral = total_e / total_w;
    *err_out = lateral - ERR_BIAS;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一帧偏差计算入口
//-------------------------------------------------------------------------------------------------------------------
uint8 Err_Compute(void)
{
    float e;
    uint8 got;

    if(!image_line_prepare())
    {
        return 0;
    }

    got = err_zone(&e);
    if(!got)
    {
        return 0;
    }

    Err = e;

    /* 低通滤波 */
    if(ERR_FILTER > 0.01f)
    {
        if(0 == err_filt_init)
        {
            err_filt_val  = Err;
            err_filt_init = 1;
        }
        err_filt_val += ERR_FILTER * (Err - err_filt_val);
        ErrUse = err_filt_val;
    }
    else
    {
        err_filt_init = 0;
        err_filt_val  = Err;
        ErrUse        = Err;
    }

    /* D 项用的每帧变化量（在中断里直接用，避免 20ms 中断和帧不同步）*/
    ErrDFrame = ErrUse - err_use_prev;
    err_use_prev = ErrUse;
    if(ErrDFrame >  ERR_D_LIMIT) { ErrDFrame =  ERR_D_LIMIT; }
    if(ErrDFrame < -ERR_D_LIMIT) { ErrDFrame = -ERR_D_LIMIT; }

    return 1;
}
