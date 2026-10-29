/*********************************************************************************************************************
* 文件名称          Contrast.c
* 功能简介          对比度巡线实现（说明见 Contrast.h）
********************************************************************************************************************/

#include "Contrast.h"

//===================================================================================================================
// Camera.c 里定义的全局量（我们只写，不重复定义；定义仍在 Camera.c，环岛 island.c 也读它们）
//===================================================================================================================
extern uint8          image_two_value[MT9V03X_H][MT9V03X_W];
extern volatile int   Left_Line[MT9V03X_H];
extern volatile int   Right_Line[MT9V03X_H];
extern volatile int   Mid_Line[MT9V03X_H];
extern volatile int   Road_Wide[MT9V03X_H];
extern int            Left_Lost_Flag[MT9V03X_H];
extern int            Right_Lost_Flag[MT9V03X_H];
extern volatile int   Search_Stop_Line;
extern volatile int   Boundry_Start_Left;
extern volatile int   Boundry_Start_Right;
extern volatile int   Left_Lost_Time;
extern volatile int   Right_Lost_Time;
extern volatile int   Both_Lost_Time;
extern volatile uint8  Img_Disappear_Flag;

//===================================================================================================================
// 本模块内部状态
//===================================================================================================================
volatile float Contrast_Ratio = CONTRAST_RATIO_DEFAULT;   // 对比度阈值（可在线调）

static uint8 reference_point;                              // 参考点灰度（底部地面平均）
static uint8 white_max_point;                              // 白点上限
static uint8 white_min_point;                              // 白点下限
static uint8 reference_col;                                // 参考列（最长白列所在列）
static uint8 reference_col_line[MT9V03X_H];                // 参考列竖线（给绘图留的）
static uint8 remote_distance[MT9V03X_W];                   // 每列白点最远行（越小=白点越长）
static uint8 left_edge_line[MT9V03X_H];                    // 搜出的左边界
static uint8 right_edge_line[MT9V03X_H];                   // 搜出的右边界

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取画面最下方 CONTRAST_REFRENCEROW 行的平均灰度，作为参考点
//              再乘系数得到 白点上限 / 白点下限（自适应阈值）
//-------------------------------------------------------------------------------------------------------------------
static void get_reference_point(const uint8 *image)
{
    const uint8 *p = &image[(MT9V03X_H - CONTRAST_REFRENCEROW) * MT9V03X_W];
    uint16 i;
    uint16 total = CONTRAST_REFRENCEROW * MT9V03X_W;
    uint32 sum = 0;

    for(i = 0; i < total; i++)
    {
        sum += p[i];
    }

    reference_point = (uint8)(sum / total);

    white_max_point = func_limit_ab((int32)reference_point * CONTRAST_WHITEMAXMUL / 10,
                                    CONTRAST_BLACKPOINT, 255);
    white_min_point = func_limit_ab((int32)reference_point * CONTRAST_WHITEMINMUL / 10,
                                    CONTRAST_BLACKPOINT, 255);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在一段数组里找极值所在下标
// 参数说明     mode = 0 找最小值；其他找最大值
//-------------------------------------------------------------------------------------------------------------------
static int find_extreme_value(const uint8 *arr, int start, int end, int mode)
{
    int i;
    int best = start;

    for(i = start; i <= end; i++)
    {
        if(0 == mode)
        {
            if(arr[i] < arr[best]) { best = i; }
        }
        else
        {
            if(arr[i] > arr[best]) { best = i; }
        }
    }
    return best;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     搜索参考列：逐列统计"从下往上白点有多长"
//
//   实现说明：参考代码里这一步用的是对比度公式，但它把"往上偏移多少"写成了 STOPROW（=0），
//             等于自己和自己比，公式恒为 0，跑不起来。这里改成更直接、更稳的写法：
//             从下往上数，像素 >= white_min_point 就算白，一直数到第一个黑点为止。
//             最终效果和"搜索每列白点长度"完全一样，只是判定方式从"对比度"换成"白点下限"。
//   对比度公式真正派上用场的地方是后面的 search_line（横向搜左右边界）。
//-------------------------------------------------------------------------------------------------------------------
static void search_reference_col(const uint8 *image)
{
    int col, row;
    int max_white = 0;

    // 先全部初始化成"白点最远行 = 最底行"
    for(col = 0; col < MT9V03X_W; col++)
    {
        remote_distance[col] = (uint8)(MT9V03X_H - 1);
    }

    for(col = 0; col < MT9V03X_W; col += CONTRAST_OFFSET)
    {
        int white_len = 0;

        // 从最底行往上数白点，碰到黑点停下
        for(row = MT9V03X_H - 1; row >= CONTRAST_STOPROW; row--)
        {
            if(image[row * MT9V03X_W + col] >= white_min_point)
            {
                white_len++;
            }
            else
            {
                break;
            }
        }

        // 白点长度 -> 换算成"白点最远行"（越小 = 白点越长，和参考代码里的 remote_distance 语义一致）
        remote_distance[col] = (uint8)(MT9V03X_H - white_len);

        if(white_len > max_white)
        {
            max_white = white_len;
        }
    }

    // 视野长度 = 最长白列的长度
    Search_Stop_Line = max_white;

    // 参考列 = 白点最长的那一列（remote_distance 最小），再补一个 CONTRAST_OFFSET 偏移
    reference_col = (uint8)(find_extreme_value(remote_distance, 10, MT9V03X_W - 10, 0) + CONTRAST_OFFSET);
    reference_col = func_limit_ab(reference_col, 1, MT9V03X_W - 2);

    for(row = 0; row < MT9V03X_H; row++)
    {
        reference_col_line[row] = reference_col;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对比度搜左右边界
//
//   每一行：
//     左边界：从参考列往左扫，当前点 temp1、左偏 CONTRAST_OFFSET 的对比点 temp2，
//             对比度 (temp1-temp2)*200/(temp1+temp2) 超过阈值，说明这里从白跳到了黑，就是边界
//     右边界：从参考列往右扫，当前点 temp1、右偏 CONTRAST_OFFSET 的对比点 temp2，原理一样
//
//   迭代收紧：这一行找到边界后，下一行只在"这一行边界 ± CONTRAST_SEARCHRANGE"里找
//             找不到就用宽范围（参考列到边缘）再找一次；再找不到就判这一行丢线
//-------------------------------------------------------------------------------------------------------------------
static void search_line(const uint8 *image)
{
    int col, row;
    int temp1, temp2, temp3;
    int leftstart, leftend, rightstart, rightend;
    int leftstop = 0, rightstop = 0;
    int ratio = (int)Contrast_Ratio;

    const int row_max = MT9V03X_H - 1;
    const int row_min = CONTRAST_STOPROW;
    const int col_min = CONTRAST_OFFSET;
    const int col_max = MT9V03X_W - CONTRAST_OFFSET;

    // 初始边界：左=0，右=W-1（和学长 Longest_White_Column 的默认值一致）
    for(row = row_max; row >= row_min; row--)
    {
        left_edge_line[row]  = 0;
        right_edge_line[row] = (uint8)(MT9V03X_W - 1);
    }

    leftstart  = reference_col;
    leftend    = 0;
    rightstart = reference_col;
    rightend   = MT9V03X_W - 1;

    for(row = row_max; row >= row_min; row--)
    {
        const uint8 *p = &image[row * MT9V03X_W];

        /* ================= 左边界 ================= */
        if(!leftstop)
        {
            int found = 0;
            int pass;

            for(pass = 0; (pass < 2) && (!found); pass++)
            {
                int start = (0 == pass) ? leftstart : reference_col;
                int end   = (0 == pass) ? leftend   : col_min;

                for(col = start; col >= end; col--)
                {
                    temp1 = p[col];
                    temp2 = p[col - CONTRAST_OFFSET];

                    /* 起点直接是黑：整幅图上面不再搜左边界 */
                    if((temp1 < white_min_point) && (col == leftstart) && (leftstart == reference_col))
                    {
                        leftstop = 1;
                        for(int r = row; r >= 0; r--) { left_edge_line[r] = col_min - CONTRAST_OFFSET; }
                        found = 1;
                        break;
                    }

                    /* 当前点直接是黑：左边界就是这里 */
                    if(temp1 < white_min_point)
                    {
                        left_edge_line[row] = (uint8)col;
                        leftstart = func_limit_ab(col + CONTRAST_SEARCHRANGE, col, col_max);
                        leftend   = func_limit_ab(col - CONTRAST_SEARCHRANGE, col_min, col);
                        found = 1;
                        break;
                    }

                    /* 对比点还是白：说明还没到边界，继续往左 */
                    if(temp2 > white_max_point)
                    {
                        continue;
                    }

                    /* 对比度判据 */
                    if((temp1 + temp2) != 0)
                    {
                        temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);

                        if((temp3 > ratio) || (col == col_min))
                        {
                            left_edge_line[row] = (uint8)(col - CONTRAST_OFFSET);
                            leftstart = func_limit_ab(col + CONTRAST_SEARCHRANGE, col, col_max);
                            leftend   = func_limit_ab(col - CONTRAST_SEARCHRANGE, col_min, col);
                            found = 1;
                            break;
                        }
                    }
                }
            }
            /* 没找到：保持默认 0 = 丢线 */
        }

        /* ================= 右边界（和左边对称） ================= */
        if(!rightstop)
        {
            int found = 0;
            int pass;

            for(pass = 0; (pass < 2) && (!found); pass++)
            {
                int start = (0 == pass) ? rightstart : reference_col;
                int end   = (0 == pass) ? rightend   : col_max;

                for(col = start; col <= end; col++)
                {
                    temp1 = p[col];
                    temp2 = p[col + CONTRAST_OFFSET];

                    if((temp1 < white_min_point) && (col == rightstart) && (rightstart == reference_col))
                    {
                        rightstop = 1;
                        for(int r = row; r >= 0; r--) { right_edge_line[r] = col_max + CONTRAST_OFFSET; }
                        found = 1;
                        break;
                    }

                    if(temp1 < white_min_point)
                    {
                        right_edge_line[row] = (uint8)col;
                        rightstart = func_limit_ab(col - CONTRAST_SEARCHRANGE, col_min, col);
                        rightend   = func_limit_ab(col + CONTRAST_SEARCHRANGE, col, col_max);
                        found = 1;
                        break;
                    }

                    if(temp2 > white_max_point)
                    {
                        continue;
                    }

                    if((temp1 + temp2) != 0)
                    {
                        temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);

                        if((temp3 > ratio) || (col == col_max))
                        {
                            right_edge_line[row] = (uint8)(col + CONTRAST_OFFSET);
                            rightstart = func_limit_ab(col - CONTRAST_SEARCHRANGE, col_min, col);
                            rightend   = func_limit_ab(col + CONTRAST_SEARCHRANGE, col, col_max);
                            found = 1;
                            break;
                        }
                    }
                }
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把搜出的边界写回学长 Camera.c 的全局量，并统计丢线信息
//-------------------------------------------------------------------------------------------------------------------
static void update_boundary_and_lost(void)
{
    int row;

    Left_Lost_Time  = 0;
    Right_Lost_Time = 0;
    Both_Lost_Time  = 0;
    Boundry_Start_Left  = 0;
    Boundry_Start_Right = 0;

    for(row = 0; row < MT9V03X_H; row++)
    {
        int l = left_edge_line[row];
        int r = right_edge_line[row];

        if(l < 0)              { l = 0; }
        if(l > MT9V03X_W - 1)  { l = MT9V03X_W - 1; }
        if(r < 0)              { r = 0; }
        if(r > MT9V03X_W - 1)  { r = MT9V03X_W - 1; }

        Left_Line[row]   = l;
        Right_Line[row]  = r;
        Mid_Line[row]    = (l + r) >> 1;
        Road_Wide[row]   = r - l;

        /* 丢线判定：搜不到时左边界停在 0、右边界停在 W-1（就是上一行的默认值）*/
        Left_Lost_Flag[row]  = (l <= 0) ? 1 : 0;
        Right_Lost_Flag[row] = (r >= (MT9V03X_W - 1)) ? 1 : 0;
    }

    /* 统计丢线行数 + 边界起始行（和学长 Longest_White_Column 的统计方式一致）*/
    for(row = MT9V03X_H - 1; row >= 0; row--)
    {
        if(1 == Left_Lost_Flag[row])  { Left_Lost_Time++; }
        if(1 == Right_Lost_Flag[row]) { Right_Lost_Time++; }
        if((1 == Left_Lost_Flag[row]) && (1 == Right_Lost_Flag[row])) { Both_Lost_Time++; }

        if((0 == Boundry_Start_Left)  && (0 == Left_Lost_Flag[row]))  { Boundry_Start_Left  = row; }
        if((0 == Boundry_Start_Right) && (0 == Right_Lost_Flag[row])) { Boundry_Start_Right = row; }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成二值图（屏幕显示 + 环岛里的 Longest_White_Column 用）
//              像素 >= 白点下限 -> 白（赛道），否则黑
//-------------------------------------------------------------------------------------------------------------------
static void build_binary_image(const uint8 *image)
{
    int row, col;

    for(row = 0; row < MT9V03X_H; row++)
    {
        const uint8 *p = &image[row * MT9V03X_W];
        for(col = 0; col < MT9V03X_W; col++)
        {
            image_two_value[row][col] = (p[col] >= white_min_point) ? IMG_WHITE : IMG_BLACK;
        }
    }

    /* 加一圈黑边，保证环岛里的 Longest_White_Column 搜线不会越界 */
    for(row = 0; row < MT9V03X_H; row++)
    {
        image_two_value[row][0]               = IMG_BLACK;
        image_two_value[row][1]               = IMG_BLACK;
        image_two_value[row][MT9V03X_W - 1]   = IMG_BLACK;
        image_two_value[row][MT9V03X_W - 2]   = IMG_BLACK;
    }
    for(col = 0; col < MT9V03X_W; col++)
    {
        image_two_value[0][col] = IMG_BLACK;
        image_two_value[1][col] = IMG_BLACK;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     巡线模块初始化
//-------------------------------------------------------------------------------------------------------------------
void Contrast_Init(void)
{
    int i;

    Contrast_Ratio       = CONTRAST_RATIO_DEFAULT;
    reference_point      = 0;
    white_max_point      = 255;
    white_min_point      = CONTRAST_BLACKPOINT;
    reference_col        = MT9V03X_W / 2;

    for(i = 0; i < MT9V03X_W; i++) { remote_distance[i] = (uint8)(MT9V03X_H - 1); }
    for(i = 0; i < MT9V03X_H; i++)
    {
        reference_col_line[i] = reference_col;
        left_edge_line[i]     = 0;
        right_edge_line[i]    = (uint8)(MT9V03X_W - 1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理一帧灰度图像
//-------------------------------------------------------------------------------------------------------------------
void Contrast_Process(const uint8 *image)
{
    if(NULL == image)
    {
        return;
    }

    get_reference_point(image);
    search_reference_col(image);
    search_line(image);
    update_boundary_and_lost();
    build_binary_image(image);

    /* 视野太短 = 基本没看到赛道，报"图像丢失"，舵机那边会保持上一拍不打飞 */
    if(Search_Stop_Line < 20)
    {
        Img_Disappear_Flag = 1;
    }
    else
    {
        Img_Disappear_Flag = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取参考点灰度
//-------------------------------------------------------------------------------------------------------------------
uint8 Contrast_GetReferencePoint(void)
{
    return reference_point;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取参考列
//-------------------------------------------------------------------------------------------------------------------
uint8 Contrast_GetReferenceCol(void)
{
    return reference_col;
}
