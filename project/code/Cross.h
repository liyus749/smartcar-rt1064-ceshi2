/*********************************************************************************************************************
* 摄像头十字：白区预判 -> 逐列上下边界 -> L角点 -> 分阶段补线 -> 图像恢复退出。
* 思路参考暮雪归烟《智能车图像处理（3）-元素识别（十字）》；实现按本工程188x100图像重写。
* https://blog.csdn.net/fgfgfdg8/article/details/126440305
********************************************************************************************************************/
#ifndef __CROSS_H
#define __CROSS_H

#include "zf_common_headfile.h"

#define CROSS_LOST_MIN          (10)
#define CROSS_CONFIRM_FRAMES    (2)
#define CROSS_HOLD_FRAMES       (15)    // 连续丢失可信出口的帧数上限；有真实出口就刷新，不因停车超时
#define CROSS_PHASE_CONFIRM     (2)     // 连续缺少入口角点才确认内部阶段，忽略单帧漏检
#define CROSS_TOP_MARGIN        (3)
#define CROSS_VALID_ABOVE       (4)
#define CROSS_VALID_BELOW       (3)
#define CROSS_SIDE_OFFSET       (6)
#define CROSS_EXIT_ROWS         (8)
#define CROSS_EXIT_FRAMES       (2)
#define CROSS_PROFILE_SUPPORT   (4)     // 横边连续支持，排除孤立黑点
#define CROSS_CORNER_JUMP       (6)
#define CROSS_FIT_ROWS          (8)
#define CROSS_FIT_SEARCH        (8)
#define CROSS_MAX_DX_DY         (1.2f)  // x=a*y+b，近竖直线a接近0，不能照搬y=k*x+b的限幅
#define CROSS_PATH_WHITE_PCT    (85)

typedef enum
{
    CROSS_STATE_NORMAL = 0,
    CROSS_STATE_CANDIDATE = 1,
    CROSS_STATE_ENTER = 2,
    CROSS_STATE_INSIDE = 3,
    CROSS_STATE_EXIT = 4
} Cross_State;

enum
{
    CROSS_UPPER_LEFT = 0, CROSS_UPPER_RIGHT,
    CROSS_LOWER_LEFT, CROSS_LOWER_RIGHT, CROSS_CORNER_COUNT
};

extern volatile int Cross_Flag;
void Cross_Init(void);
// 每帧基础巡线后先检查十字；已确认环岛时让出处理权，之后再刷新统计/计算偏差。
void Cross_Check(void);
int Cross_GetRunLen(void);              // 补线前最大连续双丢行数，保留原调试接口
Cross_State Cross_GetState(void);
uint8 Cross_GetCorner(int which, int *x, int *y); // 仅当前帧真实找到的角点
uint8 Cross_GetFillMask(int row);        // bit0左补线，bit1右补线；实测边线为0
#endif
