/* Adapter: preserve the downloaded electrical-control APIs and use VIS7 as
 * the sole ring recognizer/path generator. No IMU-based stage transitions. */
#include "Roundabout.h"
#include "island.h"
#include "Err.h"
#include <string.h>

extern volatile int Left_Line[MT9V03X_H], Right_Line[MT9V03X_H];
extern int Left_Lost_Flag[MT9V03X_H], Right_Lost_Flag[MT9V03X_H];
extern volatile int Cross_Flag;
static int learned_width[MT9V03X_H];
static volatile uint8 reset_pending;
static volatile uint16 frame_age, phase_ticks, total_ticks;
static int last_stage;
static uint8 finish_pending; /* image-thread-owned completion handoff */

void Roundabout_Init(void)
{
    int y;
    Island_Reset();
    reset_pending = finish_pending = 0; frame_age = phase_ticks = total_ticks = 0; last_stage = 0;
    for(y = 0; y < MT9V03X_H; y++) learned_width[y] = 20 + 80*y/(MT9V03X_H-1);
}

void Roundabout_Retry(void) { reset_pending = 1; }
uint8 Roundabout_ResetPending(void) { return reset_pending; }
uint8 Roundabout_GetFault(void) { return (uint8)island_fault; }
int Roundabout_GetDirection(void) { return Island_State ? island_debug.dir : 0; }
uint8 Roundabout_AllowDifferential(void)
{ return !island_fault && !reset_pending && (Island_State == 0 || Island_State == 4); }
uint8 Roundabout_GetFillMask(int row)
{
    if(row < 0 || row >= MT9V03X_H) return 0;
    return (uint8)((island_patch_left[row] ? 1 : 0) | (island_patch_right[row] ? 2 : 0));
}
int Roundabout_GetRoadWidth(int row)
{
    if(row < 0 || row >= MT9V03X_H) return 0;
    return learned_width[row];
}
uint8 Roundabout_GetPoint(int which, int *x, int *y)
{
    int dir = island_debug.dir ? island_debug.dir : island_debug.candidate;
    IslandSide *inner, *outer;
    IslandPoint p;
    if(!dir || !x || !y || island_fault) return 0;
    inner = dir > 0 ? &island_debug.left : &island_debug.right;
    outer = dir > 0 ? &island_debug.right : &island_debug.left;
    switch(which)
    {
    case ROUND_POINT_LOWER: p = inner->down; break;
    case ROUND_POINT_MIDDLE: p = inner->mid; break;
    case ROUND_POINT_UPPER: p = inner->up; break;
    case ROUND_POINT_EXIT: p = outer->down; break;
    default: return 0;
    }
    if(p.y < 0 || p.y >= MT9V03X_H || p.x < 0 || p.x >= MT9V03X_W) return 0;
    *x = p.x; *y = p.y; return 1;
}

void Roundabout_Tick20ms(uint8 moving)
{
    if(frame_age < 65535U) frame_age++;
    if(!moving || !Island_State || island_fault || reset_pending) return;
    if(frame_age >= ROUND_CAMERA_TIMEOUT_TICKS) { island_fault = 2; return; }
    if(phase_ticks < 65535U) phase_ticks++;
    if(total_ticks < 65535U) total_ticks++;
    if(phase_ticks > ROUND_PHASE_TIMEOUT_TICKS || total_ticks > ROUND_TOTAL_TIMEOUT_TICKS)
        island_fault = 3; /* Stop, never claim a successful visual exit. */
}

void Roundabout_Process(uint8 moving)
{
    int y;
    uint32 mask;
    if(reset_pending)
    {
        /* Reset heavy image/error state only in the image thread. The ISR
           remains stopped while reset_pending is set. */
        Island_Reset(); Err_Init();
        mask = __get_PRIMASK(); __disable_irq();
        frame_age = phase_ticks = total_ticks = 0; last_stage = 0; finish_pending = 0; reset_pending = 0;
        __set_PRIMASK(mask);
    }
    if(finish_pending && !island_fault) { Island_State = ROUND_NORMAL; finish_pending = 0; }
    mask = __get_PRIMASK(); __disable_irq(); frame_age = 0; __set_PRIMASK(mask);
    memset(island_patch_left, 0, sizeof(island_patch_left));
    memset(island_patch_right, 0, sizeof(island_patch_right));
    if(island_fault) return; /* A new image cannot silently clear a latched fault. */
    if(!Island_State && !Cross_Flag)
        for(y = 0; y < MT9V03X_H; y++)
        {
            int width = Right_Line[y] - Left_Line[y];
            if(!Left_Lost_Flag[y] && !Right_Lost_Flag[y] && width >= 15 && width <= 180)
                learned_width[y] = width;
        }
    island_arc_enable = 1;
    Island_SetMoving(moving);
    Island_Detect();
    if(Island_State == ROUND_NORMAL)
        for(y = 0; y < MT9V03X_H; y++)
            if(island_patch_left[y] || island_patch_right[y])
            { Island_State = ROUND_RECOVER; finish_pending = 1; break; }
    mask = __get_PRIMASK(); __disable_irq();
    if(Island_State != last_stage) { phase_ticks = 0; last_stage = Island_State; }
    if(!Island_State) total_ticks = phase_ticks = 0;
    __set_PRIMASK(mask);
}
