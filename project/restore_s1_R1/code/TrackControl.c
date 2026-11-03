#include "TrackControl.h"
#include "MotorPID.h"

static float curve_release_target;
static uint8 curve_release_ticks;
static uint8 curve_speed_valid;

float Run_CurveTarget(float target, float u, float preview)
{
    float magnitude = (u >= 0.0f) ? u : -u;
    float ahead = (preview >= 0.0f) ? preview : -preview;
    // Steering and preview select one speed tier, never multiply reductions.
    // A lower key-selected speed or the 100-count ring cap remains lower.
    if(magnitude > TURN_SHARP_LIMIT || ahead >= TURN_PREVIEW_FULL)
    {
        if(target > TURN_SPEED_SHARP_MAX) { target = TURN_SPEED_SHARP_MAX; }
    }
    else if(magnitude > TURN_MEDIUM_LIMIT || ahead > TURN_PREVIEW_START)
    {
        if(target > TURN_SPEED_MEDIUM_MAX) { target = TURN_SPEED_MEDIUM_MAX; }
    }
    // 运行减速不低于70；零目标继续作为停车命令传递。
    if(target > 0.0f && target < MOTORPID_MIN_RUN_TARGET) { target = MOTORPID_MIN_RUN_TARGET; }
    return target;
}

void Run_SpeedReset(void)
{
    curve_release_target = 0.0f;
    curve_release_ticks = 0;
    curve_speed_valid = 0;
}

float Run_SpeedTarget(float requested, float u, float preview)
{
    float desired = Run_CurveTarget(requested, u, preview);
    if(!curve_speed_valid || (desired <= curve_release_target))
    {
        curve_speed_valid = 1;
        curve_release_target = desired;
        curve_release_ticks = (desired < requested) ? TURN_RELEASE_HOLD_TICKS : 0;
    }
    else if(curve_release_ticks > 0)
    {
        curve_release_ticks--;
    }
    else
    {
        curve_release_target += TURN_RELEASE_STEP;
        if(curve_release_target > desired) { curve_release_target = desired; }
    }
    return curve_release_target;
}
