/* Pure-camera roundabouts. Stage/feature ideas: RoyZz_,
 * https://blog.csdn.net/qq_53239103/article/details/126683046
 * Independently implemented for this project's 188x100 binary images.
 */
#ifndef ROUNDABOUT_H
#define ROUNDABOUT_H
#include "zf_common_headfile.h"

#define ROUND_CONFIRM_FRAMES       3
#define ROUND_PHASE_CONFIRM        3
#define ROUND_RECOVER_FRAMES        5
#define ROUND_EXIT_ARM_FRAMES       6
// RoyZz_ lower/middle/upper entry features, adapted to the user's course:
// first recognition always requires ALL THREE corners and enters APPROACH.
// The article's missing-lower, two-point entry branch is deliberately disabled.
// Thresholds below are adapted to this project's 188x100 image.
#define ROUND_CORNER_JUMP           10 // split the island-facing contour at a large jump
#define ROUND_CORNER_MIN_HEIGHT     12 // minimum height of the middle turning contour
#define ROUND_CORNER_MIN_GAP         4 // middle peak has visible contour on both sides
#define ROUND_CORNER_PROMINENCE      4 // middle peak exceeds both ends of its contour
#define ROUND_CORNER_TRACK_X        23 // maximum candidate motion between frames
#define ROUND_CORNER_TRACK_Y        16
#define ROUND_ENTRY_CORNER_STEP      3 // immediate change at an entrance lip
#define ROUND_ENTRY_CORNER_JUMP      6 // change across three rows
#define ROUND_ENTRY_SUPPORT          3 // continuous rows on the supported side
#define ROUND_UPPER_MAX_DX          24 // upper lip stays near the middle point's x
#define ROUND_ENTRY_LOWER_ROW       85 // visible lower lip approaches the car: state 1 -> 2
#define ROUND_MAX_MISSING_FRAMES    8
#define ROUND_CAMERA_TIMEOUT_TICKS 25   // 20ms, confirmed roundabout only
#define ROUND_PHASE_TIMEOUT_TICKS 750   // 15 seconds of motion without stage progress
#define ROUND_TOTAL_TIMEOUT_TICKS 2000  // 40 seconds of motion, then stop
#define ROUND_COOLDOWN_FRAMES      20
#define ROUND_SPEED_MAX            70.0f

typedef enum {
    ROUND_NORMAL=0, ROUND_APPROACH=1, ROUND_WAIT_ENTRY=2,
    ROUND_ENTER=3, ROUND_INSIDE=4, ROUND_EXIT=5,
    ROUND_ALIGN=6, ROUND_RECOVER=7, ROUND_FAULT=8
} Roundabout_State;
enum { ROUND_POINT_LOWER, ROUND_POINT_MIDDLE, ROUND_POINT_UPPER, ROUND_POINT_EXIT, ROUND_POINT_COUNT };

void Roundabout_Init(void);
// Call once per NEW frame after Cross_Check, before statistics/Err. No gyro required.
void Roundabout_Process(uint8 moving);
void Roundabout_Tick20ms(uint8 moving);
void Roundabout_Retry(void); // C13 rearm: main loop resets on next fresh frame
uint8 Roundabout_GetFault(void); // 0 none, 1 geometry/path lost, 2 stale camera, 3 stage timeout
uint8 Roundabout_ResetPending(void);
int Roundabout_GetDirection(void); // +1 left, -1 right, 0 none
uint8 Roundabout_GetFillMask(int row); // bit0 left, bit1 right; current frame only
int Roundabout_GetRoadWidth(int row); // learned before entry; fallback for a single visible edge
uint8 Roundabout_GetPoint(int which,int *x,int *y);
uint8 Roundabout_AllowDifferential(void); // only normal/inside, never during guide transitions
#endif
