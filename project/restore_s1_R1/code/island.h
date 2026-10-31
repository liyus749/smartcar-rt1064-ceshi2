#ifndef __ISLAND_H__
#define __ISLAND_H__

#include "zf_common_headfile.h"

void Island_Detect(void);
void Image_Flag_Show(uint8 MT9V03XW,uint8(*InImg)[MT9V03XW],uint8 image_flag);
int Monotonicity_Change_Right(int start,int end);
int Monotonicity_Change_Left(int start,int end);
int Continuity_Change_Right(int start,int end);
int Continuity_Change_Left(int start,int end);
int Find_Left_Down_Point(int start,int end);
int Find_Left_Up_Point(int start,int end);
int Find_Right_Down_Point(int start,int end);
int Find_Right_Up_Point(int start,int end);
int Get_Road_Wide(int start_line,int end_line);
void K_Add_Boundry_Left(float k,int startX,int startY,int endY);
void K_Add_Boundry_Right(float k,int startX,int startY,int endY);
void K_Draw_Line(float k, int startX, int startY,int endY);
float Get_Right_K(int start_line,int end_line);
float Get_Left_K(int start_line,int end_line);


/* Initial calibration values for 188x100 images. No gyro input is used. */
#define ISLAND_CONFIRM (3)
#define ISLAND_GAP_MIN (5)
#define ISLAND_CORNER_PROM (3)
#define ISLAND_JUMP_MIN (6)
#define ISLAND_PAIR_X_MAX (40)
#define ISLAND_ARC_MIN (12) /* retained telemetry compatibility, not an entry gate */
#define ISLAND_ARC_MIN_HEIGHT (12)
#define ISLAND_GUIDE_MIN_ROWS (10)
#define ISLAND_CURVE_SHIFT (8)
#define ISLAND_MOUTH_Y (55)
#define ISLAND_ENTRY_Y (45)
#define ISLAND_EXIT_Y (35)
#define ISLAND_LOW_Y (85)
#define ISLAND_STALL_FRAMES (500)
/* x increases to the right, y increases toward the car. Missing point y=-1.
   A left island's road-facing middle is an inward x maximum; right is mirrored. */
typedef struct { int x, y; } IslandPoint;
typedef struct {
    IslandPoint up, mid, down;
    int gap, near_valid, bottom_lost, valid;
} IslandSide;
typedef struct {
    IslandSide left, right;
    int lost_l, lost_r, span, pairs;
    int dir, candidate, confirm, wait, reason, transitions, entries, exits;
    int scan, patch_l, patch_r, exit_ready;
} IslandDebug;
extern IslandDebug island_debug;
extern uint8 island_patch_left[MT9V03X_H], island_patch_right[MT9V03X_H];
extern volatile int island_fault;
extern volatile int Island_State, Left_Island_Flag, Right_Island_Flag;
extern volatile int island_arc_rows, island_arc_max, island_arc_mean_x, island_arc_peak;
extern volatile int island_arc_enable, island_trigger_reason;
void Island_Reset(void);
void Island_SetMoving(uint8 moving);

#endif