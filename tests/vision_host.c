/* Synthetic camera frames exercise the actual LineDetect/Cross/Err modules. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LineDetect.h"
#include "Cross.h"
#include "Err.h"

uint8 image_two_value[MT9V03X_H][MT9V03X_W];
volatile int Left_Line[MT9V03X_H], Right_Line[MT9V03X_H];
volatile int Mid_Line[MT9V03X_H], Road_Wide[MT9V03X_H];
int Left_Lost_Flag[MT9V03X_H], Right_Lost_Flag[MT9V03X_H];
volatile int Search_Stop_Line, Left_Lost_Time, Right_Lost_Time, Both_Lost_Time;
volatile int Boundry_Start_Left, Boundry_Start_Right, Island_State, Cross_Flag;
volatile uint8 Img_Disappear_Flag;
static uint8 gray[MT9V03X_H][MT9V03X_W];
static int checks;

/* The selected Otsu mode never invokes the alternate hardware-facing module. */
void Contrast_Process(const uint8 *image) { (void)image; abort(); }
uint8 Contrast_GetReferenceCol(void) { abort(); }

static void check(int condition, const char *name)
{
    if(!condition) { fprintf(stderr, "FAIL: %s\n", name); exit(1); }
    checks++;
}

static void reset(void)
{
    Island_State = 0;
    LineDetect_Init();
    Cross_Init();
    Err_Init();
}

/* delta shifts the whole road; bend shifts the distant part of the road. */
static void road(int delta, int bend)
{
    int row, col;
    for(row = 0; row < MT9V03X_H; row++)
    {
        int center = MT9V03X_W / 2 + delta + bend * (MT9V03X_H - 1 - row) / 99;
        for(col = 0; col < MT9V03X_W; col++)
        {
            gray[row][col] = (col >= center - 50 && col <= center + 50) ? 220 : 20;
        }
    }
}

static void gap(int top, int bottom, uint8 color)
{
    int row;
    for(row = top; row <= bottom; row++) { memset(gray[row], color, MT9V03X_W); }
}

static int process(void)
{
    LineDetect_Process(gray[0], LINE_MODE_OTSU);
    Cross_Check();
    LineDetect_RefreshStats();
    return Err_Compute();
}

static void test_straight_and_curves(void)
{
    int i, lost;
    reset(); road(0, 0);
    check(process(), "straight road has valid steering");
    check(Err > -2 && Err < 2, "centered road gives near-zero error");
    check(!Left_Lost_Flag[99] && !Right_Lost_Flag[99], "bottom row is processed");
    check(!Cross_Flag, "straight road is not a cross");
    lost = 0;
    for(i = 0; i < MT9V03X_H; i++) { lost += Left_Lost_Flag[i]; }
    check(lost == Left_Lost_Time, "statistics match final left flags");

    road(0, -75);
    check(process(), "left bend remains usable when an edge leaves the image");
    check(Err > 10, "left bend requests left steering");
    check(Left_Lost_Flag[10] && !Right_Lost_Flag[10], "single-edge loss preserves visible edge");
    check(ErrValidRows > 80, "learned width keeps single-edge rows usable");
    check(!Cross_Flag, "left bend is not a cross");

    road(0, 75);
    check(process(), "right bend remains usable");
    check(Err < -10, "right bend requests right steering");
    check(!Cross_Flag, "right bend is not a cross");
}

static void test_cross(int top, int bottom, int shift)
{
    int i;
    reset(); road(shift, 0); check(process(), "approach is valid");
    for(i = 0; i < CROSS_CONFIRM_FRAMES; i++)
    {
        road(shift, 0); gap(top, bottom, 220);
        check(process(), "cross approach still has a steering reference");
        if(i == 0)
        {
            check(!Cross_Flag, "one frame does not confirm cross");
            check(Left_Lost_Flag[(top + bottom) / 2], "unconfirmed gap is not filled");
        }
    }
    check(Cross_Flag, "cross confirmed after consecutive frames");
    check(!Left_Lost_Flag[(top + bottom) / 2], "confirmed cross gets a left guide");
    check(!Right_Lost_Flag[(top + bottom) / 2], "confirmed cross gets a right guide");
    check(Cross_GetRunLen() >= CROSS_LOST_MIN, "cross gap remains observable");
    if(shift)
    {
        check(Err < -5, "offset cross keeps useful steering instead of forcing zero");
    }

    /* A broad white arm may occupy an entire Otsu band for many frames. */
    for(i = 0; i < 20; i++)
    {
        road(shift, 0); gap(top, bottom, 220); process();
    }
    check(Cross_Flag, "prolonged white cross does not turn into a black obstruction");
    check(!Left_Lost_Flag[(top + bottom) / 2], "prolonged cross remains guided");

    /* While the gap reaches the car, the old seed must find the distant exit. */
    road(shift, 0); gap(45, 99, 220);
    check(process(), "near cross gap does not hide the distant exit");
    check(Cross_Flag, "cross stays active while near road is absent");
    check(!Left_Lost_Flag[90] && !Right_Lost_Flag[90], "confirmed near white gap keeps a guide to the car");
    check(ErrValidRows > 80, "near cross gap uses more than the distant exit for steering");
    for(i = 0; i < CROSS_EXIT_FRAMES; i++) { road(0, -55); process(); }
    check(!Cross_Flag, "cross exits as near road returns");
    check(Err > 5, "curve after cross immediately contributes steering");

    for(i = 0; i < CROSS_CONFIRM_FRAMES; i++)
    {
        road(0, 0); gap(top, bottom, 220); process();
    }
    check(Cross_Flag, "a second cross can be recognized without restart");
}

/* Small dark marks in the white cross arm must not become lane boundaries. */
static void test_cross_with_marks(void)
{
    int i, row, col;
    reset(); road(0, 0); process();
    for(i = 0; i < CROSS_CONFIRM_FRAMES; i++)
    {
        road(0, 0); gap(35, 65, 220);
        for(row = 38; row <= 62; row += 6)
        {
            int center = ((row / 6) % 2) ? 46 : 142;
            int y;
            for(y = row; y < row + 3; y++)
                for(col = center - 1; col <= center + 1; col++) gray[y][col] = 20;
        }
        process();
    }
    check(Cross_Flag, "short dark marks do not split the cross into false edge segments");
    check(!Left_Lost_Flag[50] && !Right_Lost_Flag[50], "marked cross is bridged after confirmation");
    check(Err > -3 && Err < 3, "symmetric marked cross does not pull steering sideways");

    road(0, 0); gap(35, 99, 220); process();
    check(!Left_Lost_Flag[90] && !Right_Lost_Flag[90], "exit-guided crossing covers the lower white area");
    memset(gray, 20, sizeof(gray)); process();
    check(Left_Lost_Flag[90] && Right_Lost_Flag[90], "confirmed crossing cannot bridge a new dark obstruction");
    memset(gray, 220, sizeof(gray)); process();
    check(Left_Lost_Flag[90] && Right_Lost_Flag[90], "crossing needs a real distant exit even after confirmation");
}

static void test_cross_continuation_limits(void)
{
    int i;
    reset(); road(0, 0); process();
    for(i = 0; i < CROSS_CONFIRM_FRAMES; i++)
    {
        road(0, 0); gap(35, 60, 220); process();
    }
    road(12, 0); gap(35, 99, 220); process();
    check(Err < -2, "cross continuation follows the real offset exit instead of forcing center");
    for(i = 0; i < CROSS_HOLD_FRAMES; i++)
    {
        road(12, 0); gap(35, 99, 220); process();
    }
    check(!Cross_Flag, "missing near road cannot retain crossing indefinitely");
    check(Left_Lost_Flag[90] && Right_Lost_Flag[90], "expired cross continuation stops fabricating near guides");
}

static void test_rejections(void)
{
    int i;
    reset(); road(0, 0); process();
    for(i = 0; i < 4; i++) { road(0, 0); gap(40, 60, 20); process(); }
    check(!Cross_Flag, "black obstruction is not a white crossing");
    check(Left_Lost_Flag[50] && Right_Lost_Flag[50], "black obstruction is not fabricated into a road");
    for(i = 0; i < 4; i++) { road(0, 0); gap(2, 65, 220); process(); }
    check(!Cross_Flag, "no distant exit means no cross confirmation");
    check(Left_Lost_Flag[40] && Right_Lost_Flag[40], "missing exit does not get vertical fake lines");

    reset(); road(0, 0); process();
    for(i = 0; i < 3; i++)
    {
        road(0, 0); gap(40, 55, 220); memset(gray[99], 20, MT9V03X_W); process();
    }
    check(Cross_Flag, "isolated invalid bottom row does not mask a middle cross");

    memset(gray, 20, sizeof(gray));
    check(!process(), "fully dark frame has no valid steering reference");
    check(ErrValidRows == 0 && ErrDFrame == 0, "invalid frame clears confidence and derivative");
    for(i = 0; i < CROSS_HOLD_FRAMES; i++) { process(); }
    check(!Cross_Flag, "lost image cannot latch cross forever");
}

static void test_corner_exit_response(void)
{
    int i;
    reset();
    for(i = 0; i < 5; i++) { road(-30, 0); process(); }
    check(ErrUse > 25, "turning error remains available in the bend");
    for(i = 0; i < 3; i++) { road(0, 0); process(); }
    check(ErrUse > -5 && ErrUse < 5, "old turning error releases within three straight frames");
    for(i = 0; i < 2; i++) { road(0, 0); process(); }
    check(ErrUse > -2 && ErrUse < 2, "corner exit settles near center within five frames");

    reset(); road(-30, 0); process();
    for(i = 0; i < 2; i++) { road(20, 0); process(); }
    check(ErrUse < 0, "opposite correction is not hidden by old corner error");

    reset(); road(30, 0); process();
    for(i = 0; i < 3; i++) { road(0, 0); process(); }
    check(ErrUse > -5 && ErrUse < 5, "right corner releases as quickly as left corner");
    for(i = 0; i < 20; i++) { road((i % 2) ? -1 : 1, 0); process(); }
    check(ErrUse > -1.5f && ErrUse < 1.5f, "small straight-road noise stays within steering deadband");
}

int main(void)
{
    test_straight_and_curves();
    test_cross(40, 55, 0);
    test_cross(25, 70, 12); /* wider than the old 28-row stop condition */
    test_cross_with_marks();
    test_cross_continuation_limits();
    test_rejections();
    test_corner_exit_response();
    printf("PASS: %d vision checks (real LineDetect/Cross/Err sources)\n", checks);
    return 0;
}
