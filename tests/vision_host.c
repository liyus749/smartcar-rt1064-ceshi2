/* Synthetic camera frames exercise the actual LineDetect/Cross/Err modules. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LineDetect.h"
#include "Cross.h"
#include "Err.h"
#include "Roundabout.h"

uint8 image_two_value[MT9V03X_H][MT9V03X_W];
volatile int Left_Line[MT9V03X_H], Right_Line[MT9V03X_H];
volatile int Mid_Line[MT9V03X_H], Road_Wide[MT9V03X_H];
int Left_Lost_Flag[MT9V03X_H], Right_Lost_Flag[MT9V03X_H];
volatile int Search_Stop_Line, Left_Lost_Time, Right_Lost_Time, Both_Lost_Time;
volatile int Boundry_Start_Left, Boundry_Start_Right, Island_State, Cross_Flag;
volatile uint8 Img_Disappear_Flag;
volatile int Left_Island_Flag,Right_Island_Flag;
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
    Roundabout_Init();
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
    Roundabout_Process(0);
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


/* A fully white Otsu band still has texture/noise; it must not classify white against white. */
static void test_textured_static_cross(void)
{
    int i,row,col;
    reset();road(0,0);process();
    for(i=0;i<60;i++)
    {
        road(0,0);gap(25,74,220);
        for(row=25;row<=74;row++)
            for(col=0;col<MT9V03X_W;col++)
                gray[row][col]=(uint8)(200+(col*13+row*7)%41);
        process();
        if(i>=CROSS_CONFIRM_FRAMES-1)
        {
            check(Cross_Flag && Cross_GetFillMask(50)==3,
                  "fixed textured white cross cannot drift into dark after repeated Otsu updates");
        }
    }
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
    for(i = 0; i < CROSS_HOLD_FRAMES*4; i++)
    {
        road(12, 0); gap(35, 99, 220); process();
        check(Cross_Flag && Cross_GetFillMask(90)==3,
              "fresh static cross with real exit stays active and guided beyond old timeout");
    }
    for(i = 0; i < CROSS_HOLD_FRAMES; i++) { memset(gray,20,sizeof(gray)); process(); }
    check(!Cross_Flag, "consecutive missing real exits release crossing");
    check(Left_Lost_Flag[90] && Right_Lost_Flag[90], "invalid exit never fabricates near guides");
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


/* Perspective lane plus an oblique transverse white arm, independent of tracked edges. */
static void visual_cross(int shift, int bend, int top, int bottom, int tilt)
{
    int row, col;
    for(row = 0; row < MT9V03X_H; row++)
    {
        int center = MT9V03X_W/2 + shift + bend*(MT9V03X_H-1-row)/99;
        int half_width = 26 + row*24/99;
        for(col = 0; col < MT9V03X_W; col++)
        {
            int arm_shift = tilt*(col-MT9V03X_W/2)/100;
            int lane = col >= center-half_width && col <= center+half_width;
            int arm = top >= 0 && row >= top+arm_shift && row <= bottom+arm_shift;
            gray[row][col] = (lane || arm) ? 220 : 20;
        }
    }
}

static void check_guides(const char *name)
{
    int row, filled_rows = 0;
    for(row = 0; row < MT9V03X_H; row++)
    {
        if(Cross_GetFillMask(row)) { filled_rows++; }
        if(!Left_Lost_Flag[row] && !Right_Lost_Flag[row])
        {
            check(Left_Line[row] >= 3 && Right_Line[row] < MT9V03X_W-3 &&
                  Right_Line[row]-Left_Line[row] >= 15, "guides stay ordered and within the image");
        }
    }
    check(filled_rows >= CROSS_LOST_MIN, name);
}

static void test_oblique_corner_crosses(void)
{
    int direction, i, x, y;
    for(direction = -1; direction <= 1; direction += 2)
    {
        reset(); visual_cross(direction*12, direction*15, -1, -1, 0); process();
        for(i = 0; i < CROSS_CONFIRM_FRAMES; i++)
        {
            visual_cross(direction*12, direction*15, 35, 61, direction*35); process();
        }
        check(Cross_Flag && Cross_GetState()==CROSS_STATE_ENTER, "perspective oblique cross confirms");
        check(Cross_GetCorner(CROSS_UPPER_LEFT,&x,&y), "oblique upper left is observed");
        check(Cross_GetCorner(CROSS_UPPER_RIGHT,&x,&y), "oblique upper right is observed");
        check(Cross_GetCorner(CROSS_LOWER_LEFT,&x,&y), "oblique lower left is observed");
        check(Cross_GetCorner(CROSS_LOWER_RIGHT,&x,&y), "oblique lower right is observed");
        check_guides("oblique gap is bridged");

        // Bring the arm to the car: neither lower corner is visible, use current exit geometry.
        for(i=0;i<CROSS_PHASE_CONFIRM;i++)
        { visual_cross(direction*12, direction*15, 40, 130, direction*35); process(); }
        check(Cross_Flag && Cross_GetState()==CROSS_STATE_INSIDE, "missing lower corners select inside state");
        check(!Cross_GetCorner(CROSS_LOWER_LEFT,&x,&y) && !Cross_GetCorner(CROSS_LOWER_RIGHT,&x,&y),
              "old lower corners are not reused as observations");
        check(Cross_GetFillMask(90)==3, "exit fits guide both sides near the car");
        check_guides("inside oblique crossing remains guided");
        visual_cross(0,0,-1,-1,0); process();
        check(Cross_GetState()==CROSS_STATE_EXIT, "first recovered frame reports exit state");
        visual_cross(0,0,-1,-1,0); process();
        check(!Cross_Flag && Cross_GetState()==CROSS_STATE_NORMAL, "stable real lane releases cross");
    }

    // The right near corner leaves the image before the left, and vice versa.
    for(direction = -1; direction <= 1; direction += 2)
    {
        reset(); visual_cross(0,0,-1,-1,0); process();
        for(i = 0; i < CROSS_CONFIRM_FRAMES; i++)
        { visual_cross(0,0,58,95,direction*40); process(); }
        check(Cross_Flag, "one missing entrance corner still permits oblique entry");
        check((int)Cross_GetCorner(CROSS_LOWER_LEFT,&x,&y)+(int)Cross_GetCorner(CROSS_LOWER_RIGHT,&x,&y)==1,
              "exactly one real entrance corner is reported");
        check_guides("one-sided entrance uses link plus opposite exit fit");
    }
}

static void test_exit_reacquisition(void)
{
    int i, row;
    reset(); visual_cross(0,0,-1,-1,0); process();
    for(i = 0; i < CROSS_CONFIRM_FRAMES; i++)
    {
        visual_cross(0,0,35,65,20);
        LineDetect_Process(gray[0],LINE_MODE_OTSU);
        // Model a tracker that followed the horizontal branch and failed to recover the exit.
        for(row = 2; row < 35; row++)
        {
            Left_Lost_Flag[row]=Right_Lost_Flag[row]=1;
            Left_Line[row]=0;Right_Line[row]=MT9V03X_W-1;
        }
        Cross_Check();LineDetect_RefreshStats();Err_Compute();
    }
    check(Cross_Flag, "raw column geometry can confirm when tracked exit edges are lost");
    check(!Left_Lost_Flag[15] && !Right_Lost_Flag[15], "current exit is retraced directly from binary image");
    check(!Cross_GetFillMask(15), "reacquired real edges are not labelled as artificial guides");
    check_guides("reacquired exit connects through the intersection");
}

static void test_cross_candidate_and_false_shapes(void)
{
    int i,row,col,x,y;
    reset();road(0,0);process();
    road(0,0);gap(35,65,220);process();
    check(!Cross_Flag && Cross_GetState()==CROSS_STATE_CANDIDATE, "first geometry frame stays candidate");
    memset(gray,20,sizeof(gray));process();
    check(Cross_GetState()==CROSS_STATE_NORMAL && !Cross_GetCorner(CROSS_UPPER_LEFT,&x,&y),
          "black interruption clears candidate and current corners");
    road(0,0);gap(35,65,220);process();
    check(!Cross_Flag, "interrupted evidence needs a new consecutive confirmation");

    reset();road(0,0);process();
    for(i=0;i<CROSS_CONFIRM_FRAMES;i++) { road(0,0);gap(35,65,220);process(); }
    memset(gray,20,sizeof(gray));process();
    check(Cross_Flag && Cross_GetState()==CROSS_STATE_ENTER,
          "one invalid image does not advance the physical cross stage");
    for(i=0;i<CROSS_HOLD_FRAMES*3;i++)
    {
        road(0,0);gap(35,65,220);process();
        check(Cross_Flag && Cross_GetState()==CROSS_STATE_ENTER,
              "static entry returns to entry after transient detection loss");
    }
    reset();road(0,0);process();
    // A T junction/right angle has a white transverse arm but no straight-through exit.
    for(i=0;i<4;i++)
    {
        road(0,0);gap(35,65,220);gap(0,34,20);process();
    }
    check(!Cross_Flag, "white arm without a paired far exit is not a straight-through cross");
    for(row=0;row<MT9V03X_H;row++) check(!Cross_GetFillMask(row), "T junction creates no artificial lines");

    // Black triangular splitter blocks the intended path despite a broad white region.
    reset();road(0,0);process();
    for(i=0;i<4;i++)
    {
        road(0,0);gap(30,72,220);
        for(row=15;row<72;row++)
        {
            int half=(72-row)/2;
            for(col=94-half;col<=94+half;col++) gray[row][col]=20;
        }
        process();
    }
    check(!Cross_Flag, "fork splitter cannot be bridged into a white straight path");

    reset();road(0,0);process();
    for(i=0;i<4;i++) { road(0,0);gap(35,99,220);process(); }
    check(!Cross_Flag, "without a previous cross, two far corners alone do not fabricate entrance");


    // An S bend may show white on both outer reference columns, but has no crossing geometry.
    reset();road(0,0);process();
    for(i=0;i<5;i++)
    {
        for(row=0;row<MT9V03X_H;row++)
        {
            int offset = row<25 ? row*35/25 : (row<75 ? 35-(row-25)*70/50 : -35+(row-75)*35/24);
            for(col=0;col<MT9V03X_W;col++)
            { gray[row][col]=(col>=44+offset && col<=144+offset)?220:20; }
        }
        process();
    }
    check(!Cross_Flag, "alternating S-bend side expansions are not a transverse intersection");

    reset();road(0,0);process();
    for(i=0;i<CROSS_CONFIRM_FRAMES;i++) { road(0,0);gap(35,65,220);process(); }
    road(38,0);gap(35,99,220);process();
    check(Cross_Flag && !Cross_GetFillMask(90), "inside cross rejects an abrupt unrelated exit jump");

    // Geometrically unrelated frames must not accumulate a confirmation.
    reset();road(-24,0);process();
    for(i=0;i<6;i++) { road((i%2)?24:-24,0);gap(35,65,220);process(); }
    check(!Cross_Flag, "large alternating corner jumps cannot confirm a crossing");
    check(!Cross_GetCorner(-1,&x,&y) && !Cross_GetCorner(4,&x,&y) && !Cross_GetCorner(0,0,&y),
          "debug access validates corner indices and pointers");
    check(!Cross_GetFillMask(-1) && !Cross_GetFillMask(MT9V03X_H), "debug mask handles invalid rows");
}

/* Near road stays straight while a bend first appears in the far image. */
static void bend_approach(int start_row, int bend)
{
    int row, col;
    for(row = 0; row < MT9V03X_H; row++)
    {
        int shift = (row < start_row) ? bend * (start_row - row) / start_row : 0;
        int center = MT9V03X_W / 2 + shift;
        for(col = 0; col < MT9V03X_W; col++)
            gray[row][col] = (col >= center - 50 && col <= center + 50) ? 220 : 20;
    }
}

static void test_bend_entry_preview(void)
{
    int i, direction;
    for(direction = -1; direction <= 1; direction += 2)
    {
        reset(); road(0, 0); process();
        for(i = 0; i < 8; i++)
        {
            bend_approach(40, direction * 60);
            check(process(), "distant bend still has a valid straight near reference");
        }
        check(ErrUse > -1.5f && ErrUse < 1.5f,
              "far-only bend stays inside steering deadband while near road is centered");
        for(i = 0; i < 8; i++)
        {
            bend_approach(80, direction * 60);
            check(process(), "closer bend still has a usable steering reference");
        }
        check((direction < 0) ? (ErrUse > 6) : (ErrUse < -6),
              "bend reaching the middle and near image requests the proper turn");
    }
}

/* Same generated entry sequence at 80/105/130/240; only cruise speed differs. */
static float preview_entry(float speed, int direction, float *addition)
{
    int i;
    reset(); Err_SetPreviewSpeed(speed); road(0, 0); process();
    for(i = 0; i < 2; i++) { bend_approach(40, direction * 60); process(); }
    check(ErrPreview == 0, "far-only bend does not trigger extra entry preview at high speed");
    bend_approach(60, direction * 60); process();
    check(ErrPreview == 0, "one entry frame cannot trigger extra preview");
    bend_approach(60, direction * 60); process();
    *addition = ErrPreview;
    check(ErrPreview >= -ERR_PREVIEW_MAX && ErrPreview <= ERR_PREVIEW_MAX,
          "entry preview is bounded to four image pixels");
    return ErrUse;
}

static void test_high_speed_entry_preview(void)
{
    int i, row, direction;
    float baseline, intermediate, fast, maximum, addition, old_curve;
    float intermediate_warning, fast_warning;
    for(direction = -1; direction <= 1; direction += 2)
    {
        baseline = preview_entry(80, direction, &addition);
        check(addition == 0, "80-count cruise retains the previous preview behavior");
        intermediate = preview_entry(105, direction, &addition);
        intermediate_warning = (addition >= 0) ? addition : -addition;
        fast = preview_entry(130, direction, &addition);
        fast_warning = (addition >= 0) ? addition : -addition;
        check((direction < 0) ? (addition > 0.5f) : (addition < -0.5f),
              "130-count cruise gives an early speed warning on either bend direction");
        check(intermediate_warning > 0 && intermediate_warning < fast_warning,
              "speed warning grows gradually between speed 80 and 130");
        check(baseline == intermediate && baseline == fast,
              "preview speed cannot amplify the steering error on identical entry images");
        maximum = preview_entry(240, direction, &addition);
        check(maximum == fast && ((addition >= 0) ? addition : -addition) == fast_warning,
              "higher settings cannot increase preview beyond the 130-speed cap");
        for(i = 0; i < 5; i++) { road(0, 0); process(); }
        check(ErrPreview == 0 && ErrUse > -1.5f && ErrUse < 1.5f,
              "entry preview releases on straight frames without a turn hold");

        /* Near-position correction in an established bend must remain unchanged. */
        reset(); Err_SetPreviewSpeed(80);
        for(i = 0; i < 8; i++) { road(direction * 20, direction * 30); process(); }
        old_curve = ErrUse;
        reset(); Err_SetPreviewSpeed(130);
        for(i = 0; i < 8; i++) { road(direction * 20, direction * 30); process(); }
        check(ErrPreview == 0 && ErrUse == old_curve,
              "established near-road turn keeps its original steering calculation");
    }
    /* Never derive anticipatory turning from a cross, reconstructed single edge, or S reversal. */
    reset(); road(0,0); process(); Err_SetPreviewSpeed(130);
    for(i=0;i<2;i++) { bend_approach(60,-60); process(); }
    check(ErrPreview>0, "entry preview can become active before testing invalidation");
    Cross_Flag=1;
    check(Err_Compute() && ErrPreview==0, "cross guides cannot activate entry preview");
    Cross_Flag=0;
    for(row=20;row<60;row++) { Left_Lost_Flag[row]=1; }
    for(i=0;i<3;i++) { Err_Compute(); }
    check(ErrPreview==0, "single-edge estimates cannot qualify as reliable entry preview");

    reset(); road(0,0); process(); Err_SetPreviewSpeed(130);
    for(i=0;i<5;i++) { road((i%2)?-1:1,0); process(); }
    check(ErrPreview==0 && ErrUse>-1.5f && ErrUse<1.5f, "high-speed setting does not amplify straight-road noise");
    /* Inject an opposed far/middle shape directly after real line extraction. */
    bend_approach(60,-60); process();
    for(row=20;row<40;row++) { Left_Line[row]=54; Right_Line[row]=154; }
    for(i=0;i<3;i++) { Err_Compute(); }
    check(ErrPreview==0, "opposite far and middle directions do not trigger anticipatory steering");
    memset(gray,20,sizeof(gray)); process();
    check(ErrPreview==0, "lost frame clears preview and confirmation history");
    Err_SetPreviewSpeed(0.0f/0.0f); road(0,0); process();
    check(ErrPreview==0 && ErrUse==ErrUse, "invalid preview speed cannot contaminate steering with NaN");
}

static void test_confirmed_straight_recenter(void)
{
    int i, row, direction;
    for(direction=-1;direction<=1;direction+=2)
    {
        reset(); Err_SetPreviewSpeed(180);
        for(i=0;i<5;i++) { road(direction*30,0); process(); }
        road(0,0); process();
        check((ErrUse>1.5f)||(ErrUse<-1.5f), "a single centered image cannot clear all previous turning error");
        road(0,0); process();
        check(ErrUse>=-1.5f && ErrUse<=1.5f,
              "two real centered straight frames release old corner error into the steering deadband");
        /* A new visible middle bend is never forcibly straightened. */
        reset(); road(direction*30,0); process();
        for(i=0;i<3;i++) { bend_approach(80,direction*60); process(); }
        check((direction<0)?(ErrUse>6):(ErrUse<-6), "middle bend remains steerable despite a centered bottom row");
    }
    reset(); road(-30,0); process();
    road(0,0); process();
    memset(gray,20,sizeof(gray)); process();
    road(0,0); process();
    check(ErrUse>1.5f, "invalid frame interrupts straight confirmation");

    reset(); road(-30,0); process();
    road(0,0); LineDetect_Process(gray[0],LINE_MODE_OTSU);
    LineDetect_RefreshStats();
    for(row=40;row<100;row++) { Left_Lost_Flag[row]=1; }
    for(i=0;i<2;i++) { Err_Compute(); }
    check(ErrUse>1.5f, "reconstructed single-edge straight cannot force rapid recenter");
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
    test_textured_static_cross();
    test_rejections();
    test_oblique_corner_crosses();
    test_exit_reacquisition();
    test_cross_candidate_and_false_shapes();
    test_bend_entry_preview();
    test_high_speed_entry_preview();
    test_confirmed_straight_recenter();
    test_corner_exit_response();
    printf("PASS: %d vision checks (real LineDetect/Cross/Err sources)\n", checks);
    return 0;
}
