/* Ground-plane scenes and image-level negatives exercise the production camera pipeline. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "LineDetect.h"
#include "Cross.h"
#include "Roundabout.h"
#include "Err.h"

uint8 image_two_value[MT9V03X_H][MT9V03X_W];
volatile int Left_Line[MT9V03X_H],Right_Line[MT9V03X_H],Mid_Line[MT9V03X_H],Road_Wide[MT9V03X_H];
int Left_Lost_Flag[MT9V03X_H],Right_Lost_Flag[MT9V03X_H];
volatile int Search_Stop_Line,Left_Lost_Time,Right_Lost_Time,Both_Lost_Time;
volatile int Boundry_Start_Left,Boundry_Start_Right,Island_State,Cross_Flag,Left_Island_Flag,Right_Island_Flag;
volatile uint8 Img_Disappear_Flag;
static uint8 gray[MT9V03X_H][MT9V03X_W];
static int checks;
void Contrast_Process(const uint8 *p) { (void)p;abort(); }
uint8 Contrast_GetReferenceCol(void) { abort(); }
static void check(int ok,const char *why)
{ if(!ok) { fprintf(stderr,"FAIL %s (state=%d dir=%d fault=%d)\n",why,Island_State,Roundabout_GetDirection(),Roundabout_GetFault());exit(1); }checks++; }
static void reset(void)
{ LineDetect_Init();Cross_Init();Roundabout_Init();Err_Init(); }
static unsigned checksum(void)
{ unsigned h=2166136261U;for(int y=0;y<100;y++)for(int x=0;x<188;x++)h=(h^image_two_value[y][x])*16777619U;return h; }
static void process(int moving)
{
    LineDetect_Process(gray[0],LINE_MODE_OTSU);
    unsigned before=checksum();
    Cross_Check();Roundabout_Process((uint8)moving);LineDetect_RefreshStats();Err_Compute();
    check(before==checksum(),"element processing never modifies the binary image");
    check(!(Cross_Flag&&Island_State),"cross and roundabout are mutually exclusive");
    for(int y=2;y<100;y++) if(Roundabout_GetFillMask(y))
    {
        int l=Left_Line[y],r=Right_Line[y];
        check(l>=3&&r<185&&r-l>=15,"guide pair remains bounded and ordered");
        check(image_two_value[y][(l+r)/2]==IMG_WHITE,"guide center stays on actual white pixels");
    }
    Roundabout_Tick20ms((uint8)moving);Roundabout_Tick20ms((uint8)moving);
}
static void road(int bend,int cross)
{
    for(int y=0;y<100;y++)for(int x=0;x<188;x++)
    {
        int c=94+bend*(99-y)/99;
        gray[y][x]=(abs(x-c)<=35+y/7 || (cross&&y>=35&&y<=65)) ? 220:20;
    }
}
static void ellipse(int bottom,int mirror)
{
    // Right-side through road plus a white left annulus with an isolated black island.
    road(0,0);
    if(mirror)for(int y=0;y<100;y++)for(int x=0;x<94;x++)
    { uint8 value=gray[y][x];gray[y][x]=gray[y][187-x];gray[y][187-x]=value; }
    int cy=bottom-17;
    for(int y=0;y<100;y++)for(int x=0;x<188;x++)
    {
        int xx=mirror ? 187-x:x;
        float d=(xx-45)*(xx-45)/900.0f+(y-cy)*(y-cy)/1225.0f;
        if(d<1.0f)gray[y][x]=220;
        float q=(xx-45)*(xx-45)/196.0f+(y-cy)*(y-cy)/289.0f;
        if(q<1.0f)gray[y][x]=20;
    }
}
static void curved_entry(int mirror)
{
    ellipse(85,mirror);
    for(int y=85;y<100;y++)for(int x=0;x<188;x++)
    { int xx=mirror ? 187-x:x;if(xx<70)gray[y][x]=220; }
}
static void ground(float vehicle_x,float vehicle_y,float turn,int mirror)
{
    float a=turn*0.01745329252f;
    for(int y=0;y<100;y++)for(int x=0;x<188;x++)
    {
        int xx=mirror ? 187-x:x;
        float d=(1.30f-0.00981f*y)/(1.0f+0.009436f*y);
        float lateral=(xx-94)/(1.08f*(y+106));
        float wx=vehicle_x+cosf(a)*lateral-sinf(a)*d;
        float wy=vehicle_y+sinf(a)*lateral+cosf(a)*d;
        float radius=(wx+0.38f)*(wx+0.38f)+wy*wy;
        int on_road=(fabsf(wx)<=0.18f)||(radius>=0.18f*0.18f&&radius<=0.60f*0.60f);
        gray[y][x]=on_road ? 220:20;
    }
}
static void negatives(void)
{
    for(int bend=-100;bend<=100;bend+=10)
    {
        reset();for(int k=0;k<10;k++){road(bend,0);process(1);}
        check(Island_State==0,"ordinary left/right bends do not activate roundabouts");
    }
    reset();for(int k=0;k<12;k++){road(0,1);process(1);}
    check(Cross_Flag&&Island_State==0,"cross stays a cross with roundabout enabled");
    reset();for(int k=0;k<12;k++){memset(gray,220,sizeof(gray));process(1);}
    check(Island_State==0,"full white is not a roundabout");
    reset();for(int k=0;k<12;k++){memset(gray,20,sizeof(gray));process(1);}
    check(Island_State==0,"full black is not a roundabout");
    reset();for(int k=0;k<12;k++){
        road(0,0);for(int y=25;y<70;y++)for(int x=0;x<40;x++)gray[y][x]=220;process(1);
    }check(Island_State==0,"one-sided T opening without an island is rejected");
    reset();for(int k=0;k<12;k++){
        road(0,0);for(int y=40;y<44;y++)for(int x=65;x<69;x++)gray[y][x]=20;process(1);
    }check(Island_State==0,"small dark speckles are rejected");
}
static void recognition(int mirror)
{
    reset();road(0,0);process(0);
    for(int k=0;k<ROUND_CONFIRM_FRAMES;k++)
    {
        ellipse(65,mirror);process(0);
        if(k+1<ROUND_CONFIRM_FRAMES)check(Island_State==0,"candidate needs consecutive real frames");
    }
    check(Island_State==ROUND_APPROACH,"three-corner contour activates approach");
    check(Roundabout_GetDirection()==(mirror ? -1:1),"left/right mirror direction is correct");
    for(int k=0;k<30;k++){ellipse(65,mirror);process(0);}
    check(Island_State==ROUND_APPROACH&&!Roundabout_GetFault(),"parked camera does not consume stage timers or advance phases");
    check(!Roundabout_AllowDifferential(),"entry approach suppresses differential");
    for(int k=0;k<ROUND_CAMERA_TIMEOUT_TICKS+1;k++)Roundabout_Tick20ms(1);
    check(Roundabout_GetFault()==2,"missing camera interrupts cannot keep turning indefinitely");
    Roundabout_Retry();check(Roundabout_ResetPending(),"retry waits for a new camera frame");
    road(0,0);process(0);check(!Roundabout_GetFault()&&!Island_State,"new frame rearms stopped car");
}
// Mutate the same visible contour rather than asserting internal detector math.
static void corner_scene(int mirror,int kind)
{
    ellipse(65,mirror);
    for(int y=0;y<100;y++)for(int x=0;x<188;x++)
    {
        int xx=mirror ? 187-x:x;
        if(kind==0 && y>=47 && y<=49 && xx<=45)gray[y][x]=20; // island attached to background
        if(kind==1 && y>=48 && xx>=40 && xx<=59)gray[y][x]=20; // lower corner out of view
        if(kind==2 && y<=48 && xx>=40 && xx<=59)gray[y][x]=20; // upper corner out of view
        if((kind==3 || kind==4) && y>=32 && y<=64 && xx>=31 && xx<=60)
            gray[y][x]=(xx<50 || (kind==4 && y==48 && xx<56)) ? 20:220; // no peak / one-row spike
    }
}
static void three_corner_recognition(void)
{
    int coords[2][ROUND_POINT_EXIT][2];
    for(int mirror=0;mirror<2;mirror++)
    {
        reset();
        for(int k=0;k<ROUND_CONFIRM_FRAMES;k++)
        {
            corner_scene(mirror,0);process(0);
            for(int p=0;p<ROUND_POINT_EXIT;p++)
                check(Roundabout_GetPoint(p,&coords[mirror][p][0],&coords[mirror][p][1]),
                      "all three actual corners are visible even before confirmation");
            if(k+1<ROUND_CONFIRM_FRAMES)check(!Island_State,"three corners still require three new frames");
        }
        check(Island_State==ROUND_APPROACH,"three corners recognize an island connected to the frame background");
        int *lower=coords[mirror][ROUND_POINT_LOWER],*middle=coords[mirror][ROUND_POINT_MIDDLE],*upper=coords[mirror][ROUND_POINT_UPPER];
        check(lower[1]>middle[1] && middle[1]>upper[1],"lower, middle and upper corners have correct row ordering");
        check((middle[0]-lower[0])*(mirror ? -1:1)>=4 && abs(middle[0]-upper[0])<=ROUND_UPPER_MAX_DX,
              "article middle turns inward from the lower road lip and stays near the upper road lip");
        check(lower[1]>=75 && middle[1]>=40 && middle[1]<=55 && upper[1]<=25,
              "article lower and upper points belong to the approach roads, not the island tips");
        for(int kind=1;kind<=4;kind++)
        {
            reset();
            for(int k=0;k<8;k++){corner_scene(mirror,kind);process(1);}
            check(!Island_State,"missing lower/upper/peak and isolated spikes cannot confirm a roundabout");
        }
        reset();ellipse(75,mirror);process(0);ellipse(55,mirror);process(0);ellipse(55,mirror);process(0);
        check(!Island_State,"jumping corners restart consecutive-frame confirmation");
        ellipse(55,mirror);process(0);
        check(Island_State==ROUND_APPROACH,"three stable corners after a jump can confirm afresh");
        road(0,0);process(0);
        for(int p=0;p<ROUND_POINT_EXIT;p++)
        { int x,y;check(!Roundabout_GetPoint(p,&x,&y),"lost features do not leave stale displayed corners"); }
    }
    for(int p=0;p<ROUND_POINT_EXIT;p++)
    {
        check(coords[0][p][0]+coords[1][p][0]==187 && coords[0][p][1]==coords[1][p][1],
              "all three physical corner coordinates mirror exactly");
    }
}
static void article_entry_classification(void)
{
    for(int mirror=0;mirror<2;mirror++)
    {
        int x,y;
        reset();
        for(int k=0;k<8;k++){curved_entry(mirror);process(1);}
        check(!Island_State,"user requires the lower road corner even when near-side loss and middle/upper are visible");
        reset();
        for(int k=0;k<ROUND_CONFIRM_FRAMES;k++){ellipse(75,mirror);process(0);}
        check(Island_State==ROUND_APPROACH,"a near entrance still confirms all three corners into state one");
        check(Roundabout_GetPoint(ROUND_POINT_LOWER,&x,&y) && y>=ROUND_ENTRY_LOWER_ROW,
              "near entrance retains a measured lower corner, not a fabricated image-border point");
        for(int k=0;k<ROUND_PHASE_CONFIRM;k++){ellipse(75,mirror);process(1);}
        check(Island_State==ROUND_WAIT_ENTRY,"visible lower corner approaching the car advances state one to two");

        reset();
        for(int k=0;k<8;k++){ellipse(85,mirror);process(1);}
        check(!Island_State,"middle and upper without the lower approach-road corner cannot confirm a ring");
        reset();
        for(int k=0;k<8;k++)
        {
            ellipse(65,mirror);
            for(int row=0;row<=31;row++)for(int col=0;col<188;col++)
            { int xx=mirror ? 187-col:col;if(xx<75)gray[row][col]=220; }
            process(1);
        }
        check(!Island_State,"lower and middle without a continuous far-road upper corner cannot confirm a ring");
        reset();
        for(int k=0;k<8;k++)
        {
            ellipse(65,mirror);
            for(int row=0;row<=20;row++)for(int col=0;col<188;col++)
            { int xx=mirror ? 187-col:col;if(xx<90)gray[row][col]=20; }
            process(1);
        }
        check(!Island_State,"a far-road corner too far sideways from the middle point is rejected");
        reset();
        for(int k=0;k<8;k++)
        {
            ellipse(65,mirror);
            for(int row=0;row<100;row++)for(int col=0;col<188;col++)
            { int xx=mirror ? 187-col:col;if(xx>120)gray[row][col]=220; }
            process(1);
        }
        check(!Island_State,"three-point ring entry requires independent opposite-edge support");
    }
}
static void ground_sequence(int mirror,int fresh)
{
    if(fresh) { reset(); }
    else { check(!Island_State && !Roundabout_GetFault(),"next ring starts without rebooting or resetting the module"); }
    // First expose the complete entrance required by this user's course. The
    // later projected views may crop corners after recognition has latched.
    for(int k=0;k<ROUND_CONFIRM_FRAMES;k++){ellipse(65,mirror);process(0);}
    check(Island_State==ROUND_APPROACH,"full entrance is confirmed before projected partial views");
    int stages=1<<Island_State;
    for(int step=0;step<=14;step++)
    {
        float vy=-1.0f+0.05f*step;
        for(int k=0;k<3;k++){ground(0,vy,0,mirror);process(1);stages|=1<<Island_State;}
    }
    check((stages&(1<<ROUND_APPROACH))&&(stages&(1<<ROUND_WAIT_ENTRY)),"full entrance advances through approach and waiting before entry");
    for(int deg=0;deg<=350;deg+=5)
    {
        float a=deg*0.01745329252f;
        for(int k=0;k<2;k++){
            ground(-0.38f+0.38f*cosf(a),0.38f*sinf(a),(float)deg,mirror);process(1);stages|=1<<Island_State;
        }
        if(deg<240)check(Island_State<=ROUND_INSIDE,"ordinary outer curve leaving the frame is not an exit");
        if(Island_State!=ROUND_INSIDE)check(Err*(mirror ? -1:1)>-2,"entry and exit guides steer toward the selected side");
        else check(isfinite(Err)&&fabsf(Err)<80,"inside keeps bounded real camera feedback, allowing corrective countersteer");
    }
    for(int step=0;step<=20;step++)
    {
        for(int k=0;k<3;k++){ground(0,0.05f*step,0,mirror);process(1);stages|=1<<Island_State;}
    }
    check(!Roundabout_GetFault(),"projected ring sequence retains valid camera guidance");
    check((stages&0xfe)==0xfe,"all seven ring phases are exercised");
    check(Island_State==ROUND_NORMAL,"exit reaches ordinary tracking again");
}
static void loss_and_timing(void)
{
    for(int color=0;color<2;color++)
    {
        reset();
        for(int k=0;k<ROUND_CONFIRM_FRAMES;k++){ellipse(65,0);process(0);}
        for(int k=0;k<ROUND_MAX_MISSING_FRAMES;k++){memset(gray,color ? 220:20,sizeof(gray));process(1);}
        check(Roundabout_GetFault()==1,"missing image evidence stops a confirmed ring for black AND white frames");
        for(int y=0;y<100;y++)check(!Roundabout_GetFillMask(y),"no stale guides survive image loss");
        road(0,0);process(0);check(Roundabout_GetFault()==1,"fault is latched, not automatically restarted");
    }
    reset();
    for(int k=0;k<ROUND_CONFIRM_FRAMES;k++){ellipse(65,0);process(0);}
    for(int k=0;k<ROUND_MAX_MISSING_FRAMES;k++){road(0,0);process(1);}
    check(Roundabout_GetFault()==1,"a plain straight cannot keep a vanished three-corner entrance alive");
    reset();
    for(int k=0;k<ROUND_CONFIRM_FRAMES;k++){ellipse(75,1);process(0);}
    check(Island_State==ROUND_APPROACH,"near first view still requires three points and starts in state one");
    for(int k=0;k<ROUND_PHASE_TIMEOUT_TICKS+1;k++)
    { ellipse(75,1);process(0);Roundabout_Tick20ms(1); }
    check(Roundabout_GetFault()==3,"fresh but unchanged geometry cannot turn forever while running");
    reset();
    ellipse(65,0);process(0);road(0,0);process(0);
    ellipse(65,0);process(0);ellipse(65,0);process(0);
    check(!Island_State,"interrupted candidate evidence does not accumulate confirmations");
}
int main(void)
{
    negatives();recognition(0);recognition(1);three_corner_recognition();article_entry_classification();ground_sequence(0,1);ground_sequence(1,0);loss_and_timing();
    printf("PASS: %d roundabout vision assertions\n",checks);
}
