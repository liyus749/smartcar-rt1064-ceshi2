/* Real LineDetect + complete retained VIS7 controller; generated grayscale scenes.
   These are geometry regressions, not a substitute for captured track sequences. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "island.h"
#include "LineDetect.h"
uint8 image_two_value[MT9V03X_H][MT9V03X_W];
volatile int Left_Line[MT9V03X_H],Right_Line[MT9V03X_H],Mid_Line[MT9V03X_H],Road_Wide[MT9V03X_H];
int Left_Lost_Flag[MT9V03X_H],Right_Lost_Flag[MT9V03X_H];
volatile int Search_Stop_Line,Left_Lost_Time,Right_Lost_Time,Both_Lost_Time;
volatile int Boundry_Start_Left,Boundry_Start_Right,Cross_Flag;
volatile uint8 Img_Disappear_Flag;
void Draw_Line(int a,int b,int c,int d) { (void)a;(void)b;(void)c;(void)d; }
void Contrast_Process(const uint8 *image) { (void)image;abort(); }
uint8 Contrast_GetReferenceCol(void) { abort(); }
/* Include actual ring source so extraction/patch guards can also be examined. */
#include "island.c"
static uint8 gray[MT9V03X_H][MT9V03X_W];
static int checks;
#define CHECK(x) do { checks++; if(!(x)) { fprintf(stderr,"FAIL line %d: %s (stage %d U/M/D=%d/%d/%d dir=%d)\n",__LINE__,#x,Island_State,island_debug.left.up.y,island_debug.left.mid.y,island_debug.left.down.y,island_debug.dir); return 1; } } while(0)

enum { STRAIGHT,FAR_RING,NEAR_RING,INNER_CURVE,EXIT_CORNER,LOW_EXIT,BLACKOUT,OUTER_LOST,MONOTONIC_MARK,SPIKE_MARK,BOTTOM_OCCLUDED,ROAD_BARRIER };
static void scene(int kind,int dir)
{
    int y,x;
    for(y=0;y<MT9V03X_H;y++)
    {
        int l=20,r=165;
        if(kind==INNER_CURVE)
        {
            int shift=(99-y)*1;
            l=60-shift;r-=shift;
        }
        if(kind==EXIT_CORNER || kind==LOW_EXIT)
        {
            int center=kind==EXIT_CORNER?60:88,d=abs(y-center);
            if(d<15) r=135+2*d;
        }
        if(kind==OUTER_LOST) r=187;
        for(x=0;x<188;x++) gray[y][x]=(x>=l && x<=r)?220:20;
        if(kind==FAR_RING || kind==NEAR_RING || kind==MONOTONIC_MARK || kind==SPIKE_MARK ||
           kind==BOTTOM_OCCLUDED || kind==ROAD_BARRIER)
        {
            int top=kind==NEAR_RING?50:20,bottom=kind==NEAR_RING?90:70;
            int middle=(top+bottom)/2;
            if(y>=top && y<=bottom)
            {
                int half=(bottom-top)/2,d=abs(y-middle);
                int edge=80-25*d*d/(half*half);
                if(kind==MONOTONIC_MARK) edge=55+25*(y-top)/(bottom-top);
                if(kind==SPIKE_MARK) edge=(y==middle)?85:55;
                /* Detached solid island, no unrelated stripe. The road-facing
                   left contour bulges RIGHT into the connected white lane. */
                for(x=35;x<=edge;x++) gray[y][x]=20;
            }
        }
        if(kind==BLACKOUT) memset(gray[y],20,188);
        if((kind==BOTTOM_OCCLUDED && y==99) || (kind==ROAD_BARRIER && y>=76 && y<=79))
            memset(gray[y],20,188);
        if(dir<0)
            for(x=0;x<94;x++) { uint8 t=gray[y][x];gray[y][x]=gray[y][187-x];gray[y][187-x]=t; }
    }
}
static void process(int kind,int dir)
{
    scene(kind,dir);
    LineDetect_Process(gray[0],LINE_MODE_OTSU);
    Island_Detect();
}
static void steps(int kind,int dir,int count) { while(count--) process(kind,dir); }
static void reset(void)
{ Island_Reset();island_arc_enable=1;Island_SetMoving(1);LineDetect_Init();Cross_Flag=0; }

static int traversal(int dir)
{
    IslandSide *f;
    reset();
    steps(FAR_RING,dir,1); CHECK(Island_State==0);
    steps(STRAIGHT,dir,1);steps(FAR_RING,dir,2);CHECK(Island_State==0);
    steps(FAR_RING,dir,1);CHECK(Island_State==1 && island_debug.dir==dir);
    f=dir>0?&island_debug.left:&island_debug.right;
    CHECK(f->up.y>=18 && f->up.y<=24);
    CHECK(f->mid.y>=40 && f->mid.y<=50);
    CHECK(f->down.y>=66 && f->down.y<=72);
    CHECK(dir*(f->mid.x-f->up.x)>=ISLAND_CORNER_PROM);
    CHECK(dir*(f->mid.x-f->down.x)>=ISLAND_CORNER_PROM);
    steps(FAR_RING,dir,3);CHECK(Island_State==2);
    steps(NEAR_RING,dir,3);CHECK(Island_State==3);
    steps(STRAIGHT,dir,5);CHECK(Island_State==3);
    steps(NEAR_RING,dir,3);CHECK(entry_guided>=ISLAND_CONFIRM);
    CHECK(island_debug.patch_l+island_debug.patch_r>=10);
    for(int y=0;y<100;y++)
    {
        CHECK(Left_Line[y]>=0 && Left_Line[y]<188 && Right_Line[y]>=0 && Right_Line[y]<188);
        if(island_patch_left[y]) CHECK(!Left_Lost_Flag[y]);
        if(island_patch_right[y]) CHECK(!Right_Lost_Flag[y]);
    }
    steps(INNER_CURVE,dir,3);CHECK(Island_State==4);
    steps(EXIT_CORNER,dir,6);CHECK(Island_State==4 && !island_debug.exit_ready);
    steps(OUTER_LOST,dir,4);CHECK(!island_debug.exit_ready);
    steps(INNER_CURVE,dir,3);CHECK(island_debug.exit_ready);
    steps(EXIT_CORNER,dir,3);CHECK(Island_State==5);
    steps(EXIT_CORNER,dir,1);CHECK(exit_guided);
    steps(LOW_EXIT,dir,3);CHECK(Island_State==6);
    steps(STRAIGHT,dir,3);CHECK(Island_State==7);
    steps(STRAIGHT,dir,3);CHECK(Island_State==0 && island_debug.exits==1 && !island_fault);
    CHECK(!Left_Island_Flag && !Right_Island_Flag);
    steps(FAR_RING,dir,5);CHECK(Island_State==0);
    steps(STRAIGHT,dir,3);steps(FAR_RING,dir,3);CHECK(Island_State==1);
    return 0;
}

int main(void)
{
    CHECK(traversal(1)==0);CHECK(traversal(-1)==0);
    for(int dir=-1;dir<=1;dir+=2)
    {
        reset();steps(BOTTOM_OCCLUDED,dir,3);
        CHECK(Island_State==1 && island_debug.dir==dir);
        CHECK(!contour_ok[0][99] && !contour_ok[1][99]);
        reset();steps(ROAD_BARRIER,dir,5);
        CHECK(image_two_value[78][94]==IMG_BLACK); /* survives the real 3x3 filter */
        CHECK(Island_State==0);
        CHECK(!contour_ok[0][70] && !contour_ok[1][70]);
    }
    reset();Cross_Flag=1;steps(FAR_RING,1,6);CHECK(Island_State==0);Cross_Flag=0;
    steps(STRAIGHT,1,12);CHECK(Island_State==0);
    steps(MONOTONIC_MARK,1,6);CHECK(Island_State==0);
    steps(SPIKE_MARK,1,6);CHECK(Island_State==0);
    reset();steps(BLACKOUT,1,6);CHECK(Island_State==0);
    CHECK(island_debug.left.up.y<0 && island_debug.left.mid.y<0 && island_debug.left.down.y<0);
    reset();island_arc_enable=0;steps(FAR_RING,1,6);CHECK(Island_State==0);
    reset();steps(FAR_RING,1,6);steps(NEAR_RING,1,3);CHECK(Island_State==3);
    Island_SetMoving(0);steps(NEAR_RING,1,ISLAND_STALL_FRAMES+10);CHECK(!island_fault && Island_State==3);
    Island_SetMoving(1);steps(NEAR_RING,1,ISLAND_STALL_FRAMES+10);CHECK(island_fault==1 && Island_State==3 && !island_debug.exits);
    Island_Reset();CHECK(!island_fault && Island_State==0);
    process(BLACKOUT,1);CHECK(RescanUpper(1).y<0 && RescanUpper(-1).y<0);
    CHECK(Patch(0,MissingPoint(),PointAt(0,99))==0);
    reset();Island_State=3;island_debug.dir=1;Left_Island_Flag=1;
    steps(INNER_CURVE,1,5);CHECK(Island_State==3 && entry_guided==0);
    printf("PASS: %d visual-ring checks (real grayscale -> LineDetect -> VIS7, both mirrors)\n",checks);
    return 0;
}
