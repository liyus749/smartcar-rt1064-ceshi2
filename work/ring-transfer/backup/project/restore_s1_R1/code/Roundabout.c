#include "Roundabout.h"
#include "Cross.h"
#include "LineDetect.h"
#include <string.h>

extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];
extern volatile int Left_Line[MT9V03X_H],Right_Line[MT9V03X_H];
extern int Left_Lost_Flag[MT9V03X_H],Right_Lost_Flag[MT9V03X_H];
// Keep the existing display/legacy camera ABI; the old gyro state machine is not called.
extern volatile int Island_State,Left_Island_Flag,Right_Island_Flag;

typedef struct { int x,y; uint8 valid; } Point;
typedef struct {
    int x0,x1,y0,y1,side;
    Point lower,middle,upper; // physical coordinates of this frame
    uint8 valid;
} Island;
typedef struct { Point lower,upper; int rows; uint8 valid; } Opening;

static Island observed,best,previous;
static int best_score;
static Point points[ROUND_POINT_COUNT];
static int width_history[MT9V03X_H];
static int inner[MT9V03X_H],outer[MT9V03X_H];
static uint8 inner_ok[MT9V03X_H],outer_ok[MT9V03X_H],filled[MT9V03X_H];
static uint8 clearances[MT9V03X_W];
static int costs[2][MT9V03X_W];
static signed char parents[MT9V03X_H][MT9V03X_W];
static int route[MT9V03X_H];
static volatile uint8 fault,reset_pending;
static volatile uint16 frame_age,phase_ticks,total_ticks;
static int direction,candidate_side,near_center;
static uint8 confirm,phase_confirm,missing,exit_armed,outer_stable,cooldown,recovered;
static uint8 entry_guided,exit_guided;
static int guide_goal;

static int abs_i(int v) { return v<0 ? -v:v; }
static int clip(int v,int lo,int hi) { return v<lo ? lo:(v>hi ? hi:v); }
static int nx(int physical) { return direction<0 ? MT9V03X_W-1-physical:physical; }
static int px(int normalized) { return direction<0 ? MT9V03X_W-1-normalized:normalized; }
static int white(int y,int x)
{ return y>=2 && y<MT9V03X_H && x>=2 && x<MT9V03X_W-2 && image_two_value[y][px(x)]==IMG_WHITE; }
static void set_stage(int stage)
{ if(fault) { return; } Island_State=stage; phase_ticks=0; phase_confirm=missing=0; }
static void fail(uint8 reason)
{ fault=reason; Island_State=ROUND_FAULT; }
static void clear_overlay(void)
{ memset(points,0,sizeof(points)); memset(filled,0,sizeof(filled)); }
static int active(void) { return Island_State!=ROUND_NORMAL; }

void Roundabout_Init(void)
{
    int y;
    Island_State=Left_Island_Flag=Right_Island_Flag=0;
    direction=candidate_side=0; near_center=MT9V03X_W/2;
    confirm=phase_confirm=missing=exit_armed=outer_stable=cooldown=recovered=0;
    entry_guided=exit_guided=0;guide_goal=MT9V03X_W/2;
    fault=reset_pending=0; frame_age=phase_ticks=total_ticks=0;
    memset(&previous,0,sizeof(previous)); clear_overlay();
    for(y=0;y<MT9V03X_H;y++) { width_history[y]=20+80*y/(MT9V03X_H-1); }
}
void Roundabout_Retry(void) { reset_pending=1; }
uint8 Roundabout_ResetPending(void) { return reset_pending; }
uint8 Roundabout_GetFault(void) { return fault; }
int Roundabout_GetDirection(void) { return direction; }
uint8 Roundabout_AllowDifferential(void)
{ return !fault && (Island_State==ROUND_NORMAL || Island_State==ROUND_INSIDE); }
void Roundabout_Tick20ms(uint8 moving)
{
    if(frame_age<65535U) { frame_age++; }
    if(!active() || fault || reset_pending || !moving) { return; }
    if(frame_age>ROUND_CAMERA_TIMEOUT_TICKS) { fail(2); return; }
    if(phase_ticks<65535U) { phase_ticks++; }
    if(total_ticks<65535U) { total_ticks++; }
    if(phase_ticks>ROUND_PHASE_TIMEOUT_TICKS || total_ticks>ROUND_TOTAL_TIMEOUT_TICKS) { fail(3); }
}

static int black_support(int y,int x,int left)
{
    int k;
    for(k=1;k<=3;k++) { if(white(y,x+(left ? -k:k))) { return 0; } }
    return x>5 && x<MT9V03X_W-6;
}
static int opposite_stable(int side)
{
    int y,n=0,sx=0,sy=0,sxx=0,sxy=0;
    const volatile int *line=side>0 ? Right_Line:Left_Line;
    const int *lost=side>0 ? Right_Lost_Flag:Left_Lost_Flag;
    for(y=15;y<MT9V03X_H-2;y++) if(!lost[y])
    { n++;sx+=line[y];sy+=y;sxx+=y*y;sxy+=line[y]*y; }
    if(n<35 || n*sxx-sy*sy<=0) { return 0; }
    {
        float a=(float)(n*sxy-sx*sy)/(n*sxx-sy*sy),b=((float)sx-a*sy)/n;
        int good=0;
        if(a>0.8f || a< -0.8f) { return 0; }
        for(y=15;y<MT9V03X_H-2;y++) if(!lost[y])
        { float d=line[y]-(a*y+b); if(d<4 && d> -4) { good++; } }
        return good*100>=n*85;
    }
}
static void scan_sides(void);

static int three_corner_shape(int top,int middle,int bottom,int peak)
{
    int y,trend;
    if(bottom-top<ROUND_CORNER_MIN_HEIGHT || middle-top<ROUND_CORNER_MIN_GAP ||
       bottom-middle<ROUND_CORNER_MIN_GAP || peak-inner[top]<ROUND_CORNER_PROMINENCE ||
       peak-inner[bottom]<ROUND_CORNER_PROMINENCE) { return 0; }
    // A one-row spike or two separated peaks must not masquerade as the middle turn.
    for(y=middle-1;y<=middle+1;y++) if(inner[y]<peak-2) { return 0; }
    trend=inner[top];
    for(y=top;y<=middle;y++)
    {
        if(inner[y]<trend-2) { return 0; }
        if(inner[y]>trend) { trend=inner[y]; }
    }
    trend=inner[middle];
    for(y=middle;y<=bottom;y++)
    {
        if(inner[y]>trend+2) { return 0; }
        if(inner[y]<trend) { trend=inner[y]; }
    }
    return 1;
}

// Scan the road-facing contour, not the whole black component. In mirrored
// coordinates all candidates are left islands: lower -> inward peak -> upper.
// Both end corners and a genuine reversal are REQUIRED for first recognition.
static void observe_side(int side)
{
    int bottom,old_direction=direction;
    direction=side;scan_sides();
    for(bottom=MT9V03X_H-3;bottom>=5+ROUND_CORNER_MIN_HEIGHT;bottom--)
    {
        int top,y,peak=-1,peak_first=-1,peak_last=-1,score;
        int lower_corner,upper_corner,shape;
        if(!inner_ok[bottom]) { continue; }
        lower_corner=!inner_ok[bottom+1] || inner[bottom]-inner[bottom+1]>=ROUND_CORNER_JUMP;
        if(!lower_corner && !(active() && bottom==MT9V03X_H-3)) { continue; }
        top=bottom;
        while(top>5 && inner_ok[top-1] && abs_i(inner[top]-inner[top-1])<ROUND_CORNER_JUMP) { top--; }
        upper_corner=top>5 && (!inner_ok[top-1] || inner[top]-inner[top-1]>=ROUND_CORNER_JUMP);
        if(top==bottom || (!active() && (!upper_corner || bottom-top<ROUND_CORNER_MIN_HEIGHT))) { continue; }
        memset(&observed,0,sizeof(observed));
        observed.side=side;observed.x0=MT9V03X_W;observed.x1=0;
        observed.y0=top;observed.y1=bottom;
        for(y=top;y<=bottom;y++)
        {
            int left=inner[y]-1,right=left,x0,x1;
            if(inner[y]>peak) { peak=inner[y];peak_first=peak_last=y; }
            else if(inner[y]==peak) { peak_last=y; }
            // Horizontal black support supplies entry geometry only. It is not
            // a closed-island, component-area or flood-fill recognition gate.
            while(left>2 && !white(y,left-1)) { left--; }
            x0=px(left);x1=px(right);
            if(x0>x1) { int t=x0;x0=x1;x1=t; }
            if(x0<observed.x0) { observed.x0=x0; }
            if(x1>observed.x1) { observed.x1=x1; }
        }
        y=(peak_first+peak_last)/2;
        shape=three_corner_shape(top,y,bottom,peak);
        if((!active() || Island_State<=ROUND_WAIT_ENTRY || Island_State>=ROUND_ALIGN) &&
           (!shape || nx(near_center)-peak<12)) { continue; }
        observed.lower=(Point){px(inner[bottom]),bottom,(uint8)lower_corner};
        observed.middle=(Point){px(inner[y]),y,(uint8)shape};
        observed.upper=(Point){px(inner[top]),top,(uint8)upper_corner};
        if(active() && previous.valid &&
           (abs_i(observed.x1-previous.x1)>40 || abs_i(observed.x0-previous.x0)>40 ||
            abs_i(observed.y0-previous.y0)>30)) { continue; }
        score=bottom-top;
        if(active() && previous.valid)
        { score-=2*abs_i(observed.y0-previous.y0)+abs_i(observed.x1-previous.x1); }
        if(!best.valid || score>best_score)
        { best=observed;best.valid=1;best_score=score; }
    }
    direction=old_direction;
}
static int observe_island(void)
{
    best.valid=0;best_score=-100000;
    if(active()) { observe_side(direction); }
    else
    {
        if(opposite_stable(1)) { observe_side(1); }
        if(opposite_stable(-1)) { observe_side(-1); }
    }
    if(best.valid)
    {
        if(!active() && previous.valid &&
           (best.side!=previous.side ||
            abs_i(best.lower.x-previous.lower.x)>ROUND_CORNER_TRACK_X ||
            abs_i(best.middle.x-previous.middle.x)>ROUND_CORNER_TRACK_X ||
            abs_i(best.upper.x-previous.upper.x)>ROUND_CORNER_TRACK_X ||
            abs_i(best.middle.y-previous.middle.y)>ROUND_CORNER_TRACK_Y ||
            abs_i(best.lower.y-previous.lower.y)>ROUND_CORNER_TRACK_Y ||
            abs_i(best.upper.y-previous.upper.y)>ROUND_CORNER_TRACK_Y))
        { confirm=0; }
        previous=best;
    }
    return best.valid;
}

// Trace the white component connected to the near road, using the side's actual
// black/white transitions. This scan does not assume the old left/right label survives entry.
static void scan_sides(void)
{
    int y,center=nx(near_center),connected=1;
    for(y=MT9V03X_H-2;y>=2;y--)
    {
        int l,r,d,found=connected && white(y,center);
        for(d=1;connected && !found && d<=24;d++)
        {
            if(white(y,center-d)) { center-=d; found=1; }
            else if(white(y,center+d)) { center+=d; found=1; }
        }
        inner_ok[y]=outer_ok[y]=0; inner[y]=2;outer[y]=MT9V03X_W-3;
        if(!found) { connected=0;continue; }
        l=r=center;
        while(l>2 && white(y,l-1)) { l--; }
        while(r<MT9V03X_W-3 && white(y,r+1)) { r++; }
        inner[y]=l;outer[y]=r;
        inner_ok[y]=(uint8)black_support(y,l,1); outer_ok[y]=(uint8)black_support(y,r,0);
        if(r-l<width_history[y]*3/2 && r-l>=15) { center=(l+r)/2; }
    }
}
static Opening find_opening(void)
{
    Opening o; int near;
    memset(&o,0,sizeof(o));
    for(near=MT9V03X_H-5;near>=14;near--)
    {
        int row,start,supported=0,col;
        if(!outer_ok[near] || !outer_ok[near+2] || outer_ok[near-1]) { continue; }
        // A smooth curve reaching the image border has no interior corner lip.
        if(outer[near]>MT9V03X_W-20) { continue; }
        col=outer[near]+8;
        for(row=near-1;row>=5;row--)
        {
            if(!white(row,col) || !white(row,col+3)) { break; }
            if(outer_ok[row] && outer[row]<col+3) { break; }
            supported++;
        }
        if(supported<8) { continue; }
        start=near-supported;
        o.lower=(Point){outer[near],near,1};
        // An upper lip is optional, just as in the article's lower-corner exit.
        // A fallback goal is chosen INSIDE this frame's observed white arm.
        if(row>=6 && outer_ok[row] && outer_ok[row-2] && outer[row]<MT9V03X_W-20)
        { o.upper=(Point){outer[row],row,1}; }
        else
        { o.upper=(Point){outer[near],clip(start+2,5,near-6),0}; }
        o.rows=supported;o.valid=1;return o;
    }
    return o;
}
static void current_points(void)
{
    if(!best.valid) { return; }
    points[ROUND_POINT_LOWER]=best.lower;
    points[ROUND_POINT_MIDDLE]=best.middle;
    points[ROUND_POINT_UPPER]=best.upper;
}
static void write_pair(int y,int l,int r)
{
    if(direction>0)
    { Left_Line[y]=l;Right_Line[y]=r; }
    else { Left_Line[y]=MT9V03X_W-1-r;Right_Line[y]=MT9V03X_W-1-l; }
    Left_Lost_Flag[y]=Right_Lost_Flag[y]=0; filled[y]=3;
}
static void apply_actual_curve(void)
{
    int y;
    for(y=2;y<MT9V03X_H-1;y++)
    {
        if(direction>0)
        {
            Left_Line[y]=inner[y];Right_Line[y]=outer[y];
            Left_Lost_Flag[y]=!inner_ok[y];Right_Lost_Flag[y]=!outer_ok[y];
        }
        else
        {
            Left_Line[y]=MT9V03X_W-1-outer[y];Right_Line[y]=MT9V03X_W-1-inner[y];
            Left_Lost_Flag[y]=!outer_ok[y];Right_Lost_Flag[y]=!inner_ok[y];
        }
    }
}
static int approach_guide(void)
{
    int y,count=0;
    // Keep the intact outside edge before entry; do not steer straight across the island.
    for(y=2;y<MT9V03X_H-1;y++) if(outer_ok[y])
    {
        int r=outer[y],l=r-width_history[y],c=(l+r)/2;
        if(l>=3 && white(y,c) && white(y,c-3) && white(y,c+3))
        { write_pair(y,l,r);count++; }
    }
    return count>=25;
}
static void row_clearances(int y)
{
    int x;
    memset(clearances,0,sizeof(clearances));
    for(x=3;x<MT9V03X_W-3;)
    {
        int begin,end,k;
        if(!white(y,x)) { x++; continue; }
        begin=x; while(x<MT9V03X_W-3 && white(y,x)) { x++; } end=x-1;
        for(k=begin;k<=end;k++) { clearances[k]=(uint8)((k-begin<end-k) ? k-begin:end-k); }
    }
}
// Find a connected white guide to a CURRENT feature. Bounded lateral slope,
// clearance and persistent parents prevent drawing a diagonal across a black island.
// This replaces blindly accepting a straight line when the picture does not support it.
static int guide_to(int gy,int gx,int partial_entry)
{
    const int inf=100000000;
    int y,x,buf=0,start=nx(near_center),finish=-1,best_cost=inf;
    gy=clip(gy,5,MT9V03X_H-25);gx=clip(gx,12,MT9V03X_W-13);
    for(x=0;x<MT9V03X_W;x++) { costs[buf][x]=inf; }
    row_clearances(MT9V03X_H-2);
    for(x=3;x<MT9V03X_W-3;x++) if(clearances[x]>=9 && abs_i(x-start)<=12)
    { costs[buf][x]=abs_i(x-start)*8; }
    for(y=MT9V03X_H-3;y>=gy;y--)
    {
        int next=1-buf;
        int ideal=gx+(start-gx)*(y-gy)/(MT9V03X_H-2-gy);
        row_clearances(y);
        for(x=0;x<MT9V03X_W;x++)
        {
            int d,chosen=0,cost=inf;
            costs[next][x]=inf;
            if(clearances[x]<9) { continue; }
            for(d=-3;d<=3;d++) if(x+d>=0 && x+d<MT9V03X_W)
            {
                int c=costs[buf][x+d]+abs_i(d)*5;
                if(c<cost) { cost=c; chosen=d; }
            }
            if(cost>=inf) { continue; }
            costs[next][x]=cost+abs_i(x-ideal)/3+clip(width_history[y]/3-clearances[x],0,30)*3;
            parents[y][x]=(signed char)chosen;
        }
        buf=next;
    }
    for(x=clip(gx-7,3,MT9V03X_W-4);x<=clip(partial_entry ? start-10:gx+7,3,MT9V03X_W-4);x++)
    {
        int c=costs[buf][x]+abs_i(x-gx)*20;
        if(c<best_cost) { best_cost=c;finish=x; }
    }
    if(finish<0) { return 0; }
    route[gy]=finish;
    for(y=gy;y<MT9V03X_H-2;y++) { route[y+1]=route[y]+parents[y][route[y]]; }
    for(y=gy;y<MT9V03X_H-1;y++)
    {
        int half;
        row_clearances(y);half=clip(width_history[y]/2,8,clearances[route[y]]);
        if(half<8 || !white(y,route[y])) { return 0; }
    }
    for(y=2;y<gy;y++) { Left_Lost_Flag[y]=Right_Lost_Flag[y]=1; }
    guide_goal=finish;
    for(y=gy;y<MT9V03X_H-1;y++)
    {
        int half;
        row_clearances(y);half=clip(width_history[y]/2,8,clearances[route[y]]);
        write_pair(y,route[y]-half,route[y]+half);
    }
    // Last row may have a different white shape, so validate it independently.
    y=MT9V03X_H-1; row_clearances(y);
    if(clearances[route[y-1]]>=9)
    { int half=clip(width_history[y]/2,8,clearances[route[y-1]]);write_pair(y,route[y-1]-half,route[y-1]+half); }
    return 1;
}
static int find_corridor(int *gy,int *gx)
{
    int y,x,score=-100000;
    for(y=MT9V03X_H/3;y<=MT9V03X_H/2;y+=4)
    {
        row_clearances(y);
        for(x=12;x<MT9V03X_W-12;x++) if(clearances[x]>=10 && white(y+3,x) && white(y-3,x))
        {
            int l=x,r=x;
            while(l>2 && white(y,l-1)) { l--; }
            while(r<MT9V03X_W-3 && white(y,r+1)) { r++; }
            if(!black_support(y,l,1) && !black_support(y,r,0)) { continue; }
            int s=4*clearances[x]-2*abs_i(x-guide_goal);
            if(s>score) { score=s;*gy=y;*gx=x; }
        }
    }
    return score> -100000;
}
static int near_straight(void)
{
    int y,count=0,reference=-1;
    for(y=MT9V03X_H-4;y>=MT9V03X_H/2;y--)
    {
        int c=(inner[y]+outer[y])/2;
        if(!inner_ok[y] || !outer_ok[y] || outer[y]-inner[y]<15) { return 0; }
        if(reference<0) { reference=c; }
        if(abs_i(c-reference)>12) { return 0; }
        count++;
    }
    return count>=35;
}
static int real_inside_curve(void)
{
    int y,near=-1,far=-1,count=0;
    for(y=MT9V03X_H-3;y>=MT9V03X_H*3/5;y--) if(outer_ok[y])
    {
        count++;
        if(near<0) { near=y; }
        if(y<=MT9V03X_H*2/3 && far<0) { far=y; }
    }
    if(count<20 || near<MT9V03X_H*3/4 || far<0) { return 0; }
    // Remove the ordinary perspective taper; require actual curvature into the island.
    return outer[near]-outer[far]>(width_history[near]-width_history[far])/2+8;
}
static void finish(void)
{
    if(fault) { return; }
    Island_State=Left_Island_Flag=Right_Island_Flag=0;
    direction=0;confirm=phase_confirm=missing=exit_armed=outer_stable=recovered=0;
    entry_guided=exit_guided=0;
    cooldown=ROUND_COOLDOWN_FRAMES;total_ticks=phase_ticks=0;previous.valid=0;
    clear_overlay();
}

void Roundabout_Process(uint8 moving)
{
    int found,y,near_l=-1,near_r=-1,usable=0,progress=0;
    Opening opening;
    if(reset_pending) { Roundabout_Init(); }
    frame_age=0;clear_overlay();
    if(fault) { return; }
    // Near-road seed must be observed this frame whenever possible.
    for(y=MT9V03X_H-2;y>=MT9V03X_H*3/4;y--)
    {
        if(!Left_Lost_Flag[y] && !Right_Lost_Flag[y] && Right_Line[y]-Left_Line[y]>=15)
        { near_l=Left_Line[y];near_r=Right_Line[y];near_center=(near_l+near_r)/2;break; }
    }
    if(!active() && (Cross_Flag || Cross_GetState()!=CROSS_STATE_NORMAL))
    { confirm=0;candidate_side=0;return; }
    if(cooldown)
    { if(moving) { cooldown--; } return; }
    found=observe_island();
    if(!active())
    {
        if(found)
        {
            if(candidate_side!=best.side) { confirm=0; }
            candidate_side=best.side;
            if(confirm<ROUND_CONFIRM_FRAMES) { confirm++; }
            if(confirm>=ROUND_CONFIRM_FRAMES)
            {
                direction=best.side;
                entry_guided=exit_guided=0;guide_goal=nx(near_center);
                Left_Island_Flag=(direction>0);Right_Island_Flag=(direction<0);
                total_ticks=0;set_stage(best.y1>=MT9V03X_H*3/4 ? ROUND_WAIT_ENTRY:ROUND_APPROACH);
            }
        }
        else
        {
            confirm=0;candidate_side=0;previous.valid=0;
            for(y=2;y<MT9V03X_H;y++) if(!Left_Lost_Flag[y] && !Right_Lost_Flag[y])
            {
                int w=Right_Line[y]-Left_Line[y];
                if(w>=15 && w<=width_history[y]*3/2 && w>=width_history[y]*2/3)
                { width_history[y]=(7*width_history[y]+w)/8; }
            }
        }
        if(!active()) { current_points();return; }
    }
    scan_sides();current_points();opening=find_opening();
    if(opening.valid) { points[ROUND_POINT_EXIT]=(Point){px(opening.lower.x),opening.lower.y,1}; }
    switch(Island_State)
    {
        case ROUND_APPROACH:
            usable=found && approach_guide();
            progress=found && best.y1>=MT9V03X_H*3/4;
            if(moving && progress && ++phase_confirm>=ROUND_PHASE_CONFIRM) { set_stage(ROUND_WAIT_ENTRY); }
            else if(!progress) { phase_confirm=0; }
            break;
        case ROUND_WAIT_ENTRY:
            usable=found && approach_guide();
            progress=found && best.y1>=MT9V03X_H-7 && best.y0>=MT9V03X_H/3;
            if(moving && progress && ++phase_confirm>=ROUND_PHASE_CONFIRM) { set_stage(ROUND_ENTER); }
            else if(!progress) { phase_confirm=0; }
            break;
        case ROUND_ENTER:
            if(found)
            {
                int normalized_top=best.side>0 ? best.x0:MT9V03X_W-1-best.x1;
                int goal=(best.x0+best.x1)/2;
                goal=nx(goal)-width_history[best.y0]/4;
                usable=guide_to(best.y0-5,clip(goal,normalized_top-12,MT9V03X_W-13),1);
                if(usable && entry_guided<ROUND_PHASE_CONFIRM) { entry_guided++; }
            }
            if(!usable)
            {
                int gy=0,gx=0;
                // The old island edge may be re-labelled or leave the frame during entry.
                // Re-scan current white corridors; never keep a disappeared upper corner.
                usable=find_corridor(&gy,&gx) && guide_to(gy,gx,1);
                if(usable && entry_guided<ROUND_PHASE_CONFIRM) { entry_guided++; }
            }
            // Entry must give way to a real, continuous inner/outer curve before exit can arm.
            {
                progress=entry_guided>=2 && real_inside_curve() && !opening.valid;
            }
            if(moving && progress && ++phase_confirm>=ROUND_PHASE_CONFIRM)
            { set_stage(ROUND_INSIDE);outer_stable=exit_armed=0;usable=1; }
            else if(!progress) { phase_confirm=0; }
            // If the island left the frame, trust actual curve edges, never a stale anchor.
            if(progress) { if(!usable) { apply_actual_curve(); } usable=1; }
            break;
        case ROUND_INSIDE:
            {
                int count=0,outer_count=0;
                for(y=2;y<MT9V03X_H-3;y++)
                { count+=inner_ok[y] || outer_ok[y];outer_count+=outer_ok[y]; }
                usable=count>=20;
                if(usable) { apply_actual_curve(); }
                if(!opening.valid && usable && outer_count>=20 && outer_ok[MT9V03X_H-5])
                { if(moving && outer_stable<ROUND_EXIT_ARM_FRAMES) { outer_stable++; } }
                else if(!exit_armed) { outer_stable=0; }
                if(outer_stable>=ROUND_EXIT_ARM_FRAMES) { exit_armed=1; }
                progress=exit_armed && opening.valid;
                if(moving && progress && ++phase_confirm>=ROUND_PHASE_CONFIRM) { set_stage(ROUND_EXIT);exit_guided=0; }
                else if(!progress) { phase_confirm=0; }
            }
            break;
        case ROUND_EXIT:
            if(opening.valid)
            {
                int gy=opening.upper.valid ? opening.upper.y+2:opening.upper.y;
                int gx=opening.upper.x-width_history[gy]/2;
                usable=guide_to(gy,gx,0);
                if(usable) { exit_guided=1; }
            }
            progress=!opening.valid && exit_guided;
            if(moving && progress && ++phase_confirm>=ROUND_PHASE_CONFIRM) { set_stage(ROUND_ALIGN); }
            else if(!progress) { phase_confirm=0; }
            break;
        case ROUND_ALIGN:
        case ROUND_RECOVER:
            {
                int gy=0,gx=0;
                usable=find_corridor(&gy,&gx) && guide_to(gy,gx,0);
                progress=near_straight() && !opening.valid && !found;
                if(moving && progress)
                {
                    if(Island_State==ROUND_ALIGN)
                    { if(++phase_confirm>=ROUND_PHASE_CONFIRM) { set_stage(ROUND_RECOVER);recovered=0; } }
                    else if(++recovered>=ROUND_RECOVER_FRAMES) { finish();return; }
                }
                else { phase_confirm=recovered=0; }
            }
            break;
        default: break;
    }
    if(usable) { missing=0; }
    else if(moving && ++missing>=ROUND_MAX_MISSING_FRAMES) { fail(1);clear_overlay(); }
}

uint8 Roundabout_GetFillMask(int row)
{ return row>=0 && row<MT9V03X_H ? filled[row]:0; }
int Roundabout_GetRoadWidth(int row)
{ return row>=0 && row<MT9V03X_H ? width_history[row]:0; }
uint8 Roundabout_GetPoint(int which,int *x,int *y)
{
    if(which<0 || which>=ROUND_POINT_COUNT || !x || !y || !points[which].valid) { return 0; }
    *x=points[which].x;*y=points[which].y;return 1;
}
