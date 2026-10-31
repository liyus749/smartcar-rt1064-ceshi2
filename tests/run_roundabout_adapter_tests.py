"""Link the active ring adapter, VIS7, LineDetect, Cross and Err together."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT_DIR = ROOT / 'project/mdk'
project = ET.parse(PROJECT_DIR / 'rt1064.uvprojx')
main = (PROJECT_DIR / project.find(".//Files/File[FileName='main.c']/FilePath").text.replace('\\','/')).resolve()
CODE = main.parents[2] / 'code'
STUB = '''
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
#include <stddef.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int32_t int32;
#define MT9V03X_H 100
#define MT9V03X_W 188
#define IMG_BLACK 0
#define IMG_WHITE 255
uint32 __get_PRIMASK(void); void __disable_irq(void); void __set_PRIMASK(uint32);
void Draw_Line(int,int,int,int);
#endif
'''
TEST = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Roundabout.h"
#include "island.h"
#include "LineDetect.h"
#include "Cross.h"
#include "Err.h"
uint8 image_two_value[100][188], gray[100][188];
volatile int Left_Line[100],Right_Line[100],Mid_Line[100],Road_Wide[100];
int Left_Lost_Flag[100],Right_Lost_Flag[100];
volatile int Search_Stop_Line,Left_Lost_Time,Right_Lost_Time,Both_Lost_Time;
volatile int Boundry_Start_Left,Boundry_Start_Right,Cross_Flag;
volatile uint8 Img_Disappear_Flag;
static uint32 irq;
uint32 __get_PRIMASK(void) { return irq; }
void __disable_irq(void) { irq=1; }
void __set_PRIMASK(uint32 m) { irq=m; }
void Draw_Line(int a,int b,int c,int d) { (void)a;(void)b;(void)c;(void)d; }
void Contrast_Process(const uint8 *p) { (void)p;abort(); }
uint8 Contrast_GetReferenceCol(void) { abort(); }
static void frame(void) {
    for(int y=0;y<100;y++) for(int x=0;x<188;x++) gray[y][x]=(x>=40 && x<=145)?220:20;
    LineDetect_Process(gray[0],LINE_MODE_OTSU); Cross_Check();
    Roundabout_Process(0);LineDetect_RefreshStats();assert(Err_Compute());
}
int main(void) {
    int x,y;
    Roundabout_Init();LineDetect_Init();Cross_Init();Err_Init();
    assert(!Roundabout_GetFault() && !Roundabout_ResetPending() && !Roundabout_GetDirection());
    assert(!Roundabout_GetPoint(ROUND_POINT_UPPER,&x,&y));
    frame();assert(!Island_State && ErrValidRows>80 && !irq);
    assert(Roundabout_GetRoadWidth(80)>90 && Roundabout_GetRoadWidth(80)<115);
    assert(!Roundabout_GetFillMask(-1) && !Roundabout_GetFillMask(100));
    island_patch_left[70]=island_patch_right[70]=1;assert(Roundabout_GetFillMask(70)==3);
    island_debug.dir=1;Island_State=ROUND_ENTER;
    assert(!Roundabout_AllowDifferential());
    for(int i=0;i<1000;i++)Roundabout_Tick20ms(0);assert(!Roundabout_GetFault());
    /* New frame resets stale age; no image for 500ms while moving latches fault. */
    frame();for(unsigned i=0;i<ROUND_CAMERA_TIMEOUT_TICKS;i++)Roundabout_Tick20ms(1);
    assert(Roundabout_GetFault()==2 && Island_State==ROUND_ENTER && !island_debug.exits);
    frame();assert(Roundabout_GetFault()==2); /* healthy image cannot auto-clear fault */
    Roundabout_Retry();assert(Roundabout_ResetPending() && !Roundabout_AllowDifferential());
    frame();assert(!Roundabout_ResetPending() && !Roundabout_GetFault() && !Island_State && !irq);
    assert(Roundabout_AllowDifferential());
    Island_State=ROUND_INSIDE;assert(Roundabout_AllowDifferential());
    island_debug.dir=-1;assert(Roundabout_GetDirection()==-1);
    island_debug.right.up.x=100;island_debug.right.up.y=30;
    assert(Roundabout_GetPoint(ROUND_POINT_UPPER,&x,&y) && x==100 && y==30);
    island_debug.right.up.y=-1;assert(!Roundabout_GetPoint(ROUND_POINT_UPPER,&x,&y));
    assert(!Roundabout_GetPoint(20,&x,&y));
    puts("PASS: active VIS7 adapter + actual LineDetect/Cross/Err, width/provenance, fault latch, frame-owned retry, differential and point ABI");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='vis7_adapter_') as folder:
    build = Path(folder)
    (build/'zf_common_headfile.h').write_text(STUB)
    (build/'check.c').write_text(TEST)
    exe = build/'check.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
        '-I',str(build),'-I',str(CODE),str(build/'check.c'),
        *[str(CODE/name) for name in ('island.c','Roundabout.c','LineDetect.c','Cross.c','Err.c')],
        '-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
