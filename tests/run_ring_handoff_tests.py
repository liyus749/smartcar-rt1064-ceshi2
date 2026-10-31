"""Verify the ring adapter's contract with unchanged electrical Err.c.

Uses an artificial producer only for the final-frame protocol: a completed ring
can still emit a synthetic line in that image. It must not be learned as road width.
Full image geometry is covered separately by run_island_vision_tests.py.
"""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
project_dir=ROOT/'project/mdk'
main=(project_dir/ET.parse(project_dir/'rt1064.uvprojx').find(".//Files/File[FileName='main.c']/FilePath").text.replace('\\','/')).resolve()
code=main.parents[2]/'code'
STUB='''#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
typedef uint8_t uint8;typedef uint16_t uint16;typedef uint32_t uint32;
#define MT9V03X_H 100
#define MT9V03X_W 188
uint32 __get_PRIMASK(void);void __disable_irq(void);void __set_PRIMASK(uint32);
#endif
'''
TEST=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "island.h"
#include "Roundabout.h"
volatile int Island_State,Left_Island_Flag,Right_Island_Flag,island_fault,island_arc_enable;
volatile int Left_Line[100],Right_Line[100],Cross_Flag;
int Left_Lost_Flag[100],Right_Lost_Flag[100];
uint8 island_patch_left[100],island_patch_right[100];
IslandDebug island_debug;
static uint32 irq;
static int mode,detector_calls,resets,error_resets;
uint32 __get_PRIMASK(void) { return irq; }
void __disable_irq(void) { irq=1; }
void __set_PRIMASK(uint32 m) { irq=m; }
void Err_Init(void) { error_resets++; }
void Island_Reset(void) {
    resets++; Island_State=island_fault=0;
    memset(island_patch_left,0,sizeof(island_patch_left));
    memset(island_patch_right,0,sizeof(island_patch_right));
}
void Island_SetMoving(uint8 m) { (void)m; }
void Island_Detect(void) {
    detector_calls++;
    if(mode==1) { /* visual success with a synthetic guide in its final frame */
        Island_State=0;island_patch_left[90]=1;
    }
    else assert(!Island_State);
}
int main(void) {
    Roundabout_Init();
    mode=1;Island_State=7;island_debug.dir=1;Roundabout_Process(1);
    assert(Island_State==7 && Roundabout_GetFillMask(90)==1 && !irq);
    /* Existing Err.c's !Island_State learning guard rejects this final guide. */
    mode=0;Roundabout_Process(1);
    assert(Island_State==0 && !Roundabout_GetFillMask(90) && detector_calls==2);
    /* A fault cannot be hidden by a pending successful handoff. */
    mode=1;Island_State=7;Roundabout_Process(1);island_fault=2;
    mode=0;Roundabout_Process(0);assert(Island_State==7 && island_fault==2);
    Roundabout_Retry();Roundabout_Process(0);
    assert(!Island_State && !island_fault && !Roundabout_ResetPending());
    assert(resets==2 && error_resets==1 && !irq);
    puts("PASS: synthetic final frame stays RECOVER; next clean frame resumes NORMAL; fault latch and frame-owned retry preserved");
}
'''
with tempfile.TemporaryDirectory(prefix='ring_handoff_') as folder:
    build=Path(folder)
    (build/'zf_common_headfile.h').write_text(STUB)
    (build/'check.c').write_text(TEST)
    exe=build/'check.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(build),'-I',str(code),str(build/'check.c'),str(code/'Roundabout.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
