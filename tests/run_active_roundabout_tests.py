"""Regress the code actually selected by rt1064.uvprojx, using synthetic camera frames.

Scenes approximate entrance geometry; they are not recordings from the real car.
The element call order is read from the active main.c, rather than assumed here.
"""
from pathlib import Path
import ctypes
import math
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MDK = ROOT / 'project/mdk'
PROJECT = ET.parse(MDK / 'rt1064.uvprojx')

def active_file(name):
    node = PROJECT.find(f".//Files/File[FileName='{name}']/FilePath")
    if node is None:
        raise RuntimeError(f'{name} is not selected by the active project')
    return (MDK / node.text.replace('\\', '/')).resolve()

main_source = active_file('main.c').read_text(encoding='utf-8')
frame_body = main_source.split('static void Frame_Process(const uint8 *gray)', 1)[1].split('LineDetect_RefreshStats();', 1)[0]
element_order = re.findall(r'\b(Cross_Check|Roundabout_Process)\(', frame_body)
assert sorted(element_order) == ['Cross_Check', 'Roundabout_Process'], element_order
pipeline = ''.join(name + ('((uint8)moving);' if name == 'Roundabout_Process' else '();') for name in element_order)
STUB = '''#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
#include <stddef.h>
typedef uint8_t uint8;typedef uint16_t uint16;typedef uint32_t uint32;typedef int32_t int32;
#define MT9V03X_H 100
#define MT9V03X_W 188
#define IMG_WHITE 255
#define IMG_BLACK 0
#endif
'''
SHIM = '''#include <stdlib.h>
#include "LineDetect.h"
#include "Cross.h"
#include "Roundabout.h"
#include "Err.h"
uint8 image_two_value[100][188];
volatile int Left_Line[100],Right_Line[100],Mid_Line[100],Road_Wide[100];
int Left_Lost_Flag[100],Right_Lost_Flag[100];
volatile int Search_Stop_Line,Left_Lost_Time,Right_Lost_Time,Both_Lost_Time;
volatile int Boundry_Start_Left,Boundry_Start_Right,Island_State,Cross_Flag,Left_Island_Flag,Right_Island_Flag;
volatile uint8 Img_Disappear_Flag;
void Contrast_Process(const uint8 *p) { (void)p;abort(); }
uint8 Contrast_GetReferenceCol(void) { abort(); }
static int invariant_ok;
static unsigned checksum(void) {
 unsigned h=2166136261U;
 for(int y=0;y<100;y++)for(int x=0;x<188;x++)h=(h^image_two_value[y][x])*16777619U;
 return h;
}
void test_reset(void) { LineDetect_Init();Cross_Init();Roundabout_Init();Err_Init();invariant_ok=1; }
void test_step(const uint8 *gray,int moving) {
 LineDetect_Process(gray,LINE_MODE_OTSU);
 unsigned before=checksum();
 ELEMENT_PIPELINE
 LineDetect_RefreshStats();Err_Compute();
 if(before!=checksum() || (Island_State&&Cross_Flag))invariant_ok=0;
 for(int y=2;y<100;y++)if(Roundabout_GetFillMask(y)) {
  int l=Left_Line[y],r=Right_Line[y];
  if(l<3 || r>=185 || r-l<15 || image_two_value[y][(l+r)/2]!=IMG_WHITE)invariant_ok=0;
 }
 Roundabout_Tick20ms((uint8)moving);Roundabout_Tick20ms((uint8)moving);
}
int test_value(int which) {
 switch(which) {
 case 0:return Island_State;
 case 1:return Cross_Flag;
 case 2:return Roundabout_GetDirection();
 case 3:return Roundabout_GetFault();
 case 4:return Cross_GetState();
 default:return invariant_ok;
 }
}
'''.replace('ELEMENT_PIPELINE', pipeline)
checks = 0

def check(ok, why):
    global checks
    checks += 1
    if not ok:
        raise AssertionError(f'{why}: state={value(0)}, cross={value(1)}, dir={value(2)}, fault={value(3)}')

def road(bend=0, cross=False, slant=0):
    return bytearray(220 if abs(x-(94+bend*(99-y)//99)) <= 35+y//7 or
                     cross and 35+slant*x//188 <= y <= 65+slant*x//188 else 20
                     for y in range(100) for x in range(188))

def mirror_frame(frame, mirror):
    if not mirror:
        return frame
    return bytearray(v for y in range(100) for v in frame[y*188:(y+1)*188][::-1])

def ellipse(bottom=65, mirror=False):
    frame = road()
    cy = bottom-17
    for y in range(100):
        for x in range(188):
            if (x-45)**2/900+(y-cy)**2/1225 < 1:
                frame[y*188+x] = 220
            if (x-45)**2/196+(y-cy)**2/289 < 1:
                frame[y*188+x] = 20
    return mirror_frame(frame, mirror)

def smooth_lip(mirror=False):
    frame = bytearray([20])*18800
    for y in range(100):
        left = 59-y//7
        if 27 <= y <= 73:
            if y < 32:
                left = round(56+(y-27)*2.2)
            elif y <= 68:
                left = round(66+7*math.sin(math.pi*(y-32)/36))
            else:
                left = round(66-(y-68)*3.0)
        for x in range(left, 130+y//7):
            frame[y*188+x] = 220
    return mirror_frame(frame, mirror)

def step(frame, count=1, moving=1):
    data = (ctypes.c_ubyte*18800).from_buffer_copy(frame)
    for _ in range(count):
        lib.test_step(data, moving)
        check(value(5), 'raw pixels preserved; elements exclusive; guide centers stay on white')

with tempfile.TemporaryDirectory(prefix='rt1064_active_ring_') as temp:
    build = Path(temp)
    (build/'zf_common_headfile.h').write_text(STUB, encoding='utf-8')
    (build/'pipeline.c').write_text(SHIM, encoding='utf-8')
    compiler = shutil.which('gcc')
    if not compiler:
        raise RuntimeError('gcc is required for the host camera regression')
    library = build / ('pipeline.dll' if __import__('os').name == 'nt' else 'pipeline.so')
    sources = [active_file(name) for name in ('LineDetect.c', 'Cross.c', 'Roundabout.c', 'Err.c')]
    includes = sorted({str(p.parent) for p in sources})
    subprocess.run([compiler, '-shared', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I', str(build), *[arg for inc in includes for arg in ('-I',inc)],
                    str(build/'pipeline.c'), *map(str,sources), '-o', str(library), '-lm'], check=True)
    lib = ctypes.CDLL(str(library))
    lib.test_step.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
    value = lib.test_value
    try:
        for mirror in (False, True):
            lib.test_reset()
            step(road(cross=True), 3)
            check(value(1) and not value(0), 'real cross must first be recognized')
            step(ellipse(65,mirror), 3)
            check(value(0)==1 and not value(1), 'three complete ring frames supersede a residual cross')
            check(value(2)==(-1 if mirror else 1), 'handoff preserves physical left/right direction')

            lib.test_reset()
            step(smooth_lip(mirror), 2)
            check(not value(0), 'smooth corners still require three consecutive camera frames')
            step(smooth_lip(mirror))
            check(value(0)==1 and value(2)==(-1 if mirror else 1), 'multirow real lips confirm a ring')
            step(smooth_lip(mirror), 12, moving=0)
            check(value(0)==1 and not value(3), 'parked approach cannot invent a lower point at the image border')

            lib.test_reset()
            step(ellipse(65,mirror))
            step(road())
            step(ellipse(65,mirror), 2)
            check(not value(0), 'interrupted observations do not accumulate confirmation')
            step(ellipse(65,mirror))
            check(value(0)==1, 'stable complete entrance confirms after interruption')
            for bottom in (78,80,82,85,88,91,94):
                step(ellipse(bottom,mirror))
            check(value(0)==3 and not value(3), 'measured lower corner advances into entry before leaving the frame')

            lib.test_reset()
            step(ellipse(65,mirror),3,moving=0)
            step(ellipse(65,mirror),30,moving=0)
            check(value(0)==1 and not value(3), 'stopped camera neither advances phases nor consumes motion timers')
            for _ in range(26):
                lib.Roundabout_Tick20ms(1)
            check(value(3)==2, 'stale camera still stops a confirmed ring')

            for color in (20,220):
                lib.test_reset();step(ellipse(65,mirror),3,moving=0)
                step(bytearray([color])*18800,8)
                check(value(3)==1 and value(0)==8, 'eight invalid ring frames still latch the path-loss stop')
                step(road(),3,moving=0)
                check(value(3)==1, 'new visible road cannot automatically clear a stop')

        for bend in range(-80,81,10):
            lib.test_reset();step(road(bend=bend),8)
            check(not value(0), 'ordinary left/right bends are not rings')
        for slant in (-12,0,12):
            lib.test_reset();step(road(cross=True,slant=slant),8)
            check(value(1) and not value(0), 'normal and oblique true crosses remain crosses')
        for mirror in (False,True):
            frame=road()
            for y in range(25,70):
                for x in range(40):frame[y*188+x]=220
            lib.test_reset();step(mirror_frame(frame,mirror),8)
            check(not value(0), 'one-sided T branch without inward peak is not a ring')
            frame=road()
            for y in range(40,44):
                for x in range(65,69):frame[y*188+x]=20
            lib.test_reset();step(mirror_frame(frame,mirror),8)
            check(not value(0), 'isolated dark speckles are not rings')
        for color in (20,220):
            lib.test_reset();step(bytearray([color])*18800,10)
            check(not value(0), 'featureless black/white frames do not confirm rings')
        print(f'PASS: {checks} assertions; active ring/cross handoff, smooth lips, measured entry, mirror direction, negative scenes and stop protections')
        print('Active sources:', ', '.join(str(p.relative_to(ROOT)) for p in sources))
        print('Active pipeline:', ' -> '.join(element_order))
    finally:
        if __import__('os').name=='nt':
            import _ctypes
            _ctypes.FreeLibrary(lib._handle)
