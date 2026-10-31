"""Exercise the active Keil program's physical keys and camera stop latch."""
from pathlib import Path
import re, subprocess, tempfile
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
mdk=ROOT/'project/mdk'
project=ET.parse(mdk/'rt1064.uvprojx')
main_path=project.find(".//Files/File[FileName='main.c']/FilePath").text
source=(mdk/main_path.replace('\\','/')).resolve().read_text(encoding='utf-8')
handlers=source.split('// BEGIN RUN KEY CONTROL\n',1)[1].split('// END RUN KEY CONTROL',1)[0]
names=('SERVO_P_DEFAULT','SERVO_D_DEFAULT','SERVO_P_KEY_STEP','SERVO_D_KEY_STEP',
       'SERVO_P_MIN','SERVO_P_MAX','SERVO_D_MIN','SERVO_D_MAX',
       'SPEED_DEFAULT','SPEED_KEY_STEP','SPEED_KEY_MIN','SPEED_KEY_MAX',
       'RUN_KEY_DEBOUNCE_SCANS','RUN_AUTO_START','USE_ISLAND','USE_RUN_KEYS',
       'USE_REMOTE_CONTROL','USE_ASSISTANT')
config='\n'.join(re.search(r'^#define\s+'+name+r'\s+.*$',source,re.M).group(0) for name in names)
PREFIX=r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8;
typedef int gpio_pin_enum;
enum {C15,C14,C13,C12};
#define GPIO_LOW 0
static int levels[4]={1,1,1,1},stop_calls,center_calls,ring_fault,retry_calls;
static volatile float run_p=SERVO_P_DEFAULT,run_d=SERVO_D_DEFAULT,
    run_speed=SPEED_DEFAULT,run_switch,run_enable=1;
static volatile uint8 run_auto_pending=RUN_AUTO_START;
static volatile int dbg_duty=100;
void key_init(int period) { assert(period==10); }
int gpio_get_level(int pin) { assert(pin>=0&&pin<4);return levels[pin]; }
void MotorPID_SetEnable(uint8 e) { assert(!e);stop_calls++; }
void Servo_Steer(int a) { assert(!a);center_calls++; }
uint8 Roundabout_GetFault(void) { return ring_fault; }
void Roundabout_Retry(void) { retry_calls++;ring_fault=0; }
"""
TEST=r"""
static void ticks(int n) { while(n--)Run_Keys10ms(); }
static void release(void) { for(int i=0;i<4;i++)levels[i]=1;ticks(3); }
static void tap(int key) { levels[key]=0;ticks(3);release(); }
static void near(float a,float b) { assert(fabsf(a-b)<.0001f); }
int main(void) {
    char p[5],d[6];
    assert(SPEED_DEFAULT==180&&SPEED_KEY_STEP==5);
    assert(!RUN_AUTO_START&&USE_RUN_KEYS&&!USE_REMOTE_CONTROL&&!USE_ASSISTANT);
    assert(SERVO_P_KEY_STEP==.02f&&SERVO_D_KEY_STEP==.02f);
    Run_KeysInit();ticks(30);assert(!run_switch&&run_tune_item==0);
    tap(C13);assert(!run_switch&&run_key_fault==1); // No usable image: cannot start.
    Run_KeyFrame(1);ticks(50);assert(!run_switch); // Image recovery cannot auto-start.
    run_enable=0;tap(C13);assert(!run_switch);run_enable=1;
    tap(C14);near(run_speed,185);near(run_p,1.8f);near(run_d,2);
    levels[C15]=0;ticks(3);near(run_speed,180);ticks(300);near(run_speed,180);release();
    tap(C12);assert(run_tune_item==1);
    tap(C15);near(run_p,1.78f);near(run_speed,180);tap(C14);near(run_p,1.8f);
    tap(C12);assert(run_tune_item==2);tap(C14);near(run_d,2.02f);
    tap(C15);near(run_d,2);near(run_p,1.8f);
    tap(C12);assert(run_tune_item==0);
    for(int i=0;i<300;i++)tap(C15);near(run_speed,70);
    for(int i=0;i<300;i++)tap(C14);near(run_speed,240);
    tap(C12);
    for(int i=0;i<300;i++)tap(C15);near(run_p,0);
    for(int i=0;i<300;i++)tap(C14);near(run_p,5);
    Run_FormatP(run_p,p);assert(!strcmp(p,"5.00"));
    tap(C12);
    for(int i=0;i<2600;i++)tap(C14);near(run_d,50);
    Run_FormatD(run_d,d);assert(!strcmp(d,"50.00"));
    for(int i=0;i<2600;i++)tap(C15);near(run_d,0);
    run_p=1.8f;run_d=2;run_speed=180;tap(C12);
    // Noise shorter than 30ms and opposing adjustment buttons have no effect.
    for(int key=0;key<4;key++) {
        for(int i=0;i<20;i++){levels[key]=0;ticks(2);levels[key]=1;ticks(1);}
        release();
    }
    assert(!run_switch&&run_tune_item==0);near(run_speed,180);
    levels[C15]=levels[C14]=0;ticks(3);release();near(run_speed,180);
    levels[C12]=levels[C14]=0;ticks(3);release();
    assert(run_tune_item==1);near(run_p,1.8f);near(run_speed,180);
    // Start/stop takes priority over every parameter key.
    Run_KeyFrame(1);
    for(int i=0;i<4;i++)levels[i]=0;ticks(3);
    assert(run_switch&&run_tune_item==1&&run_key_fault==0);near(run_p,1.8f);
    ticks(300);assert(run_switch);release();
    tap(C13);assert(!run_switch&&!run_key_fault&&!dbg_duty);
    // During motion, selected-only tuning still works and C13 stops immediately.
    Run_KeyFrame(1);tap(C13);assert(run_switch);tap(C14);near(run_p,1.82f);
    // Image loss and completely missing frames both stop in 500ms.
    for(int i=0;i<24;i++) {Run_KeyFrame(0);Run_KeyTick20ms();}
    assert(run_switch);Run_KeyTick20ms();
    assert(!run_switch&&run_key_fault==1&&!run_auto_pending);
    for(int i=0;i<100;i++){Run_KeyFrame(1);Run_KeyTick20ms();}
    assert(!run_switch&&run_key_fault==1);
    tap(C13);assert(run_switch&&!run_key_fault);
    for(int i=0;i<24;i++){Run_KeyFrame(1);Run_KeyTick20ms();}assert(run_switch);
    tap(C13);assert(!run_switch);
    ring_fault=1;Run_KeyFrame(1);tap(C13);assert(run_switch&&retry_calls==1);
    tap(C13);
    // Held-at-boot keys require release/new press; no accidental start/parameter changes.
    for(int i=0;i<4;i++)levels[i]=0;
    Run_KeysInit();Run_KeyFrame(1);ticks(300);
    assert(!run_switch&&run_tune_item==0);near(run_speed,180);near(run_p,1.82f);
    release();tap(C13);assert(run_switch);tap(C13);assert(!run_switch);
    Run_FormatP(1.82f,p);Run_FormatD(2.02f,d);
    assert(!strcmp(p,"1.82")&&!strcmp(d," 2.02"));
    assert(stop_calls>0&&center_calls==stop_calls);
    puts("PASS: active keys SPD +/-5, P/D +/-0.02, independent ranges, debounce/held-key handling, C13 priority, image start gate and latched camera stop, no wireless/autostart");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix='rt1064_keys_R3_') as folder:
    f=Path(folder); c=f/'keys.c';c.write_text(config+'\n'+PREFIX+handlers+TEST,encoding='utf-8')
    exe=f/'keys.exe'
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
