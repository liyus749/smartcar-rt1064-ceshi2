"""Run the actual tuning parser/controller and motor PID against simulated IO."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int32_t int32; typedef int16_t int16;
typedef enum { C15,C14,C13,C12 } gpio_pin_enum;
#define GPIO_LOW 0
uint32 __get_PRIMASK(void); void __disable_irq(void); void __set_PRIMASK(uint32);
int gpio_get_level(gpio_pin_enum); void key_init(int);
void ips200_show_string(uint16,uint16,const char*);
void ips200_show_int(uint16,uint16,int,int);
void ips200_show_float(uint16,uint16,float,int,int);
#endif
'''
USB = '''#include "zf_common_headfile.h"
uint8 usb_cdc_ready(void); uint32 usb_cdc_session(void);
uint8 usb_cdc_read_byte(uint8*); uint8 usb_cdc_rx_fault(void);
uint8 usb_cdc_try_write(const uint8*,uint32);
'''
TEST = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "MotorTune.h"
#include "MotorPID.h"
static uint32 primask,session;
static int ready=1,overflow,keys[4]={1,1,1,1},pwm_l,pwm_r;
static char rx[4096],tx[512]; static size_t ri,rw;
uint32 __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask=1; }
void __set_PRIMASK(uint32 m) { primask=m; }
int gpio_get_level(gpio_pin_enum pin) { return keys[pin]; }
void key_init(int ms) { (void)ms; }
void Servo_Steer(int angle) { assert(angle==0); }
void Motor_Set(int l,int r) { assert(l>=0&&r>=0&&l<=4000&&r<=4000); pwm_l=l;pwm_r=r; }
void Motor_Stop(void) { pwm_l=pwm_r=0; }
void ips200_show_string(uint16 x,uint16 y,const char*s) { (void)x;(void)y;(void)s; }
void ips200_show_int(uint16 x,uint16 y,int v,int n) { (void)x;(void)y;(void)v;(void)n; }
void ips200_show_float(uint16 x,uint16 y,float v,int n,int d) { (void)x;(void)y;(void)v;(void)n;(void)d; }
uint8 usb_cdc_ready(void) { return ready; }
uint32 usb_cdc_session(void) { return session; }
uint8 usb_cdc_read_byte(uint8*v) { if(ri==rw)return 0;*v=(uint8)rx[ri++];return 1; }
uint8 usb_cdc_rx_fault(void) { int f=overflow;overflow=0;return (uint8)f; }
uint8 usb_cdc_try_write(const uint8*p,uint32 n) { assert(n<sizeof(tx));memcpy(tx,p,n);tx[n]=0;return ready; }
static void feed(const char*s) {
    ri=0;rw=strlen(s);assert(rw<sizeof(rx));memcpy(rx,s,rw);MotorTune_Poll();assert(primask==0);
}
static void tick(int a,int b,int c,int d) {
    MotorTune_Sample10ms(a,b);MotorTune_Sample10ms(c,d);
    MotorTune_Control20ms();MotorTune_Poll();assert(primask==0);
}
static float ch(int index) {
    char copy[512];strcpy(copy,tx);char *p=strtok(copy,",");
    for(int i=0;i<index;i++){assert(p);p=strtok(NULL,",");} assert(p);return strtof(p,NULL);
}
static void near(float a,float b) { assert(fabsf(a-b)<0.011f); }
static void press(int pin) {
    keys[pin]=0;for(int i=0;i<3;i++)MotorTune_Keys10ms();
    keys[pin]=1;for(int i=0;i<3;i++)MotorTune_Keys10ms();
}
int main(void) {
    MotorTune_Init();tick(90,100,110,120);
    assert(!MotorPID_IsEnabled()&&pwm_l==0&&pwm_r==0);
    near(ch(2),100);near(ch(3),110);near(ch(6),100);near(ch(7),100);
    int commas=0;for(char*p=tx;*p;p++)if(*p==',')commas++;
    assert(commas==17&&strchr(tx,'\n')); /* 18 FireWater channels */
    feed("speed=170\npid=8,0.15,0\r\n");tick(0,0,0,0);
    near(ch(6),170);near(ch(7),170);near(ch(16),2);
    feed("lpid=11,0.25");tick(0,0,0,0);near(ch(8),8);
    feed(",0.02\n");tick(0,0,0,0);near(ch(8),11);near(ch(9),0.25f);near(ch(10),0.02f);near(ch(11),8);
    float accepted=ch(16);
    feed("pid=1,2,NaN\nspeed=69\nspeed=241\nspeed=nan\nrun=2\nleft=-1\nright=100junk\npid=1,2,3,4\n");
    tick(0,0,0,0);near(ch(16),accepted);near(ch(17),8);near(ch(8),11);near(ch(6),170);
    feed("left=100\nright=140\nrpid=3,0.1,0\nrun=1\n");
    tick(95,135,105,145);near(ch(0),2);near(ch(1),2.8f);near(ch(14),1);
    for(int i=0;i<60;i++)tick(95,135,105,145);
    near(ch(0),100);near(ch(1),140);near(ch(2),100);near(ch(3),140);
    near(ch(8),11);near(ch(11),3);
    /* Zero setpoint must remove that wheel's accumulated output immediately. */
    feed("left=0\n");tick(0,130,0,130);assert(pwm_l==0);near(ch(0),0);near(ch(1),140);
    feed("stop\n");assert(!MotorPID_IsEnabled()&&pwm_l==0&&pwm_r==0);
    feed("speed=100\nrun=1\n");tick(10,10,10,10);
    press(C13);assert(!MotorPID_IsEnabled()&&pwm_l==0&&pwm_r==0);
    press(C14);tick(0,0,0,0);near(ch(6),105);near(ch(7),105);
    press(C12);press(C15);tick(0,0,0,0);near(ch(6),100);near(ch(7),105);
    /* A disconnected/reset USB session cannot leave a running motor or auto-resume. */
    feed("run=1\n");tick(100,100,100,100);ready=0;MotorTune_Control20ms();
    assert(!MotorPID_IsEnabled()&&pwm_l==0&&pwm_r==0);
    ready=1;session++;tick(100,100,100,100);near(ch(14),0);near(ch(15),5);
    /* No encoder feedback, independently detected for either side. */
    feed("run=1\n");for(int i=0;i<105;i++)tick(0,100,0,100);
    near(ch(14),0);near(ch(15),2);assert(pwm_l==0&&pwm_r==0);
    feed("run=1\n");for(int i=0;i<105;i++)tick(100,0,100,0);
    near(ch(14),0);near(ch(15),3);
    feed("run=1\n");for(int i=0;i<500;i++)tick(100,100,100,100);
    near(ch(14),0);near(ch(15),1);assert(pwm_l==0&&pwm_r==0);
    /* Oversize and receive overflow must discard the whole contaminated line. */
    char huge[150];memset(huge,'x',sizeof(huge));huge[149]=0;feed(huge);feed("run=1\n");tick(0,0,0,0);near(ch(14),0);
    overflow=1;feed("run=1\n");tick(0,0,0,0);near(ch(14),0);
    feed("reset\n");tick(0,0,0,0);near(ch(8),8);near(ch(9),0.15f);near(ch(6),100);
    /* Actual unequal-load plants: same target must settle without faster side hiding slower side. */
    feed("pid=8,0.15,0\nspeed=100\nrun=1\n");
    float left=0,right=0;
    for(int i=0;i<480;i++) {
        left+=0.20f*(fmaxf(0,(pwm_l-180)/7.0f)-left);
        right+=0.30f*(fmaxf(0,(pwm_r-80)/5.0f)-right);
        tick((int)left,(int)right,(int)left,(int)right);
    }
    assert(fabsf(left-100)<2&&fabsf(right-100)<2&&pwm_l>pwm_r);
    MotorTune_Show();
    puts("PASS: production USB tuning commands, 18-channel telemetry, independent wheel PID and zero target, two-sample feedback, keys, bounds, disconnect, stall, timeout, unequal loads");
}
'''
with tempfile.TemporaryDirectory(prefix="rt1064_usb_tune_") as directory:
    build = Path(directory)
    (build / "zf_common_headfile.h").write_text(STUB)
    (build / "zf_driver_usb_cdc.h").write_text(USB)
    (build / "Servo.h").write_text("void Servo_Steer(int);\n")
    for name in ("MotorPID.c", "MotorPID.h", "Motor.h", "MotorTune.c", "MotorTune.h"):
        (build / name).write_bytes((ROOT / "project/code" / name).read_bytes())
    (build / "check.c").write_text(TEST)
    executable = build / "check.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-I",str(build),
                    str(build / "MotorPID.c"), str(build / "MotorTune.c"), str(build / "check.c"),
                    "-o", str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
