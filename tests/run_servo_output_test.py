"""Check the diagnostic's motor lockout and both bounded pulse sequences."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT_DIR = ROOT / "project/mdk"
project = ET.parse(PROJECT_DIR / "rt1064_servo_output_test.uvprojx")
servo_path = project.find(".//Files/File[FileName='Servo.c']/FilePath").text
SERVO_SOURCE = (PROJECT_DIR / servo_path.replace("\\", "/")).resolve()
STUB = r'''
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;
#define PWM4_MODULE2_CHA_C30 30
#define C30 30
#define GPIO_HIGH 1
#define GPIO_LOW 0
#define GPO 1
#define GPO_PUSH_PULL 0
#define SYSTEM_CLOCK_600M 600000000
#define BOARD_XTAL_FREQ 24000000
#define GPT2 2
#define kGPT_OutputCompare1Flag 1
#define kGPT_OutputCompare_Channel1 1
void clock_init(uint32);
uint32 interrupt_global_disable(void);
void pwm_init(int,int,int);
void pwm_set_duty(int,uint16);
void system_delay_ms(uint32);
void gpio_init(int,int,int,int);
void gpio_set_level(int,int);
void GPT_StopTimer(int);
void GPT_ClearStatusFlags(int,int);
void GPT_SetOutputCompareValue(int,int,uint32);
void GPT_StartTimer(int);
int GPT_GetStatusFlags(int,int);
void ips200_clear(void);
void ips200_show_string(int,int,const char *);
void ips200_show_uint(int,int,uint32,int);
#endif
'''
TEST = r'''
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "zf_common_headfile.h"
static jmp_buf done;
static int disabled, motor_init, stops, hw_duty, gpio_mode;
static int phase, hw_frames, gpio_frames, high, waits, complete, rounds;
static uint32 wait_us;
static int expected(int frame) {
    frame %= 550;
    if(frame<150) return 790;
    if(frame<250) return 830;
    if(frame<350) return 790;
    if(frame<450) return 750;
    return 790;
}
void clock_init(uint32 clock) { assert(clock==600000000U); }
uint32 interrupt_global_disable(void) { disabled=1;return 0; }
void Motor_Init(void) { assert(disabled);motor_init=1; }
void Motor_Stop(void) { assert(motor_init);stops++; }
void pwm_init(int pin,int freq,int duty) {
    assert(pin==30&&freq==50&&duty==0);
    gpio_mode=0;
}
void pwm_set_duty(int pin,uint16 duty) {
    assert(!gpio_mode&&pin==30&&duty>=715&&duty<=870);hw_duty=duty;
}
void system_delay_ms(uint32 ms) {
    if(ms==500) { assert(hw_duty==790);return; }
    assert(phase==1&&!gpio_mode&&ms==20&&hw_frames<1650);
    assert(hw_duty==expected(hw_frames));hw_frames++;
}
void Screen_Init(void) { assert(hw_duty==790); }
void ips200_clear(void) { }
void ips200_show_string(int x,int y,const char *s) {
    (void)x;(void)y;
    if(strstr(s,"A: HARDWARE")) {
        assert(hw_frames==rounds*550 && gpio_frames==rounds*550);
        rounds++;phase=1;
    }
    if(strstr(s,"B: GPIO")) { assert(hw_frames==rounds*550&&gpio_mode);phase=2; }
    if(strstr(s,"DONE:")) { assert(gpio_frames==1650&&rounds==3);complete=1; }
}
void ips200_show_uint(int x,int y,uint32 d,int width) {
    (void)x;(void)width;
    if(y==128) { assert(d>=1500&&d<=1660); }
    else { assert(d>=715&&d<=870); }
}
void gpio_init(int pin,int dir,int level,int mode) {
    assert(disabled&&pin==30&&dir==GPO&&level==0&&mode==GPO_PUSH_PULL);
    assert(hw_frames==rounds*550);gpio_mode=1;
}
void gpio_set_level(int pin,int level) {
    assert(gpio_mode&&phase==2&&pin==30);
    if(level) {
        assert(!high);
        if(complete&&gpio_frames==1652) longjmp(done,1);
        high=1;
    } else { assert(high);high=0; }
}
void GPT_StopTimer(int gpt) { assert(gpt==2); }
void GPT_ClearStatusFlags(int gpt,int flag) { assert(gpt==2&&flag==1); }
void GPT_SetOutputCompareValue(int gpt,int ch,uint32 ticks) {
    assert(gpt==2&&ch==1&&ticks%24U==0);
    wait_us=ticks/24U;
    if(high) { assert(wait_us==(uint32)expected(gpio_frames)*2U);waits++; }
    else {
        assert(wait_us==20000U-(uint32)expected(gpio_frames)*2U);
        gpio_frames++;waits++;
    }
}
void GPT_StartTimer(int gpt) { assert(gpt==2&&wait_us>0); }
int GPT_GetStatusFlags(int gpt,int flag) { assert(gpt==2&&flag==1);return 1; }
#define main diagnostic_main
#include "servo_output_test.c"
#undef main
int main(void) {
    if(!setjmp(done)) diagnostic_main();
    assert(complete&&rounds==3&&hw_frames==1650&&gpio_frames==1652&&waits==3304);
    assert(stops>3300&&disabled);
    puts("PASS: actual diagnostic + Servo.c: motor outputs locked off; three PWM/GPIO rounds with pin remux; 1500/1580/1660us pulses, 20ms periods; measured endpoint bounds; final center hold");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="servo_output_test_") as folder:
    build = Path(folder)
    (build / "zf_common_headfile.h").write_text(STUB, encoding="utf-8")
    (build / "fsl_gpt.h").write_text('#include "zf_common_headfile.h"\n')
    (build / "Motor.h").write_text("void Motor_Init(void);void Motor_Stop(void);\n")
    (build / "Screen.h").write_text("void Screen_Init(void);\n")
    for source in (SERVO_SOURCE, SERVO_SOURCE.with_suffix(".h"),
                   ROOT / "project/diagnostics/servo_output_test.c"):
        (build / source.name).write_bytes(source.read_bytes())
    (build / "check.c").write_text(TEST, encoding="utf-8")
    exe = build / "check.exe"
    subprocess.run(["gcc", "-std=c99", "-Wall",
                    "-Wextra", "-Werror", "-Wno-unused-variable", "-I", str(build), str(build / "check.c"),
                    str(build / "Servo.c"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
