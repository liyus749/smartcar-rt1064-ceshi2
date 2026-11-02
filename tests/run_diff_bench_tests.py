"""Run the actual timed bench controller and MotorPID.c with simulated encoder feedback."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "project/user/src/main.c").read_text(encoding="utf-8")
block = source.split("// BEGIN DIFFERENTIAL BENCH TEST\n", 1)[1].split(
    "// END DIFFERENTIAL BENCH TEST", 1)[0]
names = ("MOTOR_DIAGNOSTIC_MODE", "RUN_KEY_DEBOUNCE_SCANS", "DIFF_TEST_TARGET",
         "DIFF_TEST_PHASE_TICKS", "DIFF_TEST_SETTLE_TICKS",
         "DIFF_TEST_STALL_START", "DIFF_TEST_STALL_TICKS",
         "DIFF_TEST_SPEED_MIN", "DIFF_TEST_SPEED_MAX", "SPEED_KEY_STEP", "SPEED_KEY_LONG_SCANS")
config = "\n".join(re.search(r"^#define\s+" + name + r"\s+.*$", source, re.M).group(0)
                   for name in names)
# Exercise the bench branch even when the production firmware is in normal track mode.
config = re.sub(r"(#define\s+MOTOR_DIAGNOSTIC_MODE\s+)\([0-9]+\)",r"\g<1>(2)",config)
STUB = """
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef int32_t int32;
#endif
"""
PREFIX = r"""
#include <assert.h>
#include <stdio.h>
#include "MotorPID.h"
#define C13 13
#define C12 12
typedef int gpio_pin_enum;
#define GPIO_LOW 0
static int level=1, speed_level=1, pwm_l, pwm_r, failed_encoder;
static int16 enc_l, enc_r;
void key_init(int period) { assert(period==10); }
int gpio_get_level(int pin) { assert(pin==C13 || pin==C12); return pin==C13 ? level : speed_level; }
void Servo_Steer(int angle) { assert(angle==0); }
void Motor_Set(int left,int right) {
    assert(left>=0 && left<=4000 && right>=0 && right<=4000);
    pwm_l=left; pwm_r=right;
}
void Motor_Stop(void) { pwm_l=pwm_r=0; }
"""
TEST = r"""
static unsigned clock_ticks;
static void ticks(int count) {
    while(count--) {
        if((clock_ticks++ % 2)==0) DiffTest_Control20ms();
        enc_l=failed_encoder ? 0 : (int16)(MotorPID_GetTargetLeft()+0.5f);
        enc_r=(int16)(MotorPID_GetTargetRight()+0.5f);
        DiffTest_Keys10ms();
        DiffTest_Sample10ms();
    }
}
static void release(void) { level=1; ticks(4); }
static void press(void) { level=0; ticks(4); }
int main(void) {
    assert(MOTOR_DIAGNOSTIC_MODE==2 && MOTORPID_DIFFERENTIAL_ENABLE==1);
    MotorPID_Init(); level=0; DiffTest_Init(); ticks(100);
    assert(diff_test_state==0 && !MotorPID_IsEnabled()); /* boot-held C13 cannot start */
    release();
    for(int i=0;i<20;i++) { level=0; ticks(2); level=1; ticks(1); }
    assert(diff_test_state==0);
    release(); press(); assert(DiffTest_Active());
    ticks(100); assert(DiffTest_Active()); /* hold cannot toggle again */
    release();
    for(int i=0;i<2000 && DiffTest_Active();i++) ticks(1);
    assert(diff_test_state==5 && DiffTest_Passed() && !MotorPID_IsEnabled());
    assert(pwm_l==0 && pwm_r==0);
    assert(diff_avg_l[0]==1000 && diff_avg_r[0]==1000);
    assert(diff_avg_l[1]==1000 && diff_avg_r[1]==1600);
    assert(diff_avg_tl[1]==1000 && diff_avg_tr[1]==1600);
    assert(diff_avg_l[2]==1000 && diff_avg_r[2]==1000);
    assert(diff_avg_l[3]==1600 && diff_avg_r[3]==1000);
    /* Automatic stop retains results until a new deliberate start. */
    ticks(300); assert(diff_test_state==5 && diff_avg_r[1]==1600);
    press(); assert(DiffTest_Active()); release(); ticks(50); press();
    assert(diff_test_state==6 && pwm_l==0 && pwm_r==0);
    ticks(100); assert(diff_test_state==6);
    /* Missing encoder / stopped wheel terminates instead of driving indefinitely. */
    release(); failed_encoder=1; press(); release(); ticks(450);
    assert(diff_test_state==7 && diff_fault_side==1 && !MotorPID_IsEnabled() && pwm_l==0 && pwm_r==0);
    assert(!DiffTest_Passed());
    /* Faked command equality must not count as observed left/right differential. */
    failed_encoder=0; release(); press(); release();
    diff_test_state=2; diff_test_ticks=DIFF_TEST_PHASE_TICKS;
    diff_samples=150; diff_sum_l=diff_sum_r=15000;
    diff_sum_tl=150000; diff_sum_tr=240000;
    DiffTest_Control20ms();
    assert(diff_valid[1] && !diff_pass[1]);
    DiffTest_Stop(6); release();
    /* Idle C12 tap +5, long hold -5 once; speed never enables a motor. */
    assert(diff_test_speed==100);
    speed_level=0; ticks(4); speed_level=1; ticks(4);
    assert(diff_test_speed==105 && !DiffTest_Active() && !MotorPID_IsEnabled());
    for(int i=0;i<4;i++) assert(!diff_valid[i]);
    speed_level=0; ticks(4); ticks(SPEED_KEY_LONG_SCANS+200);
    assert(diff_test_speed==100); speed_level=1; ticks(4); assert(diff_test_speed==100);
    for(int i=0;i<50;i++) { speed_level=0; ticks(2); speed_level=1; ticks(1); }
    ticks(4); assert(diff_test_speed==100);
    for(int i=0;i<100;i++) { speed_level=0; ticks(4); speed_level=1; ticks(4); }
    assert(diff_test_speed==DIFF_TEST_SPEED_MAX);
    for(int i=0;i<100;i++) { speed_level=0; ticks(SPEED_KEY_LONG_SCANS+4); speed_level=1; ticks(4); }
    assert(diff_test_speed==DIFF_TEST_SPEED_MIN);
    /* Boot-held speed key must be released before any adjustment. */
    speed_level=0; DiffTest_Init(); ticks(300);
    assert(diff_test_speed==100 && !DiffTest_Active()); speed_level=1; ticks(4);
    assert(diff_test_speed==100);
    /* Test in progress keeps one setpoint across phases; C13 cancels a pending tap. */
    press(); release();
    speed_level=0; ticks(4); speed_level=1; ticks(4);
    assert(diff_test_speed==100 && DiffTest_Active());
    press(); assert(!DiffTest_Active()); release();
    speed_level=0; ticks(4); press();
    speed_level=1; release(); assert(diff_test_speed==100 && DiffTest_Active());
    DiffTest_Stop(6);
    puts("PASS: real bench sequence, measured averages versus targets, reject no measured differential, auto/early/low-wheel stop, idle speed keys/limits/freeze, debounce and retained results");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="rt1064_diff_bench_") as directory:
    build = Path(directory)
    (build / "zf_common_headfile.h").write_text(STUB, encoding="utf-8")
    (build / "check.c").write_text(config + PREFIX + block + TEST, encoding="utf-8")
    executable = build / "check.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(build), "-I", str(ROOT / "project/code"),
                    str(build / "check.c"), str(ROOT / "project/code/MotorPID.c"),
                    "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
