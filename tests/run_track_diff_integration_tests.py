"""Link the production image-error gate and 20ms ISR with the real motor controller."""
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
project_dir = ROOT / "project/mdk"
main_path = ET.parse(project_dir / "rt1064.uvprojx").find(".//Files/File[FileName='main.c']/FilePath").text
main_source = (project_dir / main_path.replace("\\", "/")).resolve()
code_dir = main_source.parents[2] / "code"
source = main_source.read_text(encoding="utf-8")
names = (
    "SPEED_DEFAULT", "USE_ISLAND", "ISLAND_SLOW_RATIO", "CROSS_SLOW_RATIO", "TURN_SLOW_MEDIUM", "TURN_SLOW_SHARP",
    "TURN_MEDIUM_LIMIT", "TURN_SHARP_LIMIT", "TURN_SPEED_MEDIUM_MAX",
    "TURN_SPEED_SHARP_MAX", "STEER_LIMIT_HOLD_TICKS", "STEER_DEADBAND",
    "STEER_BOOST_START", "STEER_BOOST_K", "SERVO_U_LIMIT", "SERVO_P_DEFAULT",
    "TURN_PREVIEW_START", "TURN_PREVIEW_FULL", "TURN_ENTRY_SPEED_MAX",
    "TURN_RELEASE_HOLD_TICKS", "TURN_RELEASE_STEP", "DIFF_STEER_DEADBAND",
    "ERROR_DIFF_ON", "ERROR_DIFF_OFF", "ERROR_DIFF_FULL", "ERROR_DIFF_MIN_LEVEL",
    "ERROR_DIFF_CONFIRM", "ERROR_DIFF_MAX_AGE",
)
config = "\n".join(re.search(r"^#define\s+" + name + r"\s+.*$", source, re.M).group(0)
                   for name in names)


def block(name):
    return source.split(f"// BEGIN {name}\n", 1)[1].split(f"// END {name}", 1)[0]


controller = "void pit_handler0(void)" + source.split("void pit_handler0(void)", 1)[1].split(
    "//===================================================================================================================", 1)[0]
STUB = """
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef int16_t int16;
#endif
"""
PREFIX = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "Servo.h"
#include "MotorPID.h"
#include "Roundabout.h"
#define ERR_D_LIMIT 20.0f
#define B9 9
#define MOTOR_DIAGNOSTIC_MODE 0
static float run_enable=1,run_switch=0,run_speed=SPEED_DEFAULT,run_p=SERVO_P_DEFAULT,run_d=0;
static float ErrUse,ErrPreview,err_last,steer_prev_error,steer_d_filter;
static uint8 steer_pd_init,Img_Disappear_Flag;
static int Cross_Flag,enc_l,enc_r,dbg_duty,hb_cnt,pwm_l,pwm_r;
static int Island_State;
static uint8 round_fault,round_pending;
void Roundabout_Tick20ms(uint8 moving) { (void)moving; }
uint8 Roundabout_GetFault(void) { return round_fault; }
uint8 Roundabout_ResetPending(void) { return round_pending; }
uint8 Roundabout_AllowDifferential(void) { return !Island_State || Island_State==ROUND_INSIDE; }
static int servo_duty=SERVO_DUTY_MIDDLE;
void Servo_Steer(int angle) {
    if(angle>SERVO_ANGLE_LEFT_MAX) angle=SERVO_ANGLE_LEFT_MAX;
    if(angle<SERVO_ANGLE_RIGHT_MAX) angle=SERVO_ANGLE_RIGHT_MAX;
    servo_duty=SERVO_DUTY_MIDDLE+angle;
}
uint16 Servo_GetDuty(void) { return (uint16)servo_duty; }
int16 Servo_GetAngle(void) { return (int16)(servo_duty-SERVO_DUTY_MIDDLE); }
void Motor_Set(int left,int right) {
    assert(left>=0 && left<=4000 && right>=0 && right<=4000);
    pwm_l=left;pwm_r=right;
}
void Motor_Stop(void) { pwm_l=pwm_r=0; }
void gpio_toggle_level(int pin) { assert(pin==B9); }
"""
TEST = r"""
static void near(float a,float b) { assert(fabsf(a-b)<0.002f); }
static void tick(void) {
    /* Simulated counts are test inputs, not evidence of real motor response. */
    enc_l=(int)(MotorPID_GetTargetLeft()+0.5f);
    enc_r=(int)(MotorPID_GetTargetRight()+0.5f);
    pit_handler0();
}
static void frames(float error,int count) {
    while(count--) {
        ErrUse=error;
        Run_ErrorDiffFrame(error,error,!Cross_Flag && !Img_Disappear_Flag && run_switch>0.5f);
        tick();tick();
    }
}
int main(void) {
    MotorPID_Init();frames(24,10);
    near(MotorPID_GetTarget(),0);assert(!pwm_l && !pwm_r);
    run_switch=1;frames(0,80);
    assert(SPEED_DEFAULT==180);
    near(MotorPID_GetTargetLeft(),180);near(MotorPID_GetTargetRight(),180);
    /* The old 20-pixel gate missed this confirmed moderate error. */
    frames(16,15);
    assert(MotorPID_GetTargetRight()>MotorPID_GetTargetLeft()*1.30f);
    assert(MotorPID_GetTargetLeft()>=70);
    frames(24,15);
    near(MotorPID_GetTargetLeft(),70);near(MotorPID_GetTargetRight(),112);
    near(MotorPID_GetDiffPercent(),60);
    frames(-24,15);
    near(MotorPID_GetTargetLeft(),112);near(MotorPID_GetTargetRight(),70);
    /* Recenter / cross / missing frames revoke differential at the motor output. */
    frames(0,1);near(MotorPID_GetTargetLeft(),MotorPID_GetTargetRight());
    frames(24,15);Cross_Flag=1;frames(24,1);
    near(MotorPID_GetTargetLeft(),MotorPID_GetTargetRight());
    Cross_Flag=0;frames(24,15);Img_Disappear_Flag=1;tick();
    near(MotorPID_GetTargetLeft(),MotorPID_GetTargetRight());
    Img_Disappear_Flag=0;frames(24,15);
    for(int i=0;i<ERROR_DIFF_MAX_AGE;i++) tick();
    near(MotorPID_GetTargetLeft(),MotorPID_GetTargetRight());
    run_speed=40;frames(24,20);near(MotorPID_GetTargetLeft(),70);
    near(MotorPID_GetTargetRight(),112);
    run_speed=0;frames(24,1);near(MotorPID_GetTarget(),0);
    run_speed=80;frames(24,80);near(MotorPID_GetTargetLeft(),70);
    near(MotorPID_GetTargetRight(),112);
    run_switch=0;tick();near(MotorPID_GetTarget(),0);
    assert(!pwm_l && !pwm_r && servo_duty==790);
    near(dbg_diff_peak,60); /* command peak survives manual stop */
    run_switch=1;frames(0,1);near(dbg_diff_peak,0);
    /* Ring guidance cannot inherit a 60% yaw assist; actual inside curves may use it. */
    Island_State=ROUND_ENTER;frames(24,80);
    near(MotorPID_GetTargetLeft(),70);near(MotorPID_GetTargetRight(),70);
    Island_State=ROUND_INSIDE;frames(24,80);
    near(MotorPID_GetTargetLeft(),70);near(MotorPID_GetTargetRight(),112);
    Island_State=ROUND_EXIT;frames(24,1);
    near(MotorPID_GetTargetLeft(),70);near(MotorPID_GetTargetRight(),70);
    Island_State=ROUND_APPROACH;frames(0,50);
    near(MotorPID_GetTarget(),70); /* cap applies before steering becomes large */
    round_fault=1;tick();assert(run_switch==0 && !pwm_l && !pwm_r && servo_duty==790);
    round_pending=1;run_switch=1;tick();assert(!pwm_l && !pwm_r && dbg_duty==0);
    round_fault=round_pending=0;Island_State=0;frames(0,60);assert(MotorPID_IsEnabled());
    puts("PASS: real image gate -> normal ISR -> real MotorPID, straight180, 70/112 left and right, earlier assist, invalid/cross/stale/recenter release and stop");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="rt1064_track_diff_") as directory:
    build = Path(directory)
    (build / "zf_common_headfile.h").write_text(STUB, encoding="utf-8")
    (build / "check.c").write_text(config + PREFIX + block("STEERING BOOST") +
                                   block("RUN CURVE CONTROL") + controller + TEST,
                                   encoding="utf-8")
    executable = build / "check.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(build), "-I", str(code_dir),
                    str(build / "check.c"), str(code_dir / "MotorPID.c"),
                    "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
