"""Run the production 20ms controller with mocked hardware and speed feedback."""
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


def block(name):
    return source.split(f"// BEGIN {name}\n", 1)[1].split(f"// END {name}", 1)[0]


names = (
    "USE_ISLAND", "ISLAND_SLOW_RATIO", "CROSS_SLOW_RATIO", "TURN_SLOW_MEDIUM", "TURN_SLOW_SHARP",
    "TURN_MEDIUM_LIMIT", "TURN_SHARP_LIMIT", "TURN_SPEED_MEDIUM_MAX",
    "TURN_SPEED_SHARP_MAX", "STEER_LIMIT_HOLD_TICKS", "STEER_DEADBAND",
    "STEER_BOOST_START", "STEER_BOOST_K", "SERVO_U_LIMIT", "SERVO_P_DEFAULT",
    "SPEED_KEY_MAX", "TURN_PREVIEW_START", "TURN_PREVIEW_FULL", "TURN_ENTRY_SPEED_MAX",
    "TURN_RELEASE_HOLD_TICKS", "TURN_RELEASE_STEP", "DIFF_STEER_DEADBAND",
    "ERROR_DIFF_ON", "ERROR_DIFF_OFF", "ERROR_DIFF_FULL", "ERROR_DIFF_MIN_LEVEL", "ERROR_DIFF_CONFIRM", "ERROR_DIFF_MAX_AGE",
)
config = "\n".join(re.search(r"^#define\s+" + n + r"\s+.*$", source, re.M).group(0)
                   for n in names)
controller = "void pit_handler0(void)" + source.split("void pit_handler0(void)", 1)[1].split(
    "//===================================================================================================================", 1)[0]
PREFIX = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef int16_t int16;
#include "Servo.h"
#include "MotorPID.h"
#include "Roundabout.h"
#define ERR_D_LIMIT 20.0f
#define B9 9
#define MOTOR_DIAGNOSTIC_MODE 0 /* Check the normal controller, independent of selected bench firmware. */
static float run_enable=1, run_switch=0, run_speed=80, run_p=SERVO_P_DEFAULT, run_d=0;
static float ErrUse, ErrPreview, err_last, steer_prev_error, steer_d_filter;
static uint8 steer_pd_init, Img_Disappear_Flag;
static int Cross_Flag, enc_l, enc_r, dbg_duty, hb_cnt;
static int motor_enabled, servo_duty=SERVO_DUTY_MIDDLE, updates;
static int Island_State;
static uint8 round_fault,round_pending;
void Roundabout_Tick20ms(uint8 moving) { (void)moving; }
uint8 Roundabout_GetFault(void) { return round_fault; }
uint8 Roundabout_ResetPending(void) { return round_pending; }
uint8 Roundabout_AllowDifferential(void) { return !Island_State || Island_State==ROUND_INSIDE; }
static float target, diff_turn;
uint8 MotorPID_IsEnabled(void) { return motor_enabled; }
void MotorPID_SetEnable(uint8 enabled) { motor_enabled=enabled; }
void MotorPID_SetTarget(float speed) { target=speed; }
void MotorPID_SetTurn(float turn) { diff_turn=turn; }
float MotorPID_GetDiffPercent(void) { return fabsf(diff_turn)*MOTORPID_DIFF_MAX_RATIO*100.0f; }
int MotorPID_Update(int left,int right) { (void)left; (void)right; updates++; return 100; }
void Servo_Steer(int angle) {
    if(angle>SERVO_ANGLE_LEFT_MAX) angle=SERVO_ANGLE_LEFT_MAX;
    if(angle<SERVO_ANGLE_RIGHT_MAX) angle=SERVO_ANGLE_RIGHT_MAX;
    servo_duty=SERVO_DUTY_MIDDLE+angle;
}
uint16 Servo_GetDuty(void) { return (uint16)servo_duty; }
int16 Servo_GetAngle(void) { return (int16)(servo_duty-SERVO_DUTY_MIDDLE); }
void gpio_toggle_level(int pin) { assert(pin==B9); }
"""
TEST = r"""
static void tick(int count) { while(count--) pit_handler0(); }
static void near(float actual,float expected) { assert(fabsf(actual-expected)<0.001f); }
int main(void) {
    assert(SERVO_SETUP_MODE==SERVO_SETUP_RUN && Island_State==ROUND_NORMAL);
    assert(SPEED_KEY_MAX==240 && TURN_SPEED_MEDIUM_MAX==80 && TURN_SPEED_SHARP_MAX==70);
    assert(Run_DifferentialTurn(0)==0 && Run_DifferentialTurn(10)==0 && Run_DifferentialTurn(-10)==0);
    near(Run_DifferentialTurn(80),1); near(Run_DifferentialTurn(-75),-1);
    near(Run_DifferentialTurn(1000),1); near(Run_DifferentialTurn(-1000),-1);
    /* Verify the explicit 70 floor, symmetric caps and zero-speed stop. */
    for(int speed=0;speed<=240;speed++) {
        float bounded=speed>0 && speed<70 ? 70 : (float)speed;
        float previous=bounded;
        for(int angle=0;angle<=120;angle++) {
            float next=Run_CurveTarget((float)speed,(float)angle,0);
            assert(next>=0 && next<=bounded && next<=previous+0.001f);
            near(next,Run_CurveTarget((float)speed,(float)-angle,0));
            if(angle>30) assert(next<=80);
            if(angle>70) assert(next<=70);
            previous=next;
        }
    }
    /* Predictive speed caps are sign symmetric, respect the 70 floor, and cannot
       cancel stricter medium/sharp limits. They apply even before appreciable steering. */
    for(int speed=0;speed<=240;speed++) {
        float bounded=speed>0 && speed<70 ? 70 : (float)speed;
        float previous=bounded;
        for(int i=0;i<=40;i++) {
            float warning=i*0.1f;
            float next=Run_CurveTarget((float)speed,0,warning);
            assert(next>=0 && next<=bounded && next<=previous+0.001f);
            near(next,Run_CurveTarget((float)speed,0,-warning));
            if(warning>=TURN_PREVIEW_FULL) assert(next<=100);
            if(speed<=100) near(next,bounded);
            assert(Run_CurveTarget((float)speed,40,warning)<=80);
            assert(Run_CurveTarget((float)speed,70,warning)<=70);
            previous=next;
        }
    }
    /* Actual ISR: before-turn warning slows an 180 setting without extra steering. */
    run_switch=1; tick(1);
    near(target,80); assert(servo_duty==790 && motor_enabled && updates==1);
    run_speed=180; tick(50); near(target,180);
    ErrPreview=2; tick(1); near(target,100); assert(servo_duty==790);
    ErrPreview=-2; tick(1); near(target,100); assert(servo_duty==790);
    ErrPreview=0;
    tick(TURN_RELEASE_HOLD_TICKS); near(target,100);
    tick(1); near(target,100+TURN_RELEASE_STEP);
    tick(50); near(target,180);
    ErrUse=20; tick(1); near(target,80);
    ErrUse=25; tick(1); near(target,70);
    ErrUse=-25; tick(1); near(target,70); assert(diff_turn==0); /* no confirmed frames */
    /* Brief straight gaps in an S bend cannot immediately accelerate to 180. */
    ErrUse=0; tick(5); near(target,70);
    ErrUse=-25; tick(1); near(target,70);
    ErrUse=0; tick(TURN_RELEASE_HOLD_TICKS); near(target,70);
    float previous=target;
    for(int i=0;i<80;i++) {
        tick(1); assert(target>=previous && target-previous<=TURN_RELEASE_STEP+0.001f);
        previous=target;
    }
    near(target,180);
    /* Setpoint decreases, including stopping at a zero setting, take effect immediately. */
    run_speed=40; tick(1); near(target,70);
    Cross_Flag=1; ErrUse=25; tick(1); near(target,70);
    run_speed=0; tick(1); near(target,0);
    /* A fresh start uses the requested speed and the motor PID's existing soft start. */
    run_switch=0; tick(1); run_speed=180; Cross_Flag=0; ErrUse=0; run_switch=1;
    tick(1); near(target,180);
    /* Crossing then sharp turn applies both limits; recovery stays gradual. */
    Cross_Flag=1; tick(1); near(target,126); assert(diff_turn==0);
    ErrUse=25; tick(1); near(target,70);
    Cross_Flag=0; run_speed=80; tick(1); near(target,70);

    /* Confirmed straight error enters the production deadband, clears residual D,
       and issues exact center even after a right turn with the configured D=2. */
    run_d=2; ErrUse=-30; tick(4); assert(servo_duty<790);
    ErrUse=-13.5f; tick(1);
    ErrUse=0; tick(1); assert(servo_duty==790 && steer_d_filter==0);
    tick(3); assert(servo_duty==790);
    run_d=0;

    /* Error assistance counts fresh frames rather than repeated timer ticks. */
    Cross_Flag=0; Img_Disappear_Flag=0; ErrUse=25;
    Run_ErrorDiffReset();
    Run_ErrorDiffFrame(16,16,1); tick(3); assert(diff_turn==0);
    Run_ErrorDiffFrame(16,16,1); tick(1); assert(diff_turn>0);
    assert(diff_turn<1); /* intermediate image error gives intermediate assist */
    Run_ErrorDiffFrame(30,30,1); tick(1); near(diff_turn,1);
    /* PD remains unchanged and wrong-direction or small-angle commands cannot get assist. */
    assert(Run_ErrorDiffCommand(-50)==0 && Run_ErrorDiffCommand(0)==0);
    assert(Run_ErrorDiffCommand(9)==0);
    Run_ErrorDiffFrame(18,18,1); tick(1); assert(diff_turn>0); /* hysteresis */
    Run_ErrorDiffFrame(7,25,1); tick(1); assert(diff_turn==0); /* raw error releases early */
    Run_ErrorDiffFrame(30,30,1); Run_ErrorDiffFrame(30,30,1); tick(1); assert(diff_turn>0);
    tick(ERROR_DIFF_MAX_AGE); assert(diff_turn==0); /* stale camera cannot keep assisting */
    Run_ErrorDiffFrame(-25,-25,1); ErrUse=-25; tick(1); assert(diff_turn==0);
    Run_ErrorDiffFrame(-25,-25,1); tick(1); assert(diff_turn<0);
    Run_ErrorDiffFrame(30,30,1); ErrUse=30; tick(1); assert(diff_turn==0); /* direction needs fresh confirmation */
    Run_ErrorDiffFrame(30,30,1); tick(1); assert(diff_turn>0);
    Cross_Flag=1; tick(1); assert(diff_turn==0); Cross_Flag=0;
    Run_ErrorDiffFrame(30,30,0); tick(1); assert(diff_turn==0); /* poor/invalid line data */
    Run_ErrorDiffFrame(30,30,1); Run_ErrorDiffFrame(30,30,1); tick(1); assert(diff_turn>0);
    Img_Disappear_Flag=1; tick(1); assert(diff_turn==0); Img_Disappear_Flag=0;
    Run_ErrorDiffFrame(30,30,1); Run_ErrorDiffFrame(30,30,1); tick(1); assert(diff_turn>0);
    run_switch=0; tick(1); run_switch=1; tick(1); assert(diff_turn==0);

    /* Peak is immediate; LIM needs 10 uninterrupted ticks and latches across stop. */
    run_switch=0; tick(1); run_switch=1; ErrUse=40; tick(9);
    assert(dbg_peak_left==870 && dbg_peak_right==790 && dbg_limit_flags==0);
    tick(1); assert(dbg_limit_flags==1);
    ErrUse=-40; tick(9); assert(dbg_peak_right==715 && dbg_limit_flags==1);
    tick(1); assert(dbg_limit_flags==3);
    Img_Disappear_Flag=1; tick(1); assert(dbg_image_lost==1 && diff_turn==0);
    run_switch=0; tick(20);
    assert(!motor_enabled && servo_duty==790 && dbg_peak_left==870 && dbg_peak_right==715);
    assert(dbg_limit_flags==3 && dbg_image_lost==1);
    /* A stopped/live image must not contaminate the next run. */
    Img_Disappear_Flag=0; ErrUse=0; run_switch=1; tick(1);
    assert(dbg_peak_left==790 && dbg_peak_right==790 && !dbg_limit_flags && !dbg_image_lost);
    /* Interrupted bursts are not falsely counted as one continuous saturation. */
    for(int i=0;i<4;i++) { ErrUse=40; tick(9); ErrUse=0; tick(1); }
    assert(!dbg_limit_flags);
    run_switch=0; tick(1); Img_Disappear_Flag=1; tick(10);
    assert(!dbg_image_lost);
    puts("PASS: actual controller fresh-frame large-error assist and release, stale/invalid/cross rejection, speed caps, recovery and stop/restart");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="rt1064_curve_") as directory:
    build = Path(directory)
    (build / "zf_common_headfile.h").write_text("/* No hardware needed. */\n", encoding="utf-8")
    (build / "check.c").write_text(config + PREFIX + block("STEERING BOOST") +
                                   block("RUN CURVE CONTROL") + controller + TEST,
                                   encoding="utf-8")
    executable = build / "check.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(build), "-I", str(code_dir),
                    str(build / "check.c"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
