"""Exercise production MotorPID.c in both differential and equal-target modes."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = """
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
typedef uint8_t uint8;
#endif
"""
TEST = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "MotorPID.h"
static int pwm_l, pwm_r;
void Motor_Set(int left,int right) {
    assert(left>=0 && left<=MOTOR_DUTY_MAX && right>=0 && right<=MOTOR_DUTY_MAX);
    pwm_l=left; pwm_r=right;
}
void Motor_Stop(void) { pwm_l=pwm_r=0; }
static void near(float actual,float expected) { assert(fabsf(actual-expected)<0.001f); }
static void settle(float speed,float turn) {
    MotorPID_SetTarget(speed); MotorPID_SetTurn(turn);
    for(int i=0;i<100;i++) MotorPID_Update(0,0);
}
int main(void) {
    MotorPID_Init();
    MotorPID_SetTurn(1); MotorPID_Update(0,0);
    assert(pwm_l==0 && pwm_r==0 && MotorPID_GetTarget()==0);
    MotorPID_SetEnable(1); MotorPID_SetTarget(70); MotorPID_SetTurn(1);
    float previous=0;
    for(int i=0;i<50;i++) {
        MotorPID_Update(0,0);
        float left=MotorPID_GetTargetLeft(), right=MotorPID_GetTargetRight();
        assert(left>=previous && left-previous<=1.401f && left<=70.001f);
        assert(right>=left && right<=left*(1.0f+MOTORPID_DIFF_MAX_RATIO+0.00001f));
        near(MotorPID_GetTarget(),(left+right)*0.5f);
        previous=left;
    }
    near(MotorPID_GetTargetLeft(),70);
#if MOTORPID_DIFFERENTIAL_ENABLE
    near(MotorPID_GetTargetRight(),(70*(1.0f+MOTORPID_DIFF_MAX_RATIO)));
    /* Reverse swaps the boosted side immediately, ramping the new assist from zero. */
    MotorPID_SetTurn(-1); MotorPID_Update(0,0);
    near(MotorPID_GetTargetRight(),70);
    assert(MotorPID_GetTargetLeft()>70 && MotorPID_GetTargetLeft()<=70*(1.0f+MOTORPID_DIFF_STEP)+0.001f);
    settle(70,-1); near(MotorPID_GetTargetLeft(),(70*(1.0f+MOTORPID_DIFF_MAX_RATIO))); near(MotorPID_GetTargetRight(),70);
    /* Straight resets all differential immediately; no yaw assist is retained. */
    MotorPID_SetTurn(0); MotorPID_Update(0,0);
    near(MotorPID_GetTargetLeft(),70); near(MotorPID_GetTargetRight(),70);
    /* Smaller turn produces less assist; deadband itself is in the real main controller. */
    settle(70,0.5f); near(MotorPID_GetTargetRight(),(70*(1.0f+0.5f*MOTORPID_DIFF_MAX_RATIO))); near(MotorPID_GetTargetLeft(),70);
#else
    near(MotorPID_GetTargetRight(),70);
    settle(70,-1); near(MotorPID_GetTargetLeft(),70); near(MotorPID_GetTargetRight(),70);
#endif
    near(MotorPID_GetDiffPercent(),MOTORPID_DIFFERENTIAL_ENABLE ? 50*MOTORPID_DIFF_MAX_RATIO : 0);
    /* High setpoints cannot escape per-wheel bounds, and zero never forces a minimum. */
    settle(999,1); assert(MotorPID_GetTargetLeft()<=240 && MotorPID_GetTargetRight()<=240); near(MotorPID_GetDiffPercent(),MOTORPID_DIFFERENTIAL_ENABLE ? 100*MOTORPID_DIFF_MAX_RATIO : 0);
    MotorPID_SetTarget(0); MotorPID_Update(0,0);
    near(MotorPID_GetTargetLeft(),0); near(MotorPID_GetTargetRight(),0);
    MotorPID_SetEnable(0); assert(!MotorPID_IsEnabled() && pwm_l==0 && pwm_r==0);
    near(MotorPID_GetTargetLeft(),0); near(MotorPID_GetTargetRight(),0);
    MotorPID_SetEnable(1); MotorPID_SetTarget(70); MotorPID_Update(0,0);
    near(MotorPID_GetTargetLeft(),MotorPID_GetTargetRight()); /* stale turn was reset */
    MotorPID_SetTarget(NAN); MotorPID_Update(0,0); near(MotorPID_GetTarget(),0);
    MotorPID_SetTurn(NAN); settle(70,0); near(MotorPID_GetTargetLeft(),MotorPID_GetTargetRight());
    /* Every nonzero requested speed respects 70 after soft start, and saturation
       preserves the commanded ratio rather than clipping both wheels to equality. */
    for(int speed=1;speed<=260;speed++) for(int direction=-1;direction<=1;direction++) {
        settle((float)speed,(float)direction);
        float left=MotorPID_GetTargetLeft(),right=MotorPID_GetTargetRight();
        assert(left>=70 && right>=70 && left<=240 && right<=240);
        if(MOTORPID_DIFFERENTIAL_ENABLE && direction) {
            float inner=direction>0 ? left : right,outer=direction>0 ? right : left;
            near(outer/inner,1.0f+MOTORPID_DIFF_MAX_RATIO);
            assert(outer>inner);
        } else near(left,right);
    }
    /* One stalled wheel needs its own feedback; fast other wheel cannot conceal it. */
    MotorPID_SetEnable(0); MotorPID_SetEnable(1);
    MotorPID_SetGains(8,0.15f,0); MotorPID_SetTarget(70); MotorPID_SetTurn(0);
    for(int i=0;i<300;i++) MotorPID_Update(0,70);
    assert(pwm_l>pwm_r+1000 && pwm_l<=MOTOR_DUTY_MAX);
    MotorPID_SetEnable(0); MotorPID_SetEnable(1); MotorPID_SetTarget(70);
    for(int i=0;i<300;i++) MotorPID_Update(70,0);
    assert(pwm_r>pwm_l+1000 && pwm_r<=MOTOR_DUTY_MAX);
    /* No unbounded duty increase during a stall in a turn. */
    settle(70,1); for(int i=0;i<10000;i++) MotorPID_Update(0,0);
    assert(pwm_l<=MOTOR_DUTY_MAX && pwm_r<=MOTOR_DUTY_MAX);
    MotorPID_SetEnable(0); MotorPID_Update(0,0); assert(pwm_l==0 && pwm_r==0);
    printf("PASS: differential=%d, correct outer wheel, 70 minimum, differential survives saturation, bounded slew/targets/duty, independent stalled-wheel feedback, stop/restart\n",MOTORPID_DIFFERENTIAL_ENABLE);
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="rt1064_motor_diff_") as directory:
    build = Path(directory)
    (build / "zf_common_headfile.h").write_text(STUB, encoding="utf-8")
    for name in ("MotorPID.c", "Motor.h"):
        (build / name).write_bytes((ROOT / "project/code" / name).read_bytes())
    (build / "check.c").write_text(TEST, encoding="utf-8")
    original = (ROOT / "project/code/MotorPID.h").read_text(encoding="utf-8")
    for enabled in (0, 1):
        # Select each feature branch only in the temporary build; no project edits.
        header = re.sub(r"(#define\s+MOTORPID_DIFFERENTIAL_ENABLE\s+)\([01]\)",
                        rf"\g<1>({enabled})", original)
        (build / "MotorPID.h").write_text(header, encoding="utf-8")
        executable = build / f"check_{enabled}.exe"
        subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I", str(build), str(build / "check.c"),
                        str(build / "MotorPID.c"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
