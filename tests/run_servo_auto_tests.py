"""Exercise the real Servo.c auto-sweep state machine without physical PWM."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = """
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
typedef uint8_t uint8;
typedef int16_t int16;
typedef uint16_t uint16;
#define PWM4_MODULE2_CHA_C30 30
void pwm_init(int pin, int freq, int duty);
void pwm_set_duty(int pin, uint16 duty);
void system_delay_ms(int ms);
#endif
"""
TEST = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "Servo.h"
static int output, writes;
void pwm_init(int pin,int freq,int duty) {
    assert(pin == 30 && freq == 50 && duty == 0);
}
void pwm_set_duty(int pin,uint16 duty) {
    int amp = Servo_AutoTestAmplitude();
    assert(pin == 30);
    assert(duty >= SERVO_DUTY_MIDDLE - amp);
    assert(duty <= SERVO_DUTY_MIDDLE + amp);
    output = duty; writes++;
}
void system_delay_ms(int ms) { (void)ms; }
static void complete_sweep(int amplitude) {
    int low = SERVO_DUTY_MIDDLE, high = SERVO_DUTY_MIDDLE;
    int previous = output;
    for(int i=0; i<6000; i++) {
        Servo_AutoTestStep();
        assert(output - previous <= 1 && previous - output <= 1);
        if(output < low) low = output;
        if(output > high) high = output;
        previous = output;
    }
    assert(low == SERVO_DUTY_MIDDLE - amplitude);
    assert(high == SERVO_DUTY_MIDDLE + amplitude);
    assert(Servo_AutoTestCycles() == SERVO_AUTO_CYCLES);
    assert(Servo_AutoTestPaused());
    assert(output == SERVO_DUTY_MIDDLE);
}
int main(void) {
    assert(SERVO_SETUP_MODE == SERVO_SETUP_AUTO);
    assert(!SERVO_TEST_USE_WIRELESS);
    Servo_Init();
    assert(writes && output == 760);
    assert(Servo_AutoTestAmplitude() == 40);
    for(int i=0; i<SERVO_AUTO_START_TICKS; i++) {
        Servo_AutoTestStep(); assert(output == 760);
    }
    complete_sweep(40);
    assert(Servo_AutoTestAmplitude() == 40); /* Never grows unattended. */
    int count = writes;
    for(int i=0; i<1000; i++) Servo_AutoTestStep();
    assert(writes == count);
    assert(!Servo_CalibrationSetTarget(800));
    assert(!Servo_CalibrationSetTarget(NAN));

    Servo_AutoTestPause(0);
    for(int i=0; i<200; i++) Servo_AutoTestStep();
    assert(output > 760);
    Servo_AutoTestPause(1);
    assert(output == 760 && Servo_AutoTestPaused());
    for(int i=0; i<1000; i++) Servo_AutoTestStep();
    assert(output == 760);

    Servo_AutoTestAdjust(10000);
    assert(Servo_AutoTestAmplitude() == 50); /* At most +10 per request. */
    assert(output == 760 && !Servo_AutoTestPaused());
    for(int i=0; i<40; i++) Servo_AutoTestAdjust(10);
    assert(Servo_AutoTestAmplitude() == SERVO_AUTO_MAX_AMP);
    complete_sweep(SERVO_AUTO_MAX_AMP);
    for(int i=0; i<40; i++) Servo_AutoTestAdjust(-10);
    assert(Servo_AutoTestAmplitude() == SERVO_AUTO_AMP_STEP);
    complete_sweep(SERVO_AUTO_AMP_STEP);
    Servo_SetDuty(0); assert(output == 760);
    Servo_SetDuty(10000); assert(output == 760);
    Servo_Init();
    assert(output == 760 && Servo_AutoTestAmplitude() == 40);
    puts("PASS: real Servo.c: startup center, both sweep endpoints, one-unit slew, three-cycle stop, pause/resume, bounded manual amplitude, no wireless commands");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="su400_auto_test_") as directory:
    build = Path(directory)
    (build / "zf_common_headfile.h").write_text(STUB, encoding="utf-8")
    # Exercise the auto-sweep branch without changing the production RUN config.
    servo_header = (ROOT / "project/code/Servo.h").read_text(encoding="utf-8")
    run_mode = "#define SERVO_SETUP_MODE        (SERVO_SETUP_RUN)"
    auto_mode = "#define SERVO_SETUP_MODE        (SERVO_SETUP_AUTO)"
    if run_mode not in servo_header and auto_mode not in servo_header:
        raise RuntimeError("Unexpected servo setup mode: cannot select AUTO for this test")
    (build / "Servo.h").write_text(servo_header.replace(run_mode, auto_mode), encoding="utf-8")
    (build / "Servo.c").write_bytes((ROOT / "project/code/Servo.c").read_bytes())
    (build / "auto_test.c").write_text(TEST, encoding="utf-8")
    executable = build / "auto_test.exe"
    subprocess.run([
        "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(build), "-I", str(ROOT / "project/code"),
        str(build / "auto_test.c"), str(build / "Servo.c"),
        "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
