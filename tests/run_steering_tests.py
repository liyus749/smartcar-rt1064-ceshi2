"""Check real turn compensation and RUN-mode Servo.c with a mocked PWM output."""
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
boost = source.split("// BEGIN STEERING BOOST\n", 1)[1].split("// END STEERING BOOST", 1)[0]
config = "\n".join(re.search(r"^#define\s+" + key + r"\s+.*$", source, re.MULTILINE).group(0)
                   for key in ("SERVO_P_DEFAULT", "STEER_BOOST_START", "STEER_BOOST_K"))
STUB = """
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
#include <stddef.h>
typedef uint8_t uint8;
typedef int16_t int16;
typedef uint16_t uint16;
#define PWM4_MODULE2_CHA_C30 30
void pwm_init(int pin, int frequency, int duty);
void pwm_set_duty(int pin, uint16 duty);
void system_delay_ms(int ms);
#endif
"""
TEST = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "Servo.h"
static int last, writes;
void pwm_init(int pin,int frequency,int duty) { assert(pin==30 && frequency==50 && duty==0); }
void pwm_set_duty(int pin,uint16 duty) {
    assert(pin==30 && duty>=SERVO_DUTY_RIGHT && duty<=SERVO_DUTY_LEFT);
    last=duty; writes++;
}
void system_delay_ms(int ms) { (void)ms; }
int main(void) {
    assert(SERVO_SETUP_MODE == SERVO_SETUP_RUN);
    assert(SERVO_DUTY_MIDDLE==790 && SERVO_DUTY_LEFT==870 && SERVO_DUTY_RIGHT==715);
    assert(Steering_Boost(0)==0 && Steering_Boost(10)==0 && Steering_Boost(-10)==0);
    assert(Steering_Boost(12)==0 && Steering_Boost(-12)==0);
    assert(Steering_Boost(15) < 0.03f*15*15);
    assert(Steering_Boost(25) > 0.03f*25*25);
    assert(fabsf(Steering_Boost(-25)+Steering_Boost(25)) < 0.0001f);
    assert(Steering_Boost(12.01f) > 0 && Steering_Boost(12.01f) < 0.1f);

    Servo_Init(); assert(last==790 && writes);
    Servo_Steer(10); assert(last==800);
    int before=writes;
    for(int i=0;i<100;i++) { Servo_Steer((i%2) ? 9 : 11); }
    assert(last==800 && writes==before); /* adjacent-unit chatter is suppressed */
    Servo_Steer(12); assert(last==802 && writes==before+1);
    Servo_Steer(1); assert(last==791);
    Servo_Steer(0); assert(last==790); /* exact recenter still works across a one-unit delta */

    Servo_Steer((int)(SERVO_P_DEFAULT*25 + Steering_Boost(25))); assert(last>850);
    Servo_Steer(SERVO_ANGLE_LEFT_MAX-1); assert(last==869);
    Servo_Steer(SERVO_ANGLE_LEFT_MAX); assert(last==870); /* endpoint cannot be filtered as chatter */
    Servo_Steer(SERVO_ANGLE_RIGHT_MAX+1); assert(last==716);
    Servo_Steer(SERVO_ANGLE_RIGHT_MAX); assert(last==715);
    Servo_Steer(1000); assert(last==870);
    Servo_Steer(-1000); assert(last==715); /* no slew delay on a large correction */
    for(int angle=-1000;angle<=1000;angle++) { Servo_Steer(angle); }
    Servo_Steer(0); assert(last==790);
    puts("PASS: smooth thresholded bend boost, small PWM chatter suppression, fast large corrections and recenter, measured limit clamps");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="rt1064_steering_") as directory:
    build=Path(directory)
    (build/"zf_common_headfile.h").write_text(STUB,encoding="utf-8")
    # Put the compiled production helper before main while keeping Servo.h visible to the mocks.
    (build/"check.c").write_text(config + "\n#include \"Servo.h\"\n" + boost + TEST,encoding="utf-8")
    executable=build/"check.exe"
    subprocess.run(["gcc","-std=c99","-O2","-Wall","-Wextra","-Werror","-Wno-unused-variable",
                    "-I",str(build),"-I",str(code_dir),str(build/"check.c"),
                    str(code_dir/"Servo.c"),"-o",str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
