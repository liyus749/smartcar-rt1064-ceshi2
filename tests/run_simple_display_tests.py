"""Exercise the real compact display block selected by the active Keil project."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
project_dir = ROOT / "project/mdk"
project = ET.parse(project_dir / "rt1064.uvprojx")
screen_path = project.find(".//Files/File[FileName='Screen.c']/FilePath").text
screen = (project_dir / screen_path.replace("\\", "/")).resolve()
source = screen.read_text(encoding="utf-8")
block = source.split("// BEGIN SIMPLE RUN DISPLAY\n", 1)[1].split("// END SIMPLE RUN DISPLAY", 1)[0]
PREFIX = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef uint8_t uint8;
#define SCR_TXT_Y 104
static int speed,left,ltarget,right,rtarget,pwm,cr,cs,is;
static float error;
static char state[8],spdsel[8],psel[8],dsel[8],dir[8],cam[16],reason[24];
static void bounds(int x,int y,int chars) {
    assert(x>=0&&x+chars*8<=240&&y>=104&&y+16<=320);
}
void ips200_show_string(int x,int y,const char *s) {
    bounds(x,y,(int)strlen(s));
    if(x==176&&y==104)strcpy(state,s);
    if(x==0&&y==128)strcpy(spdsel,s);
    if(x==0&&y==152)strcpy(psel,s);
    if(x==112&&y==152)strcpy(dsel,s);
    if(x==128&&y==248)strcpy(dir,s);
    if(x==0&&y==272)strcpy(cam,s);
    if(x==88&&y==272)strcpy(reason,s);
}
void ips200_show_int(int x,int y,int v,int digits) {
    bounds(x,y,digits+1); // Official library reserves a sign/space.
    if(x==40&&y==128)speed=v;
    else if(x==24&&y==176)left=v;
    else if(x==88&&y==176)ltarget=v;
    else if(x==24&&y==200)right=v;
    else if(x==88&&y==200)rtarget=v;
    else if(x==160&&y==176)pwm=v;
    else if(x==24&&y==224)cr=v;
    else if(x==112&&y==224)cs=v;
    else if(x==24&&y==248)is=v;
    else assert(0);
}
void ips200_show_float(int x,int y,float v,int a,int b) {
    bounds(x,y,a+b+2);assert(x==144&&y==128);error=v;
}
"""
TEST = r"""
int main(void) {
    Screen_ShowDriving(-16.5f,180,"1.80"," 2.00",175,180,172,180,1,0);
    Screen_ShowTrackFlags(1,3,0,0,0,0,790,0);
    assert(speed==180&&left==175&&ltarget==180&&right==172&&rtarget==180);
    assert(fabsf(error+16.5f)<.001f&&!strcmp(state,"RUN "));
    assert(!strcmp(spdsel,">SPD")&&!strcmp(psel," P")&&!strcmp(dsel," D"));
    assert(cr==1&&cs==3&&is==0&&pwm==790&&!strcmp(dir,"--   "));
    Screen_ShowTrackFlags(0,0,4,1,0,0,870,1);
    assert(!strcmp(psel,">P")&&!strcmp(spdsel," SPD")&&is==4&&!strcmp(dir,"LEFT "));
    Screen_ShowTrackFlags(0,0,8,-1,1,2,715,2);
    assert(!strcmp(dsel,">D")&&!strcmp(psel," P")&&!strcmp(dir,"RIGHT"));
    assert(is==8&&!strcmp(cam,"CAM:LOST")&&!strcmp(reason,"STOP:RING       "));
    Screen_ShowRunState(0,0);Screen_ShowHealth(1,1);
    assert(!strcmp(state,"STOP")&&!strcmp(reason,"STOP:CAMERA     "));
    Screen_ShowHealth(0,0);assert(!strcmp(cam,"CAM:OK  ")&&!strcmp(reason,"STOP:--         "));
    puts("PASS: active LCD coordinates including footer within 240x320; actual/target values, selected SPD/P/D, cross/ring/direction, fault/LOST and PWM");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix='rt1064_keys_display_') as folder:
    f=Path(folder);c=f/'check.c';c.write_text(PREFIX+block+TEST,encoding='utf-8');exe=f/'check.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
