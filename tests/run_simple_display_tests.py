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
PREFIX = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8;
#define SCR_TXT_Y 104
static int speed,left,ltarget,right,rtarget;
static float error;
static char state[8];
static void bounds(int x,int y) { assert(x>=0&&x<240&&y>=104&&y<=224); }
void ips200_show_string(int x,int y,const char *s) {
    static const char *allowed[]={"AUTO180 R2","RUN ","WAIT","STOP","SPD","ERR",
        "P","D","L:","R:","/","L/R: actual / target","1.80","2.00"};
    int valid=0;bounds(x,y);
    assert(x+(int)strlen(s)*8<=240);
    for(unsigned i=0;i<sizeof(allowed)/sizeof(allowed[0]);i++)valid|=!strcmp(s,allowed[i]);
    assert(valid);
    if(x==176&&y==104)strcpy(state,s);
}
void ips200_show_int(int x,int y,int v,int digits) {
    bounds(x,y);assert(digits==3||digits==4);
    if(x==40&&y==128)speed=v;
    else if(x==24&&y==176)left=v;
    else if(x==88&&y==176)ltarget=v;
    else if(x==24&&y==200)right=v;
    else if(x==88&&y==200)rtarget=v;
    else assert(0);
}
void ips200_show_float(int x,int y,float v,int a,int b) {
    bounds(x,y);assert(x==144&&y==128&&a==3&&b==1);error=v;
}
'''
TEST = r'''
int main(void) {
    Screen_ShowDriving(-16.5f,180,"1.80","2.00",175,180,172,180,1,0);
    assert(speed==180&&left==175&&ltarget==180&&right==172&&rtarget==180);
    assert(fabsf(error+16.5f)<.001f&&!strcmp(state,"RUN "));
    Screen_ShowRunState(0,1);assert(!strcmp(state,"WAIT"));
    Screen_ShowRunState(0,0);assert(!strcmp(state,"STOP"));
    puts("PASS: actual compact screen: distinct actual/target values, SPD180, image error, P/D, RUN/WAIT/STOP; only allowed labels within LCD bounds");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="rt1064_simple_display_") as folder:
    build = Path(folder)
    check = build / "check.c"
    check.write_text(PREFIX + block + TEST, encoding="utf-8")
    exe = build / "check.exe"
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(check), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
