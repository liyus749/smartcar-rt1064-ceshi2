"""Check real grayscale -> LineDetect -> retained VIS7 with generated geometry."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
PROJECT_DIR = ROOT / 'project/mdk'
MAIN = ET.parse(PROJECT_DIR/'rt1064.uvprojx').find(".//Files/File[FileName='main.c']/FilePath").text
CODE = (PROJECT_DIR/MAIN.replace('\\','/')).resolve().parents[2]/'code'
STUB = '''
#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int32_t int32;
#define MT9V03X_H 100
#define MT9V03X_W 188
#define IMG_BLACK 0
#define IMG_WHITE 255
void Draw_Line(int,int,int,int);
#endif
'''
with tempfile.TemporaryDirectory(prefix='ring_vision_') as directory:
    build = Path(directory)
    (build/'zf_common_headfile.h').write_text(STUB)
    executable = build/'test.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(build),
                    '-I',str(CODE),str(ROOT/'tests/island_vision_host.c'),str(CODE/'LineDetect.c'),
                    '-o',str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
