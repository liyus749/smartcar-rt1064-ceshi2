"""Build production LineDetect/Cross/Roundabout/Err against synthetic binary/ground scenes."""
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
STUB='''#ifndef HOST_COMMON_H
#define HOST_COMMON_H
#include <stdint.h>
#include <stddef.h>
typedef uint8_t uint8;typedef uint16_t uint16;typedef uint32_t uint32;typedef int32_t int32;
#define MT9V03X_H 100
#define MT9V03X_W 188
#define IMG_WHITE 255
#define IMG_BLACK 0
#endif
'''
with tempfile.TemporaryDirectory(prefix="rt1064_roundabout_") as directory:
    build=Path(directory)
    (build/'zf_common_headfile.h').write_text(STUB)
    executable=build/'roundabout.exe'
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-I',str(build),
                    '-I',str(ROOT/'project/code'),str(ROOT/'tests/roundabout_host.c'),
                    *[str(ROOT/'project/code'/name) for name in ('LineDetect.c','Cross.c','Roundabout.c','Err.c')],
                    '-o',str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
