"""Exercise the production frame snapshot during simulated camera buffer changes."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "project/user/src/main.c").read_text(encoding="utf-8")
snapshot = source.split("// BEGIN FRAME SNAPSHOT\n", 1)[1].split("// END FRAME SNAPSHOT", 1)[0]
prefix = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
#define MT9V03X_H 100
#define MT9V03X_W 188
static uint8 buffers[2][MT9V03X_H][MT9V03X_W];
static uint8 frame_gray[MT9V03X_H][MT9V03X_W];
static uint8 (*mt9v03x_image)[MT9V03X_W] = buffers[0];
static volatile uint8 mt9v03x_finish_flag;
static uint32 irq_mask;
static int swap_during_copy;
static uint32 __get_PRIMASK(void) { return irq_mask; }
static void __disable_irq(void) { irq_mask = 1; }
static void __set_PRIMASK(uint32 value) { irq_mask = value; }
static void __DMB(void) { }
static void *camera_copy(void *dst, const void *src, size_t count)
{
    /* Copying must not suppress the normal control/camera interrupts. */
    assert(irq_mask == 0);
    memcpy(dst, src, count/2);
    if(swap_during_copy)
    {
        swap_during_copy = 0;
        mt9v03x_image = buffers[1];
        mt9v03x_finish_flag = 1;
    }
    memcpy((uint8 *)dst+count/2,(const uint8 *)src+count/2,count-count/2);
    return dst;
}
#define memcpy camera_copy
"""
suffix = r"""
#undef memcpy
int main(void)
{
    memset(buffers[0],33,sizeof(buffers[0]));
    memset(buffers[1],211,sizeof(buffers[1]));
    assert(!Frame_TakeSnapshot() && irq_mask==0);
    irq_mask=1;
    assert(!Frame_TakeSnapshot() && irq_mask==1); /* do not unexpectedly re-enable interrupts */
    irq_mask=0;
    mt9v03x_finish_flag=1;
    assert(Frame_TakeSnapshot() && !mt9v03x_finish_flag);
    assert(!memcmp(frame_gray,buffers[0],sizeof(frame_gray)));
    /* The frozen image survives later DMA/camera updates. */
    memset(buffers[0],99,sizeof(buffers[0]));
    assert(frame_gray[0][0]==33 && frame_gray[99][187]==33);
    mt9v03x_finish_flag=1;swap_during_copy=1;
    assert(!Frame_TakeSnapshot() && mt9v03x_finish_flag);
    assert(Frame_TakeSnapshot() && !mt9v03x_finish_flag);
    assert(!memcmp(frame_gray,buffers[1],sizeof(frame_gray)));
    puts("PASS: production snapshot, frozen content, interrupt mask preservation, concurrent swap rejection and retry");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="rt1064_frame_sync_") as directory:
    build=Path(directory)
    cfile=build/"frame_sync.c"
    cfile.write_text(prefix+snapshot+suffix,encoding="utf-8")
    executable=build/"frame_sync.exe"
    subprocess.run(["gcc","-std=c99","-O2","-Wall","-Wextra","-Werror",str(cfile),"-o",str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
