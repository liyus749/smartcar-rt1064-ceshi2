"""Build the real vision modules with a host-only hardware stub. Requires GCC."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = """
#ifndef HOST_ZF_COMMON_HEADFILE_H
#define HOST_ZF_COMMON_HEADFILE_H
#include <stdint.h>
#include <stddef.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int32_t int32;
#define MT9V03X_H 100
#define MT9V03X_W 188
#define IMG_BLACK 0
#define IMG_WHITE 255
#endif
"""

with tempfile.TemporaryDirectory(prefix="ceshi_vision_tests_") as directory:
    build = Path(directory)
    (build / "zf_common_headfile.h").write_text(STUB, encoding="utf-8")
    executable = build / "vision_tests.exe"
    subprocess.run([
        "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(build), "-I", str(ROOT / "project/code"),
        str(ROOT / "tests/vision_host.c"),
        *[str(ROOT / "project/code" / name)
          for name in ("LineDetect.c", "Cross.c", "Err.c")],
        "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
