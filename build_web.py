#!/usr/bin/env python3
"""
Build the goniometer web app and copy the output into spiffs_image/.

Usage:
    python build_web.py
"""

import subprocess
import sys
import shutil
from pathlib import Path

WEB_DIR     = Path(r"C:\Users\njtan\Documents\GitHub\goniometer\web")
DIST_DIR    = WEB_DIR / "dist-esp32"
SPIFFS_DIR  = Path(__file__).parent / "spiffs_image"

def run(cmd, cwd):
    print(f"\n> {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, shell=True)
    if result.returncode != 0:
        print(f"Error: command exited with code {result.returncode}")
        sys.exit(result.returncode)

def main():
    # 1. Build
    run(["npm", "run", "build:esp32"], cwd=WEB_DIR)

    if not DIST_DIR.exists():
        print(f"Error: expected build output at {DIST_DIR}")
        sys.exit(1)

    # 2. Clear spiffs_image/, recreate it
    if SPIFFS_DIR.exists():
        shutil.rmtree(SPIFFS_DIR)
    SPIFFS_DIR.mkdir()

    # 3. Copy dist-esp32/ → spiffs_image/, skipping the C header
    skip = {"web_assets.h"}
    copied = 0
    for src in DIST_DIR.rglob("*"):
        if src.is_dir() or src.name in skip:
            continue
        dst = SPIFFS_DIR / src.relative_to(DIST_DIR)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"  {src.relative_to(DIST_DIR)}")
        copied += 1

    print(f"\nCopied {copied} file(s) to {SPIFFS_DIR}")
    print("\nNext: flash the SPIFFS image to the external chip:")
    print("  idf.py build")
    print("  esptool.py --chip esp32c3 --port <PORT> --spi-connection 4,5,6,7 write_flash 0x0 build\\spiffs_ext.bin")

if __name__ == "__main__":
    main()
