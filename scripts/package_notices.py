"""Collect upstream notices from the inputs to the STM32 and Windows release builds."""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("west", "boot-build", "arm-toolchain", "mingw", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    boot = args.west / "bootloader/mcuboot"
    modules = args.west / "modules"
    zephyr = args.west / "zephyr"
    drivers = repo / "firmware/Drivers"
    st = modules / "hal/stm32/stm32cube/stm32f1xx"
    mingw = args.mingw / "licenses"
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    for name, source in {
        "Apache-2.0.txt": zephyr / "LICENSE",
        "MCUboot-LICENSE.txt": boot / "LICENSE",
        "MCUboot-NOTICE.txt": boot / "NOTICE",
        "TinyCrypt.txt": boot / "ext/tinycrypt/LICENSE",
        "zcbor.txt": modules / "lib/zcbor/LICENSE",
        "CMSIS-Core.txt": drivers / "CMSIS/LICENSE.txt",
        "CMSIS-Device-ST.txt": drivers / "CMSIS/Device/ST/STM32F1xx/LICENSE.txt",
        "STM32F1-HAL-terms.txt": drivers / "STM32F1xx_HAL_Driver/LICENSE.txt",
        "STM32F1-HAL-BSD-3-Clause.txt": st / "drivers/LICENSE.md",
        "STM32F1-SoC.txt": st / "soc/LICENSE.md",
        "GNU-Tools-for-STM32-third-party.txt": args.arm_toolchain / "LICENSE.THIRDPARTY",
        "GPL-3.0.txt": mingw / "gcc/COPYING3",
        "GCC-runtime-exception.txt": mingw / "gcc/COPYING.RUNTIME",
        "MinGW-w64-runtime.txt": mingw / "mingw-w64/COPYING.MinGW-w64-runtime.txt",
        "MinGW-w64.txt": mingw / "mingw-w64/COPYING.MinGW-w64.txt",
        "winpthreads.txt": mingw / "winpthreads/COPYING",
    }.items():
        shutil.copyfile(source, out / name)

    # Include the original notices from the compiler inputs, including pulled-in headers.
    build = args.boot_build
    paths = {Path(row["file"]) for row in json.loads((build / "compile_commands.json").read_text())}
    deps = subprocess.check_output(["ninja", "-C", str(build), "-t", "deps"], text=True)
    paths.update(build / line.strip() for line in deps.splitlines() if line.startswith("    "))
    paths.update(p for p in (repo / "firmware").rglob("*") if p.suffix in (".c", ".h", ".s")
                 and "build" not in p.parts[len(repo.parts):])
    roots = [(p.resolve(), name) for p, name in ((zephyr, "Zephyr"), (boot, "MCUboot"),
             (modules, "Zephyr modules"), (repo / "firmware", "Baize firmware"))]
    notices = {}
    for path in paths:
        path = path.resolve()
        for root, name in roots:
            if path.is_relative_to(root) and path.is_file():
                source = path.read_text(encoding="utf-8", errors="replace")
                for block in re.findall(r"/\*.*?\*/|(?:(?://|@)[^\n]*\n)+", source, re.S):
                    if re.search(r"copyright|SPDX-License-Identifier", block, re.I):
                        notices.setdefault(block.strip(), set()).add(f"{name}/{path.relative_to(root)}")
                break
    with (out / "source-notices.txt").open("w", encoding="utf-8") as stream:
        for block, files in sorted(notices.items()):
            stream.write("\n".join(sorted(files)) + "\n" + block + "\n\n")


if __name__ == "__main__":
    main()
