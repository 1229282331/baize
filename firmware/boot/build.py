"""Build the Baize bootloader from an external Zephyr v4.4.0 workspace."""
import argparse
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = args.output.resolve()
    port = Path(__file__).resolve().parent
    boot = workspace / "bootloader/mcuboot"
    patch = str(port / "serial-policy.patch")
    # git apply also works with source archives. Refuse an unrelated source revision.
    if (boot / ".git").exists():
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=boot, text=True).strip()
        if revision != "ee39e2d694bd827ffd1bebbce2f571a9154e6ec2":
            raise SystemExit("use the MCUboot revision from Zephyr v4.4.0 west.yml")
    if subprocess.run(["git", "apply", "--reverse", "--check", patch], cwd=boot,
                      stderr=subprocess.DEVNULL).returncode:
        subprocess.run(["git", "apply", "--check", patch], check=True, cwd=boot)
        subprocess.run(["git", "apply", patch], check=True, cwd=boot)
    modules = [boot] + [workspace / p for p in
                        ("modules/hal/cmsis", "modules/hal/cmsis_6", "modules/hal/stm32",
                         "modules/lib/zcbor")]
    env = dict(os.environ, ZEPHYR_BASE=str(workspace / "zephyr"))
    subprocess.run(["cmake", "-G", "Ninja", "-S", str(boot / "boot/zephyr"), "-B", str(output),
                    "-DBOARD=baize_chassis", f"-DBOARD_ROOT={port.as_posix()}",
                    f"-DCONF_FILE={(port / 'boot.conf').as_posix()}",
                    "-DZEPHYR_MODULES=" + ";".join(p.as_posix() for p in modules)], check=True, env=env)
    config = set((output / "zephyr/.config").read_text().splitlines())
    required = {"CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=72000000", "CONFIG_FLASH_SIZE=256",
                "CONFIG_SRAM_SIZE=48", "CONFIG_FLASH_LOAD_SIZE=0x10000",
                "CONFIG_BOOT_SWAP_USING_OFFSET=y", "CONFIG_MCUBOOT_UUID_CID=y",
                "CONFIG_BOOT_WATCHDOG_SETUP_AT_BOOT=y", "CONFIG_BOOT_WATCHDOG_FEED=y"}
    if not required <= config:
        raise SystemExit("incorrect board configuration: " + ", ".join(sorted(required - config)))
    subprocess.run(["cmake", "--build", str(output)], check=True, env=env)


if __name__ == "__main__":
    main()
