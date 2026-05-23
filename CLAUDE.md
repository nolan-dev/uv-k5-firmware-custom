# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Bare-metal ARM Cortex-M0 firmware for the Quansheng UV-K5/K6/5R radio (DP32G030 SoC, BK4819 transceiver). Fork lineage: DualTachyon's open firmware → OneOfEleven custom → fagci spectrum analyzer → egzumer. No RTOS, no heap to speak of (`_Min_Heap_Size = 0` in `firmware.ld`). Code, rodata, and .data initializers must fit in **60 KB FLASH**; RAM is **16 KB**. Every feature is gated on a compile-time flag because of this — keep that in mind before adding code unconditionally.

## Build

Three build paths, all produce `firmware.bin` and (if `python` + `crcmod` are present) `firmware.packed.bin`:

- **Docker** (most reproducible, smallest binaries — README claims up to ~1 KB savings): `./compile-with-docker.sh` (Linux/Mac) or `compile-with-docker.bat` (Windows). Output goes to `compiled-firmware/`.
- **Native Windows**: `win_make.bat`. Expects `gcc-arm-none-eabi-10.3-2021.10` installed at the default `C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\` and GnuWin32 make on PATH (the script prepends these).
- **Native Linux / WSL**: `make clean && make`. Requires `arm-none-eabi-gcc` 10.3.1 (Ubuntu 22.04's default) and `pip install crcmod` for packing.

Useful Makefile knobs (override on the command line, e.g. `make ENABLE_VOX=1`):

- `ENABLE_*` flags at the top of `Makefile` — '0'/'1'. Each one both gates an object in `OBJS` and emits a matching `-DENABLE_*` to the compiler. **If you add a new flag, you must update both halves.**
- `ENABLE_LTO` and `ENABLE_OVERLAY` are mutually exclusive (the Makefile forces this). `ENABLE_CLANG=1` also forces `ENABLE_LTO=0` because GCC's `ld` can't read LLVM bitcode.
- `AUTHOR_STRING` (max 7 chars) and `VERSION_STRING` are baked into the binary; `VERSION_STRING` defaults to `git describe`/`git rev-parse --short HEAD`/`NOGIT`.
- `make flash` and `make debug` shell out to a hardcoded `/opt/openocd` with a JLink config — ignore unless you actually have SWD hardware (also requires `ENABLE_SWD=1`).

There are no unit tests and no linter beyond `-Wall -Werror -Wextra`. Verification means flashing a radio.

## Architecture

### Boot and main loop

Entry is `start.S` → `init.c` (copies `.data`, zeros `.bss`) → `Main()` in `main.c`. `Main()` initializes clocks, peripherals, EEPROM-backed settings, the BK4819 transceiver, then enters an infinite loop:

```c
while (true) {
    APP_Update();                          // app/app.c — drives RX/TX, scan, menus
    if (gNextTimeslice)      APP_TimeSlice10ms();
    if (gNextTimeslice_500ms) APP_TimeSlice500ms();
}
```

`scheduler.c::SystickHandler()` fires every 10 ms, increments `gGlobalSysTickCounter`, sets the `gNextTimeslice*` flags, and decrements a pile of `*Countdown_10ms` counters. **This is the only "scheduling" — everything is cooperative.** A long-running loop in `app/` will stall scanning, audio, key repeat, etc. When adding work, hang it off a countdown + flag pattern like the existing modules.

### Layer layout

- **`hardware/dp32g030/*.def`** — chip register layouts written as macros. The Makefile rule `bsp/dp32g030/%.h: hardware/dp32g030/%.def` generates the matching `bsp/` headers; include those (`bsp/dp32g030/gpio.h`, etc.), never the `.def` directly.
- **`driver/`** — peripheral drivers: `bk4819` (main 18 MHz–1.3 GHz transceiver, the heart of the radio), `bk1080` (broadcast FM RX), `st7565` (128×64 LCD), `eeprom`, `adc`, `keyboard`, `uart`, `i2c`, `spi`, `systick`, `backlight`. The BK4819 register map is in `driver/bk4819-regs.h`; AGC, AM-fix, spectrum sweep, modulation switching all poke this chip.
- **`app/`** — top-level state machines and feature modules. `app.c` is the dispatcher (`APP_Update`, `APP_TimeSlice10ms/500ms`). Sibling modules: `main` (VFO screen logic — distinct from `main.c`/`Main()`), `menu`, `scanner`, `chFrScanner` (channel/freq scan), `spectrum` (fagci analyzer), `fm`, `dtmf`, `aircopy`, `flashlight`, `uart` (serial protocol for PC programming).
- **`ui/`** — pure rendering for each screen (`ui/main.c`, `ui/menu.c`, `ui/status.c`, `ui/welcome.c`, etc.) plus `ui/helper.c` (font/bitmap blits to the framebuffer) and `ui/inputbox.c`. UI modules write to a framebuffer; `driver/st7565.c` pushes it to the LCD. Keep render logic out of `app/` and state out of `ui/`.
- **Top-level C files** — `radio.c` (VFO/channel/modulation/squelch wiring around BK4819), `settings.c` (EEPROM layout, calibration, build-options blob), `audio.c`, `frequencies.c` (band tables, step sizes), `functions.c` (the `FUNCTION_*` state — RECEIVE/TRANSMIT/POWER_SAVE/MONITOR/FOREGROUND), `am_fix.c` (dynamic LNA/PGA gain to fight AM saturation), `misc.c` (the big globals soup: `gEeprom`, all the countdown timers, scan state).

### State conventions

- Globals are pervasive and prefixed `g` (e.g. `gEeprom`, `gCurrentFunction`, `gScanStateDir`, `gTxTimerCountdown_500ms`). They are not protected — `SystickHandler` decrements counters and sets flags; mainline code reads and clears them. This works only because the CPU is single-core and the ISR is short.
- Settings persisted across reboots live in `struct EEPROM_Config gEeprom` (see `settings.h`), serialized to the 24Cxx EEPROM via `driver/eeprom.c`. Calibration data is at a separate EEPROM region loaded by `SETTINGS_LoadCalibration()`.
- Boot modes (`BOOT_MODE_NORMAL`, `BOOT_MODE_F_LOCK`, etc.) come from key combos held at power-on — see `helper/boot.c`. `BOOT_MODE_F_LOCK` unlocks hidden menu items past `FIRST_HIDDEN_MENU_ITEM`.

### Adding a feature

1. Pick or add an `ENABLE_FOO` flag in `Makefile`. Gate your `.o` in `OBJS` and add the matching `-DENABLE_FOO` block lower down.
2. Wrap every reference to your code in `#ifdef ENABLE_FOO` — including the `#include`. Other configurations must still build (CI is whoever rebuilds with different flags).
3. After building, check `firmware` size with `arm-none-eabi-size firmware` — if you've blown past ~60 KB the linker will fail. Common ways to claw back bytes: keep strings short, prefer `uint8_t`/`int8_t` for small fields, avoid `printf` family unless you already pulled it in via `external/printf`.

## Firmware packing

`fw-pack.py` XORs the raw `firmware.bin` with a fixed key and adds a CRC header so the stock Quansheng Windows uploader will accept it. The unpacked `firmware.bin` works with `k5prog`-family tools. Both are produced by the default `make` target when `python` + `crcmod` are available.
