# 0) Goals & Scope
- Build **aarch64** user-space apps for **Raspberry Pi OS 64-bit** from an **Ubuntu host**.
- Keep a **clean directory layout** under `~/rpi-dev/`.
- Use **CMake + Ninja**, **aarch64 cross-GCC**, a **Pi sysroot** (headers/libs rsynced from the device), and **pkg-config** wired to that sysroot.
- Support **Debug / Release** builds, **scp/rsync deploy**, and **remote gdbserver** debugging.
- Keep the sysroot reasonably **lean**, expandable later.
---
# 1) Host (Ubuntu) one-time setup
```bash
sudo apt update
sudo apt install \
  gcc-aarch64-linux-gnu g++-aarch64-linux-gnu gdb-multiarch \
  cmake ninja-build rsync pkg-config ccache file
```
Notes:
- `gcc-aarch64-linux-gnu` / `g++-aarch64-linux-gnu` = cross compilers.
- `gdb-multiarch` = debugger for foreign architectures.
- `rsync` syncs the Pi sysroot.
- `pkg-config` provides correct compile/link flags from `.pc` files.
- `ccache` speeds up rebuilds.
---
# 2) Pi (target) preparation
On the Pi (hostname example: `hydrapi.local`):
```bash
sudo apt update
# runtime basics
sudo apt install libc6 libstdc++6 libgcc-s1 ca-certificates zlib1g libssl3
# development headers used by your app (extend later as needed)
sudo apt install build-essential pkg-config zlib1g-dev libssl-dev
```
Whenever you add more libs later (OpenSSL already shown; later maybe OpenCV, GStreamer, BlueZ, libcamera, etc.), install their `*-dev` packages **on the Pi first**, then re-sync the sysroot (next step).
---
# 3) Directory layout on the host
```
~/rpi-dev/
├── hydrapi-env.sh            # environment activator for Pi-4 builds
├── hydrapi-sysroots/         # rsynced sysroot (target headers/libs/pkgconfig)
└── toolchains/
  └── rpi-aarch64.cmake     # CMake toolchain
```
You can add more boards later (e.g., `pi5-env.sh`, `pi5-sysroots/`) but reuse the same `toolchains/`.
---
# 4) Minimal sysroot sync script (lean but sufficient)
Create: `~/rpi-dev/sync-sysroot.sh` and make it executable (`chmod +x`).
```bash
#!/usr/bin/env bash
# Minimal Raspberry Pi sysroot sync (Pi-4 aarch64)
# Syncs only what cross builds need; extend SYNC_ITEMS if required.
set -euo pipefail
DEVHOME="$HOME/rpi-dev"
SYSROOT="$DEVHOME/hydrapi-sysroots"
RPI_REMOTE="pi@hydrapi.local"   # change if needed
# Lean set: headers, libs, pkg-config
SYNC_ITEMS=(
  "/usr/include"
  "/usr/lib/aarch64-linux-gnu"
  "/lib/aarch64-linux-gnu"
  "/usr/lib/pkgconfig"
  "/usr/share/pkgconfig"
)
RSYNC_FLAGS=(-a --delete)
EXCLUDES=(--exclude="/dev/*" --exclude="/proc/*" --exclude="/sys/*"
        --exclude="/tmp/*" --exclude="/run/*" --exclude="/mnt/*"
        --exclude="/media/*" --exclude="/lost+found" --exclude="/boot/*"
        --exclude="/var/*" --exclude="/etc/*")
echo "[sync] SYSROOT      = $SYSROOT"
echo "[sync] RPI_REMOTE   = $RPI_REMOTE"
mkdir -p "$SYSROOT"
for SRC in "${SYNC_ITEMS[@]}"; do
  DEST_DIR="$SYSROOT$(dirname "$SRC")"
  echo "[sync] $RPI_REMOTE:$SRC -> $DEST_DIR/"
  mkdir -p "$DEST_DIR"
  rsync "${RSYNC_FLAGS[@]}" "${EXCLUDES[@]}" "$RPI_REMOTE:$SRC" "$DEST_DIR/"
done
echo "[sync] Done. Sysroot at: $SYSROOT"
```
- This **avoids `/etc`** and other privileged/virtual paths (no `sudoers` permission errors).
- Expect size ≈ **1–3 GB** depending on what's installed on the Pi — normal for a useful sysroot.
- Extend `SYNC_ITEMS` later (e.g., `"/usr/lib/cmake"`, `"/usr/local/include"`, `"/usr/local/lib"`, `"/opt/vc"` if you ever need them).
Run it:
```bash
~/rpi-dev/sync-sysroot.sh
```
Re-run after you install new `*-dev` packages on the Pi.
---
# 5) Environment activator (`hydrapi-env.sh`)
Create: `~/rpi-dev/hydrapi-env.sh`
```bash
#!/usr/bin/env bash
set -euo pipefail
# Central dev root
export DEVHOME="$HOME/rpi-dev"
# Sysroot for this Pi
export SYSROOT="$DEVHOME/hydrapi-sysroots"
# Cross compilers
export CC=aarch64-linux-gnu-gcc
export CXX=aarch64-linux-gnu-g++
# pkg-config points into the sysroot
export PKG_CONFIG_DIR=
export PKG_CONFIG_SYSROOT_DIR="$SYSROOT"
export PKG_CONFIG_LIBDIR="$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/share/pkgconfig"
# Optional: ccache
export CC="ccache $CC"
export CXX="ccache $CXX"
# CPU tuning (Pi-4 Cortex-A72)
export RPI_CFLAGS="-mcpu=cortex-a72 -pipe"
export RPI_LDFLAGS="-mcpu=cortex-a72"
# Toolchain file path (for convenience)
export RPI_TOOLCHAIN="$DEVHOME/toolchains/rpi-aarch64.cmake"
echo "[env] DEVHOME=$DEVHOME"
echo "[env] SYSROOT=$SYSROOT"
echo "[env] TOOLCHAIN=$RPI_TOOLCHAIN"
```
Activate it in each shell before configuring:
```bash
source ~/rpi-dev/hydrapi-env.sh
```
---
# 6) CMake toolchain file (`toolchains/rpi-aarch64.cmake`)
Create: `~/rpi-dev/toolchains/rpi-aarch64.cmake`
```cmake
# Minimal, clean toolchain: only cross environment; dependencies belong in your project CMakeLists.txt
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT "$ENV{SYSROOT}")
set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
add_compile_options(--sysroot=${CMAKE_SYSROOT} $ENV{RPI_CFLAGS})
add_link_options(--sysroot=${CMAKE_SYSROOT} $ENV{RPI_LDFLAGS})
```
> Keep the toolchain **pure**: no `find_package(Threads)` here — put dependencies in your project CMake.
---
# 7) Your project CMake (example)
**Top-level `CMakeLists.txt`:**
```cmake
cmake_minimum_required(VERSION 3.20)
project(hello_pi CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(hello src/main.cpp)
# Example dependency declarations belong here (not in toolchain!)
find_package(Threads REQUIRED)
target_link_libraries(hello PRIVATE Threads::Threads)
```
**`src/main.cpp`:**
```cpp
#include <iostream>
#include <thread>
int main() {
  std::cout << "Hello from Pi4 aarch64!\n";
  std::thread t([]{ std::cout << "Thread works.\n"; });
  t.join();
  return 0;
}
```
---
# 8) Configure & build (Debug / Release)
> Don’t bake `-O2` in the env; let **CMake build types** control optimization.
```bash
source ~/rpi-dev/hydrapi-env.sh
# Debug (-g -O0 or -Og if you override)
cmake -S . -B build-debug -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=$RPI_TOOLCHAIN \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug -j
# Release (-O3 -DNDEBUG by default; you can change to -O2)
cmake -S . -B build-release -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=$RPI_TOOLCHAIN \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-release -j
```
**Optional**: Prefer `Release = -O2`? Add in your project CMake:
```cmake
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)
endif()
```
---
# 9) Deploy & run
```bash
scp build-release/hello pi@hydrapi.local:/home/pi/hello
ssh pi@hydrapi.local ./hello
```
If runtime can’t find a shared lib, check:
- The lib actually exists on Pi (`ldd ./hello`).
- Your binary’s `rpath` isn’t mandatory here; the default system lib paths on Pi are fine for standard libs.
---
# 10) Remote debugging
On the **Pi**:
```bash
gdbserver :1234 ./hello
```
On the **host**:
```bash
gdb-multiarch build-release/hello
(gdb) set architecture aarch64
(gdb) target remote hydrapi.local:1234
(gdb) b main
(gdb) c
```
**VS Code**: use `C/C++` (ms-vscode.cpptools) with `cppdbg`, set:
- `miDebuggerPath: "gdb-multiarch"`
- `miDebuggerServerAddress: "hydrapi.local:1234"`
- Program path = your host-built binary (`build-.../hello`), source mapping normally not needed if paths match.
---
# 11) Adding dependencies later (the cycle)
1. On the **Pi**: `sudo apt install <libname>-dev`
2. On the **host**: re-run `~/rpi-dev/sync-sysroot.sh`
3. In your **project CMake**:
   - Use `find_package(<Pkg> REQUIRED)` if the library provides CMake config files and they’re in the sysroot (sometimes under `/usr/lib/cmake` → add to SYNC_ITEMS if needed).
   - Or use `pkg_check_modules()` (from `FindPkgConfig`) / `pkg-config`-style and link the resulting `CFLAGS`/`LDFLAGS`.
   - Or link manually (`target_include_directories`, `target_link_libraries`) if no `.pc` file is present.
---
# 12) Common pitfalls & fixes
- **Link errors `__atomic_*`**
  Rare on aarch64; if you hit them (unusual wide atomics), add `-latomic` to link flags or `target_link_libraries(... atomic)`.
- **“wrong ELF class: ELFCLASS64/32”**
  You linked x86_64 libs by accident. Ensure `PKG_CONFIG_LIBDIR` points **only** to the sysroot’s aarch64 directories and you passed `--sysroot` (we do).
- **“not found: libXYZ.so” on Pi**
  Install the runtime package on the Pi (`apt install libxyz`), not just the `-dev`. Re-check `ldd ./yourbin`.
- **pkg-config finds host libs**
  Always activate `hydrapi-env.sh` first; those `PKG_CONFIG_*` vars must point into the sysroot.
- **Huge sysroot**
  You already use the lean set. Only add dirs when needed (e.g., `"/usr/lib/cmake"`).
---
# 13) Sanity checks (quick)
```bash
# Check compilers
aarch64-linux-gnu-gcc -v
aarch64-linux-gnu-g++ -v
# Check sysroot has key dirs
test -d "$SYSROOT/usr/include" && echo OK include
test -d "$SYSROOT/usr/lib/aarch64-linux-gnu" && echo OK libs
# pkg-config uses target
PKG_CONFIG_SYSROOT_DIR="$SYSROOT" \
PKG_CONFIG_LIBDIR="$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/share/pkgconfig" \
pkg-config --cflags --libs zlib
```
---
# 14) Multi-Pi scaling (later)
- Duplicate the env script: `pi5-env.sh` with `SYSROOT="$DEVHOME/pi5-sysroots"` and different CPU flags (e.g., `-mcpu=cortex-a76` for Pi 5 or keep `-mcpu=cortex-a72 -moutline-atomics` for Pi4/5 portability).
- Reuse `toolchains/rpi-aarch64.cmake`.
- Keep one sysroot directory **per image** to avoid mixing libs.
---
## TL;DR
- **Install** cross toolchain on Ubuntu.
- **Sync** a lean **sysroot** from the Pi: headers + aarch64 libs + pkgconfig.
- **Activate** env (`hydrapi-env.sh`) to point compilers and `pkg-config` into the sysroot.
- **Use** a clean **toolchain.cmake** (just environment; no deps).
- **Declare deps** in your project CMake (e.g., `Threads`, later OpenSSL/Zlib/etc.).
- **Build** Debug/Release via `-DCMAKE_BUILD_TYPE`.
- **Deploy** with `scp`, **debug** with `gdbserver` or VS Code remote.
If you want, I can package this into a tiny starter repo (folders + scripts + a "hello" project) so you can drop it into place and run immediately.
