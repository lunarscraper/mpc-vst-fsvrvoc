#!/usr/bin/env bash
# Build FSVR as a VST2 instrument for the MPC OS plugin host (armhf), engine IN-PROCESS.
#   vst/build/fsvr.so               -> /sdcard/vst/ on the device
#   vst/build/pluginlist-entry.xml  the <PLUGIN> line for MPC.settings' pluginList-arm
#   vst/build/skin/                 -> /sdcard/Synths/ on the device
# Same pattern as mpc-vst-euclidier: FSVR's engine (../src, see ../src/VENDORED.md) is compiled
# straight into the .so together with fsvr_vst.cpp; the factory banks (../data) are embedded by
# banks.S. No JUCE, no GUI, no ALSA. vst.json/module.json only feed tools/gen_vst.py for params.h
# + the skin. Offline test + benchmark: test.sh.
set -euo pipefail
cd "$(dirname "$0")"
MPC_VST="$(cd "${MPC_VST:-../../mpc-vst}" && pwd)"
U="$(id -u):$(id -g)"
mkdir -p build

# 1. skin artwork renderer (host binary; the renderer vendored in mpc-vst-plugins)
if command -v gcc >/dev/null; then
  gcc -O2 -I"$MPC_VST/tools/vendor/force-shadow/tools" -o build/shadow_art "$MPC_VST/tools/shadow_art.c" -lm
else
  docker run --rm -u "$U" -v "$PWD":/w -v "$MPC_VST":/mv:ro -w /w gcc:12 \
    gcc -O2 -I/mv/tools/vendor/force-shadow/tools -o build/shadow_art /mv/tools/shadow_art.c -lm
fi

# 2. params.h, skin, pluginlist-entry.xml (needs Pillow)
if python3 -c "import PIL" 2>/dev/null; then
  python3 "$MPC_VST/tools/gen_vst.py" vst.json
else
  docker run --rm -u "$U" -v "$PWD":/w -v "$MPC_VST":/mv:ro -w /w python:3.11-slim sh -c \
    "pip install -q --no-warn-script-location --target /tmp/p pillow >/dev/null 2>&1; PYTHONPATH=/tmp/p python3 /mv/tools/gen_vst.py vst.json"
fi
cp "$MPC_VST/wrapper/popup.h" build/   # popup open-flag handling shared with mpc-vst's own wrapper

# 3. the plugin (armhf, glibc 2.36 so it loads on the device's 2.39).
#    Every first-generation MPC OS unit is a Rockchip RK3288 (Cortex-A17, NEON/VFPv4), hence the
#    tuning flags. FSVR_MATH=1 is FSVR's own default sample-loop maths backend (lookup tables).
#    QEMU makes this slow (a few minutes); the engine files compile in parallel.
docker run --rm --platform linux/arm/v7 -v "$PWD/..":/b -w /b/vst arm32v7/gcc:12 bash -euxc '
  CXXF="-O2 -fPIC -fvisibility=hidden -std=c++17 -DFSVR_MATH=1 -mtune=cortex-a17 -mfpu=neon-vfpv4 -mfloat-abi=hard -fno-math-errno"
  mkdir -p build/obj
  ls ../src/fs1r/chips/ymp706.cpp ../src/fs1r/firmware/*.cpp ../src/fsvr/device.cpp ../src/fsvr/fastmath.cpp \
     ../src/fsvr/selftest.cpp | xargs -P "$(nproc)" -I{} sh -c \
     "g++ $CXXF -w -I../src -c {} -o build/obj/\$(basename {} .cpp).o"
  g++ $CXXF -Wall -Wextra -Wno-unused-parameter -Ibuild -I../src -c fsvr_vst.cpp -o build/obj/vst.o
  g++ -c banks.S -o build/obj/banks.o -Wa,-I,../data
  g++ -shared -o build/fsvr.so build/obj/*.o \
      -static-libstdc++ -static-libgcc -lpthread -lm -Wl,--no-undefined
  strip build/fsvr.so
  echo "-- size --"; ls -la build/fsvr.so
  echo "-- needed --"; readelf -d build/fsvr.so | grep NEEDED
  echo "-- highest glibc (device has 2.39) --"; readelf -V build/fsvr.so | grep -o "GLIBC_[0-9.]*" | sort -uV | tail -1
  chown -R '"$U"' build
'
md5sum build/fsvr.so
