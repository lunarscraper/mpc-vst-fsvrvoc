#!/usr/bin/env bash
# Offline x86 test (ASan/UBSan) of the in-process build, then a benchmark of every factory
# performance (-O2): run after build.sh (needs build/params.h, build/popup.h). Prints
# PASSED/FAILED; exit code follows. The benchmark numbers are for the CI machine's x86 core: on the
# Force's Cortex-A17 expect several times more - /tmp/fsvr_vst.log on the device has the real load.
set -euo pipefail
cd "$(dirname "$0")"
docker run --rm -v "$PWD/..":/b -w /b/vst gcc:12 bash -euc 'set -o pipefail
  ENG="../src/fs1r/chips/ymp706.cpp ../src/fs1r/firmware/*.cpp ../src/fsvr/device.cpp ../src/fsvr/fastmath.cpp ../src/fsvr/selftest.cpp"
  # -fno-sanitize=shift: FSVR notes.cpp shifts a negative pitch-EG level left (arithmetic on gcc, harmless)
  SAN="-fsanitize=address,undefined -fno-sanitize=shift"
  mkdir -p build/x86 build/x86o
  for f in $ENG; do g++ -O0 -g $SAN -std=c++17 -DFSVR_MATH=1 -w -fPIC -I../src -c $f -o build/x86/$(basename $f .cpp).o; done
  g++ -O0 -g $SAN -std=c++17 -DFSVR_MATH=1 -fPIC -Ibuild -I../src -c fsvr_vst.cpp -o build/x86/vst.o
  g++ -c banks.S -o build/x86/banks.o -Wa,-I,../data
  g++ -shared $SAN -o build/x86/fsvr-x86.so build/x86/*.o -lpthread -ldl
  g++ -O0 -g $SAN -std=c++17 -o build/x86/host_test host_test.cpp -ldl
  ASAN_OPTIONS=detect_leaks=0 ./build/x86/host_test ./build/x86/fsvr-x86.so

  for f in $ENG; do g++ -O2 -std=c++17 -DFSVR_MATH=1 -w -fPIC -fvisibility=hidden -I../src -c $f -o build/x86o/$(basename $f .cpp).o; done
  g++ -O2 -std=c++17 -DFSVR_MATH=1 -fPIC -fvisibility=hidden -Ibuild -I../src -c fsvr_vst.cpp -o build/x86o/vst.o
  g++ -c banks.S -o build/x86o/banks.o -Wa,-I,../data
  g++ -shared -o build/x86o/fsvr-x86.so build/x86o/*.o -lpthread
  g++ -O2 -std=c++17 -o build/x86o/host_test host_test.cpp -ldl
  ./build/x86o/host_test ./build/x86o/fsvr-x86.so bench
'
