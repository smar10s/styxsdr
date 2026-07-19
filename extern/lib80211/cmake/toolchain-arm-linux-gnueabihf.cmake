# CMake toolchain file for ADALM-Pluto (ARM Cortex-A9, hard-float)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_AR arm-linux-gnueabihf-ar)

# Cortex-A9 with NEON (Pluto's Zynq 7010)
set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard")

# NOTE: We do NOT set -static globally here. Instead, offline tools opt into
# static linking via target properties. OTA tools link dynamically (libiio.so
# lives on the device, and we can't statically link against glibc 2.25 anyway
# with a 2.35 toolchain — but the Pluto's own dynamic linker handles it fine
# since we only depend on libiio + basic glibc symbols).

set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Force FFTW3 backend (NEON-enabled, cross-compiled in Docker)
set(LIB80211_FFT_BACKEND "fftw3" CACHE STRING "FFT backend" FORCE)

# Enable OTA tools for Pluto builds (libiio available on device)
set(LIB80211_BUILD_OTA ON CACHE BOOL "Build OTA tools" FORCE)
