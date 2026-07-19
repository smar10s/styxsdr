cmake_minimum_required(VERSION 3.20)
project(lib80211
    VERSION 0.2.0
    LANGUAGES C
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Sanitizers (for development/CI)
option(LIB80211_SANITIZERS "Enable ASan + UBSan" OFF)
if(LIB80211_SANITIZERS)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# FFT backend selection: vdsp (macOS) or fftw3 (Linux/ARM)
if(NOT DEFINED LIB80211_FFT_BACKEND)
    if(APPLE)
        set(LIB80211_FFT_BACKEND "vdsp")
    else()
        set(LIB80211_FFT_BACKEND "fftw3")
    endif()
endif()

message(STATUS "FFT backend: ${LIB80211_FFT_BACKEND}")

if(LIB80211_FFT_BACKEND STREQUAL "vdsp")
    if(NOT APPLE)
        message(FATAL_ERROR "vDSP backend requires macOS/iOS")
    endif()
    find_library(ACCELERATE_FW Accelerate REQUIRED)
elseif(LIB80211_FFT_BACKEND STREQUAL "fftw3")
    # Find FFTW3 single-precision (libfftw3f)
    find_library(FFTW3F_LIB fftw3f REQUIRED)
    find_path(FFTW3F_INCLUDE fftw3.h)
    if(NOT FFTW3F_INCLUDE)
        message(FATAL_ERROR "fftw3.h not found. Install FFTW3 (brew install fftw / apt install libfftw3-dev)")
    endif()
else()
    message(FATAL_ERROR "Unknown FFT backend: ${LIB80211_FFT_BACKEND}. Use 'vdsp' or 'fftw3'.")
endif()

include(GNUInstallDirs)

add_subdirectory(src)

# Tests
option(LIB80211_BUILD_TESTS "Build test suite" ON)
if(LIB80211_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Tools
add_subdirectory(tools)

# ─── Install rules ───────────────────────────────────────────────────────────

install(TARGETS lib80211
    EXPORT lib80211Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/lib80211
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT lib80211Targets
    FILE lib80211Targets.cmake
    NAMESPACE lib80211::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lib80211
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/lib80211Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/lib80211Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lib80211
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/lib80211ConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/lib80211Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/lib80211ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lib80211
)
