# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(styx-firmware C)

set(CMAKE_C_STANDARD 11)

# ---- Styx HAL library ----
add_library(styx_hal STATIC
    src/hal.c
    src/dma_rx.c
    src/dma_tx.c
    src/convert.c
)
target_include_directories(styx_hal PUBLIC src/)

# ---- On-device tools (ARM only) ----
if(NOT STYX_HOST_BUILD)
    # Link statically for deployment to Pluto
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

    # Tools with no lib80211 dependency
    add_executable(hil_inject tools/hil_inject.c)
    target_link_libraries(hil_inject PRIVATE styx_hal m)

    add_executable(pluto_dma_test tools/platform/pluto_dma_test.c)
    target_link_libraries(pluto_dma_test PRIVATE styx_hal m)

    # Tools requiring lib80211
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../extern/lib80211/CMakeLists.txt")
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../extern/lib80211
                         ${CMAKE_CURRENT_BINARY_DIR}/lib80211)

        add_executable(adc_capture tools/adc_capture.c)
        target_link_libraries(adc_capture PRIVATE styx_hal lib80211 m)

        add_executable(pluto_sigladder tools/platform/pluto_sigladder.c)
        target_link_libraries(pluto_sigladder PRIVATE styx_hal lib80211 fftw3f m)

        add_executable(pluto_tx_beacon tools/platform/pluto_tx_beacon.c)
        target_link_libraries(pluto_tx_beacon PRIVATE styx_hal lib80211 m)

        add_executable(pluto_loopback tools/platform/pluto_loopback.c)
        target_link_libraries(pluto_loopback PRIVATE styx_hal lib80211 m)

        add_executable(pluto_burst_loopback tools/platform/pluto_burst_loopback.c)
        target_link_libraries(pluto_burst_loopback PRIVATE styx_hal lib80211 m)
    endif()
endif()

# ---- Host-native tests ----
if(STYX_HOST_BUILD)
    enable_testing()

    # Stub HAL for host compilation
    add_library(styx_hal_stub STATIC test/stubs.c src/convert.c)
    target_include_directories(styx_hal_stub PUBLIC src/)

    # Host-native unit tests for data conversion layer
    add_executable(test_convert test/test_convert.c)
    target_link_libraries(test_convert PRIVATE styx_hal_stub m)
    add_test(NAME test_convert COMMAND test_convert)
endif()
