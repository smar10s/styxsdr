# Tools — offline (fully static on ARM, no IIO dependency)

add_executable(rx_file rx_file.c)
target_link_libraries(rx_file PRIVATE lib80211)

add_executable(gen_test_cf32 gen_test_cf32.c)
target_link_libraries(gen_test_cf32 PRIVATE lib80211)

add_executable(tx_beacon tx_beacon.c)
target_link_libraries(tx_beacon PRIVATE lib80211)

if(NOT APPLE)
    target_link_libraries(rx_file PRIVATE m)
    target_link_libraries(gen_test_cf32 PRIVATE m)
    target_link_libraries(tx_beacon PRIVATE m)
endif()

# Static linking for offline tools on ARM (glibc version mismatch workaround)
if(CMAKE_CROSSCOMPILING)
    foreach(tgt rx_file gen_test_cf32 tx_beacon)
        target_link_options(${tgt} PRIVATE -static)
    endforeach()
endif()

# Tools — OTA (require libiio, dynamically linked on ARM)
option(LIB80211_BUILD_OTA "Build OTA tools (require libiio)" OFF)

if(LIB80211_BUILD_OTA)
    find_path(IIO_INCLUDE_DIR iio.h)
    find_library(IIO_LIBRARY iio)

    if(NOT IIO_INCLUDE_DIR OR NOT IIO_LIBRARY)
        message(FATAL_ERROR "libiio not found. Install libiio-dev or set IIO paths.")
    endif()

    # OTA tools link dynamically — libiio.so + glibc live on the device.
    # The device has glibc 2.25 which lacks GLIBC_2.29 log/logf.
    # We intercept those calls with --wrap and a local shim.
    add_executable(ota_rx ota_rx.c)
    target_include_directories(ota_rx PRIVATE ${PROJECT_SOURCE_DIR}/include ${IIO_INCLUDE_DIR})
    target_link_libraries(ota_rx PRIVATE lib80211 ${IIO_LIBRARY})

    add_executable(ota_tx_beacon ota_tx_beacon.c)
    target_include_directories(ota_tx_beacon PRIVATE ${PROJECT_SOURCE_DIR}/include ${IIO_INCLUDE_DIR})
    target_link_libraries(ota_tx_beacon PRIVATE lib80211 ${IIO_LIBRARY})

    if(NOT APPLE)
        target_link_libraries(ota_rx PRIVATE m)
        target_link_libraries(ota_tx_beacon PRIVATE m)
    endif()

    if(CMAKE_CROSSCOMPILING)
        # glibc 2.25 compat: wrap log/logf with local shim
        add_library(compat_glibc OBJECT compat_glibc225.c)
        target_compile_options(compat_glibc PRIVATE -fno-builtin)

        foreach(tgt ota_rx ota_tx_beacon)
            target_sources(${tgt} PRIVATE $<TARGET_OBJECTS:compat_glibc>)
            target_link_options(${tgt} PRIVATE
                -Wl,--wrap=log
                -Wl,--wrap=logf
                -Wl,--allow-shlib-undefined
            )
        endforeach()
    endif()
endif()

# Test: generate a beacon cf32 and decode it (only if tests are enabled)
if(LIB80211_BUILD_TESTS)
    add_test(NAME test_rx_file_roundtrip
        COMMAND ${CMAKE_COMMAND} -E env bash -c
        "$<TARGET_FILE:gen_test_cf32> /tmp/test_beacon_rx.cf32 && $<TARGET_FILE:rx_file> -q /tmp/test_beacon_rx.cf32 | grep -q '\"fcs_ok\": true'"
    )
endif()
