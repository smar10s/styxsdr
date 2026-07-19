# Test suite for lib80211

# Common test utilities + vendored cJSON
add_library(test_util STATIC
    test_util.c
    vendor/cJSON.c
)
target_include_directories(test_util PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(test_util PUBLIC lib80211)
# cJSON uses some features that trigger warnings
target_compile_options(test_util PRIVATE -Wno-unused-parameter)

# Pass vectors directory to tests at configure time
set(VECTORS_DIR "${CMAKE_SOURCE_DIR}/vectors")

# Macro to add a test executable
macro(add_lib80211_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE test_util lib80211 m)
    target_compile_definitions(${name} PRIVATE
        VECTORS_DIR="${VECTORS_DIR}"
    )
    add_test(
        NAME ${name}
        COMMAND ${name}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endmacro()

# FFT tests
add_lib80211_test(test_fft)

# Scratch buffer allocator
add_lib80211_test(test_scratch)

# Phase 1: Legacy TX building blocks
add_lib80211_test(test_scrambler)
add_lib80211_test(test_conv_encode)
add_lib80211_test(test_interleaver)
add_lib80211_test(test_modulation)
add_lib80211_test(test_ofdm)
add_lib80211_test(test_tx_legacy)

# Phase 2: HT TX
add_lib80211_test(test_htsig)
add_lib80211_test(test_tx_ht)

# Phase 3: VHT TX
add_lib80211_test(test_tx_vht)

# HT/VHT intermediate TX pipeline stages (scramble, encode)
add_lib80211_test(test_tx_stages)

# Phase 4: LDPC encoder
add_lib80211_test(test_ldpc)

# Phase 5: Viterbi decoder
add_lib80211_test(test_viterbi)

# Phase 5: Soft demapper
add_lib80211_test(test_demap)

# Phase 5.5: Synchronization (STF detection + CFO estimation)
add_lib80211_test(test_sync)

# Phase 5.8: Legacy frame decoder
add_lib80211_test(test_rx_legacy)

# Phase 6: HT frame decoder
add_lib80211_test(test_rx_ht)

# Phase 7: VHT frame decoder
add_lib80211_test(test_rx_vht)

# Benchmarks (not ctests — run manually)
add_executable(bench_decode bench_decode.c)
target_link_libraries(bench_decode PRIVATE lib80211 m)
target_compile_definitions(bench_decode PRIVATE
    VECTORS_DIR="${VECTORS_DIR}"
)

# Per-stage decode microbenchmark (not a ctest — run manually)
add_executable(bench_stages bench_stages.c)
target_link_libraries(bench_stages PRIVATE lib80211 m)

# Impairment robustness tests (statistical — longer runtime)
add_lib80211_test(test_impairments)

# Edge case tests (large frames, boundary sizes, seed exhaustion)
add_lib80211_test(test_edge_cases)

# MAC frame builders (beacon, deauth, FCS)
add_lib80211_test(test_mac)

# Coverage gap tests (rate_lookup, sync edge cases, channel, equalize, LDPC, RNG)
add_lib80211_test(test_coverage)

# Fuzz target (requires clang with libFuzzer)
option(LIB80211_BUILD_FUZZ "Build fuzz targets" OFF)
if(LIB80211_BUILD_FUZZ)
    add_executable(fuzz_rx fuzz_rx.c)
    target_link_libraries(fuzz_rx PRIVATE lib80211 m)
    target_compile_options(fuzz_rx PRIVATE -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer)
    target_link_options(fuzz_rx PRIVATE -fsanitize=fuzzer,address,undefined)
endif()

# Static linking for all test/bench executables on ARM (glibc version mismatch)
if(CMAKE_CROSSCOMPILING)
    get_property(_all_targets DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_tgt IN LISTS _all_targets)
        get_target_property(_type ${_tgt} TYPE)
        if(_type STREQUAL "EXECUTABLE")
            target_link_options(${_tgt} PRIVATE -static)
        endif()
    endforeach()
endif()
