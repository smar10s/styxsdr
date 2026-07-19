# lib80211 static library

# FFT backend source
if(LIB80211_FFT_BACKEND STREQUAL "vdsp")
    set(FFT_SRC fft_vdsp.c)
elseif(LIB80211_FFT_BACKEND STREQUAL "fftw3")
    set(FFT_SRC fft_fftw3.c)
endif()

add_library(lib80211 STATIC
    ${FFT_SRC}
    scratch.c
    tables.c
    scrambler.c
    conv_encode.c
    viterbi.c
    puncture.c
    interleaver.c
    modulation.c
    demap.c
    ofdm.c
    signal.c
    htsig.c
    vhtsig.c
    ldpc_tables.c
    ldpc_encode.c
    ldpc_decode.c
    tx_legacy.c
    tx_ht.c
    tx_vht.c
    channel.c
    sync.c
    rx_common.c
    rx_legacy.c
    rx_ht.c
    rx_vht.c
    impairments.c
    mac.c
)

target_include_directories(lib80211 PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Link platform libraries
if(LIB80211_FFT_BACKEND STREQUAL "vdsp")
    target_link_libraries(lib80211 PUBLIC ${ACCELERATE_FW})
elseif(LIB80211_FFT_BACKEND STREQUAL "fftw3")
    target_include_directories(lib80211 PRIVATE ${FFTW3F_INCLUDE})
    target_link_libraries(lib80211 PUBLIC ${FFTW3F_LIB})
endif()
target_link_libraries(lib80211 PUBLIC m)

# pthread needed for pthread_once in viterbi.c
if(NOT APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(lib80211 PUBLIC Threads::Threads)
endif()

# Compiler warnings
target_compile_options(lib80211 PRIVATE
    -Wall -Wextra -Wpedantic -Werror
    -Wno-unused-parameter
)
