# lib80211

Portable 802.11 OFDM PHY library in C and Python. Decodes and encodes 20 MHz
single-stream frames from raw IQ: Legacy (802.11a), HT-mixed (802.11n),
and VHT (802.11ac).

## Why

- **Educational.** Built from the IEEE 802.11-2020 spec to understand
  the PHY layer end-to-end — every stage from STF detection through FCS.
- **Reusable.** Zero-allocation hot path (bump allocator), no dynamic
  memory in decode/encode, dual FFT backend (Apple vDSP / FFTW3).
  Targets both workstation and embedded ARM (PlutoSDR).
- **Golden vectors.** 459 test vectors with full provenance from the
  IEEE TGn and TGac reference waveform generators. See
  [PROVENANCE.md](PROVENANCE.md).
- **Python and C.** Python for prototyping and SDR scripting, C for
  performance and firmware deployment. Both validate against the same
  golden vectors.

## Coverage

| Standard | Rates | FEC |
|----------|-------|-----|
| 802.11a (Legacy) | 6–54 Mbps (all 8) | BCC (Viterbi) |
| 802.11n (HT-mixed) | MCS 0–7, 20 MHz | BCC + LDPC |
| 802.11ac (VHT) | MCS 0–8, 20 MHz | BCC + LDPC |

Full RX pipeline: STF detection, coarse/fine CFO estimation, LTF timing,
channel estimation, MMSE equalization, pilot tracking (CPE + SFO),
soft-demap, FEC decode, descramble, FCS check.

Full TX pipeline: frame generation for all supported rates with proper
preamble construction (STF/LTF/SIG fields).

## Building (C)

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

Requires CMake 3.20+ and a C17 compiler. On macOS, the FFT backend uses
Accelerate.framework (no external dependencies). On Linux/ARM, link
against `libfftw3f`.

Optional build flags:
- `-DLIB80211_SANITIZERS=ON` — enable ASan + UBSan
- `-DLIB80211_BUILD_FUZZ=ON` — build the libFuzzer target

## Quick Start (C)

### Decode a frame from IQ samples

```c
#include <lib80211/rx.h>
#include <lib80211/fft.h>
#include <stdio.h>

int main(void) {
    /* Load IQ from file or SDR into float arrays */
    float *iq_real = /* ... */;
    float *iq_imag = /* ... */;
    size_t n_samples = /* ... */;

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    lib80211_rx_result result;

    if (lib80211_rx_decode(plan, iq_real, iq_imag, n_samples, &result) == 0) {
        printf("Frame: %zu bytes, FCS %s\n",
               result.psdu_len,
               result.fcs_valid ? "OK" : "FAIL");
    }

    lib80211_fft_plan_destroy(plan);
}
```

### Generate a beacon frame

```c
#include <lib80211/tx.h>
#include <lib80211/mac.h>
#include <lib80211/fft.h>

/* Build a beacon PSDU */
uint8_t psdu[512];
uint8_t bssid[6] = {0x02, 0x00, 0x00, 0xDE, 0xAD, 0x01};
size_t len = lib80211_build_beacon(psdu, sizeof(psdu) - 4,
                                   "MyNetwork", bssid, 36, 100, 0);
lib80211_append_fcs(psdu, len);
len += 4;

/* Generate IQ at 6 Mbps */
lib80211_tx_legacy_params params = {
    .rate_mbps = 6,
    .psdu = psdu,
    .psdu_len = len,
    .scrambler_seed = 0x5D,
};

lib80211_fft_plan *plan = lib80211_fft_plan_create();
size_t n_out = lib80211_tx_legacy_samples(&params);
float *out_re = malloc(n_out * sizeof(float));
float *out_im = malloc(n_out * sizeof(float));

lib80211_tx_legacy(plan, &params, out_re, out_im);
/* out_re/out_im now contain baseband IQ ready for a DAC or file */
```

### Zero-allocation path (embedded / real-time)

```c
#include <lib80211/rx.h>
#include <lib80211/scratch.h>

/* Allocate scratch once at init (static, stack, or heap) */
static uint8_t mem[LIB80211_SCRATCH_MAX];
lib80211_scratch scratch;
lib80211_scratch_init(&scratch, mem, sizeof(mem));

/* Decode with no malloc — scratch is reset internally per call */
lib80211_rx_result result;
lib80211_rx_decode_s(plan, iq_re, iq_im, n, &scratch, &result);
```

## Python

`py80211` is a complete Python implementation of the same PHY pipeline —
encode, decode, channel simulation, impairments. Usable standalone for
host-side SDR work (e.g., with pyadi-iio and a PlutoSDR) without
touching the C library.

```sh
pip install -e python/
```

### Decode from IQ (Python)

```python
import numpy as np
from py80211.decode_frame import detect_stf, DecodeContext

# Load IQ capture (complex64, 20 MSPS)
iq = np.load("capture.npy")

ctx = DecodeContext()
offset = 0
while offset < len(iq) - 1000:
    stf = detect_stf(iq[offset:], threshold=0.5, min_periods=6)
    if stf < 0:
        break

    result = ctx.decode_frame(iq, offset + stf)
    if result and result.get("fcs_ok"):
        print(f"Frame: {len(result['psdu'])} bytes at offset {offset + stf}")

    offset += stf + max(result.get("signal", {}).get("n_symbols", 1) * 80, 320)
```

### Generate a frame (Python)

```python
from py80211.gen_ofdm_frame import generate_ofdm_frame

# Generate a legacy 24 Mbps frame
psdu = b"\x80\x00" + b"\x00" * 50  # example beacon stub
iq = generate_ofdm_frame(psdu, rate_mbps=24)
# iq is complex64 ndarray of baseband samples
```

### Channel simulation

```python
from py80211.impairments import add_awgn, add_cfo, add_multipath
from py80211.gen_ofdm_frame import generate_ofdm_frame

iq = generate_ofdm_frame(psdu, rate_mbps=54)
iq = add_multipath(iq, delays=[0, 3, 7], gains=[1.0, 0.4, 0.2])
iq = add_cfo(iq, cfo_hz=1200, sample_rate=20e6)
iq = add_awgn(iq, snr_db=20)
```

## CLI Tools

The `tools/` directory contains ready-to-use command-line utilities:

| Tool | Description |
|------|-------------|
| `tx_beacon` | Generate beacon IQ as a cf32 file |
| `rx_file` | Decode all frames from a cf32 capture, output JSON |
| `ota_rx` | Live OTA decode using PlutoSDR + libiio |
| `ota_tx_beacon` | Transmit beacons over the air via PlutoSDR |

```sh
# Generate beacons, then decode them:
./build/tx_beacon -s "TestNet" -c 36 -n 10 -o beacons.cf32
./build/rx_file -v beacons.cf32
```

## Hardware Validation (PlutoSDR)

The `tools/hardware/` directory contains Python scripts for over-the-air
testing with an ADALM-Pluto SDR:

| Script | Description |
|--------|-------------|
| `capture_iq.py` | Capture raw IQ to .npy files |
| `test_passive_rx.py` | Passive capture + decode of real AP traffic |
| `test_cable_loopback.py` | TX/RX loopback via RF cable |
| `test_ota_loopback.py` | TX/RX loopback over the air |
| `test_tx_beacon.py` | Transmit beacons and verify with monitor |
| `test_sensitivity.py` | RX sensitivity sweep across SNR levels |
| `traffic_analysis.py` | Decode and classify live traffic |

```sh
# Passive capture on channel 36 (5180 MHz), decode all frames
python tools/hardware/test_passive_rx.py --freq 5180000000 --duration 10

# Save IQ capture for offline replay
python tools/hardware/capture_iq.py --freq 5180e6 --duration 5 --file ch36.npy
python tools/hardware/test_passive_rx.py --load ch36.npy
```

These scripts require a PlutoSDR and `pyadi-iio`. See
`docker/Dockerfile.pluto` for ARM cross-compilation targeting the Pluto's
Cortex-A9.

## Project Layout

```
include/lib80211/  Public API (18 headers)
src/               C library (29 modules, zero-alloc hot path)
python/py80211/    Complete Python implementation
vectors/           459 golden test vectors (JSON)
test/              C test suite (27 tests), benchmarks, fuzzer
tools/             CLI utilities and PlutoSDR hardware scripts
cmake/             Toolchain files and CMake config template
```

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) — PHY pipeline reference
  (frame structures, bit fields, subcarrier maps, pilot formulas)
- [DECISIONS.md](DECISIONS.md) — non-obvious design rationale
- [PROVENANCE.md](PROVENANCE.md) — vector origins, generation method,
  and legal basis for redistribution

## License

MIT
