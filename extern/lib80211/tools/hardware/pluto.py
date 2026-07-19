"""Shared PlutoSDR interface for OTA test scripts.

Provides device discovery, RX/TX configuration, capture, and transmit
helpers built on pyadi-iio. SDR libraries are optional; the module exits
cleanly if they are not installed.
"""

import sys
import time

import numpy as np

try:
    import adi
    import iio
except ImportError as e:
    sys.exit(
        f"SDR libraries not available. Install pyadi-iio and pylibiio.\n"
        f"  pip install pyadi-iio\n"
        f"Error: {e}"
    )

SAMPLE_RATE = 20_000_000  # 20 MSPS — standard 802.11a/g/n 20 MHz channel
MAX_RX_GAIN_5G = 62       # AD9364 max RX gain at 5 GHz (73 dB at 2.4 GHz)


def find_pluto(uri=None):
    """Find a PlutoSDR device.

    Args:
        uri: Explicit IIO context URI (e.g. "usb:1.4.5" or "ip:192.168.2.1").
             If None, scans available contexts preferring USB over IP.

    Returns:
        (uri_string, adi.Pluto instance)

    Raises:
        SystemExit if no device is found.
    """
    if uri is not None:
        sdr = adi.Pluto(uri)
        return uri, sdr

    ctxs = iio.scan_contexts()
    usb_uri = None
    ip_uri = None
    for ctx in ctxs:
        if ctx.startswith("usb:"):
            usb_uri = ctx
        elif ctx.startswith("ip:"):
            ip_uri = ctx

    found = usb_uri or ip_uri
    if not found:
        sys.exit("No PlutoSDR found. Is it connected via USB or network?")

    sdr = adi.Pluto(found)
    return found, sdr


def configure_rx(sdr, freq_hz, gain_db, sample_rate=20e6, buffer_size=2_000_000):
    """Configure PlutoSDR RX chain.

    Sets LO, sample rate, bandwidth, gain (manual mode), and buffer size.
    Flushes 3 buffers to clear AGC/PLL transients.

    Args:
        sdr: adi.Pluto instance.
        freq_hz: Center frequency in Hz.
        gain_db: Hardware RX gain (0–73 dB).
        sample_rate: Sample rate in Hz (default 20 MSPS).
        buffer_size: RX buffer size in samples.

    Returns:
        The configured sdr instance.
    """
    sdr.rx_lo = int(freq_hz)
    sdr.sample_rate = int(sample_rate)
    sdr.rx_rf_bandwidth = int(sample_rate)
    sdr.rx_buffer_size = int(buffer_size)
    sdr.gain_control_mode_chan0 = "manual"
    # Clamp gain to hardware max (varies by frequency)
    max_gain = MAX_RX_GAIN_5G if freq_hz > 3e9 else 73
    if gain_db > max_gain:
        gain_db = max_gain
    sdr.rx_hardwaregain_chan0 = gain_db

    # Let PLL/AGC settle then flush transient buffers
    time.sleep(0.1)
    for _ in range(3):
        try:
            sdr.rx()
        except Exception:
            pass

    return sdr


def configure_tx(sdr, freq_hz, attenuation_db=10, sample_rate=20e6):
    """Configure PlutoSDR TX chain for cyclic buffer mode.

    Args:
        sdr: adi.Pluto instance.
        freq_hz: Center frequency in Hz.
        attenuation_db: TX attenuation (positive value, applied as negative gain).
        sample_rate: Sample rate in Hz (default 20 MSPS).

    Returns:
        The configured sdr instance.
    """
    sdr.tx_lo = int(freq_hz)
    sdr.sample_rate = int(sample_rate)
    sdr.tx_rf_bandwidth = int(sample_rate)
    sdr.tx_hardwaregain_chan0 = -abs(attenuation_db)
    sdr.tx_cyclic_buffer = True

    return sdr


def capture(sdr, duration_sec, buffer_size=2_000_000):
    """Capture IQ samples for a given duration.

    Reads multiple buffers from the SDR and concatenates them into a
    single contiguous complex64 array.

    Args:
        sdr: Configured adi.Pluto instance (call configure_rx first).
        duration_sec: Capture duration in seconds.
        buffer_size: Buffer size per read (must match configure_rx setting).

    Returns:
        np.ndarray of dtype complex64.
    """
    n_samples = int(duration_sec * SAMPLE_RATE)
    n_buffers = max(1, n_samples // buffer_size)
    total = n_buffers * buffer_size

    all_iq = np.zeros(total, dtype=np.complex64)
    for i in range(n_buffers):
        buf = sdr.rx()
        start = i * buffer_size
        end = min(start + buffer_size, total)
        available = min(len(buf), end - start)
        all_iq[start:start + available] = np.asarray(buf[:available], dtype=np.complex64)

    return all_iq[:n_samples]


def transmit(sdr, iq):
    """Scale IQ waveform and load into TX cyclic buffer.

    Normalizes peak amplitude to 2^14 (one bit of headroom below int16 max)
    then sends as int16 I/Q pairs.

    Args:
        sdr: Configured adi.Pluto instance (call configure_tx first).
        iq: Complex waveform (any numeric dtype).
    """
    iq = np.asarray(iq, dtype=np.complex128)
    peak = np.max(np.abs(iq))
    if peak == 0:
        raise ValueError("Cannot transmit all-zero waveform")

    scale = (2**14) / peak
    iq_scaled = (iq * scale).astype(np.complex64)

    # pyadi-iio accepts complex arrays directly for single-channel TX
    sdr.tx(iq_scaled)


def stop_tx(sdr):
    """Stop TX DMA and release the cyclic buffer."""
    sdr.tx_destroy_buffer()
