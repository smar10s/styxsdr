"""Shared fixtures and helpers for lib80211 PHY golden vector tests.

Common utilities used by both test_tx.py (TX path verification) and
test_rx.py (RX path verification).
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

# ============================================================================
# Project paths
# ============================================================================
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
VECTORS_DIR = PROJECT_ROOT / "vectors"
sys.path.insert(0, str(PROJECT_ROOT / "python"))


# ============================================================================
# Vector loading
# ============================================================================

def load_vector(name: str) -> dict:
    """Load a golden vector JSON file by name (without .json extension)."""
    path = VECTORS_DIR / f"{name}.json"
    with open(path) as f:
        return json.load(f)


# ============================================================================
# Subcarrier / FFT bin index conversion
# ============================================================================

def subcarrier_to_fft_bin(subcarrier_index: int) -> int:
    """Convert subcarrier index (-32..+31) to FFT bin index (0..63).

    FFT bin 0 = DC (subcarrier 0), bin 1 = subcarrier +1, ...
    bin 32 = subcarrier -32, bin 33 = subcarrier -31, ...
    """
    return subcarrier_index % 64


def json_index_to_fft_bin(json_idx: int) -> int:
    """JSON arrays are indexed 0..63 with index 0 = subcarrier -32.

    So json_idx i represents subcarrier (i - 32).
    """
    return subcarrier_to_fft_bin(json_idx - 32)


# ============================================================================
# PSDU constants — IEEE 802.11-2020 Annex I.1 (100-byte "Ode to Joy")
# ============================================================================

ANNEX_I1_PSDU_HEX = [
    "04", "02", "00", "2E", "00", "60", "08", "CD", "37", "A6",
    "00", "20", "D6", "01", "3C", "F1", "00", "60", "08", "AD",
    "3B", "AF", "00", "00", "4A", "6F", "79", "2C", "20", "62",
    "72", "69", "67", "68", "74", "20", "73", "70", "61", "72",
    "6B", "20", "6F", "66", "20", "64", "69", "76", "69", "6E",
    "69", "74", "79", "2C", "0A", "44", "61", "75", "67", "68",
    "74", "65", "72", "20", "6F", "66", "20", "45", "6C", "79",
    "73", "69", "75", "6D", "2C", "0A", "46", "69", "72", "65",
    "2D", "69", "6E", "73", "69", "72", "65", "64", "20", "77",
    "65", "20", "74", "72", "65", "61", "67", "33", "21", "B6",
]
ANNEX_I1_PSDU_BYTES = bytes(int(h, 16) for h in ANNEX_I1_PSDU_HEX)
ANNEX_I1_SCRAMBLER_SEED = 0x5D


def golden_psdu_bytes_from_vec(psdu_vec: dict) -> bytes:
    """Return the 100-byte PSDU from Annex I.1 vector dict as bytes."""
    return bytes(int(h, 16) for h in psdu_vec["octets_hex"])
