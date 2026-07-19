"""IEEE 802.11 LDPC encoder and decoder."""
import functools
import numpy as np
from .ldpc_tables import H_SUB, P_GEN, Z as _Z_MAP

_RATE_K = {"1/2": (1, 2), "2/3": (2, 3), "3/4": (3, 4), "5/6": (5, 6)}


def build_H(rate: str, cw_len: int) -> np.ndarray:
    """Build the full (N-K) x N binary parity check matrix from circulant sub-matrix.

    Uses H_SUB convention: 0 = zero block, 1..Z = shift by (val % Z).
    """
    H_sub = H_SUB[rate][cw_len]
    Z = _Z_MAP[cw_len]
    M, N_blocks = H_sub.shape
    H = np.zeros((M * Z, N_blocks * Z), dtype=np.int8)
    for i in range(M):
        for j in range(N_blocks):
            val = int(H_sub[i, j])
            if val != 0:  # 0 = zero block
                shift = val % Z
                for k in range(Z):
                    H[i * Z + k, j * Z + (k + shift) % Z] = 1
    return H


def ldpc_encode(info_bits: np.ndarray, rate: str, cw_len: int) -> np.ndarray:
    """Systematic LDPC encoding: codeword = [info | P_GEN @ info mod 2].

    Args:
        info_bits: Information bit vector of length K.
        rate: Code rate ("1/2", "2/3", "3/4", "5/6").
        cw_len: Codeword length (648, 1296, 1944).

    Returns:
        Codeword of length cw_len as int8 array.
    """
    num, den = _RATE_K[rate]
    K = cw_len * num // den
    assert len(info_bits) == K, f"Expected {K} info bits, got {len(info_bits)}"
    P = P_GEN[rate][cw_len]
    # P is sparse (csc_matrix) — sparse @ dense is efficient
    parity = np.asarray(P @ info_bits.astype(np.float64)).flatten().astype(np.int64) % 2
    return np.concatenate([info_bits.astype(np.int8), parity.astype(np.int8)])


@functools.lru_cache(maxsize=16)
def _build_adjacency(rate: str, cw_len: int):
    """Build and cache adjacency lists from the parity check matrix."""
    H = build_H(rate, cw_len)
    num_checks, num_vars = H.shape
    # check_to_var[i] = array of variable indices connected to check i
    check_to_var = [np.where(H[i] == 1)[0] for i in range(num_checks)]
    # var_to_check[j] = array of check indices connected to variable j
    var_to_check = [np.where(H[:, j] == 1)[0] for j in range(num_vars)]
    return check_to_var, var_to_check, num_checks, num_vars


def ldpc_decode(llr: np.ndarray, rate: str, cw_len: int,
                max_iter: int = 30, scale: float = 0.75) -> np.ndarray:
    """Min-sum LDPC decoder.

    Args:
        llr: Channel LLR values, length = cw_len.
             Positive = bit 0 more likely.
        rate: Code rate ("1/2", "2/3", "3/4", "5/6").
        cw_len: Codeword length (648, 1296, 1944).
        max_iter: Maximum BP iterations.
        scale: Min-sum normalization factor (0.75 typical for 802.11).

    Returns:
        Hard-decision decoded bits, length = cw_len (int8 array of 0/1).
    """
    check_to_var, var_to_check, num_checks, num_vars = _build_adjacency(rate, cw_len)
    llr = np.asarray(llr, dtype=np.float64)
    assert len(llr) == cw_len

    # R[i][local_idx] = check-to-variable message from check i to its local_idx-th variable
    # Store as list of arrays for efficient access
    R = [np.zeros(len(check_to_var[i]), dtype=np.float64) for i in range(num_checks)]

    # Precompute reverse index: for variable j and check i in var_to_check[j],
    # what is the local index of j in check_to_var[i]?
    # var_in_check_idx[j][local_k] = local index of j in check_to_var[var_to_check[j][local_k]]
    var_in_check_idx = []
    for j in range(num_vars):
        indices = []
        for i in var_to_check[j]:
            idx = np.searchsorted(check_to_var[i], j)
            indices.append(idx)
        var_in_check_idx.append(np.array(indices, dtype=np.intp))

    for _iteration in range(max_iter):
        # --- Check-to-variable (R) update using min-sum ---
        for i in range(num_checks):
            var_indices = check_to_var[i]
            deg = len(var_indices)
            # Compute Q[i, var_indices[k]] for each k
            # Q[i,j] = llr[j] + sum of R[m,j] for m != i
            # But we store R by check, so we need to gather R messages for each variable
            Q_local = np.empty(deg, dtype=np.float64)
            for k in range(deg):
                j = var_indices[k]
                # Sum of all R[m,j] for m connected to j
                total_r = 0.0
                for local_m, m in enumerate(var_to_check[j]):
                    if m != i:
                        total_r += R[m][var_in_check_idx[j][local_m]]
                Q_local[k] = llr[j] + total_r

            # Min-sum: R[i,j] = scale * prod_signs(Q except j) * min_abs(Q except j)
            signs = np.sign(Q_local)
            signs[signs == 0] = 1  # treat zero LLR as slightly positive
            abs_Q = np.abs(Q_local)
            # Product of all signs
            total_sign = np.prod(signs)
            # Find min and second min for efficient "min excluding j"
            sorted_idx = np.argsort(abs_Q)
            min_val = abs_Q[sorted_idx[0]]
            second_min = abs_Q[sorted_idx[1]] if deg > 1 else min_val
            min_pos = sorted_idx[0]

            for k in range(deg):
                # sign excluding k
                s = total_sign * signs[k]  # dividing out sign[k]
                # min excluding k
                if k == min_pos:
                    m_val = second_min
                else:
                    m_val = min_val
                R[i][k] = scale * s * m_val

        # --- Total LLR and hard decision ---
        L = llr.copy()
        for j in range(num_vars):
            for local_m, m in enumerate(var_to_check[j]):
                L[j] += R[m][var_in_check_idx[j][local_m]]

        hard = (L < 0).astype(np.int8)

        # --- Early termination: check syndrome ---
        syndrome_ok = True
        for i in range(num_checks):
            if np.sum(hard[check_to_var[i]]) % 2 != 0:
                syndrome_ok = False
                break
        if syndrome_ok:
            return hard

    return hard
