# ...existing code...
import numpy as np
from typing import Tuple, Union
try:
    from scipy.io import loadmat  # optional, only if user has scipy and a .mat file
except Exception:
    loadmat = None


def estimate_cap_params(P: Union[str, np.ndarray]) -> Tuple[float, float]:
    """
    Übersetzung des gegebenen MATLAB-Skripts:
    Erwartet eine Matrix P mit mindestens 5 Spalten oder einen Pfad zu einer .mat-Datei
    (falls scipy.io.loadmat verfügbar ist). Verwendet die Zeilen ab Index 3 (MATLAB 4:end).

    Rückgabe:
      Rest  -> geschätzter Serienwiderstand (realer Anteil von x[0])
      Cest  -> geschätzte Kapazität (realer Anteil von 1 / x[1])

    Beispiel:
      Rest, Cest = estimate_cap_params(P_array)
    """
    if isinstance(P, str):
        if loadmat is None:
            raise RuntimeError("scipy not available: cannot load .mat file")
        data = loadmat(P)
        # falls Strukturname bekannt, muss der Nutzer anpassen; hier versuchen wir heuristisch:
        # erstes Array im dict nehmen, das 2D ist und ausreichend Spalten hat
        arrays = [v for k, v in data.items() if isinstance(v, np.ndarray) and v.ndim == 2 and v.shape[1] >= 5]
        if not arrays:
            raise ValueError("No suitable 2D array with >=5 columns found in .mat file")
        P = arrays[0]

    P = np.asarray(P)
    if P.ndim != 2 or P.shape[1] < 5:
        raise ValueError("P must be 2D array with at least 5 columns")

    # MATLAB: P(4:end, 3..5) -> Python zero-based: rows from 3:, cols 2,3,4
    t = P[3:, 2].astype(float)
    u = P[3:, 3].astype(complex)  # allow complex if measured
    i = -P[3:, 4].astype(complex)

    # sampling
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError("time vector must be strictly increasing")
    fs = 1.0 / np.mean(dt)
    N = t.size

    # FFT (shifted)
    fU = np.fft.fftshift(np.fft.fft(u))
    fI = np.fft.fftshift(np.fft.fft(i))

    # frequency axis (robust)
    f = np.fft.fftshift(np.fft.fftfreq(N, d=1.0 / fs))
    omega = 2.0 * np.pi * f

    # choose positive frequencies (exclude DC)
    pos_idx = np.where(f > 0)[0]
    if pos_idx.size == 0:
        raise RuntimeError("no positive frequencies found (N too small?)")
    # optional: skip the first positive bin if very small omega; mimic MATLAB's (N/2+2):N by skipping first positive
    if pos_idx.size > 1:
        idx = pos_idx[1:]
    else:
        idx = pos_idx

    # build linear system A x = b
    FI = fI[idx]
    OM = omega[idx]
    # avoid division by zero (should not happen because f>0)
    A = np.column_stack([FI, (-1j / OM) * FI])
    b = fU[idx]

    # least squares solution
    x, *_ = np.linalg.lstsq(A, b, rcond=None)

    Rest = float(np.real(x[0]))
    Cest = float(np.real(1.0 / x[1]))

    return Rest, Cest


if __name__ == "__main__":
    # kurzer CLI-Test:
    import sys
    if len(sys.argv) == 2:
        src = sys.argv[1]
        if src.endswith(".mat") and loadmat is not None:
            R, C = estimate_cap_params(src)
            print(f"Rest = {R:.6g}  Cest = {C:.6g}")
        else:
            # versuchen, CSV zu laden (ohne Header)
            arr = np.loadtxt(src, delimiter=",")
            R, C = estimate_cap_params(arr)
            print(f"Rest = {R:.6g}  Cest = {C:.6g}")
    else:
        print("Usage: python calc_cap_params.py <data.mat|data.csv>")
```# filepath: /Users/peer/Documents/GITLAB_1/03_HM_Code/07_SoSe25/MEXT/picoscope/calc_cap_params.py
# ...existing code...
import numpy as np
from typing import Tuple, Union
try:
    from scipy.io import loadmat  # optional, only if user has scipy and a .mat file
except Exception:
    loadmat = None


def estimate_cap_params(P: Union[str, np.ndarray]) -> Tuple[float, float]:
    """
    Übersetzung des gegebenen MATLAB-Skripts:
    Erwartet eine Matrix P mit mindestens 5 Spalten oder einen Pfad zu einer .mat-Datei
    (falls scipy.io.loadmat verfügbar ist). Verwendet die Zeilen ab Index 3 (MATLAB 4:end).

    Rückgabe:
      Rest  -> geschätzter Serienwiderstand (realer Anteil von x[0])
      Cest  -> geschätzte Kapazität (realer Anteil von 1 / x[1])

    Beispiel:
      Rest, Cest = estimate_cap_params(P_array)
    """
    if isinstance(P, str):
        if loadmat is None:
            raise RuntimeError("scipy not available: cannot load .mat file")
        data = loadmat(P)
        # falls Strukturname bekannt, muss der Nutzer anpassen; hier versuchen wir heuristisch:
        # erstes Array im dict nehmen, das 2D ist und ausreichend Spalten hat
        arrays = [v for k, v in data.items() if isinstance(v, np.ndarray) and v.ndim == 2 and v.shape[1] >= 5]
        if not arrays:
            raise ValueError("No suitable 2D array with >=5 columns found in .mat file")
        P = arrays[0]

    P = np.asarray(P)
    if P.ndim != 2 or P.shape[1] < 5:
        raise ValueError("P must be 2D array with at least 5 columns")

    # MATLAB: P(4:end, 3..5) -> Python zero-based: rows from 3:, cols 2,3,4
    t = P[3:, 2].astype(float)
    u = P[3:, 3].astype(complex)  # allow complex if measured
    i = -P[3:, 4].astype(complex)

    # sampling
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError("time vector must be strictly increasing")
    fs = 1.0 / np.mean(dt)
    N = t.size

    # FFT (shifted)
    fU = np.fft.fftshift(np.fft.fft(u))
    fI = np.fft.fftshift(np.fft.fft(i))

    # frequency axis (robust)
    f = np.fft.fftshift(np.fft.fftfreq(N, d=1.0 / fs))
    omega = 2.0 * np.pi * f

    # choose positive frequencies (exclude DC)
    pos_idx = np.where(f > 0)[0]
    if pos_idx.size == 0:
        raise RuntimeError("no positive frequencies found (N too small?)")
    # optional: skip the first positive bin if very small omega; mimic MATLAB's (N/2+2):N by skipping first positive
    if pos_idx.size > 1:
        idx = pos_idx[1:]
    else:
        idx = pos_idx

    # build linear system A x = b
    FI = fI[idx]
    OM = omega[idx]
    # avoid division by zero (should not happen because f>0)
    A = np.column_stack([FI, (-1j / OM) * FI])
    b = fU[idx]

    # least squares solution
    x, *_ = np.linalg.lstsq(A, b, rcond=None)

    Rest = float(np.real(x[0]))
    Cest = float(np.real(1.0 / x[1]))

    return Rest, Cest


if __name__ == "__main__":
    # kurzer CLI-Test:
    import sys
    if len(sys.argv) == 2:
        src = sys.argv[1]
        if src.endswith(".mat") and loadmat is not None:
            R, C = estimate_cap_params(src)
            print(f"Rest = {R:.6g}  Cest = {C:.6g}")
        else:
            # versuchen, CSV zu laden (ohne Header)
            arr = np.loadtxt(src, delimiter=",")
            R, C = estimate_cap_params(arr)
            print(f"Rest = {R:.6g}  Cest = {C:.6g}")
    else:
        print("Usage: python calc_cap_params.py <data.mat|data.csv>")