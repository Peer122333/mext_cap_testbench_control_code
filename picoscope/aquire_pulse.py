"""
Einfaches Acquisition-Skript:
- Holt einen Block vom PicoScope (PS3000A)
- Speichert Signal + Zeit + Metadaten als .npz
"""

import os
import time
import ctypes as ct
import numpy as np
from datetime import datetime
from picosdk.ps3000a import ps3000a as ps
from picosdk.functions import assert_pico_ok

# ------------------- Einstellungen -------------------
CHANNEL      = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_A"]
COUPLING     = ps.PS3000A_COUPLING["PS3000A_DC"]
V_RANGE      = ps.PS3000A_RANGE["PS3000A_10V"]    # Messbereich: ±5 V
ENABLE       = 1

TRIG_LEVEL_V = 8.0  # Volt (Rising)
TARGET_FS    = 20e6
N_SAMPLES    = 20000
OVERSAMPLE   = 1

# Absoluter Speicherordner (hart verdrahtet)
SAVE_DIR = r"C:\Users\Prüfstand\Documents\Control\mext_cap_testbench_control_code\picoscope\Messungen"
os.makedirs(SAVE_DIR, exist_ok=True)
# ------------------------------------------------------


def next_filename(prefix="pulse", ext="npz"):
    """Erzeuge den nächsten freien Dateinamen: pulse_001.npz, pulse_002.npz, ..."""
    existing = [f for f in os.listdir(SAVE_DIR) if f.startswith(prefix + "_") and f.endswith("." + ext)]
    nums = []
    for f in existing:
        base = f[:-(len(ext) + 1)]  # ohne .ext
        try:
            nums.append(int(base.split("_")[-1]))
        except ValueError:
            pass
    next_num = max(nums) + 1 if nums else 1
    return os.path.join(SAVE_DIR, f"{prefix}_{next_num:03d}.{ext}")


def range_fullscale_volts(v_range_enum):
    """Map Pico-Range-Enum -> Fullscale (±X V)."""
    table = {
        ps.PS3000A_RANGE["PS3000A_20MV"]: 0.02,
        ps.PS3000A_RANGE["PS3000A_50MV"]: 0.05,
        ps.PS3000A_RANGE["PS3000A_100MV"]: 0.1,
        ps.PS3000A_RANGE["PS3000A_200MV"]: 0.2,
        ps.PS3000A_RANGE["PS3000A_500MV"]: 0.5,
        ps.PS3000A_RANGE["PS3000A_1V"]: 1.0,
        ps.PS3000A_RANGE["PS3000A_2V"]: 2.0,
        ps.PS3000A_RANGE["PS3000A_5V"]: 5.0,
        ps.PS3000A_RANGE["PS3000A_10V"]: 10.0,
        ps.PS3000A_RANGE["PS3000A_20V"]: 20.0,
        ps.PS3000A_RANGE["PS3000A_50V"]: 50.0,
    }
    return table[v_range_enum]


def pick_timebase(handle, target_fs, n_samples):
    """Suche eine Timebase nahe target_fs (ps3000aGetTimebase2)."""
    best = None
    for tb in range(1, 50000):
        time_interval_ns = ct.c_float()
        max_samples = ct.c_int32()
        status = ps.ps3000aGetTimebase2(
            handle, tb, n_samples, ct.byref(time_interval_ns), 0, ct.byref(max_samples), 0
        )
        if status == 0:  # PICO_OK
            dt = time_interval_ns.value * 1e-9
            if dt <= 0:
                continue
            fs = 1.0 / dt
            err = abs(fs - target_fs) / target_fs
            if best is None or err < best[0]:
                best = (err, tb, dt, fs)
            if err < 0.02:
                break
    if best is None:
        raise RuntimeError("Keine gültige Timebase gefunden.")
    _, tb, dt, fs = best
    return tb, dt, fs


def acquire_block():
    """Erfasst genau einen Block und gibt (t, y, meta) zurück."""
    # Gerät öffnen (mit Power-Source-Fallback)
    handle = ct.c_int16()
    status = ps.ps3000aOpenUnit(ct.byref(handle), None)
    try:
        assert_pico_ok(status)
    except:
        if status in (ps.PICO_POWER_SUPPLY_NOT_CONNECTED, ps.PICO_USB3_0_DEVICE_NON_USB3_0_PORT):
            status = ps.ps3000aChangePowerSource(handle, status)
            assert_pico_ok(status)
        else:
            raise

    try:
        # Kanal konfigurieren
        assert_pico_ok(ps.ps3000aSetChannel(handle, CHANNEL, ENABLE, COUPLING, V_RANGE, 0.0))

        # Max-ADC (ctypes-Objekt behalten!)
        max_adc = ct.c_int16()
        assert_pico_ok(ps.ps3000aMaximumValue(handle, ct.byref(max_adc)))

        # Timebase wählen
        timebase, dt, fs = pick_timebase(handle, TARGET_FS, N_SAMPLES)
        print(f"[Info] Timebase={timebase}, dt={dt*1e9:.2f} ns, fs={fs/1e6:.2f} MS/s")

        # Trigger (einfach, Rising auf Kanal A)
        vfs = range_fullscale_volts(V_RANGE)  # ±X V
        trig_adc = int((TRIG_LEVEL_V / vfs) * max_adc.value)
        assert_pico_ok(
            ps.ps3000aSetSimpleTrigger(
                handle,
                1,                              # enable
                CHANNEL,                        # source
                trig_adc,                       # level (ADC)
                ps.PS3000A_THRESHOLD_DIRECTION["PS3000A_RISING"],
                0,                              # delay (samples)
                500                             # autoTrigger ms
            )
        )

        # Puffer
        buf = (ct.c_int16 * N_SAMPLES)()
        assert_pico_ok(
            ps.ps3000aSetDataBuffer(
                handle, CHANNEL, ct.byref(buf), N_SAMPLES, 0,
                ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"]
            )
        )

        # Block starten (pre=0, post=N_SAMPLES)
        time_indisposed_ms = ct.c_int32(0)
        assert_pico_ok(
            ps.ps3000aRunBlock(
                handle,
                0,                   # preTriggerSamples
                N_SAMPLES,           # postTriggerSamples
                timebase,
                int(OVERSAMPLE),
                ct.byref(time_indisposed_ms),
                0,                   # segmentIndex
                None,
                None,
            )
        )

        # Warten bis fertig
        ready = ct.c_int16(0)
        while not ready.value:
            ps.ps3000aIsReady(handle, ct.byref(ready))
            time.sleep(0.001)

        # Werte holen
        n = ct.c_int32(N_SAMPLES)
        overflow = ct.c_int16()
        assert_pico_ok(
            ps.ps3000aGetValues(
                handle, 0, ct.byref(n), 1,
                ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"],
                0, ct.byref(overflow)
            )
        )

        # In Volt umrechnen (NumPy, ohne adc2mV)
        data_adc = np.frombuffer(buf, dtype=np.int16, count=n.value).astype(np.float64, copy=False)
        scale = vfs / max_adc.value    # Volt pro ADC-Count
        y = data_adc * scale
        t = np.arange(len(y)) * dt

        meta = dict(
            fs=fs,
            v_range=vfs,
            trigger_level_v=TRIG_LEVEL_V,
            overflow=int(overflow.value),
            time_indisposed_ms=int(time_indisposed_ms.value),
            timestamp=str(datetime.now()),
        )

        return t, y, meta

    finally:
        try:
            ps.ps3000aStop(handle)
        except Exception:
            pass
        ps.ps3000aCloseUnit(handle)


def acquire_and_save():
    """Erfasst einen Block und speichert ihn als nächste freie Datei im SAVE_DIR."""
    t, y, meta = acquire_block()
    fname = next_filename(prefix="pulse", ext="npz")
    np.savez(fname, time=t, signal=y, meta=meta)
    print(f"[OK] Daten gespeichert in {fname}")
    if meta.get("overflow", 0) != 0:
        print("[Warn] Overflow-Flag gesetzt – Signal könnte den Bereich überschreiten.")


if __name__ == "__main__":
    acquire_and_save()
