"""
PicoScope 3205A – Minimalbeispiel
Block-Mode, optionaler Rising-Trigger, 1 Kanal (A)
Getestet mit PicoSDK Python Wrappers (ps3000a)
"""

import time
import ctypes as ct
import numpy as np
import matplotlib.pyplot as plt

from picosdk.ps3000a import ps3000a as ps
from picosdk.functions import assert_pico_ok

# ========= Nutzereinstellungen (Basics) =========
CHANNEL        = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_A"]
COUPLING       = ps.PS3000A_COUPLING["PS3000A_DC"]     # DC-Kopplung
V_RANGE        = ps.PS3000A_RANGE["PS3000A_5V"]        # ±5 V Messbereich
ENABLE         = 1

USE_TRIGGER    = True
TRIG_SOURCE    = CHANNEL
TRIG_DIR       = ps.PS3000A_THRESHOLD_DIRECTION["PS3000A_RISING"]
TRIG_LEVEL_V   = 1.0          # Volt
TRIG_DELAY     = 0            # Samples
AUTO_TRIGGER_MS= 500          # ms

TARGET_FS      = 20e6         # Ziel-Samplingrate (Hz); passende Timebase wird gesucht
N_SAMPLES      = 20000        # Samples pro Block
OVERSAMPLE     = 1
PRETRIG_RATIO  = 0.2          # Anteil vor dem Trigger
# ================================================


def print_settings():
    print("\n" + "="*46)
    print("           PICO-SCOPE MINIMAL-SETUP")
    print("="*46)
    print("[ Kanal ]")
    print(f"  CHANNEL       : {CHANNEL} (A=0)")
    print(f"  ENABLE        : {ENABLE}")
    print(f"  COUPLING      : {COUPLING} (0=AC, 1=DC)")
    print(f"  V_RANGE (idx) : {V_RANGE}  (z.B. 8 = ±5V)")
    print("\n[ Trigger ]")
    print(f"  USE_TRIGGER   : {USE_TRIGGER}")
    print(f"  SOURCE        : {TRIG_SOURCE}")
    print(f"  DIRECTION     : {TRIG_DIR} (RISING)")
    print(f"  LEVEL [V]     : {TRIG_LEVEL_V}")
    print(f"  DELAY [samples]: {TRIG_DELAY}")
    print(f"  AUTO_TRIGGER  : {AUTO_TRIGGER_MS} ms")
    print("\n[ Erfassung ]")
    print(f"  TARGET_FS     : {TARGET_FS/1e6:.2f} MS/s")
    print(f"  N_SAMPLES     : {N_SAMPLES}")
    print(f"  OVERSAMPLE    : {OVERSAMPLE}")
    print(f"  PRETRIGGER    : {PRETRIG_RATIO*100:.1f} %")
    print("="*46 + "\n")


def pick_timebase(handle, target_fs, n_samples):
    """
    Sucht eine Timebase, die nahe an target_fs liegt (ps3000aGetTimebase2).
    Gibt (timebase, dt_s, fs) zurück.
    """
    best = None
    for tb in range(1, 50000):
        time_interval_ns = ct.c_float()
        max_samples      = ct.c_int32()
        status = ps.ps3000aGetTimebase2(
            handle, tb, n_samples, ct.byref(time_interval_ns), 0, ct.byref(max_samples), 0
        )
        if status == 0:  # PICO_OK
            dt = time_interval_ns.value * 1e-9
            fs = 1.0 / dt if dt > 0 else 0.0
            err = abs(fs - target_fs) / target_fs
            if best is None or err < best[0]:
                best = (err, tb, dt, fs)
            if err < 0.02:  # gut genug
                break
    if best is None:
        raise RuntimeError("Keine gültige Timebase gefunden.")
    _, tb, dt, fs = best
    return tb, dt, fs


def main():
    print_settings()

    # Gerät öffnen (mit Power-Source-Fallback)
    handle = ct.c_int16()
    status = ps.ps3000aOpenUnit(ct.byref(handle), None)
    try:
        assert_pico_ok(status)
    except:
        if status == ps.PICO_POWER_SUPPLY_NOT_CONNECTED or status == ps.PICO_USB3_0_DEVICE_NON_USB3_0_PORT:
            status = ps.ps3000aChangePowerSource(handle, status)
            assert_pico_ok(status)
        else:
            raise

    try:
        # Kanal konfigurieren
        assert_pico_ok(ps.ps3000aSetChannel(handle, CHANNEL, ENABLE, COUPLING, V_RANGE, 0.0))

        # Max-ADC (als ctypes-Objekt behalten!)
        max_adc = ct.c_int16()
        assert_pico_ok(ps.ps3000aMaximumValue(handle, ct.byref(max_adc)))

        # Timebase wählen
        timebase, dt, fs = pick_timebase(handle, TARGET_FS, N_SAMPLES)
        print(f"[Info] Timebase={timebase}, dt={dt*1e9:.2f} ns, fs={fs/1e6:.2f} MS/s")

        # Trigger (optional, simpel)
        if USE_TRIGGER:
            range_volts = {
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
            }[V_RANGE]
            trig_adc = int((TRIG_LEVEL_V / range_volts) * max_adc.value)
            assert_pico_ok(
                ps.ps3000aSetSimpleTrigger(
                    handle, ENABLE, TRIG_SOURCE, trig_adc, TRIG_DIR, TRIG_DELAY, int(AUTO_TRIGGER_MS)
                )
            )
        else:
            assert_pico_ok(ps.ps3000aSetSimpleTrigger(handle, 0, CHANNEL, 0, TRIG_DIR, 0, 0))

        # Puffer
        buf = (ct.c_int16 * N_SAMPLES)()
        assert_pico_ok(
            ps.ps3000aSetDataBuffer(
                handle,
                CHANNEL,
                ct.byref(buf),               # Buffer-Pointer
                N_SAMPLES,
                0,                           # SegmentIndex
                ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"],
            )
        )

        # Block-Parameter
        preTrig  = int(PRETRIG_RATIO * N_SAMPLES)
        postTrig = N_SAMPLES - preTrig
        time_indisposed_ms = ct.c_int32(0)

        # Capture starten
        assert_pico_ok(
            ps.ps3000aRunBlock(
                handle,
                preTrig,
                postTrig,
                timebase,
                int(OVERSAMPLE),
                ct.byref(time_indisposed_ms),
                0,          # SegmentIndex
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
                handle,
                0,                               # startIndex
                ct.byref(n),                     # noOfSamples (in/out)
                1,                               # downSampleRatio
                ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"],
                0,                               # segmentIndex
                ct.byref(overflow),
            )
        )

        # ----- In Volt umrechnen (NumPy, ohne adc2mV) -----
        # Rohdaten holen und direkt in float64 casten (vermeidet Overflow)
        data_adc = np.frombuffer(buf, dtype=np.int16, count=n.value).astype(np.float64, copy=False)

        # Full-Scale-Spannung für den gesetzten Bereich (±X V)
        range_volts = {
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
        }[V_RANGE]

        # Volt pro ADC-Count (MAX = max_adc.value, i.d.R. 32767)
        scale = range_volts / max_adc.value
        y = data_adc * scale  # Ergebnis in Volt
        # ---------------------------------------------------

        # Zeitachse
        t = np.arange(len(y)) * dt

        # Kurze Konsolen-Zusammenfassung
        print(f"[Info] Erhaltene Samples: {len(y)}")
        print(f"[Info] Overflow-Flags  : {overflow.value}")
        print(f"[Info] time_indisposed: {time_indisposed_ms.value} ms")
        if overflow.value != 0:
            print("[Warn] Overflow-Flag gesetzt – Signal könnte den Bereich überschreiten.")

        # Plot (einmalig)
        plt.figure(figsize=(9,4))
        plt.plot(t, y)
        plt.xlabel("Zeit [s]")
        plt.ylabel("Spannung [V]")
        plt.title("PicoScope – Blockaufnahme Kanal A")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    finally:
        try:
            ps.ps3000aStop(handle)
        except Exception:
            pass
        ps.ps3000aCloseUnit(handle)
        print("Gerät geschlossen.")


if __name__ == "__main__":
    main()
