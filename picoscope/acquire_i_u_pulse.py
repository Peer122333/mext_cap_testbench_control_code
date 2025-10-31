"""
PS3000A Acquisition – U/I synchron, Append-CSV pro Messlauf (Mehrfach-Pulse)
- Kanal A: Spannung (AC, kleiner Bereich)
- Kanal B: Rogowski (AC, optional Volt->Ampere)
- Jede Erfassung wird als neuer 'pulse_id' in EINE CSV angehängt
  Spalten: pulse_id, sample_idx, time_s, u_V, i_{V|A}
"""

import os
import time
import json
import ctypes as ct # C-Typen für Picoscope SDK
import numpy as np  
from datetime import datetime
from picosdk.ps3000a import ps3000a as ps    # Picoscope PS3000A SDK 
from picosdk.functions import assert_pico_ok # Fehlerprüfung SDK-Aufrufe

# =================== CONTROL ===================
RUN_NAME            = "90V_DC_300A-3"  # Messlauf-Name (Ordner+Datei) Pulse_Test_30V_Source_1
AUTO_TRIG_MS        = 0            # Fallback-Trigger
TRIG_LEVEL_V        = -0.2           # Trigger auf CH A (AC), in Volt

# Sampling / Block
TARGET_FS           = 20e6
OVERSAMPLE          = 1
PRETRIG_RATIO       = 0.2            # 20% vor Trigger
N_SAMPLES           = 400_000 + int(PRETRIG_RATIO * 400_000)   # Gesamtanzahl Samples

# Anzahl Pulse in einer Session + Wartezeit zwischen Pulsen
N_PULSES            = 3
INTER_PULSE_DELAY_S = 0.0            # z.B. 0.01 für 10 ms Pause

# Kanal A: Spannung (kleiner Bereich für höhere Auflösung)
CH_A                = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_A"]
COUPLING_A          = ps.PS3000A_COUPLING["PS3000A_AC"]
RANGE_A             = ps.PS3000A_RANGE["PS3000A_50MV"]   # ±2 V -> eigentlich sollte beim Spannungsmessung hier bei 1:100 kleinere Werte besser klappen

# Kanal B: Rogowski (Strom)
CH_B                = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_B"]
COUPLING_B          = ps.PS3000A_COUPLING["PS3000A_AC"]
RANGE_B             = ps.PS3000A_RANGE["PS3000A_10V"]   # anpassen

# Optional: Volt -> Ampere (Integratorfaktor der Rogowski-Kette)
ROGOWSKI_V_PER_A    = 0.02          # z.B. 0.1 (V/A). None => CSV in Volt
U_PROBE_ATTENUATION = 50.0 # 1:10 Tastkopf
# ==================================================

# Basisordner & Run-Verzeichnis
BASE_DIR   = r"C:\Users\mext\Documents\02 Python Schnittstelle STM32 serielle Steuerung\mext_cap_testbench_control_code\picoscope"
RUN_DIR    = os.path.join(BASE_DIR, "Runs", RUN_NAME) 
CSV_PATH   = os.path.join(RUN_DIR, f"{RUN_NAME}.csv")
META_PATH  = os.path.join(RUN_DIR, f"{RUN_NAME}.meta.json")
os.makedirs(RUN_DIR, exist_ok=True)

# ---------- SDK-Helfer ----------
def range_fullscale_volts(v_range_enum):
    """Gibt den vollen Messbereich in Volt für den gegebenen Bereichs-Enum zurück."""
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
    """Wählt einen geeigneten Timebase-Index für das PS3000A-Gerät.

    Die Funktion probiert mehrere Timebase-Indizes aus und
    bestimmt für jeden das vom Treiber zurückgegebene Zeitintervall pro
    Sample. Es wird die Timebase gewählt, deren effektive Abtastrate am
    nächsten an ``target_fs`` liegt und für die der Treiber einen gültigen
    Status zurückgibt.

    Parameters
    ----------
    handle : ctypes.c_int16 oder ähnliches
        Gerät-Handle wie von :func:`ps.ps3000aOpenUnit` zurückgegeben.
    target_fs : float
        Gewünschte Abtastrate in Hertz (z. B. ``20e6`` für 20 MHz).
    n_samples : int
        Gewünschte Anzahl Samples im Block (inkl. Pre/Post-Trigger).

    Returns
    -------
    tuple
        Ein Tripel ``(timebase_index, dt_seconds, fs_hz)``:
        - ``timebase_index`` (int): Index, der später an ``ps3000aRunBlock``
          übergeben werden soll.
        - ``dt_seconds`` (float): Zeitabstand zwischen zwei Samples in
          Sekunden (``time_interval_ns * 1e-9``).
        - ``fs_hz`` (float): Effektive Abtastfrequenz in Hertz (``1/dt_seconds``).

    Raises
    ------
    RuntimeError
        Falls keine gültige Timebase gefunden werden konnte (z. B. wenn der
        Treiber alle abgefragten Indizes als ungültig meldet).
    """
    best = None
    for tb in range(1, 50000):
        time_interval_ns = ct.c_float()  # Zeitintervall pro Sample in ns
        max_samples = ct.c_int32()       # Maximal mögliche Samples bei dieser Timebase
        status = ps.ps3000aGetTimebase2( # Versuch mit Timebase 'tb'
            handle, tb, n_samples, ct.byref(time_interval_ns), 0, ct.byref(max_samples), 0
        )
        if status == 0:
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

# ---------- CSV-Helfer ----------
def _csv_header(i_unit):
    return (
        f"# RUN_NAME={RUN_NAME}\n"
        f"# created={datetime.now().isoformat()}\n"
        f"# columns: pulse_id,sample_idx,time_s,u_V,i_{i_unit}\n"
    )

def _csv_exists_write_header(i_unit):
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", encoding="utf-8") as f:
            f.write(_csv_header(i_unit))

def _next_pulse_id_scan():
    """Liest die höchste pulse_id aus der CSV (nur einmal zu Beginn nötig)."""
    if not os.path.exists(CSV_PATH):
        return 1
    last_id = 0
    with open(CSV_PATH, "r", encoding="utf-8") as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            try:
                pid = int(line.split(",")[0])
                if pid > last_id:
                    last_id = pid
            except Exception:
                pass
    return last_id + 1

def append_csv_with_id(t, u, i, i_unit, pulse_id):
    """Hängt einen Puls mit gegebener pulse_id an die CSV an (ohne erneuten Scan)."""
    _csv_exists_write_header(i_unit)
    n = len(t)
    data = np.column_stack([
        np.full(n, pulse_id, dtype=np.int64),
        np.arange(n, dtype=np.int64),
        t.astype(np.float64, copy=False),
        u.astype(np.float64, copy=False),
        i.astype(np.float64, copy=False),
    ])
    with open(CSV_PATH, "a", encoding="utf-8") as f:
        np.savetxt(f, data, delimiter=",",
                   fmt=["%d", "%d", "%.9e", "%.9e", "%.9e"])

def write_meta_once(meta):
    """Schreibt/aktualisiert eine Meta-JSON zum Run (Sampling, Bereiche etc.)."""
    meta_out = dict(meta)
    meta_out["run_name"] = RUN_NAME
    meta_out["csv_path"] = CSV_PATH
    meta_out["created_or_updated"] = datetime.now().isoformat()
    with open(META_PATH, "w", encoding="utf-8") as f:
        json.dump(meta_out, f, indent=2)

# ---------- Session-Erfassung für N Pulse ----------
def acquire_n_pulses(n_pulses=N_PULSES, inter_pulse_delay_s=INTER_PULSE_DELAY_S):
    """
    Erfasst n_pulses synchron auf CH A/B und hängt sie an die Run-CSV an.
    Gerät wird nur einmal geöffnet/konfiguriert.
    """
    # Gerät öffnen
    handle = ct.c_int16() # Gerät-Handle
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
        # Kanäle
        assert_pico_ok(ps.ps3000aSetChannel(handle, CH_A, 1, COUPLING_A, RANGE_A, 0.0))
        assert_pico_ok(ps.ps3000aSetChannel(handle, CH_B, 1, COUPLING_B, RANGE_B, 0.0))

        # Max-ADC
        max_adc = ct.c_int16() 
        assert_pico_ok(ps.ps3000aMaximumValue(handle, ct.byref(max_adc)))

        # Timebase
        timebase, dt, fs = pick_timebase(handle, TARGET_FS, N_SAMPLES) 
        print(f"[Info] Timebase={timebase}, dt={dt*1e9:.2f} ns, fs={fs/1e6:.2f} MS/s")

        # Trigger (CH A, Rising)
        vfs_a = range_fullscale_volts(RANGE_A)
        vfs_b = range_fullscale_volts(RANGE_B)
        trig_adc = int((TRIG_LEVEL_V / vfs_a) * max_adc.value)
        assert_pico_ok(ps.ps3000aSetSimpleTrigger(
            handle, 1, CH_A, trig_adc,
            ps.PS3000A_THRESHOLD_DIRECTION["PS3000A_FALLING"],
            0, int(AUTO_TRIG_MS)
        ))

        # Puffer
        bufA = (ct.c_int16 * N_SAMPLES)()
        bufB = (ct.c_int16 * N_SAMPLES)()
        assert_pico_ok(ps.ps3000aSetDataBuffer(
            handle, CH_A, ct.byref(bufA), N_SAMPLES, 0,
            ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"]))
        assert_pico_ok(ps.ps3000aSetDataBuffer(
            handle, CH_B, ct.byref(bufB), N_SAMPLES, 0,
            ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"]))

        pre  = int(PRETRIG_RATIO * N_SAMPLES)
        post = N_SAMPLES - pre
        t = np.arange(N_SAMPLES) * dt

        # Header & Meta einmalig schreiben
        i_unit = "A" if (ROGOWSKI_V_PER_A and ROGOWSKI_V_PER_A > 0) else "V"
        _csv_exists_write_header(i_unit)
        write_meta_once(dict(
            run_name=RUN_NAME, fs=fs, dt_s=dt,
            pretrigger_samples=pre, posttrigger_samples=post,
            ch_a=dict(coupling="AC", v_range=vfs_a),
            ch_b=dict(coupling="AC", v_range=vfs_b, rogowski_v_per_a=ROGOWSKI_V_PER_A),
            trigger_level_v=TRIG_LEVEL_V,
        ))

        # Start-pulse_id nur einmal ermitteln
        pulse_id = _next_pulse_id_scan()


        time.sleep(10)
        # Schleife über Pulse
        for k in range(n_pulses):
            time_indisposed_ms = ct.c_int32(0)
            assert_pico_ok(ps.ps3000aRunBlock(
                handle, pre, post, timebase, int(OVERSAMPLE),
                ct.byref(time_indisposed_ms), 0, None, None))

            # Warten bis fertig
            ready = ct.c_int16(0)
            while not ready.value:
                ps.ps3000aIsReady(handle, ct.byref(ready))
                time.sleep(0.001)

            # Werte holen
            n = ct.c_int32(N_SAMPLES)
            overflow = ct.c_int16()
            assert_pico_ok(ps.ps3000aGetValues(
                handle, 0, ct.byref(n), 1,
                ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"],
                0, ct.byref(overflow)))

            # Umrechnen
            adcA = np.frombuffer(bufA, dtype=np.int16, count=n.value).astype(np.float64, copy=False)
            adcB = np.frombuffer(bufB, dtype=np.int16, count=n.value).astype(np.float64, copy=False)
            
            
            u = adcA * (vfs_a / max_adc.value) * U_PROBE_ATTENUATION
            i_v = adcB * (vfs_b / max_adc.value)
            i = i_v / ROGOWSKI_V_PER_A

            print(f'Spannung U (CH A): min={u.min():.3f} V  max={u.max():.3f} V')
            print(f'Strom I (CH B): min={i.min():.3f} {i_unit}  max={i.max():.3f} {i_unit}')
            print(f"{k}")

            # An CSV anhängen (mit fixer pulse_id)
            append_csv_with_id(t, u, i, i_unit, pulse_id)
            print(f"[OK] pulse_id={pulse_id}  rows={n.value}  overflow={overflow.value}  timeIndisposed={time_indisposed_ms.value} ms")

            pulse_id += 1  # nächster Puls
            if inter_pulse_delay_s > 0:
                time.sleep(inter_pulse_delay_s)

            # (Optional) Stop ist bei Block-Mode nicht notwendig; Treiber handled nächste Armierung.
            # Falls nötig: ps.ps3000aStop(handle)

    finally:
        try:
            ps.ps3000aStop(handle)
        except Exception:
            pass
        ps.ps3000aCloseUnit(handle)

# --------- Start ---------
if __name__ == "__main__":
    acquire_n_pulses()
