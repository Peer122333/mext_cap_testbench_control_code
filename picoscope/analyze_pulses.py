"""
Analyse CSV-basierter U/I-Pulse (ohne CLI)
- Liest Runs/<RUN_NAME>/<RUN_NAME>.csv und die zugehörige meta.json
- Plottet Spannung U und Strom I in einem großen Fenster (2 Zeilen, gemeinsame Zeitachse)
- Y-Achsen automatisch auf vollen Messbereich (±v_range aus Meta)
- Optional: mehrere Pulse überlagern (per Liste OVERLAY_IDS)
- Optional: FFT (nur der Hauptpuls)
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt

# ===================== CONTROL =====================
BASE_DIR     = r"C:\Users\mext\Documents\02 Python Schnittstelle STM32 serielle Steuerung\mext_cap_testbench_control_code\picoscope"
RUN_NAME     = "90V_DC_300A-3"   # muss zum Messlauf passen (CSV + meta.json)
USE_LAST     = True            # True: neuesten pulse_id verwenden; False: PULSE_ID nutzen
PULSE_ID     = 3               # nur wenn USE_LAST=False

OVERLAY_IDS  = [1,2,3]              # z.B. [1,2,5] -> zusätzliche Pulse überlagern
SHOW_FFT     = False           # FFT des Hauptpulses
FIG_SIZE     = (13, 8)         # großes Fenster
LINEWIDTH    = 1.1
GRID_ALPHA   = 0.25
# ===================================================

# Pfade
RUN_DIR   = os.path.join(BASE_DIR, "Runs", RUN_NAME)
print(RUN_DIR)
CSV_PATH  = os.path.join(RUN_DIR, f"{RUN_NAME}.csv")
META_PATH = os.path.join(RUN_DIR, f"{RUN_NAME}.meta.json")


# --------- Helpers ---------
def read_meta():
    if not os.path.isfile(META_PATH):
        raise FileNotFoundError(f"Meta-Datei fehlt: {META_PATH}")
    with open(META_PATH, "r", encoding="utf-8") as f:
        return json.load(f)

def detect_i_unit_from_header():
    """Liest Kopfzeilen (# ...) und erkennt die I-Spaltenbezeichnung (i_A oder i_V)."""
    if not os.path.isfile(CSV_PATH):
        raise FileNotFoundError(f"CSV nicht gefunden: {CSV_PATH}")
    i_colname = "i_V"
    with open(CSV_PATH, "r", encoding="utf-8") as f:
        for line in f:
            if not line.startswith("#"):
                break
            if "columns:" in line:
                # Beispiel-Zeile: "# columns: pulse_id,sample_idx,time_s,u_V,i_A"
                cols = line.split("columns:")[-1].strip()
                parts = [p.strip() for p in cols.split(",")]
                for p in parts:
                    if p.startswith("i_"):
                        i_colname = p
                        break
    return i_colname  # "i_A" oder "i_V"

def get_last_pulse_id():
    """Liest die größte pulse_id aus der CSV (letzte Zeilen scannen)."""
    if not os.path.isfile(CSV_PATH):
        raise FileNotFoundError(f"CSV nicht gefunden: {CSV_PATH}")
    last_id = None
    # effizient genug für typische Dateigrößen; für extrem große CSV ggf. anders lösen
    with open(CSV_PATH, "r", encoding="utf-8") as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            try:
                pid = int(line.split(",")[0])
                last_id = pid if (last_id is None or pid > last_id) else last_id
            except Exception:
                pass
    if last_id is None:
        raise ValueError("Keine Datenzeilen in CSV gefunden.")
    return last_id

def read_pulse_from_csv(pulse_id, i_colname):
    """
    Liest einen Puls (alle Zeilen mit pulse_id) aus der CSV.
    Gibt (t, u, i) zurück.
    """
    if not os.path.isfile(CSV_PATH):
        raise FileNotFoundError(f"CSV nicht gefunden: {CSV_PATH}")

    # Spalten: pulse_id,sample_idx,time_s,u_V,i_{V|A}
    # Wir lesen nur Datenzeilen (skip '#'), dann filtern wir auf pulse_id.
    # Für moderate Dateigrößen ist das okay. Bei >GB CSV später auf chunking umstellen.
    rows = []
    with open(CSV_PATH, "r", encoding="utf-8") as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            parts = line.strip().split(",")
            if len(parts) < 5:
                continue
            try:
                pid = int(parts[0])
            except:
                continue
            if pid == pulse_id:
                # sample_idx = int(parts[1])  # würde die Reihenfolge sichern, falls nötig
                t = float(parts[2])
                u = float(parts[3])
                i = float(parts[4])
                rows.append((t, u, i))

    if not rows:
        raise FileNotFoundError(f"Pulse-ID {pulse_id} nicht in CSV gefunden.")

    # nach Zeit sortieren (falls Zeilenreihenfolge nicht sicher)
    rows.sort(key=lambda r: r[0])
    data = np.array(rows, dtype=float)
    t = data[:, 0]
    u = data[:, 1]
    i = data[:, 2]
    return t, u, i

def plot_fft(y, fs, title="FFT"):
    N = len(y)
    if N == 0 or not fs:
        return
    win = np.hanning(N)
    Y = np.fft.rfft(y * win)
    freqs = np.fft.rfftfreq(N, 1.0 / fs)
    amp = np.abs(Y) / N * 2.0
    plt.figure()
    plt.semilogy(freqs, amp)
    plt.xlabel("Frequenz [Hz]")
    plt.ylabel("Amplitude")
    plt.title(title)
    plt.grid(True)


# --------- Hauptlogik ---------
def analyze_pulse_csv():
    if not os.path.isdir(RUN_DIR):
        raise NotADirectoryError(f"Run-Ordner nicht gefunden: {RUN_DIR}")
    meta = read_meta()
    fs   = meta.get("fs")
    dt   = meta.get("dt_s")
    v_range_u = meta.get("ch_a", {}).get("v_range", None)
    v_range_i_v = meta.get("ch_b", {}).get("v_range", None)
    rogowski_v_per_a = meta.get("ch_b", {}).get("rogowski_v_per_a", None)

    # I-Einheit erkennen (A oder V)
    i_colname = detect_i_unit_from_header()
    i_unit = "A" if i_colname == "i_A" else "V"

    # Hauptpuls-ID
    pid = get_last_pulse_id() if USE_LAST else int(PULSE_ID)

    # Hauptpuls laden
    t, u, i = read_pulse_from_csv(pid, i_colname)

    # y-Limits aus Meta: voller Messbereich ±v_range
    if v_range_u:
        u_ylim = (-abs(v_range_u), abs(v_range_u))
    else:
        u_ylim = (-max(abs(u.min()), abs(u.max())), max(abs(u.min()), abs(u.max())))

    if i_unit == "A" and (v_range_i_v is not None) and (rogowski_v_per_a and rogowski_v_per_a > 0):
        # v_range_i_v ist der Spannungsbereich am Kanal B; in A umrechnen
        i_range = v_range_i_v / rogowski_v_per_a
        i_ylim  = (-abs(i_range), abs(i_range))
    elif (i_unit == "V") and (v_range_i_v is not None):
        i_ylim  = (-abs(v_range_i_v), abs(v_range_i_v))
    else:
        i_ylim  = (-max(abs(i.min()), abs(i.max())), max(abs(i.min()), abs(i.max())))

    # ---------- Plot: großes Fenster, 2 Zeilen ----------
    fig, (ax_u, ax_i) = plt.subplots(2, 1, figsize=FIG_SIZE, sharex=True, constrained_layout=True)

    # Spannung
    ax_u.plot(t, u, linewidth=LINEWIDTH, label=f"U  (pulse {pid})")
    ax_u.set_ylabel("Spannung U [V]", fontsize=12)
    ax_u.set_ylim(u_ylim)
    ax_u.set_title(f"{RUN_NAME} – Spannung & Strom (pulse_id={pid})", fontsize=15)
    ax_u.grid(True, alpha=GRID_ALPHA)

    # Strom
    ax_i.plot(t, i, linewidth=LINEWIDTH, label=f"I  (pulse {pid})")
    ax_i.set_ylabel(f"Strom I [{i_unit}]", fontsize=12)
    ax_i.set_xlabel("Zeit t [s]", fontsize=12)
    ax_i.set_ylim(i_ylim)
    ax_i.grid(True, alpha=GRID_ALPHA)

    # Overlays (optional)
    for pid_ov in OVERLAY_IDS:
        try:
            t_o, u_o, i_o = read_pulse_from_csv(int(pid_ov), i_colname)
            ax_u.plot(t_o, u_o, linewidth=0.9, alpha=0.65, label=f"U (pulse {pid_ov})")
            ax_i.plot(t_o, i_o, linewidth=0.9, alpha=0.65, label=f"I (pulse {pid_ov})")
        except Exception as e:
            print(f"[Warn] Overlay {pid_ov}: {e}")

    ax_u.legend(loc="best", fontsize=9)
    ax_i.legend(loc="best", fontsize=9)

    # Schöne Ticks
    for ax in (ax_u, ax_i):
        ax.tick_params(axis='both', labelsize=11)

    plt.show()

    # FFT (optional, nur Hauptpuls)
    if SHOW_FFT and fs:
        plot_fft(u, fs, f"FFT U (pulse {pid})")
        plot_fft(i, fs, f"FFT I (pulse {pid})")
        plt.show()


# Auto-Start
if __name__ == "__main__":
    analyze_pulse_csv()
