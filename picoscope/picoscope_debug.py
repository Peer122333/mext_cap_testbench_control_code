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