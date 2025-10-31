import pandas as pd
import matplotlib.pyplot as plt


################## DATEN INPUT ##################
# Pfad zu CSV
csv_path = "MEXT/picoscope/Runs/90V_DC_300A/90V_DC_300A.csv"

# CSV einlesen in panda Dataframe
df = pd.read_csv(
    csv_path, 
    comment="#", # alle Zeilen die mit # starten werden übersprungen 
    header=None, # Spaltennamen werden selber angegeben
)



################## DATEN VERARBEITUNG ##################
# Spaltennamen setzen, da CSV nicht passend konfiguriert
df.columns = ["pulse_id", "sample_idx", "time_s", "u_V", "i_A"]

df["time_us"] = df["time_s"] * 1e6 # neue Spalte in micro sekunden

# als test nur den ersten Puls nehmen
pulse_1_df  = df[df["pulse_id"] == 1]



################## DATEN DARSTELLUNG ##################
# Plot erstellen
"""
plt.figure()
plt.plot(pulse_1_df["time_s"], pulse_1_df["u_V"], label="Spannung u_V")
plt.plot(pulse_1_df["time_s"], pulse_1_df["i_A"], label="Strom i_A")
"""
fig, (ax1, ax2) = plt.subplots(2,1,sharex=True, figsize=(8,6))

ax1.plot(pulse_1_df["time_us"], pulse_1_df["u_V"], color="tab:blue")
ax1.set_ylabel("Spannung am Kondensator [V]")
ax1.grid(True)

ax2.plot(pulse_1_df["time_us"], pulse_1_df["i_A"], color="tab:red")
ax2.set_ylabel("Strom Kondensator -> Vollbrücke[V]")
ax2.set_xlabel("Zeit [µs]")
ax2.grid(True)

plt.title("Messung 90V DC geschätzter Spulenstrom 300A")
plt.legend()

print(len(df["time_us"]), len(df["u_V"]), len(df["i_A"]))
print(df["u_V"].unique()[:20])
print(len(df["u_V"].unique()))


plt.show()

