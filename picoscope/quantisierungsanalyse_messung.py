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


print(len(df["time_us"]), len(df["u_V"]), len(df["i_A"]))
print(df["u_V"].unique()[:20])
print(len(df["u_V"].unique()))

print("u_V unique:", len(df["u_V"].unique()))
print("i_A unique:", len(df["i_A"].unique()))

print("u_V min/max:", df["u_V"].min(), df["u_V"].max())
print("i_A min/max:", df["i_A"].min(), df["i_A"].max())



# plt.show()

