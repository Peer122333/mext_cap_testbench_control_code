import pandas as pd
import matplotlib.pyplot as plt

################## DATEN INPUT ##################
# Pfad zu CSV (anpassen!)
csv_path = "MEXT/picoscope/Picoscope_Oszi_Tool_Export/20251027-0003_15Pulse_02.csv"

# CSV einlesen: Semikolon, Komma als Dezimaltrennzeichen, 2 Kopfzeilen überspringen
df = pd.read_csv(
    csv_path,
    sep=";",          # Trenner ;
    decimal=",",      # Dezimaltrennzeichen ,
    skiprows=2,       # erste zwei Zeilen: "Zeit;Kanal A;Kanal B" und Einheiten
    header=None,      # wir setzen Namen selbst
)

################## DATEN VERARBEITUNG ##################
# Spalten benennen gemäß Datei
df.columns = ["time_ms", "u_V", "i_A"]

# Zeit in µs umrechnen
df["time_us"] = df["time_ms"] * 1_000.0

################## DATEN DARSTELLUNG ##################
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

# Spannung oben
ax1.plot(df["time_us"], df["u_V"], color="tab:blue")
ax1.set_ylabel("Kanal A / Spannung [V]")
ax1.grid(True)

# Strom unten
ax2.plot(df["time_us"], df["i_A"], color="tab:red", label="Kanal B / Strom [A]")
ax2.set_xlabel("Zeit [µs]")
ax2.set_ylabel("Strom [A]")
ax2.grid(True)
ax2.legend()

# Titel lieber auf die Figure legen, nicht auf plt (sonst sitzt er über beiden Achsen komisch)
fig.suptitle("Messung – CSV PicoScope Import")

plt.tight_layout()
plt.show()

################## DEBUG INFOS ##################
print(len(df["time_us"]), len(df["u_V"]), len(df["i_A"]))
print(df["u_V"].unique()[:20])
print(len(df["u_V"].unique()))
