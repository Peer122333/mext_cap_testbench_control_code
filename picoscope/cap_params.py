"""
Kapazitätsparameter-Berechnung aus Puls-Messdaten.

Dieses Modul berechnet die äquivalenten Schaltkreismodell-Parameter
(ESR: Equivalent Series Resistance, Kapazität) eines Kondensators
aus gemessenen Spannungs- und Strompulsen.

Die Berechnung basiert auf einer FFT-basierten Methode, die ein
lineares Gleichungssystem löst, um die Impedanz-Parameter zu bestimmen.
"""

import numpy as np
from typing import Tuple


def estimate_cap_params(t: np.ndarray, u: np.ndarray, i: np.ndarray) -> Tuple[float, float]:
    """
    Schätzt ESR (Equivalent Series Resistance) und Kapazität aus Puls-Messdaten.
    
    Die Funktion verwendet eine FFT-basierte Analyse, um die Impedanz des
    Kondensators zu bestimmen. Dabei wird ein vereinfachtes Ersatzschaltbild
    angenommen: Serie aus Widerstand (ESR) und Kapazität.
    
    Die Methode löst ein lineares Gleichungssystem im Frequenzbereich:
    U(ω) = ESR * I(ω) + (1/jωC) * I(ω)
    
    Parameters
    ----------
    t : np.ndarray
        Zeitvektor in Sekunden (1D-Array).
        Muss streng monoton steigend sein.
    u : np.ndarray
        Spannungswerte in Volt (1D-Array, gleiche Länge wie t).
        Gemessene Spannung am Kondensator.
    i : np.ndarray
        Stromwerte in Ampere (1D-Array, gleiche Länge wie t).
        Gemessener Strom durch den Kondensator.
        Hinweis: Das Vorzeichen sollte so sein, dass positiver Strom
        eine Entladung darstellt (konsistent mit dem MATLAB-Code).
    
    Returns
    -------
    esr_ohm : float
        Geschätzter Equivalent Series Resistance in Ohm.
        Repräsentiert den Serienwiderstand des Kondensators.
    capacitance_f : float
        Geschätzte Kapazität in Farad.
        Kann in µF oder mF umgerechnet werden (1 µF = 1e-6 F).
    
    Raises
    ------
    ValueError
        Wenn Zeitvektor nicht streng monoton steigend ist,
        oder wenn die Arrays unterschiedliche Längen haben,
        oder wenn keine positiven Frequenzen gefunden werden.
    
    Notes
    -----
    - Die Funktion verwendet Least-Squares-Methode zur Lösung des
      überbestimmten Gleichungssystems im Frequenzbereich.
    - DC-Komponente (f=0) wird ausgeschlossen, da sie keine
      Impedanz-Information liefert.
    - Erste positive Frequenz wird übersprungen (ähnlich MATLAB-Code)
      um numerische Instabilitäten bei sehr kleinen Frequenzen zu vermeiden.
    - Nur positive Frequenzen werden verwendet (einseitiges Spektrum).
    
    Examples
    --------
    >>> import numpy as np
    >>> # Beispiel: Synthetische Daten
    >>> t = np.linspace(0, 1e-3, 1000)  # 1 ms, 1000 Samples
    >>> u = np.exp(-t / 1e-4) * 10  # Exponentieller Abfall
    >>> i = -np.diff(np.pad(u, (0, 1)))  # Vereinfachter Strom
    >>> esr, cap = estimate_cap_params(t, u, i)
    >>> print(f"ESR: {esr:.6f} Ω, C: {cap*1e6:.6f} µF")
    """
    # Eingabevalidierung
    t = np.asarray(t, dtype=float)
    u = np.asarray(u, dtype=complex)  # Komplex erlauben (für FFT)
    i = np.asarray(i, dtype=complex)
    
    if len(t) != len(u) or len(t) != len(i):
        raise ValueError("Arrays t, u, i müssen gleiche Länge haben")
    
    # Zeitvektor prüfen: muss streng monoton steigend sein
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError("Zeitvektor muss streng monoton steigend sein")
    
    # Abtastfrequenz aus mittlerem Zeitabstand berechnen
    fs = 1.0 / np.mean(dt)
    N = t.size
    
    # FFT berechnen (shifted für symmetrische Darstellung)
    fU = np.fft.fftshift(np.fft.fft(u))
    fI = np.fft.fftshift(np.fft.fft(i))
    
    # Frequenzachse generieren (shifted)
    f = np.fft.fftshift(np.fft.fftfreq(N, d=1.0 / fs))
    omega = 2.0 * np.pi * f  # Kreisfrequenz
    
    # Nur positive Frequenzen verwenden (DC ausschließen)
    pos_idx = np.where(f > 0)[0]
    if pos_idx.size == 0:
        raise ValueError("Keine positiven Frequenzen gefunden (N zu klein?)")
    
    # Erste positive Frequenz überspringen (wie im MATLAB-Code)
    # Vermeidet numerische Instabilitäten bei sehr kleinen Omega
    if pos_idx.size > 1:
        idx = pos_idx[1:]  # Ab Index 1
    else:
        idx = pos_idx  # Fallback: nur eine positive Frequenz
    
    # Lineares Gleichungssystem aufbauen: A * x = b
    # U(ω) = ESR * I(ω) + (1/jωC) * I(ω)
    # A = [I(ω), -I(ω)/(jω)]
    # x = [ESR, 1/C]
    # b = U(ω)
    FI = fI[idx]  # Strom-FFT bei ausgewählten Frequenzen
    OM = omega[idx]  # Kreisfrequenzen
    
    # Vermeide Division durch Null (sollte nicht passieren, da f > 0)
    A = np.column_stack([
        FI,  # Spalte 1: Strom-FFT (für ESR)
        (-1j / OM) * FI  # Spalte 2: Strom-FFT / jω (für 1/C)
    ])
    b = fU[idx]  # Spannungs-FFT (Zielvektor)
    
    # Least-Squares-Lösung (überbestimmtes System)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    
    # Parameter extrahieren
    esr_ohm = float(np.real(x[0]))  # Realteil von x[0] ist ESR
    capacitance_f = float(np.real(1.0 / x[1]))  # Realteil von 1/x[1] ist C
    
    return esr_ohm, capacitance_f

