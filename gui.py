import tkinter as tk                # TKinter-Basis (Widgets, Fenster)
from tkinter import ttk, messagebox # Themed Widgets, Messageboxen
import threading                    # für Threads, hier genutzt für nicht-blockierende UI
import time 
import queue                        # für Thread-Kommunikation (RX-Monitor)



try:
    from serial.tools import list_ports # für Port-Auswahl in GUI
except Exception:
    list_ports = None # fallback

from nucleo_uart import NucleoUART # Import der NucleoUART-Klasse

class App:
    def __init__(self, root):
        self.root = root                        # Referenz auf das Hauptfenster
        self.root.title("STM32 UART Control")   # Fenstertitel
        self.nuc = None                         # Platzhalter für NucleoUART-Instanz
        self._build_ui()                        # UI aufbauen
        self._set_connected(False)              # Initialer Verbindungsstatus

        # --- Serial Monitor (RX) ---
        self._rx_q = queue.Queue()          # Queue für empfangene Daten
        self._reader_stop = True               # Flag zum Stoppen des Lesethreads

    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        # Verbindung
        frm_conn = ttk.LabelFrame(self.root, text="Verbindung")
        frm_conn.grid(row=0, column=0, sticky="ew", **pad)

        ttk.Label(frm_conn, text="Port:").grid(row=0, column=0, sticky="w")
        self.cmb_port = ttk.Combobox(frm_conn, width=28, state="readonly")
        self.cmb_port.grid(row=0, column=1, sticky="ew")
        ttk.Button(frm_conn, text="↻", width=3, command=self.refresh_ports).grid(row=0, column=2)
        ttk.Label(frm_conn, text="Baud:").grid(row=0, column=3, sticky="w")
        self.cmb_baud = ttk.Combobox(frm_conn, width=8, state="readonly",
                                     values=["115200","230400","460800","921600"])
        self.cmb_baud.set("115200")
        self.cmb_baud.grid(row=0, column=4, sticky="w")

        self.btn_connect = ttk.Button(frm_conn, text="Verbinden", command=self.connect)
        self.btn_disconnect = ttk.Button(frm_conn, text="Trennen", command=self.disconnect)
        self.btn_connect.grid(row=0, column=5, padx=(12,0))
        self.btn_disconnect.grid(row=0, column=6)

        # SET
        frm_set = ttk.LabelFrame(self.root, text="Timer konfigurieren (SET)")
        frm_set.grid(row=1, column=0, sticky="ew", **pad)

        self.timer_var = tk.IntVar(value=1)
        ttk.Radiobutton(frm_set, text="Timer 1 (µs)", variable=self.timer_var, value=1).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(frm_set, text="Timer 2 (ms)", variable=self.timer_var, value=2).grid(row=0, column=1, sticky="w")

        ttk.Label(frm_set, text="Periode:").grid(row=0, column=2, sticky="e")
        self.ent_period = ttk.Entry(frm_set, width=10)
        self.ent_period.insert(0, "200")
        self.ent_period.grid(row=0, column=3, sticky="w")

        # (Optionales Flags-Feld kann bleiben; SET sendet intern flags=0)
        ttk.Label(frm_set, text="Flags (ignoriert):").grid(row=0, column=4, sticky="e")
        self.ent_flags = ttk.Entry(frm_set, width=6)
        self.ent_flags.insert(0, "0")
        self.ent_flags.grid(row=0, column=5, sticky="w")

        self.btn_set = ttk.Button(frm_set, text="SET", command=self.on_set)
        self.btn_set.grid(row=0, column=6, padx=(12,0))

        # Sequenz
        frm_seq = ttk.LabelFrame(self.root, text="Sequenz")
        frm_seq.grid(row=2, column=0, sticky="ew", **pad)

        ttk.Label(frm_seq, text="Pulse-Count (0=endlos):").grid(row=0, column=0, sticky="e")
        self.ent_pulses = ttk.Entry(frm_seq, width=10)
        self.ent_pulses.insert(0, "0")
        self.ent_pulses.grid(row=0, column=1, sticky="w")

        self.btn_start = ttk.Button(frm_seq, text="START", command=self.on_start)
        self.btn_start.grid(row=0, column=2, padx=(12,0))

        # ⬇️ NEU: Dropdown für Soft/Hard Stop
        ttk.Label(frm_seq, text="Stop-Mode:").grid(row=0, column=3, sticky="e")
        self.stop_mode = ttk.Combobox(frm_seq, state="readonly", width=8, values=["Soft", "Hard"])
        self.stop_mode.set("Soft")
        self.stop_mode.grid(row=0, column=4, sticky="w")

        self.btn_stop  = ttk.Button(frm_seq, text="STOP", command=self.on_stop)
        self.btn_stop.grid(row=0, column=5)

        # Readback
        frm_rb = ttk.LabelFrame(self.root, text="Readback")
        frm_rb.grid(row=3, column=0, sticky="ew", **pad)

        self.btn_rb_t1 = ttk.Button(frm_rb, text="READBACK T1", command=lambda: self.on_readback(1))
        self.btn_rb_t2 = ttk.Button(frm_rb, text="READBACK T2", command=lambda: self.on_readback(2))
        self.btn_rb_t1.grid(row=0, column=0, padx=(0,6))
        self.btn_rb_t2.grid(row=0, column=1, padx=(6,0))

        # Log
        frm_log = ttk.LabelFrame(self.root, text="Log")
        frm_log.grid(row=4, column=0, sticky="nsew", **pad)

        self.txt = tk.Text(frm_log, height=14, width=90)
        self.txt.grid(row=0, column=0, sticky="nsew")
        self.txt.configure(state="disabled")
        scroll = ttk.Scrollbar(frm_log, command=self.txt.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.txt["yscrollcommand"] = scroll.set

        self.root.columnconfigure(0, weight=1)
        frm_log.rowconfigure(0, weight=1)
        frm_log.columnconfigure(0, weight=1)


        # --- Serial Monitor (RX) ---
        frm_mon = ttk.LabelFrame(self.root, text="Serial Monitor (RX)")
        frm_mon.grid(row=5, column=0, sticky="nsew", **pad)

        self.txt_mon = tk.Text(frm_mon, height=10, width=90)
        self.txt_mon.grid(row=0, column=0, sticky="nsew")
        self.txt_mon.configure(state="disabled")
        scroll_mon = ttk.Scrollbar(frm_mon, command=self.txt_mon.yview)
        scroll_mon.grid(row=0, column=1, sticky="ns")
        self.txt_mon["yscrollcommand"] = scroll_mon.set

        frm_mon.rowconfigure(0, weight=1)
        frm_mon.columnconfigure(0, weight=1)
        self.root.rowconfigure(5, weight=1)

        # Clear-Button fürs Leeren beider Fenster
        btn_clear = ttk.Button(frm_log, text="Clear All", command=self.clear_logs)
        btn_clear.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4,0))


        self.refresh_ports()

    # Helpers, Connect/Disconnect und Actions bleiben unverändert …
    def _set_connected(self, ok: bool):
        self.btn_connect.configure(state=("disabled" if ok else "normal"))      # Verbinden deaktivieren bei Verbindung
        self.btn_disconnect.configure(state=("normal" if ok else "disabled"))   # Trennen aktivieren bei Verbindung
        state = "normal" if ok else "disabled"                                  # Buttons aktivieren/deaktivieren           
        for w in [self.btn_set, self.btn_start, self.btn_stop, self.btn_rb_t1, self.btn_rb_t2]:
            w.configure(state=state)                                            # alle Buttons aktivieren/deaktivieren

    def log(self, msg: str):
        self.txt.configure(state="normal")      # Log-Textfeld beschreibbar machen
        self.txt.insert("end", msg + "\n")      # Nachricht einfügen
        self.txt.see("end")                     # zum Ende scrollen
        self.txt.configure(state="disabled")    # Log-Textfeld wieder schreibgeschützt machen

    def refresh_ports(self):
        ports = []                              # Liste der verfügbaren Ports
        if list_ports:
            ports = [p.device for p in list_ports.comports()]   # verfügbare Ports abfragen
        if not ports:
            ports = ["/dev/tty.usbmodem1103", "/dev/tty.usbserial", "/dev/cu.usbmodem", "/dev/cu.usbserial"] # Fallback
        self.cmb_port["values"] = ports
        if ports and not self.cmb_port.get():
            self.cmb_port.set(ports[0]) # ersten Port auswählen

    def connect(self):
        port = self.cmb_port.get().strip()      # ausgewählten Port holen
        baud = int(self.cmb_baud.get().strip()) # ausgewählte Baudrate holen
        if not port:                            # kein Port ausgewählt
            messagebox.showwarning("Hinweis", "Bitte zuerst einen Port auswählen.") # Warnung
            return
        try:
            self.nuc = NucleoUART(port=port, baudrate=baud, timeout=1.0) # Verbindung aufbauen
            self._set_connected(True)                                    # UI anpassen
            self.log(f"[OK] Verbunden mit {port} @ {baud} Baud")         # Log-Eintrag
            # RX-Monitor starten
            self._reader_stop = False
            threading.Thread(target=self._serial_monitor_reader, daemon=True).start()
            self.root.after(50, self._drain_monitor_queue)

        except Exception as e:             # Verbindungsfehler abfangen         
            messagebox.showerror("Verbindung fehlgeschlagen", str(e))   # Fehlermeldung
            self.log(f"[ERR] {e}")                                      # Log-Eintrag

    def disconnect(self):
        self._reader_stop = True   # RX-Thread stoppen
        if self.nuc:    # Verbindung besteht
            try:
                self.nuc.close() # serielle Schnittstelle schließen
            except Exception:
                pass
            self.nuc = None     # Referenz auf NucleoUART-Instanz entfernen
        self._set_connected(False) # UI zurücksetzen
        self.log("[i] Verbindung getrennt") # Log-Eintrag

    def _in_thread(self, target):                             # Hilfsmethode für nicht-blockierende UI
        threading.Thread(target=target, daemon=True).start() # Daemon-Thread starten
        # ausführlichere Erläuterung für _in_thread:
        # - threading.Thread: Erstellt einen neuen Thread
        # - target=target: Die Funktion, die im neuen Thread ausgeführt wird
        # - daemon=True: Der Thread wird als Daemon-Thread gestartet, d.h.
        #   er beendet sich automatisch, wenn das Hauptprogramm endet
        # - .start(): Startet den Thread und führt die Ziel-Funktion aus
        # Diese Methode wird genutzt, um lange laufende Operationen (z.B. serielle Kommunikation)
        # in einem separaten Thread auszuführen, damit die GUI reaktionsfähig bleibt.

    def on_set(self):
        def work():
            if not self.nuc: return                                     # keine Verbindung
            try:                                                        
                timer = self.timer_var.get()                            # ausgewählten Timer holen           
                period = int(self.ent_period.get())                     # Periode holen        
                self.log(f"> SET T{timer} period={period} (flags=0)")   # Log-Eintrag
                self.nuc.set_timer(timer, period)  # flags intern 0 gesetzt
            except Exception as e:  
                self.log(f"[ERR] SET: {e}")
        self._in_thread(work)

    def on_start(self):
        def work():
            if not self.nuc: return
            try:
                pulses = int(self.ent_pulses.get())                 # Pulse-Count holen
                self.log(f"> START sequence pulse_count={pulses}")  # Log-Eintrag
                self.nuc.start_sequence(pulses)                     # START senden
            except Exception as e:
                self.log(f"[ERR] START: {e}")
        self._in_thread(work)

    def on_stop(self):
        def work():
            if not self.nuc:
                return
            try:
                mode = self.stop_mode.get()                     # Stop-Mode holen
                hard = (mode == "Hard")                         # Hard-Flag setzen
                self.log(f"> STOP ({mode})")                    # Log-Eintrag
                self.nuc.stop_timer(hard=hard, timer_for_cmd=1)  # 0x30/0x31
            except Exception as e:
                self.log(f"[ERR] STOP: {e}")
        self._in_thread(work)


    def on_readback(self, timer: int):
        def work():
            if not self.nuc: return
            try:
                val = self.nuc.readback(timer)
                unit = "µs" if timer == 1 else "ms"
                self.log(f"< READBACK T{timer}: {val} {unit}, flags=0x{0:02X}")
            except Exception as e:
                self.log(f"[ERR] READBACK T{timer}: {e}")
        self._in_thread(work)


    # --- Serial Monitor (RX) ---
    def mon(self, msg: str):
        """Schreibt eine Zeile in den RX-Monitor."""
        self.txt_mon.configure(state="normal")
        self.txt_mon.insert("end", msg + "\n")
        self.txt_mon.see("end")
        self.txt_mon.configure(state="disabled")

    def _drain_monitor_queue(self):
        """holt periodisch RX-Zeilen aus der Queue und zeigt sie im Monitor an"""
        try:
            while True:
                line = self._rx_q.get_nowait()
                self.mon(line)
        except queue.Empty:
            pass
        self.root.after(50, self._drain_monitor_queue)  # alle 50ms erneut

    def _serial_monitor_reader(self):
        ser = self.nuc.ser if (self.nuc and hasattr(self.nuc, "ser")) else None
        if not ser:
            return

        buf = bytearray()
        while not self._reader_stop and ser and ser.is_open:
            try:
                n = ser.in_waiting
                if n:
                    chunk = ser.read(n)
                    if not chunk:
                        continue
                    buf.extend(chunk)

                    # so lange Zeilenende gefunden wird -> eine Zeile extrahieren
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        text = line.decode(errors="replace").strip()
                        if text:
                            self._rx_q.put(text)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self._rx_q.put(f"[RX-ERR] {e}")
                break

 
    def clear_logs(self):
        """Leert Log- und RX-Monitor-Fenster."""
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")

        self.txt_mon.configure(state="normal")
        self.txt_mon.delete("1.0", "end")
        self.txt_mon.configure(state="disabled")


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
