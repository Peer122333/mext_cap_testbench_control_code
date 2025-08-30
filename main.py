import serial
import time
from enum import IntEnum

PREAMBLE = 0xFF # Startbyte für jedes Frame
FRAME_SIZE = 5 # Frame-Größe in Bytes

class Cmd(IntEnum):
    SET   = 0x10  # Timer1: 0x10, Timer2: 0x11
    START = 0x20  # Timer1: 0x20, Timer2: 0x21
    STOP  = 0x30  # Timer1: 0x30, Timer2: 0x31
    READ  = 0x40  # Timer1: 0x40, Timer2: 0x41

def cmd_for_timer(base: Cmd, timer: int) -> int: # Befehlscode für Timer berechnen
    if timer not in (1, 2):                         # nur Timer 1 oder 2 erlaubt
        raise ValueError("Timer must be 1 or 2")    # Fehler bei ungültigem Timer
    return int(base) + (timer - 1)                  # Timer 1: +0, Timer 2: +1

def u16_to_lsb_msb(val: int) -> tuple[int, int]: # 16-Bit- Zahl in 2 Bytes aufteilen
    v = val & 0xFFFF                                # nur die unteren 16 Bit verwenden     
    return v & 0xFF, (v >> 8) & 0xFF   # LSB, MSB   # zuerst LSB, dann MSB zurückgeben

def lsb_msb_to_u16(lsb: int, msb: int) -> int:  # 2 Bytes zu 16-Bit-Zahl zusammensetzen
    return ((msb & 0xFF) << 8) | (lsb & 0xFF)       # MSB und LSB kombinieren

class NucleoUART:
    """
    Frameformat (6 Byte):
      [0]=0xFF, [1]=CMD, [2]=LSB, [3]=MSB, [4]=FLAGS, [5]=0x00
    """
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0): # seriellen Port öffnen
        self.ser = serial.Serial(
            port=port, 
            baudrate=baudrate, 
            timeout=timeout, 
            bytesize=serial.EIGHTBITS,      # 8 Datenbits
            parity=serial.PARITY_NONE,      # keine Parität
            stopbits=serial.STOPBITS_ONE,   # 1 Stoppbit
            write_timeout=timeout,          # Schreib-Timeout   
        )

    def _build_packet(self, cmd: int, period: int = 0, flags: int = 0) -> bytes: # Frame zusammenbauen
        lsb, msb = u16_to_lsb_msb(period)                                           # Periode in LSB/MSB aufteilen                  
        return bytes([PREAMBLE, cmd, lsb, msb, flags & 0xFF])                       # Frame erstellen

    def _write_packet(self, pkt: bytes):            # Frame senden
        if len(pkt) != 5 or pkt[0] != PREAMBLE:         # Frame muss 5 Byte lang sein und mit PREAMBLE beginnen
            raise ValueError("invalid packet")          # Fehler bei ungültigem Frame
        self.ser.reset_output_buffer()                  # Output-Puffer leeren
        self.ser.write(pkt)                             # Frame senden      
        self.ser.flush()                                # buffer leeren 

    def _read_packet(self) -> bytes:                # Frame empfangen
        # warte auf PREAMBLE
        while True:                                     
            b = self.ser.read(1)                   # ein Byte lesen 
            if not b:                              # Timeout
                raise TimeoutError("UART read timeout waiting for preamble")
            if b[0] == PREAMBLE:                    # PREAMBLE gefunden
                break                               # weitere 5 Bytes lesen       
        return b + self.ser.read(5)                 # restlichen 4 Bytes lesen        

    # -------- High-Level API --------
    def set_timer(self, timer: int, period: int, flags: int = 0): # Timer konfigurieren
        cmd = cmd_for_timer(Cmd.SET, timer)                         # Befehlscode für Timer holen       
        self._write_packet(self._build_packet(cmd, period, flags))  # Frame senden

    def start_timer(self, timer: int):                  # Timer starten
        cmd = cmd_for_timer(Cmd.START, timer)           # Befehlscode für Timer holen
        self._write_packet(self._build_packet(cmd))     # Frame senden

    def stop_timer(self, timer: int):                   # Timer stoppen
        cmd = cmd_for_timer(Cmd.STOP, timer)            # Befehlscode für Timer holen
        self._write_packet(self._build_packet(cmd))     # Frame senden

    def readback(self, timer: int) -> int:              # Timer-Periode auslesen
        cmd = cmd_for_timer(Cmd.READ, timer)            # Befehlscode für Timer holen
        self._write_packet(self._build_packet(cmd))     # Frame senden
        pkt = self._read_packet()                       # Antwort-Frame empfangen
        if pkt[1] != cmd:                               # Antwort prüfen
            raise ValueError(f"unexpected response: got 0x{pkt[1]:02X}") # Fehler bei falscher Antwort
        return lsb_msb_to_u16(pkt[2], pkt[3])           # Periode aus Antwort extrahieren

    def close(self):                                    # Verbindung schließen
        self.ser.close()

    # -------- Testfunktionen --------
    TEST_FRAMES = [
        "FF 10 0A 00 00 00",  # SET Timer1, period=10
        "FF 11 64 00 00 00",  # SET Timer2, period=100
        "FF 20 00 00 00 00",  # START Sequence
        "FF 30 00 00 00 00",  # STOP (soft)
        "FF 40 00 00 00 00",  # READBACK Timer1
        "FF 41 00 00 00 00",  # READBACK Timer2
    ]

    # ------------------------
    # Definiere alle Testfälle
    # ------------------------
    tests = [
        # 1) Happy-Path
        ("T1 SET 250us",    ("set",  1, 250)),
        ("T2 SET 5ms",      ("set",  2, 5)),
        ("START",           ("start",1, 0)),
        ("READBACK T1",     ("read", 1, 0)),
        ("READBACK T2",     ("read", 2, 0)),
        ("STOP (soft)",     ("stop", 1, 0)),

        # 2) Grenzwerte
        ("T1 SET min 10us", ("set",  1, 10)),
        ("T1 SET max 1ms",  ("set",  1, 1000)),
        ("T2 SET min 1ms",  ("set",  2, 1)),
        ("T2 SET max 10s",  ("set",  2, 10000)),

        # 3) Clamping
        ("T1 SET 0us",      ("set",  1, 0)),
        ("T1 SET 2000us",   ("set",  1, 2000)),
        ("T2 SET 0ms",      ("set",  2, 0)),
        ("T2 SET 20000ms",  ("set",  2, 20000)),
    ]
    tests2 = [
        # 1) Happy-Path
        ("T1 SET 250us",    ("set",  1, 250)),
        ("T2 SET 1s",      ("set",  2, 1000)),
        ("START",           ("start",1, 0)),
        ("READBACK T1",     ("read", 1, 0)),
        ("READBACK T2",     ("read", 2, 0))
    ]


    def run_tests(nuc, tests=tests2):
        for name, (cmd, timer, val) in tests:
            print(f"\n>>> {name}")
            try:
                if cmd == "set":
                    nuc.set_timer(timer, val)
                elif cmd == "start":
                    nuc.start_timer(timer)
                elif cmd == "stop":
                    nuc.stop_timer(timer)
                elif cmd == "read":
                    result = nuc.readback(timer)
                    print(f"READBACK T{timer} -> period={result}")
                else:
                    print(f"Unknown cmd {cmd}")
            except Exception as e:
                print("Fehler:", e)
            time.sleep(0.2)  # kleine Pause

    def soft_stop_test(nuc):
        print("\n>>> Soft-Stop-Test")
        nuc.set_timer(1, 500)   # 500 µs
        nuc.set_timer(2, 50)    # 50 ms
        nuc.start_timer(1)      # START (Timerbit egal)
        time.sleep(0.05)        # kurz warten
        nuc.stop_timer(1)       # Soft-Stop anfordern
        time.sleep(0.5)         # auf all_off() warten


raw_tests = [
    ("T1 SET period=200us", "FF 10 C8 00 00"),  # Timer1 Periode 200 µs
    ("T1 SET pulse=200us",  "FF 10 C8 00 01"),  # Beispiel: Flags=1 für „Pulse“ (je nach FW-Interpretation)
    ("T2 SET period=1000ms","FF 11 E8 03 00"),  # Timer2 Periode 1000 ms (1 s)
    ("START Sequence",      "FF 20 00 00 00"),  # Start
    ("READBACK T1",         "FF 40 00 00 00"),  # zurücklesen T1
    ("READBACK T2",         "FF 41 00 00 00"),  # zurücklesen T2
    ("STOP (soft)",         "FF 30 00 00 00"),  # weiches Stoppen
]


def send_raw_command(nuc: NucleoUART, hex_cmd: str, timeout: float = 1.0):
    data = bytes.fromhex(hex_cmd)
    nuc._write_packet(data)
    end = time.time() + timeout
    buf = bytearray()
    while time.time() < end:
        n = nuc.ser.in_waiting
        if n:
            buf.extend(nuc.ser.read(n))
        else:
            time.sleep(0.01)
    if buf:
        try:
            print(buf.decode(errors="replace"), end="")
        except Exception:
            print("RX:", " ".join(f"{b:02X}" for b in buf))
    else:
        print("[keine Antwort]")

def test_all(nuc: NucleoUART):
    for frame in TEST_FRAMES:
        print(f"\n>>> TX: {frame}")
        send_raw_command(nuc, frame)

def demo_api(nuc: NucleoUART):
    print("\n--- High-Level API Demo ---")
    nuc.set_timer(1, 50000)
    print("T1 READBACK:", nuc.readback(1))
    nuc.start_timer(1)
    time.sleep(0.5)
    nuc.stop_timer(1)
    print("T1 READBACK:", nuc.readback(1))

# -------- Main --------
if __name__ == "__main__":
    PORT = "/dev/tty.usbmodem11103"  # anpassen!
    nuc = NucleoUART(port=PORT, baudrate=115200, timeout=1.0)
    try:
        for frame in raw_tests:
            print(f"\n>>> TX: {frame[0]}")
            send_raw_command(nuc, frame[1])
            time.sleep(0.2)
    finally:
        nuc.close()
        print("\nVerbindung geschlossen.")

