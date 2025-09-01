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
    Frameformat (5 Byte):
      [0]=0xFF, [1]=CMD, [2]=LSB, [3]=MSB, [4]=FLAGS
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

    def _build_packet(self, cmd: int, value: int = 0, flags: int = 0) -> bytes: # Frame zusammenbauen
        lsb = value & 0xFF
        msb = (value >> 8) & 0xFF
        return bytes([PREAMBLE, cmd, lsb, msb, flags & 0xFF])  # nur 5 Bytes

    def _write_packet(self, pkt: bytes):            # Frame senden
        if len(pkt) != FRAME_SIZE or pkt[0] != PREAMBLE:         # Frame muss 5 Byte lang sein und mit PREAMBLE beginnen
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
        return b + self.ser.read(FRAME_SIZE - 1)                 # restlichen 4 Bytes lesen        
    
    def _read_ascii_response(self, timeout: float = 0.5) -> str:
        end = time.time() + timeout
        buf = bytearray()
        while time.time() < end:
            n = self.ser.in_waiting
            if n:
                buf.extend(self.ser.read(n))
            else:
                time.sleep(0.01)
        return buf.decode(errors="replace").strip() if buf else ""




    # -------- High-Level API --------
    def set_timer(self, timer: int, period: int, flags: int = 0): # Timer konfigurieren
        cmd = cmd_for_timer(Cmd.SET, timer)                         # Befehlscode für Timer holen       
        self._write_packet(self._build_packet(cmd, value=period, flags=flags))  # Frame senden
        resp = self._read_ascii_response()
        if resp:
            print("STM32 response:", resp)
        else:
            print("No response from STM32")

    def start_sequence(self, pulse_count:int):                  # Timer starten
        """
        START Sequenz (global): FW liest pulse_count aus value (LSB/MSB).
        pulse_count=0 -> endlos bis STOP.
        """
        cmd = cmd_for_timer(Cmd.START, timer=1)           # Befehlscode für Timer holen
        self._write_packet(self._build_packet(cmd, value=pulse_count, flags=0))     # Frame senden
        resp = self._read_ascii_response()
        if resp:
            print("STM32 response:", resp)
        else:
            print("No response from STM32")

    def stop_timer(self, timer: int, flags):                   # Timer stoppen
        flags: int = 0
        cmd = cmd_for_timer(Cmd.STOP, timer)            # Befehlscode für Timer holen
        self._write_packet(self._build_packet(cmd, value=0, flags=0))     # Frame senden
        resp = self._read_ascii_response()
        if resp:
            print("STM32 response:", resp)
        else:
            print("No response from STM32")


    def readback(self, timer: int) -> int:              # Timer-Periode auslesen
        cmd = cmd_for_timer(Cmd.READ, timer)        # 0x40 / 0x41
        self._write_packet(self._build_packet(cmd, value=0, flags=0))
        pkt = self._read_packet()
        if pkt[1] != cmd:
            raise ValueError(f"unexpected response: got 0x{pkt[1]:02X}")
        return ((pkt[3] & 0xFF) << 8) | (pkt[2] & 0xFF)

    def close(self):                                    # Verbindung schließen
        self.ser.close()

   
api_tests = [
    # Name,           Funktion,   Args
    ("T1 SET period=200us",  lambda nuc: nuc.set_timer(1, 200, flags=0)),
    ("T2 SET period=1000ms", lambda nuc: nuc.set_timer(2, 1000, flags=0)),
    ("START Sequence",       lambda nuc: nuc.start_sequence(0)),  # 0 = endlos
    ("READBACK T1",          lambda nuc: print("READBACK T1 ->", nuc.readback(1))),
    ("READBACK T2",          lambda nuc: print("READBACK T2 ->", nuc.readback(2))),
    ("READBACK T1",          lambda nuc: print("READBACK T1 ->", nuc.readback(1))),
    ("READBACK T2",          lambda nuc: print("READBACK T2 ->", nuc.readback(2))),
    ("READBACK T1",          lambda nuc: print("READBACK T1 ->", nuc.readback(1))),
    ("READBACK T2",          lambda nuc: print("READBACK T2 ->", nuc.readback(2))),
    ("READBACK T1",          lambda nuc: print("READBACK T1 ->", nuc.readback(1))),
    ("READBACK T2",          lambda nuc: print("READBACK T2 ->", nuc.readback(2))),
    ("READBACK T1",          lambda nuc: print("READBACK T1 ->", nuc.readback(1))),
    ("READBACK T2",          lambda nuc: print("READBACK T2 ->", nuc.readback(2))),
    ("READBACK T1",          lambda nuc: print("READBACK T1 ->", nuc.readback(1))),
    ("READBACK T2",          lambda nuc: print("READBACK T2 ->", nuc.readback(2))),
    ("STOP (soft)",          lambda nuc: nuc.stop_timer()),
]

def run_api_tests(nuc, tests=api_tests, delay=0.2):
    for name, func in tests:
        print(f"\n>>> {name}")
        try:
            func(nuc)
        except Exception as e:
            print("Fehler:", e)
        time.sleep(delay)


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


# -------- Main --------
if __name__ == "__main__":
    PORT = "/dev/tty.usbmodem1103"  # anpassen!
    nuc = NucleoUART(port=PORT, baudrate=115200, timeout=1.0)
    try:
        run_api_tests(nuc)
    finally:
        nuc.close()
        print("\nVerbindung geschlossen.")

