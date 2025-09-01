import serial
from enum import IntEnum
import time

PREAMBLE = 0xFF
FRAME_SIZE = 5  # Frame-Größe in Bytes

class CmdBase(IntEnum):
    """Befehl-Basiscodes für Timer-Kommandos.

    Parameters
    ----------
    IntEnum : enum.IntEnum
        Basisklasse für Kommando-Enumeration.
    """
    SET      = 0x10  # Timer1: 0x10, Timer2: 0x11
    START    = 0x20  # Timer1: 0x20, Timer2: 0x21
    STOP     = 0x30  # Timer1: 0x30, Timer2: 0x31
    READBACK = 0x40  # Timer1: 0x40, Timer2: 0x41



""" 
######################## Hilsfunktionen ########################
"""
def _code_for_timer(base: CmdBase, timer: int) -> int:
    """Erzeugt den Befehlscode für den gegebenen Timer.

    Parameters
    ----------
    base : CmdBase
        Basiskommando, z.B. CmdBase.SET
    timer : int
        Timer-Nummer (1 oder 2)

    Returns
    -------
    int
        Der Befehlscode für den gegebenen Timer.

    Raises
    ------
    ValueError
        Wenn der Timer nicht 1 oder 2 ist.
    """
    if timer not in (1, 2):
        raise ValueError("timer must be 1 or 2")
    return int(base) + (0 if timer == 1 else 1)

def _u16_to_lsb_msb(val: int) -> tuple[int, int]:
    """Wandelt einen 16-Bit-Wert in MSB/LSB um.

    Parameters
    ----------
    val : int
        16-Bit-Wert (0..65535)

    Returns
    -------
    tuple[int, int]
        (MSB, LSB)
    """
    v = int(val) & 0xFFFF
    return (v >> 8) & 0xFF, v & 0xFF

def _lsb_msb_to_u16(lsb: int, msb: int) -> int:
    """Wandelt MSB/LSB in einen 16-Bit-Wert um.

    Parameters
    ----------
    msb : int   
        Most Significant Byte
    lsb : int
        Least Significant Byte

    Returns
    -------
    int
        16-Bit-Wert
    """
    return ((msb & 0xFF) << 8) | (lsb & 0xFF)



class NucleoUART:
    """
    Protokoll: 5 Bytes pro Frame
      [0]=0xFF, [1]=CMD, [2]=LSB(Value), [3]=MSB(Value), [4]= FLAGS
    """
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2):
        self.ser = serial.Serial(
            port=port,                      # Gerätepfad
            baudrate=baudrate,              # Baudrate (sollte mit der Firmware übereinstimmen)
            timeout=timeout,                # Lese-Timeout in Sekunden
            bytesize=serial.EIGHTBITS,      # 8 Datenbits
            parity=serial.PARITY_NONE,      # keine Parität
            stopbits=serial.STOPBITS_ONE,   # 1 Stoppbit
            write_timeout=timeout,          # Schreib-Timeout   
        )

    # -------- low level --------
    def _build_packet(self, cmd: int, value: int = 0, flags: int = 0) -> bytes:
        lsb, msb = _u16_to_lsb_msb(value)
        return bytes([PREAMBLE, cmd, lsb, msb, flags & 0xFF])

    def _write_packet(self, pkt: bytes) -> None:
        if len(pkt) != FRAME_SIZE or pkt[0] != PREAMBLE:
            raise ValueError("invalid packet")
        self.ser.reset_output_buffer()
        self.ser.write(pkt)
        self.ser.flush()

    def _read_packet(self) -> bytes:
        # Resync auf PREAMBLE, dann restliche 4 Bytes lesen
        while True:
            b = self.ser.read(1)
            if not b:
                raise TimeoutError("UART read timeout (waiting for preamble)")
            if b[0] == PREAMBLE:
                break
        rest = self.ser.read(FRAME_SIZE - 1)
        if len(rest) != FRAME_SIZE - 1:
            raise TimeoutError("UART read timeout (reading frame body)")
        return b + rest

    def drain_text(self, timeout: float = 0.5) -> str:
        """Liest ASCII-Logs (printf) der FW, falls vorhanden."""
        end = time.time() + timeout
        buf = bytearray()
        while time.time() < end:
            n = self.ser.in_waiting
            if n:
                buf.extend(self.ser.read(n))
            else:
                time.sleep(0.01)
        return buf.decode(errors="replace")

    # ---------- High-Level API ----------
    def set_timer(self, timer: int, period: int) -> None:
        """
        SET für Timer:
          - T1: period in µs
          - T2: period in ms
        Flags werden pauschal 0 gesendet (wie gewünscht).
        """
        cmd = _code_for_timer(CmdBase.SET, timer)
        self._write_packet(self._build_packet(cmd, value=period, flags=0))

    def start_sequence(self, pulse_count: int, timer_for_cmd: int = 1) -> None:
        """
        START Sequenz (global).
        - pulse_count = 0 → endlos bis STOP
        - timer_for_cmd setzt nur das LSB des CMD (0x20/0x21); FW-seitig egal
        """
        cmd = _code_for_timer(CmdBase.START, timer_for_cmd)
        self._write_packet(self._build_packet(cmd, value=pulse_count, flags=0))

    def stop_timer(self, *, hard: bool = False, timer_for_cmd: int = 1) -> None:
        """
        STOP Sequenz:
          - flags = 0 → Soft
          - flags = 1 → Hard   (ggf. Mapping an FW anpassen)
        """
        cmd = _code_for_timer(CmdBase.STOP, timer_for_cmd)
        flags = 1 if hard else 0
        self._write_packet(self._build_packet(cmd, value=0, flags=flags))

    def readback(self, timer: int) -> int:
        """
        READBACK liefert einen 16-Bit-Wert:
          - T1: µs
          - T2: ms
        Ablauf: Befehl senden → Binär-Frame (FF 40/41 …) einlesen → Wert zurückgeben.
        """
        cmd = _code_for_timer(CmdBase.READBACK, timer)
        self._write_packet(self._build_packet(cmd, value=0, flags=0))

        pkt = self._read_packet()
        if pkt[1] != cmd:
            raise ValueError(f"unexpected response: got 0x{pkt[1]:02X}")
        value = _lsb_msb_to_u16(pkt[2], pkt[3])
        return value

    def close(self) -> None:
        self.ser.close()



def test_timer(nuc: NucleoUART, timer: int, period: int, pulse: int) -> None:
    print(f"\n=== Timer {timer}: SET period={period} pulse={pulse} ===")
    nuc.set_timer(timer, period, pulse)
    print(nuc.drain_text(0.2), end="")  # zeigt ggf. „CMD: SET (…) OK“
    p, w = nuc.readback(timer)
    print(f"READBACK -> period={p} pulse={w}")

    print(f"=== Timer {timer}: START ===")
    nuc.start_timer(timer)
    print(nuc.drain_text(0.2), end="")

    print(f"=== Timer {timer}: READBACK während Lauf ===")
    p, w = nuc.readback(timer)
    print(f"READBACK -> period={p} pulse={w}")

    print(f"=== Timer {timer}: STOP ===")
    nuc.stop_timer(timer)
    print(nuc.drain_text(0.2), end="")

    print(f"=== Timer {timer}: READBACK nach STOP ===")
    p, w = nuc.readback(timer)
    print(f"READBACK -> period={p} pulse={w}")