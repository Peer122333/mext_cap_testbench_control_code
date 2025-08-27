import serial
from enum import IntEnum

PREAMBLE = 0xFF

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

def _u16_to_msb_lsb(val: int) -> tuple[int, int]:
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

def _msb_lsb_to_u16(msb: int, lsb: int) -> int:
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

def checksum6(frame5: bytes) -> int:
    """Berechnet die Prüfsumme für ein 6-Byte-Frame (letztes Byte)."""
    return sum(frame5) & 0xFF




class NucleoUART:
    """
    Protokoll: 6 Bytes pro Frame
      [0]=0xFF, [1]=CMD, [2]=P0 LSB, [3]=P1 MSB, [4]= FLAGS, [5]= CHKSUM
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
    def _build_packet(self, cmd: int, period: int = 0, pulse: int = 0) -> bytes:
        ph, pl = _u16_to_msb_lsb(period)
        wh, wl = _u16_to_msb_lsb(pulse)
        return bytes([PREAMBLE, cmd, ph, pl, wh, wl])

    def _write_packet(self, pkt: bytes) -> None:
        if len(pkt) != 6 or pkt[0] != PREAMBLE:
            raise ValueError("invalid packet")
        self.ser.reset_output_buffer()
        self.ser.write(pkt)
        self.ser.flush()

    def _read_exact(self, n: int) -> bytes:
        """Liest exakt n Bytes oder wirft TimeoutError."""
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                raise TimeoutError("UART read timeout")
            buf.extend(chunk)
        return bytes(buf)

    def _read_packet(self) -> bytes:
        """
        Resync: lies bis PREAMBLE, dann noch 5 Bytes.
        So sind wir robust, falls Bytes im Stream standen.
        """
        # sync auf 0xFF
        while True:
            b = self.ser.read(1)
            if not b:
                raise TimeoutError("UART read timeout (waiting for preamble)")
            if b[0] == PREAMBLE:
                break
        rest = self._read_exact(5)
        return b + rest  # 6 bytes



    # -------- high level API --------
    def set_timer(self, timer: int, period: int, pulse: int) -> None:
        """
        period/pulse: 0..65535 (deine Firmware: [2..5] als u16 big-endian).
        """
        cmd = _code_for_timer(CmdBase.SET, timer)
        self._write_packet(self._build_packet(cmd, period, pulse))

    def start_timer(self, timer: int) -> None:
        cmd = _code_for_timer(CmdBase.START, timer)
        self._write_packet(self._build_packet(cmd))

    def stop_timer(self, timer: int) -> None:
        cmd = _code_for_timer(CmdBase.STOP, timer)
        self._write_packet(self._build_packet(cmd))

    def readback(self, timer: int) -> tuple[int, int]:
        """
        Erwartet 6-Byte-Response im gleichen Format:
        [0]=0xFF, [1]=0x40/0x41, [2..3]=PERIOD, [4..5]=PULSE
        """
        cmd = _code_for_timer(CmdBase.READBACK, timer)
        self._write_packet(self._build_packet(cmd))
        pkt = self._read_packet()
        if pkt[1] != cmd:
            raise ValueError(f"unexpected READBACK cmd in response: 0x{pkt[1]:02X}")
        period = _msb_lsb_to_u16(pkt[2], pkt[3])
        pulse  = _msb_lsb_to_u16(pkt[4], pkt[5])
        return period, pulse
    
    
    # Convenience: SET + READBACK + Plausibilitätscheck
    def set_and_verify(self, timer: int, period: int, pulse: int, tol: int = 0) -> tuple[int, int]:
        """
        Setzt period/pulse und verifiziert per READBACK.
        tol: Toleranz in Ticks (0 = exakt).
        """
        self.set_timer(timer, period, pulse)
        rb_period, rb_pulse = self.readback(timer)
        if abs(rb_period - (period & 0xFFFF)) > tol or abs(rb_pulse - (pulse & 0xFFFF)) > tol:
            raise AssertionError(
                f"READBACK mismatch: got (P={rb_period},W={rb_pulse}) "
                f"expected (P={(period & 0xFFFF)},W={(pulse & 0xFFFF)})"
            )
        return rb_period, rb_pulse

    def close(self):
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