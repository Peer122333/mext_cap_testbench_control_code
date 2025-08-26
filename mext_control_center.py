from nucleo_uart import NucleoUART
import time

# Optional: ungenutzt, kann raus
T1_PERIOD, T1_PULSE = 50000, 15000
T2_PERIOD, T2_PULSE = 40000, 10000

HEX = "FF 10 C3 50 3A 98"  # SET Timer1: period=50000, pulse=15000
PORT = "/dev/tty.usbmodem21303"

def test_commands(nuc: NucleoUART) -> None:

    test_arr = [
        "FF 10 00 00 00 00",  # SET  T1
        "FF 11 00 00 00 00",  # SET  T2
        "FF 20 00 00 00 00",  # START T1
        "FF 21 00 00 00 00",  # START T2
        "FF 30 00 00 00 00",  # STOP  T1
        "FF 31 00 00 00 00",  # STOP  T2
        "FF 40 00 00 00 00",  # READBACK T1
        "FF 41 00 00 00 00",  # READBACK T2
    ]

    for HEX in test_arr:
        # Input-Buffer leeren
        nuc.ser.reset_input_buffer()

        # 6-Byte-Frame senden
        data = bytes.fromhex(HEX)
        nuc._write_packet(data)          # alternativ: nuc.ser.write(data)
        nuc.ser.flush()

        # Antwort einsammeln (printf-Text oder binär)
        end = time.time() + 2.0
        buf = bytearray()
        while time.time() < end:
            n = nuc.ser.in_waiting
            if n:
                buf.extend(nuc.ser.read(n))
            else:
                time.sleep(0.01)

        if buf:
            # Versuch als Text zu dekodieren; wenn nicht darstellbar, als Hex zeigen
            try:
                print(buf.decode(errors="replace"), end="")
            except Exception:
                print("RX:", " ".join(f"{b:02X}" for b in buf))
        else:
            print("[keine Antwort erhalten]")


def main():
    nuc = NucleoUART(port=PORT, baudrate=115200, timeout=2.0)  # Port ggf. anpassen
    try:
        test_commands(nuc)

    finally:
        nuc.close()

if __name__ == "__main__":
    main()
