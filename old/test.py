
import time
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

def test_all(nuc: NucleoUART):
    for frame in TEST_FRAMES:
        print(f"\n>>> TX: {frame}")
        send_raw_command(nuc, frame)