#!/usr/bin/env python3
"""
Kaya wheel-mode probe v2 — instrumented single-servo pulse test.

Prints every write's (comm,error), reads back Mode to confirm the switch
actually took effect, and polls Present Position during the pulse so we
can tell from encoder counts alone whether the servo is moving, even if
it's not visually obvious.

Register map (Feetech SMS/STS control table):
    0x21 (33) Mode            0=position 1=wheel(speed) 2=PWM 3=step
    0x28 (40) Torque Enable
    0x2E (46) Goal Speed (2B) sign-magnitude, bit15=sign, bits0-14=magnitude
    0x37 (55) Lock            0=EEPROM writable 1=locked
    0x38 (56) Present Position (2B)
    0x3A (58) Present Speed (2B) signed

Usage (robot on blocks, wheels off the floor):
    python3 kaya_drive/tools/wheel_probe.py <id> <signed_speed> <duration_s>
    e.g.  python3 kaya_drive/tools/wheel_probe.py 1 300 1.0

Pulse each ID in turn and note which wheel moves. The kinematics assume
    servo 1 -> front-right (300 deg from forward, counter-clockwise)
    servo 2 -> front-left  ( 60 deg)
    servo 3 -> rear        (180 deg)
with "forward" where the camera looks. This is the script that resolved the
mapping on 2026-09-03 (commit 371f15b); it lived only on the robot until
2026-10-12. Spin direction is then checked with teleop (GETTING_STARTED.md
step 5); on this robot no per-wheel sign flip was needed.
"""
import sys, time, signal
# The SDK is vendored in kaya_drive; `source ~/ros2_ws/install/setup.bash` first.
from kaya_drive.scservo_sdk import PortHandler, PacketHandler, SCS_TOSCS, SCS_TOHOST, COMM_SUCCESS

PORT = "/dev/ttyACM0"
BAUD = 1000000

ADDR_MODE = 33
ADDR_TORQUE = 40
ADDR_GOAL_SPEED = 46
ADDR_LOCK = 55
ADDR_PRESENT_POS = 56
ADDR_PRESENT_SPEED = 58

MAX_SPEED_MAGNITUDE = 600
MAX_DURATION = 2.0

def main():
    if len(sys.argv) != 4:
        print(__doc__); sys.exit(2)
    sid = int(sys.argv[1]); speed = int(sys.argv[2]); dur = float(sys.argv[3])

    if abs(speed) > MAX_SPEED_MAGNITUDE:
        print(f"REFUSING: |speed|={abs(speed)} exceeds probe cap {MAX_SPEED_MAGNITUDE}"); sys.exit(1)
    if dur > MAX_DURATION:
        print(f"REFUSING: duration {dur}s exceeds probe cap {MAX_DURATION}"); sys.exit(1)

    port = PortHandler(PORT)
    ph = PacketHandler(0)
    if not port.openPort(): print("ERR: cannot open port"); sys.exit(1)
    if not port.setBaudRate(BAUD): print("ERR: cannot set baud"); sys.exit(1)

    # NOTE: the SDK's clearPort() only flushes the OUTPUT buffer (pyserial
    # ser.flush() semantics), never the input side. A delayed ACK (common on
    # this servo's EEPROM-area writes, e.g. Mode) then sits in the OS input
    # buffer and gets misread as the response to the *next* transaction —
    # e.g. a stale 6-byte write-ack's checksum byte gets misread as a 1-byte
    # read's data byte. Always purge input immediately before every request.
    def w1(addr, data, tries=3):
        for i in range(tries):
            port.ser.reset_input_buffer()
            r = ph.write1ByteTxRx(port, sid, addr, data)
            if r[0] == COMM_SUCCESS:
                return r
            time.sleep(0.03)
        return r

    def w2(addr, data, tries=3):
        for i in range(tries):
            port.ser.reset_input_buffer()
            r = ph.write2ByteTxRx(port, sid, addr, data)
            if r[0] == COMM_SUCCESS:
                return r
            time.sleep(0.03)
        return r

    def r1(addr, tries=3):
        for i in range(tries):
            port.ser.reset_input_buffer()
            v, c, e = ph.read1ByteTxRx(port, sid, addr)
            if c == COMM_SUCCESS:
                return v, c, e
            time.sleep(0.03)
        return v, c, e

    def r2(addr, tries=3):
        for i in range(tries):
            port.ser.reset_input_buffer()
            v, c, e = ph.read2ByteTxRx(port, sid, addr)
            if c == COMM_SUCCESS:
                return v, c, e
            time.sleep(0.03)
        return v, c, e

    stopped = {"done": False}
    def stop_all(*_):
        if stopped["done"]: return
        stopped["done"] = True
        try:
            w2(ADDR_GOAL_SPEED, 0)
            w1(ADDR_TORQUE, 0)
            w1(ADDR_MODE, 0)
        except Exception as e:
            print(f"stop_all error: {e}")
        print(f"servo {sid}: stopped, torque off, mode=position")

    signal.signal(signal.SIGINT, lambda *a: (stop_all(), sys.exit(1)))
    signal.signal(signal.SIGTERM, lambda *a: (stop_all(), sys.exit(1)))

    try:
        pos0, c0, e0 = r2(ADDR_PRESENT_POS)
        mode0, cm0, em0 = r1(ADDR_MODE)
        print(f"servo {sid}: baseline pos={pos0} mode={mode0} (comm pos={c0} mode={cm0})")

        r = w1(ADDR_LOCK, 0)
        print(f"  lock=0        -> comm/err {r}")
        r = w1(ADDR_TORQUE, 0)
        print(f"  torque=0      -> comm/err {r}")
        r = w1(ADDR_MODE, 1)
        print(f"  mode=1(wheel) -> comm/err {r}")
        time.sleep(0.02)

        mode1, cm1, em1 = r1(ADDR_MODE)
        print(f"  mode readback = {mode1} (comm={cm1})")
        if mode1 != 1:
            print("  WARNING: mode did not switch to wheel — aborting pulse, servo left safe.")
            return

        r = w1(ADDR_TORQUE, 1)
        print(f"  torque=1      -> comm/err {r}")

        val = SCS_TOSCS(speed, 15)
        r = w2(ADDR_GOAL_SPEED, val)
        print(f"servo {sid}: goal_speed={speed} (encoded 0x{val:04x}) -> comm/err {r}, running {dur}s")

        t_end = time.time() + dur
        last_pos = pos0
        while time.time() < t_end:
            p, c, e = r2(ADDR_PRESENT_POS, tries=1)
            spd_raw, cs, es = r2(ADDR_PRESENT_SPEED, tries=1)
            spd = SCS_TOHOST(spd_raw, 15) if cs == COMM_SUCCESS else None
            if c == COMM_SUCCESS:
                print(f"  t={dur-(t_end-time.time()):.2f}s pos={p} (d={p-last_pos:+d}) present_speed={spd}")
                last_pos = p
            time.sleep(0.08)
    finally:
        stop_all()
        posf, cf, ef = r2(ADDR_PRESENT_POS)
        print(f"servo {sid}: final pos={posf} (net change from baseline: {posf - pos0 if cf==COMM_SUCCESS else 'unknown'})")
        port.closePort()

if __name__ == "__main__":
    main()
