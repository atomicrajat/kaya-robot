#!/usr/bin/env python3
"""Give a wheel servo a new bus ID (written to its EEPROM).

    source ~/ros2_ws/install/setup.bash
    python3 kaya_drive/tools/servo_set_id.py --from 1 --to 2 --yes

Connect ONE servo at a time: new servos all ship as ID 1, and two servos with
the same ID answer the same packet and corrupt each other's replies.

Register 5 is the ID on Feetech STS/SCS-protocol servos (the vendored
scservo_sdk's own register map). The EEPROM must be unlocked first (register
55 = 0) and is locked again afterwards; the new ID is read back before the
script reports success.

UNTESTED on hardware: the project's servos were assigned IDs with the vendor
tool before this script existed. It follows the same unlock / write / read-back
pattern as FeetechBus.enter_wheel_mode(), which is exercised on every drive.
"""
import argparse

from kaya_drive.feetech_bus import ADDR_LOCK, FeetechBus
from kaya_drive.scservo_sdk import COMM_SUCCESS

ADDR_ID = 5

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="/dev/ttyACM0")
ap.add_argument("--baud", type=int, default=1000000)
ap.add_argument("--from", dest="old", type=int, required=True, help="the servo's current ID")
ap.add_argument("--to", dest="new", type=int, required=True, help="the ID it should have (1-253)")
ap.add_argument("--yes", action="store_true", help="actually write; without it, only check the servo answers")
args = ap.parse_args()
if not 1 <= args.new <= 253:
    raise SystemExit("new ID must be 1..253")

bus = FeetechBus(args.port, args.baud, [args.old, args.new])
cur, comm, _ = bus._read1(args.old, ADDR_ID)
if comm != COMM_SUCCESS:
    raise SystemExit(f"no servo answers as ID {args.old} on {args.port}")
print(f"servo {args.old} answers; its ID register reads {cur}")
other, comm2, _ = bus._read1(args.new, ADDR_ID)
if comm2 == COMM_SUCCESS and args.new != args.old:
    raise SystemExit(f"another servo already answers as ID {args.new}; disconnect it first")
if not args.yes:
    print(f"dry run: would set ID {args.old} -> {args.new}. Re-run with --yes to write it.")
    raise SystemExit(0)

bus._write1(args.old, ADDR_LOCK, 0)          # unlock EEPROM
bus._write1(args.old, ADDR_ID, args.new)     # the servo answers as new_id from here on
got, comm3, _ = bus._read1(args.new, ADDR_ID)
bus._write1(args.new, ADDR_LOCK, 1)          # lock again, so a brown-out cannot corrupt it
if comm3 == COMM_SUCCESS and got == args.new:
    print(f"OK: servo now answers as ID {args.new} (read back from the ID register)")
else:
    raise SystemExit(f"FAILED: read-back as ID {args.new} returned comm={comm3} value={got}; "
                     f"power-cycle the servo and scan IDs 1..253 before retrying")
