"""Low-level driver for the three Feetech-protocol (Waveshare ST3020) wheel
servos on the Kaya robot.

Wraps the vendored kaya_drive.scservo_sdk (see that package's LICENSE --
Unlicense/public domain, mirrored from the official FEETECH SCServoSDK).
Vendored rather than pip-installed so the workspace build stays apt-only and
self-contained; it is not packaged for apt/rosdep and is small and stable.

Two lessons learned probing this bus by hand (see ~/wheel_probe.py) are baked
in here:

1. The SDK's clearPort() only flushes the OUTPUT side (pyserial semantics).
   A delayed ACK -- routine on this servo's EEPROM-area writes such as Mode --
   is left sitting in the OS input buffer and corrupts the *next*
   transaction's response (a stale write-ack's checksum byte gets misread as
   a read's data byte). Every request in here flushes input immediately
   before it is sent.
2. Goal Speed / Present Speed share the same unit as Present Position: raw
   encoder counts per second, on a 4096-count/rev magnetic encoder. Confirmed
   empirically -- a commanded magnitude of 400 moved ~400-425 counts over a
   1.0 s pulse, matching 1:1 net of accel ramp.
"""

import time

from kaya_drive.scservo_sdk import (
    PortHandler, PacketHandler, GroupSyncWrite,
    SCS_TOSCS, COMM_SUCCESS,
)

ADDR_MODE = 33
ADDR_TORQUE = 40
ADDR_GOAL_SPEED = 46
ADDR_LOCK = 55
ADDR_PRESENT_POS = 56
ADDR_PRESENT_SPEED = 58

COUNTS_PER_REV = 4096
MODE_POSITION = 0
MODE_WHEEL = 1


class FeetechBus:

    def __init__(self, port_name, baud, servo_ids, retries=3, retry_delay=0.03):
        self.servo_ids = list(servo_ids)
        self.retries = retries
        self.retry_delay = retry_delay
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(0)  # protocol_end=0: little-endian, matches SCS wire format
        if not self.port.openPort():
            raise IOError(f'cannot open {port_name}')
        if not self.port.setBaudRate(baud):
            raise IOError(f'cannot set baud {baud} on {port_name}')
        self._sync_write = GroupSyncWrite(self.port, self.ph, ADDR_GOAL_SPEED, 2)

    # ------------------------------------------------------------- transport
    # Every call flushes the input buffer immediately before transmitting --
    # see the module docstring for why that matters on this bus.
    def _write1(self, sid, addr, val):
        r = (COMM_SUCCESS + 1, 0)  # sentinel != COMM_SUCCESS
        for _ in range(self.retries):
            self.port.ser.reset_input_buffer()
            r = self.ph.write1ByteTxRx(self.port, sid, addr, val)
            if r[0] == COMM_SUCCESS:
                return r
            time.sleep(self.retry_delay)
        return r

    def _read1(self, sid, addr):
        r = (0, COMM_SUCCESS + 1, 0)
        for _ in range(self.retries):
            self.port.ser.reset_input_buffer()
            r = self.ph.read1ByteTxRx(self.port, sid, addr)
            if r[1] == COMM_SUCCESS:
                return r
            time.sleep(self.retry_delay)
        return r

    def _read2(self, sid, addr):
        r = (0, COMM_SUCCESS + 1, 0)
        for _ in range(self.retries):
            self.port.ser.reset_input_buffer()
            r = self.ph.read2ByteTxRx(self.port, sid, addr)
            if r[1] == COMM_SUCCESS:
                return r
            time.sleep(self.retry_delay)
        return r

    # ---------------------------------------------------------------- setup
    def enter_wheel_mode(self):
        """Unlock EEPROM, disable torque, switch every servo to wheel mode,
        verify the switch landed, then re-enable torque. Returns True iff
        every servo confirmed wheel mode."""
        for sid in self.servo_ids:
            self._write1(sid, ADDR_LOCK, 0)
            self._write1(sid, ADDR_TORQUE, 0)
            self._write1(sid, ADDR_MODE, MODE_WHEEL)

        ok = True
        for sid in self.servo_ids:
            mode, comm, _err = self._read1(sid, ADDR_MODE)
            if comm != COMM_SUCCESS or mode != MODE_WHEEL:
                ok = False

        for sid in self.servo_ids:
            self._write1(sid, ADDR_TORQUE, 1)
        return ok

    # --------------------------------------------------------------- motion
    def set_wheel_speeds(self, speeds):
        """speeds: {servo_id: signed raw counts/sec}. One broadcast packet,
        no per-servo ACK -- SYNC WRITE goes to the broadcast ID, so there is
        nothing here for a stale response to contaminate."""
        self._sync_write.clearParam()
        for sid in self.servo_ids:
            enc = SCS_TOSCS(speeds.get(sid, 0), 15)
            self._sync_write.addParam(sid, [enc & 0xFF, (enc >> 8) & 0xFF])
        self._sync_write.txPacket()

    def read_positions(self):
        """Returns {servo_id: raw_count (0..4095)} for every servo that answered."""
        out = {}
        for sid in self.servo_ids:
            v, comm, _err = self._read2(sid, ADDR_PRESENT_POS)
            if comm == COMM_SUCCESS:
                out[sid] = v
        return out

    # -------------------------------------------------------------- safety
    def stop_and_release(self):
        """Zero every wheel, drop torque, restore position mode. Best-effort:
        never raises, so it is safe to call from a shutdown/exception handler."""
        try:
            self._sync_write.clearParam()
            for sid in self.servo_ids:
                self._sync_write.addParam(sid, [0, 0])
            self._sync_write.txPacket()
        except Exception:
            pass
        for sid in self.servo_ids:
            try:
                self._write1(sid, ADDR_TORQUE, 0)
                self._write1(sid, ADDR_MODE, MODE_POSITION)
            except Exception:
                pass

    def close(self):
        self.stop_and_release()
        self.port.closePort()
