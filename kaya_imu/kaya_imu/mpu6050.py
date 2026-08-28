"""Register-level driver for the InvenSense MPU-6050 on the Kaya robot.

Deliberately free of any ROS dependency: this file must be runnable with plain
`python3 mpu6050.py` so you can iterate on the hardware without a colcon build.

Datasheet: "MPU-6000/MPU-6050 Register Map and Descriptions, rev 4.2".
"""

import math
import time

import smbus2

# --------------------------------------------------------------------------
# Register map (datasheet section 3). These are addresses, not values.
# --------------------------------------------------------------------------
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_TEMP_OUT_H   = 0x41
REG_GYRO_XOUT_H  = 0x43
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

ACCEL_RANGES = {2: (0x00, 16384.0), 4: (0x08, 8192.0), 8: (0x10, 4096.0), 16: (0x18, 2048.0)}
GYRO_RANGES  = {250: (0x00, 131.0), 500: (0x08, 65.5), 1000: (0x10, 32.8), 2000: (0x18, 16.4)}

STANDARD_GRAVITY = 9.80665  # m/s^2, CODATA. REP-145 wants accel in m/s^2, not g.

# Gyro output rate feeding the sample-rate divider: 8 kHz with DLPF disabled
# (DLPF_CFG 0 or 7), 1 kHz whenever the filter is active (DLPF_CFG 1..6).
_GYRO_OUTPUT_RATE = {0: 8000.0, 7: 8000.0}


class MPU6050:
    """Blocking I2C driver. One instance owns one SMBus handle."""

    def __init__(self, bus=7, address=0x68, accel_range=2, gyro_range=250,
                 dlpf=3, sample_rate_hz=100):
        if accel_range not in ACCEL_RANGES:
            raise ValueError(f'accel_range must be one of {sorted(ACCEL_RANGES)}, got {accel_range}')
        if gyro_range not in GYRO_RANGES:
            raise ValueError(f'gyro_range must be one of {sorted(GYRO_RANGES)}, got {gyro_range}')
        if not 0 <= dlpf <= 7:
            raise ValueError(f'dlpf must be 0..7, got {dlpf}')

        self.address = address
        self.bus = smbus2.SMBus(bus)
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self._accel_lsb = ACCEL_RANGES[accel_range][1]
        self._gyro_lsb = GYRO_RANGES[gyro_range][1]

        # (1) Wake the device. Bit 6 (SLEEP) is set on power-up and the chip
        # returns stale zeros until it is cleared. Writing 0x01 clears SLEEP and
        # simultaneously selects clock source 1 = PLL with X-gyro reference,
        # which InvenSense recommend over the internal 8 MHz RC oscillator: the
        # RC oscillator drifts several percent over the temperature range this
        # board will see, and that drift lands directly in integrated heading.
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.1)  # PLL lock

        # (2) Identity check before trusting anything else.
        who = self.bus.read_byte_data(self.address, REG_WHO_AM_I)
        if who != 0x68:
            raise RuntimeError(
                f'WHO_AM_I returned 0x{who:02x}, expected 0x68 at address '
                f'0x{address:02x} on i2c-{bus}. Wrong device, wrong bus, or bad wiring.')

        # (3) Filter, ranges, sample rate.
        self.bus.write_byte_data(self.address, REG_CONFIG, dlpf)
        self.bus.write_byte_data(self.address, REG_GYRO_CONFIG, GYRO_RANGES[gyro_range][0])
        self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, ACCEL_RANGES[accel_range][0])

        base_rate = _GYRO_OUTPUT_RATE.get(dlpf, 1000.0)
        div = int(round(base_rate / float(sample_rate_hz))) - 1
        div = max(0, min(255, div))
        self.bus.write_byte_data(self.address, REG_SMPLRT_DIV, div)

        self.sample_rate_hz = base_rate / (1 + div)  # what we ACTUALLY got
        self.dlpf = dlpf
        time.sleep(0.05)

    # ----------------------------------------------------------------- helpers
    @staticmethod
    def _to_signed16(high, low):
        """Combine two big-endian bytes into a signed 16-bit value."""
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value

    # -------------------------------------------------------------------- read
    def read_raw(self):
        """One burst read of all 14 data registers.

        Returns (ax, ay, az, temp_raw, gx, gy, gz) in raw signed ADC counts.

        The 14 bytes are read in a single transaction because the data registers
        are latched together at each sample instant. Seven separate reads can
        straddle a sample boundary and yield an orientation the robot was never
        physically in.
        """
        d = self.bus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H, 14)
        s = self._to_signed16
        return (
            s(d[0], d[1]),    # accel X
            s(d[2], d[3]),    # accel Y
            s(d[4], d[5]),    # accel Z
            s(d[6], d[7]),    # temperature
            s(d[8], d[9]),    # gyro X
            s(d[10], d[11]),  # gyro Y
            s(d[12], d[13]),  # gyro Z
        )

    def read(self):
        """Scaled reading in SI units: accel m/s^2, gyro rad/s, temp degrees C."""
        ax, ay, az, temp_raw, gx, gy, gz = self.read_raw()
        a_scale = STANDARD_GRAVITY / self._accel_lsb        # counts -> m/s^2
        g_scale = (math.pi / 180.0) / self._gyro_lsb        # counts -> rad/s
        return {
            'accel': (ax * a_scale, ay * a_scale, az * a_scale),
            'gyro': (gx * g_scale, gy * g_scale, gz * g_scale),
            'temp': temp_raw / 340.0 + 36.53,               # datasheet 4.19
        }

    def close(self):
        self.bus.close()


if __name__ == '__main__':
    imu = MPU6050()
    print(f'WHO_AM_I ok. accel +/-{imu.accel_range}g  gyro +/-{imu.gyro_range}dps  '
          f'dlpf={imu.dlpf}  chip rate={imu.sample_rate_hz:.1f} Hz')
    print('streaming for 3 s...')
    n, t_end = 0, time.time() + 3.0
    while time.time() < t_end:
        d = imu.read()
        n += 1
        if n % 25 == 0:
            ax, ay, az = d['accel']
            gx, gy, gz = d['gyro']
            mag = math.sqrt(ax * ax + ay * ay + az * az)
            print(f"a=({ax:+7.3f},{ay:+7.3f},{az:+7.3f}) |a|={mag:6.3f} m/s^2  "
                  f"g=({gx:+6.3f},{gy:+6.3f},{gz:+6.3f}) rad/s  {d['temp']:.1f}C")
        time.sleep(0.01)
    print(f'{n} samples in 3 s -> {n / 3.0:.1f} Hz')
    imu.close()
