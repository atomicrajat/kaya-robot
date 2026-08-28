"""Six-position accelerometer + gyro bias calibration for the Kaya MPU-6050.

Why six positions
-----------------
A single stationary average cannot separate the two error terms in

    measured = scale * true + offset

because one equation has two unknowns. Point an axis UP and then DOWN and you
get two equations:

    up   = scale * (+g) + offset
    down = scale * (-g) + offset

    =>  offset = (up + down) / 2          <- flips sign with gravity, so it survives the sum
        scale  = (up - down) / (2g)       <- constant under inversion, so it survives the difference

Three axes x two directions = six placements. The gyro needs no such trick: it
should read zero whenever the board is still, so its bias is just the mean.
"""

import argparse
import math
import os
import statistics
import sys
import time

from kaya_imu.mpu6050 import MPU6050, STANDARD_GRAVITY

AXES = ('x', 'y', 'z')
SETTLE_S = 1.0          # discard this much after you stop moving the board
SAMPLES = 400           # ~4 s at 100 Hz
STILL_GYRO_MAX = 0.05   # rad/s; above this the board is still moving
STILL_ACC_STD_MAX = 0.15  # m/s^2


def collect(imu, n=SAMPLES):
    """Gather n samples, returning per-axis means and standard deviations."""
    acc = [[], [], []]
    gyr = [[], [], []]
    for _ in range(n):
        d = imu.read()
        for i in range(3):
            acc[i].append(d['accel'][i])
            gyr[i].append(d['gyro'][i])
        time.sleep(0.01)
    return {
        'acc_mean': [statistics.fmean(a) for a in acc],
        'acc_std': [statistics.stdev(a) for a in acc],
        'gyro_mean': [statistics.fmean(g) for g in gyr],
    }


def dominant_axis(acc_mean):
    """Which axis is vertical, and pointing which way?

    Returns (index, sign). The largest-magnitude component is the one aligned
    with gravity. Sign is positive when that axis points UP -- an accelerometer
    at rest measures the normal force opposing gravity ("proper acceleration"),
    so the axis pointing skyward reads +g, not -g.
    """
    i = max(range(3), key=lambda k: abs(acc_mean[k]))
    return i, (1 if acc_mean[i] > 0 else -1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bus', type=int, default=7)
    ap.add_argument('--address', type=lambda s: int(s, 0), default=0x68)
    ap.add_argument('--output', default=os.path.expanduser(
        '~/ros2_ws/src/kaya/kaya_imu/config/imu_calibration.yaml'))
    ap.add_argument('--samples', type=int, default=SAMPLES)
    args = ap.parse_args(argv)

    imu = MPU6050(bus=args.bus, address=args.address)
    print(f'MPU6050 ready on i2c-{args.bus} @ 0x{args.address:02x}\n')
    print('Six placements needed: each of X, Y, Z pointing UP and DOWN.')
    print('Rest the board on a flat, solid surface each time and let go of it.')
    print('Order does not matter -- the axis is detected automatically.\n')

    # readings[(axis_index, sign)] = mean of that axis in that placement
    readings = {}
    gyro_samples = []

    while len(readings) < 6:
        missing = [f'{AXES[i]}{"+" if s > 0 else "-"}'
                   for i in range(3) for s in (1, -1) if (i, s) not in readings]
        print(f'--- {len(readings)}/6 done. Still need: {" ".join(missing)}')
        try:
            input('    Place the board, let go, then press Enter: ')
        except (EOFError, KeyboardInterrupt):
            print('\naborted'); imu.close(); return 1

        time.sleep(SETTLE_S)
        s = collect(imu, args.samples)

        gyro_mag = math.sqrt(sum(v * v for v in s['gyro_mean']))
        worst_std = max(s['acc_std'])
        if gyro_mag > STILL_GYRO_MAX or worst_std > STILL_ACC_STD_MAX:
            print(f'    NOT STILL (gyro {gyro_mag:.3f} rad/s, accel sd {worst_std:.3f}) '
                  f'-- retaking. Make sure nothing is touching the board.\n')
            continue

        idx, sign = dominant_axis(s['acc_mean'])
        key = (idx, sign)
        label = f'{AXES[idx]}{"+" if sign > 0 else "-"}'
        if key in readings:
            print(f'    Already have {label}. Turn the board to a new face.\n')
            continue

        readings[key] = s['acc_mean'][idx]
        gyro_samples.append(s['gyro_mean'])
        print(f'    captured {label}: {s["acc_mean"][idx]:+.4f} m/s^2 '
              f'(noise sd {s["acc_std"][idx]:.4f})\n')

    # ------------------------------------------------------------ solve
    accel_offset, accel_scale = [], []
    for i in range(3):
        up, down = readings[(i, 1)], readings[(i, -1)]
        accel_offset.append((up + down) / 2.0)
        accel_scale.append((up - down) / (2.0 * STANDARD_GRAVITY))

    # Gyro bias: mean across every stationary placement. Averaging over six
    # different orientations also averages out any small g-sensitivity the
    # gyro has, which a single-position average would bake in.
    gyro_offset = [statistics.fmean([g[i] for g in gyro_samples]) for i in range(3)]

    print('=' * 62)
    print('RESULT')
    for i, ax in enumerate(AXES):
        print(f'  accel {ax}: offset {accel_offset[i]:+8.4f} m/s^2   '
              f'scale {accel_scale[i]:.4f}')
    for i, ax in enumerate(AXES):
        print(f'  gyro  {ax}: offset {gyro_offset[i]:+8.5f} rad/s '
              f'({math.degrees(gyro_offset[i]):+7.3f} deg/s)')
    print('=' * 62)

    # ------------------------------------------------------------ verify
    print('\nVerification -- leave the board still in any orientation.')
    input('Press Enter: ')
    time.sleep(SETTLE_S)
    v = collect(imu, args.samples)
    corr = [(v['acc_mean'][i] - accel_offset[i]) / accel_scale[i] for i in range(3)]
    raw_mag = math.sqrt(sum(x * x for x in v['acc_mean']))
    cor_mag = math.sqrt(sum(x * x for x in corr))
    gcorr = [v['gyro_mean'][i] - gyro_offset[i] for i in range(3)]
    print(f'  |a| raw        {raw_mag:7.4f} m/s^2   (error {raw_mag - STANDARD_GRAVITY:+.4f})')
    print(f'  |a| corrected  {cor_mag:7.4f} m/s^2   (error {cor_mag - STANDARD_GRAVITY:+.4f})')
    print(f'  gyro corrected ({gcorr[0]:+.5f}, {gcorr[1]:+.5f}, {gcorr[2]:+.5f}) rad/s')
    drift = math.degrees(math.sqrt(sum(x * x for x in gcorr))) * 60
    print(f'  residual heading drift ~{drift:.2f} deg/min '
          f'(was {math.degrees(math.sqrt(sum(x*x for x in v["gyro_mean"]))) * 60:.1f})')

    # ------------------------------------------------------------ write
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w') as fh:
        fh.write('# MPU-6050 calibration for the Kaya robot.\n')
        fh.write('# Generated by: ros2 run kaya_imu calibrate\n')
        fh.write(f'# {time.strftime("%Y-%m-%d %H:%M:%S")}  chip temp during run matters:\n')
        fh.write('# these biases drift with temperature; recalibrate if the board\n')
        fh.write('# will run far from the temperature it was calibrated at.\n')
        fh.write('# Model: true = (measured - offset) / scale\n\n')
        fh.write('accel_offset: [%.6f, %.6f, %.6f]\n' % tuple(accel_offset))
        fh.write('accel_scale:  [%.6f, %.6f, %.6f]\n' % tuple(accel_scale))
        fh.write('gyro_offset:  [%.8f, %.8f, %.8f]\n' % tuple(gyro_offset))
        fh.write('gyro_scale:   [1.0, 1.0, 1.0]\n')
    print(f'\nwritten -> {args.output}')
    imu.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
