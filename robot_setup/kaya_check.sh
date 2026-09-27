#!/usr/bin/env bash
# ============================================================================
#  kaya_check.sh - NVIDIA Kaya robot system & peripheral health check
#                  (JetPack 7.x / L4T R39 / ROS 2 Jazzy / Isaac ROS 4.x)
#
#  Verifies: Jetson platform, Super Mode, storage, thermals, Feetech ST3215
#            servos, MPU6050 IMU, Orbbec Gemini 335L, network, ROS + Isaac ROS.
#
#  Runs fully unprivileged on JetPack 7 - the SKU fuse, I2C and the servo bus
#  are all readable as a normal user once group membership is correct.
#
#  Usage:  ./kaya_check.sh [options]
#     -q, --quiet      only print WARN/FAIL lines and the summary
#     -j, --json       emit machine-readable JSON instead of a report
#     -n, --no-color   disable ANSI colour
#     -h, --help       show this help
#
#  Exit codes:  0 = all good (may include warnings)   1 = one or more failures
# ============================================================================

set -uo pipefail

# ------------------------- expected hardware config -------------------------
# Every value can be overridden from the environment, e.g.
#   SERVO_IDS_EXPECTED="1 2 3 4" ./kaya_check.sh
SERVO_BAUD=${SERVO_BAUD:-1000000}
SERVO_IDS_EXPECTED=${SERVO_IDS_EXPECTED:-"1 2 3"}
SERVO_PORT_GLOB=${SERVO_PORT_GLOB:-"/dev/ttyACM* /dev/ttyUSB*"}

IMU_ADDR=${IMU_ADDR:-0x68}
IMU_BUS_HINT=${IMU_BUS_HINT:-7}   # 40-pin header I2C (c250000.i2c); auto-detected

CAM_VID=${CAM_VID:-2bc5}
CAM_PID=${CAM_PID:-0804}
CAM_NAME=${CAM_NAME:-"Orbbec Gemini 335L"}

ROS_WS=${ROS_WS:-"$HOME/ros2_ws"}

# thresholds
SERVO_V_MIN=9.5 ; SERVO_V_WARN=10.5 ; SERVO_V_MAX=13.0
SERVO_T_WARN=55 ; SERVO_T_FAIL=70
SOC_T_WARN=75   ; SOC_T_FAIL=90
DISK_WARN=85    ; DISK_FAIL=95
MEM_WARN_MB=500
SWAP_WARN_GB=4          # builds on 8GB need real swap - see SETUP_LOG.md §8.1

# Super Mode: the GPU OPP table must reach this, else Super Mode is not active.
# nvpmodel -q is NOT proof - it reports the config file's label, not the hardware.
GPU_SUPER_HZ=${GPU_SUPER_HZ:-1020000000}

# ------------------------------- plumbing -----------------------------------
QUIET=0; JSON=0; COLOR=1
while [ $# -gt 0 ]; do
  case "$1" in
    -q|--quiet)    QUIET=1 ;;
    -j|--json)     JSON=1 ;;
    -n|--no-color) COLOR=0 ;;
    -h|--help)     awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
  esac; shift
done

[ -t 1 ] || COLOR=0
if [ "$COLOR" = 1 ]; then
  R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'; B=$'\033[34m'; C=$'\033[36m'
  BOLD=$'\033[1m'; DIM=$'\033[2m'; N=$'\033[0m'
else
  R=""; G=""; Y=""; B=""; C=""; BOLD=""; DIM=""; N=""
fi

NPASS=0; NWARN=0; NFAIL=0
JSON_ROWS=()

_json_esc(){ printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'; }
_record(){ JSON_ROWS+=("{\"check\":\"$(_json_esc "$2")\",\"status\":\"$1\",\"detail\":\"$(_json_esc "$3")\"}"); }
pass(){ NPASS=$((NPASS+1)); _record PASS "$1" "${2:-}"
        [ "$JSON" = 1 ] && return; [ "$QUIET" = 1 ] && return
        printf '  %s[ OK ]%s %-26s %s\n' "$G" "$N" "$1" "${2:-}"; }
warn(){ NWARN=$((NWARN+1)); _record WARN "$1" "${2:-}"
        [ "$JSON" = 1 ] && return
        printf '  %s[WARN]%s %-26s %s\n' "$Y" "$N" "$1" "${2:-}"; }
fail(){ NFAIL=$((NFAIL+1)); _record FAIL "$1" "${2:-}"
        [ "$JSON" = 1 ] && return
        printf '  %s[FAIL]%s %-26s %s\n' "$R" "$N" "$1" "${2:-}"; }
info(){ [ "$JSON" = 1 ] && return; [ "$QUIET" = 1 ] && return
        printf '  %s      %-26s %s%s\n' "$DIM" "$1" "${2:-}" "$N"; }
section(){ [ "$JSON" = 1 ] && return; [ "$QUIET" = 1 ] && return
        printf '\n%s%s%s\n' "$BOLD$C" "$1" "$N"; }

have(){ command -v "$1" >/dev/null 2>&1; }
fcmp(){ awk -v a="$1" -v b="$3" "BEGIN{exit !(a $2 b)}"; }

IS_ROOT=0; [ "$(id -u)" = 0 ] && IS_ROOT=1

# resolve the GPU devfreq node once (path is stable across JP6/JP7 but glob it anyway)
GPUDEV=$(ls -d /sys/devices/platform/bus@0/*.gpu/devfreq/*.gpu 2>/dev/null | head -1)
gpu_read(){ [ -n "$GPUDEV" ] && cat "$GPUDEV/$1" 2>/dev/null; }

if [ "$JSON" = 0 ] && [ "$QUIET" = 0 ]; then
  printf '%s' "$BOLD$B"
  echo "=============================================================="
  echo "  KAYA ROBOT SYSTEM CHECK   $(date '+%Y-%m-%d %H:%M:%S')"
  echo "=============================================================="
  printf '%s' "$N"
fi

# ============================== 1. PLATFORM =================================
section "[1] Jetson platform"

MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null)
[ -n "$MODEL" ] && info "carrier" "$MODEL"

# The module SKU is fused into the silicon and is the only identification that
# does not route through the (rewritable) module EEPROM. See SETUP_LOG.md §2.
SKU_FUSE=$(python3 -c "print(hex(open('/sys/bus/nvmem/devices/fuse/nvmem','rb').read()[0x10]))" 2>/dev/null)
case "$SKU_FUSE" in
  0xd3) MODULE="Orin NX 16GB" ;;
  0xd4) MODULE="Orin NX 8GB" ;;
  0xd5) MODULE="Orin Nano 8GB" ;;
  0xd6) MODULE="Orin Nano 4GB" ;;
  *)    MODULE="" ;;
esac

# fall back to the device-tree SKU if the fuse is unreadable
PNUM=""
for f in /proc/device-tree/compatible /proc/device-tree/chosen/ids; do
  [ -e "$f" ] || continue
  PNUM=$(tr '\0' '\n' < "$f" | grep -o '3767-[0-9]\{4\}' | head -1)
  [ -n "$PNUM" ] && break
done
if [ -z "$MODULE" ]; then
  case "$PNUM" in
    3767-0000) MODULE="Orin NX 16GB" ;; 3767-0001) MODULE="Orin NX 8GB" ;;
    3767-0003) MODULE="Orin Nano 8GB" ;; 3767-0004|3767-0005) MODULE="Orin Nano 4GB" ;;
    *) MODULE="unknown" ;;
  esac
fi
CPUMAX=$(awk '{printf "%d", $1/1000}' /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null)
info "module" "$MODULE${SKU_FUSE:+  (SKU fuse $SKU_FUSE)}${PNUM:+ p$PNUM}${CPUMAX:+  CPU max ${CPUMAX}MHz}"
case "$MODEL:$MODULE" in
  *"Orin NX"*:"Orin Nano"*)
    info "" "note: carrier reports 'Orin NX' but the fitted module is $MODULE" ;;
esac

if [ -f /etc/nv_tegra_release ]; then
  L4T=$(awk -F'[ ,]+' '/# R/{r=$2; sub("R","",r)} {for(i=1;i<=NF;i++) if($i=="REVISION:") rev=$(i+1)} END{print r"."rev}' /etc/nv_tegra_release 2>/dev/null | tr -d ':')
  [ -n "$L4T" ] && pass "L4T" "$L4T" || warn "L4T" "unparsed"
else
  warn "L4T" "/etc/nv_tegra_release missing"
fi

JP=$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null)
if [ -n "$JP" ]; then
  pass "JetPack" "$JP"
else
  # the meta-package is removed by the Isaac ROS OpenCV resolution; the SDK
  # components themselves survive, so check for those instead of failing.
  CU=$(ls -d /usr/local/cuda-* 2>/dev/null | head -1)
  if [ -n "$CU" ]; then
    info "JetPack meta" "not installed (removed by Isaac ROS dep resolution - expected)"
  else
    fail "JetPack" "meta-package and CUDA both absent"
  fi
fi

info "kernel" "$(uname -r)"
info "OS" "$(. /etc/os-release 2>/dev/null; echo "${PRETTY_NAME:-unknown}")"

CUDA_V=$(/usr/local/cuda/bin/nvcc --version 2>/dev/null | grep -oE 'release [0-9.]+' | awk '{print $2}')
if [ -n "$CUDA_V" ]; then pass "CUDA" "$CUDA_V (nvcc responds)"
elif [ -f /usr/local/cuda/version.json ]; then
  pass "CUDA" "$(python3 -c "import json;print(json.load(open('/usr/local/cuda/version.json'))['cuda']['version'])" 2>/dev/null)"
else warn "CUDA" "not found"; fi

TRT=$(dpkg-query -W -f='${Version}' libnvinfer10 2>/dev/null)
[ -n "$TRT" ] && info "TensorRT" "$TRT"
CUDNN=$(dpkg-query -W -f='${Version}' libcudnn9-cuda-13 2>/dev/null || dpkg-query -W -f='${Version}' libcudnn9 2>/dev/null)
[ -n "$CUDNN" ] && info "cuDNN" "$CUDNN"

# ============================ 2. SUPER MODE =================================
section "[2] Super Mode"

# THE test. nvpmodel -q reports the label from the active config file and will
# happily say MAXN_SUPER on a board running 15W clocks. The OPP table cannot lie:
# those frequency points come from the BPMP firmware device tree.
GPU_TABLE=$(gpu_read available_frequencies)
GPU_MAX=$(gpu_read max_freq)
GPU_TOP=$(echo "$GPU_TABLE" | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -n | tail -1)

if [ -n "$GPU_TOP" ] && [ "$GPU_TOP" -ge "$GPU_SUPER_HZ" ] 2>/dev/null; then
  pass "GPU clock table" "tops out at $((GPU_TOP/1000000)) MHz - Super Mode ACTIVE"
elif [ -n "$GPU_TOP" ]; then
  fail "GPU clock table" "tops out at $((GPU_TOP/1000000)) MHz - Super Mode NOT active"
  info "" "requires reflash with 'jetson-orin-nano-devkit-super' (SETUP_LOG.md §3)"
else
  warn "GPU clock table" "unreadable at ${GPUDEV:-<no devfreq node>}"
fi
[ -n "$GPU_MAX" ] && info "GPU max_freq" "$((GPU_MAX/1000000)) MHz"
[ -n "$CPUMAX" ] && info "CPU max_freq" "${CPUMAX} MHz"

# corroborate with the flashed board spec
TNSPEC=$( { grep TNSPEC /etc/nv_boot_control.conf 2>/dev/null || \
            sudo -n grep TNSPEC /etc/nv_boot_control.conf 2>/dev/null; } | awk '{print $2}')
if [ -n "$TNSPEC" ]; then
  case "$TNSPEC" in
    *-super-*) info "flashed spec" "$TNSPEC" ;;
    *)         warn "flashed spec" "$TNSPEC  <- lacks '-super', clocks are capped" ;;
  esac
fi
case "$(tr '\0' ' ' < /proc/device-tree/compatible 2>/dev/null)" in
  *-super*) info "device tree" "carries the -super suffix" ;;
  *)        info "device tree" "no -super suffix" ;;
esac

PMODE=$(nvpmodel -q 2>/dev/null | grep -i 'power mode' | sed 's/.*: *//')
[ -n "$PMODE" ] && info "power mode" "$PMODE  ${DIM}(label only - not proof)${N}"

# ========================== 3. RESOURCES / THERMAL ==========================
section "[3] Resources & thermals"

MEM_AVAIL=$(awk '/MemAvailable/{printf "%d", $2/1024}' /proc/meminfo)
MEM_TOTAL=$(awk '/MemTotal/{printf "%d", $2/1024}' /proc/meminfo)
if [ "$MEM_AVAIL" -lt "$MEM_WARN_MB" ]; then
  warn "memory" "${MEM_AVAIL}MB available of ${MEM_TOTAL}MB"
else
  pass "memory" "${MEM_AVAIL}MB available of ${MEM_TOTAL}MB"
fi

# 8GB board: without real swap, any sizeable colcon build OOMs the machine.
SWAP_GB=$(awk '/SwapTotal/{printf "%.1f", $2/1048576}' /proc/meminfo)
if fcmp "$SWAP_GB" ">=" "$SWAP_WARN_GB"; then
  pass "swap" "${SWAP_GB}GB configured"
else
  warn "swap" "only ${SWAP_GB}GB - large builds will OOM (see SETUP_LOG.md §8.1)"
fi

DISK_PCT=$(df --output=pcent / 2>/dev/null | tail -1 | tr -dc '0-9')
DISK_H=$(df -h --output=used,avail,size / 2>/dev/null | tail -1 | awk '{print $1" used, "$2" free of "$3}')
if   [ "${DISK_PCT:-0}" -ge "$DISK_FAIL" ]; then fail "rootfs" "${DISK_PCT}% full - $DISK_H"
elif [ "${DISK_PCT:-0}" -ge "$DISK_WARN" ]; then warn "rootfs" "${DISK_PCT}% full - $DISK_H"
else pass "rootfs" "${DISK_PCT}% used - $DISK_H"; fi

ROOTDEV=$(findmnt -no SOURCE / 2>/dev/null)
case "$ROOTDEV" in
  *nvme*)   info "boot media" "NVMe ($ROOTDEV)" ;;
  *mmcblk*) info "boot media" "SD/eMMC ($ROOTDEV)" ;;
  *)        info "boot media" "$ROOTDEV" ;;
esac

TMAX=0; TMAXNAME=""
for z in /sys/class/thermal/thermal_zone*; do
  [ -r "$z/temp" ] || continue
  t=$(cat "$z/temp" 2>/dev/null); [ -z "$t" ] && continue
  [ "$t" -le 0 ] 2>/dev/null && continue
  tc=$((t/1000)); n=$(cat "$z/type" 2>/dev/null)
  if [ "$tc" -gt "$TMAX" ]; then TMAX=$tc; TMAXNAME=$n; fi
done
if   [ "$TMAX" -ge "$SOC_T_FAIL" ]; then fail "temperature" "${TMAX}C on ${TMAXNAME} - throttling risk"
elif [ "$TMAX" -ge "$SOC_T_WARN" ]; then warn "temperature" "${TMAX}C on ${TMAXNAME}"
elif [ "$TMAX" -gt 0 ]; then pass "temperature" "max ${TMAX}C (${TMAXNAME})"
else warn "temperature" "no readable thermal zones"; fi

info "uptime / load" "$(uptime | sed 's/.*up //; s/,  *[0-9]* user.*load average:/ | load/')"

# =============================== 4. SERVOS ==================================
section "[4] Feetech ST3215 servos"

SERVO_PORT=""
for g in $SERVO_PORT_GLOB; do
  for p in $g; do [ -e "$p" ] && { SERVO_PORT="$p"; break 2; }; done
done

if [ -z "$SERVO_PORT" ]; then
  fail "servo bus" "no /dev/ttyACM* or /dev/ttyUSB* present - controller unplugged?"
elif [ ! -r "$SERVO_PORT" ] || [ ! -w "$SERVO_PORT" ]; then
  fail "servo bus" "$SERVO_PORT not accessible - add user to 'dialout' group, then re-login"
else
  BRIDGE=$(udevadm info -q property -n "$SERVO_PORT" 2>/dev/null | awk -F= '/^ID_VENDOR_FROM_DATABASE=/{print $2}')
  info "bus" "$SERVO_PORT${BRIDGE:+  ($BRIDGE)}"

  SERVO_OUT=$(SERVO_PORT="$SERVO_PORT" SERVO_BAUD="$SERVO_BAUD" \
              SERVO_IDS="$SERVO_IDS_EXPECTED" python3 - <<'PY' 2>&1
import os, sys, time
try:
    import serial
except ImportError:
    print("ERR|pyserial not installed (pip3 install pyserial)"); sys.exit(0)

PORT = os.environ["SERVO_PORT"]; BAUD = int(os.environ["SERVO_BAUD"])
WANT = [int(x) for x in os.environ["SERVO_IDS"].split()]

def cks(p): return (~sum(p[2:]) & 0xFF) & 0xFF
def pkt(sid, instr, params=()):
    p = [0xFF, 0xFF, sid, len(params) + 2, instr, *params]; p.append(cks(p))
    return bytearray(p)
def txrx(ser, sid, instr, params=(), rlen=0):
    ser.reset_input_buffer(); ser.write(pkt(sid, instr, params))
    time.sleep(0.006)
    r = ser.read(6 + rlen)
    if len(r) >= 6 + rlen and r[0] == 0xFF and r[1] == 0xFF and r[2] == sid:
        return r[5:5 + rlen], r[4]
    return None, None

EBITS = [(0x01,"voltage"),(0x02,"angle"),(0x04,"overheat"),
         (0x08,"overcurrent"),(0x20,"overload")]
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
except Exception as e:
    print(f"ERR|cannot open {PORT}: {e}"); sys.exit(0)

seen = {}
for sid in WANT + [i for i in range(1, 31) if i not in WANT]:
    _, err = txrx(ser, sid, 0x01)
    if err is not None: seen[sid] = err

for sid, err in sorted(seen.items()):
    pos,  _ = txrx(ser, sid, 0x02, (0x38, 2), 2)
    load, _ = txrx(ser, sid, 0x02, (0x3C, 2), 2)
    volt, _ = txrx(ser, sid, 0x02, (0x3E, 1), 1)
    temp, _ = txrx(ser, sid, 0x02, (0x3F, 1), 1)
    P = int.from_bytes(pos, "little") if pos else -1
    L = int.from_bytes(load, "little") if load else -1
    V = volt[0] / 10.0 if volt else -1
    T = temp[0] if temp else -1
    flags = ",".join(n for b, n in EBITS if err & b) or "none"
    print(f"SERVO|{sid}|{P}|{V}|{T}|{L}|{flags}")
ser.close()
print("FOUND|" + " ".join(str(s) for s in sorted(seen)))
PY
)

  if echo "$SERVO_OUT" | grep -q '^ERR|'; then
    fail "servo bus" "$(echo "$SERVO_OUT" | sed -n 's/^ERR|//p')"
  else
    FOUND=$(echo "$SERVO_OUT" | sed -n 's/^FOUND|//p')
    while IFS='|' read -r tag sid pos volt temp load flags; do
      [ "$tag" = "SERVO" ] || continue
      det="pos=$pos  ${volt}V  ${temp}C  load=$load"
      if [ "$flags" != "none" ]; then
        fail "servo id=$sid" "$det  errors=$flags"
      elif fcmp "$volt" "<" "$SERVO_V_MIN" || fcmp "$volt" ">" "$SERVO_V_MAX"; then
        fail "servo id=$sid" "$det  <- voltage out of range"
      elif [ "$temp" -ge "$SERVO_T_FAIL" ] 2>/dev/null; then
        fail "servo id=$sid" "$det  <- overheating"
      elif fcmp "$volt" "<" "$SERVO_V_WARN"; then
        warn "servo id=$sid" "$det  <- battery low"
      elif [ "$temp" -ge "$SERVO_T_WARN" ] 2>/dev/null; then
        warn "servo id=$sid" "$det  <- running hot"
      else
        pass "servo id=$sid" "$det"
      fi
    done <<< "$SERVO_OUT"

    for want in $SERVO_IDS_EXPECTED; do
      echo " $FOUND " | grep -q " $want " || fail "servo id=$want" "MISSING - no ping response"
    done
    for got in $FOUND; do
      echo " $SERVO_IDS_EXPECTED " | grep -q " $got " || warn "servo id=$got" "unexpected extra servo on bus"
    done
    [ -z "$FOUND" ] && fail "servos" "none responded at ${SERVO_BAUD} baud - check power rail"
  fi
fi

# ================================ 5. IMU ====================================
section "[5] MPU6050 IMU"

IMU_BUS=""
for b in $IMU_BUS_HINT $(ls /dev/i2c-* 2>/dev/null | sed 's|/dev/i2c-||' | sort -n); do
  [ -e "/dev/i2c-$b" ] || continue
  if python3 -c "
import smbus2,sys
try:
    bus=smbus2.SMBus($b); v=bus.read_byte_data($IMU_ADDR,0x75); bus.close()
    sys.exit(0 if v==0x68 else 1)
except Exception: sys.exit(1)" 2>/dev/null; then IMU_BUS=$b; break; fi
done

if [ -z "$IMU_BUS" ]; then
  if ! python3 -c "import smbus2" 2>/dev/null; then
    warn "MPU6050" "python3-smbus2 not installed - cannot probe"
  else
    fail "MPU6050" "not responding at $IMU_ADDR on any I2C bus - check SDA/SCL wiring, or 'i2c' group membership"
  fi
else
  BUSNAME=$(cat "/sys/bus/i2c/devices/i2c-$IMU_BUS/name" 2>/dev/null)
  pass "MPU6050 present" "addr $IMU_ADDR on /dev/i2c-$IMU_BUS${BUSNAME:+ ($BUSNAME)}  WHO_AM_I=0x68"

  IMU_OUT=$(IMU_BUS="$IMU_BUS" IMU_ADDR="$IMU_ADDR" python3 - <<'PY' 2>&1
import os, time, math, smbus2
bus_n = int(os.environ["IMU_BUS"]); addr = int(os.environ["IMU_ADDR"], 16)
b = smbus2.SMBus(bus_n)
def r16(reg):
    hi, lo = b.read_byte_data(addr, reg), b.read_byte_data(addr, reg + 1)
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v
try:
    orig = b.read_byte_data(addr, 0x6B); woke = bool(orig & 0x40)
    if woke:
        b.write_byte_data(addr, 0x6B, 0x01)      # wake, PLL w/ X-gyro ref
        b.write_byte_data(addr, 0x1A, 0x03)      # DLPF ~44Hz
        time.sleep(0.2)
    accel_g = 16384.0 / (1 << ((b.read_byte_data(addr, 0x1C) >> 3) & 3))
    gyro_d  = 131.0   / (1 << ((b.read_byte_data(addr, 0x1B) >> 3) & 3))
    N, sa, sg = 20, [0.0]*3, [0.0]*3
    for _ in range(N):
        for i, r in enumerate((0x3B, 0x3D, 0x3F)): sa[i] += r16(r)
        for i, r in enumerate((0x43, 0x45, 0x47)): sg[i] += r16(r)
        time.sleep(0.005)
    ax, ay, az = [v / N / accel_g for v in sa]
    gx, gy, gz = [v / N / gyro_d  for v in sg]
    tc = r16(0x41) / 340.0 + 36.53
    mag = math.sqrt(ax*ax + ay*ay + az*az)
    gbias = math.sqrt(gx*gx + gy*gy + gz*gz)
    print(f"IMU|{mag:.3f}|{ax:.2f}|{ay:.2f}|{az:.2f}|{gx:.1f}|{gy:.1f}|{gz:.1f}|{tc:.1f}|{orig:#04x}|{gbias:.1f}")
    if woke: b.write_byte_data(addr, 0x6B, orig)  # leave the chip as we found it
except Exception as e:
    print(f"ERR|{e}")
finally:
    b.close()
PY
)
  if echo "$IMU_OUT" | grep -q '^ERR|'; then
    warn "MPU6050 readback" "$(echo "$IMU_OUT" | sed -n 's/^ERR|//p')"
  else
    IFS='|' read -r _ mag ax ay az gx gy gz tc orig gbias <<< "$(echo "$IMU_OUT" | grep '^IMU|')"
    det="accel=[$ax $ay $az]g  ${tc}C"
    # At rest |a| must be 1g in ANY orientation - independent of how the robot sits.
    if fcmp "$mag" ">" 0.90 && fcmp "$mag" "<" 1.10; then
      pass "MPU6050 accel" "|a|=${mag}g  $det"
    else
      warn "MPU6050 accel" "|a|=${mag}g, expected 1.0 at rest - needs offset calibration  $det"
      info "" "consistent |a| error = accelerometer bias, not a bus fault; run a 6-point calibration"
    fi
    if fcmp "$gbias" "<" 3.0; then
      pass "MPU6050 gyro" "bias ${gbias} d/s at rest  gyro=[$gx $gy $gz]"
    else
      warn "MPU6050 gyro" "bias ${gbias} d/s at rest (expected <3) - subtract offsets in software"
    fi
    [ "$orig" = "0x40" ] && info "" "chip was asleep (normal power-on state); test woke it and restored it"
  fi
fi

# =============================== 6. CAMERA ==================================
section "[6] Orbbec Gemini 335L depth camera"

if ! have lsusb; then
  warn "camera" "lsusb not available (install usbutils)"
elif ! lsusb -d ${CAM_VID}:${CAM_PID} >/dev/null 2>&1; then
  fail "camera" "$CAM_NAME (${CAM_VID}:${CAM_PID}) NOT detected on USB"
  info "" "check: USB 3.0 cable + port; camera draws ~900mA"
else
  CAMPATH=""
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    if [ "$(cat "$d/idVendor" 2>/dev/null)" = "$CAM_VID" ] &&
       [ "$(cat "$d/idProduct" 2>/dev/null)" = "$CAM_PID" ]; then CAMPATH="$d"; break; fi
  done
  SPEED=$(cat "$CAMPATH/speed" 2>/dev/null)
  SERIAL=$(cat "$CAMPATH/serial" 2>/dev/null)

  if [ "${SPEED:-0}" -ge 5000 ] 2>/dev/null; then
    pass "camera link" "${SPEED} Mbps SuperSpeed${SERIAL:+  sn=$SERIAL}"
  elif [ -n "$SPEED" ]; then
    fail "camera link" "only ${SPEED} Mbps - USB 2.0 link, depth+RGB will not stream"
  else
    warn "camera link" "speed unreadable"
  fi

  NVID=$(ls /dev/video* 2>/dev/null | wc -l)
  RD=0; for v in /dev/video*; do [ -r "$v" ] && RD=$((RD+1)); done
  if [ "$NVID" -gt 0 ] && [ "$RD" -gt 0 ]; then
    pass "video nodes" "$NVID nodes, $RD readable"
  elif [ "$NVID" -gt 0 ]; then
    fail "video nodes" "$NVID present but none readable - add user to 'video' group"
  else
    fail "video nodes" "camera enumerated but no /dev/video* - uvcvideo not bound?"
  fi

  # The SDK drives camera control through hidraw; without the vendor udev rules
  # it is root-only and the node fails at init.
  HIDOK=0; for h in /dev/hidraw*; do [ -r "$h" ] && [ -w "$h" ] && HIDOK=1; done
  if ls /dev/hidraw* >/dev/null 2>&1; then
    [ "$HIDOK" = 1 ] && pass "hidraw access" "read/write - SDK control interface OK" \
                     || warn "hidraw access" "root-only; install Orbbec udev rules (SETUP_LOG.md §7)"
  fi

  # ROS driver: built in the workspace, not shipped as a deb
  if [ -d "$ROS_WS/install/orbbec_camera" ]; then
    LDIR="$ROS_WS/install/orbbec_camera/share/orbbec_camera/launch"
    if [ -f "$LDIR/gemini_330_series.launch.py" ]; then
      pass "orbbec_camera driver" "built in $ROS_WS  (launch: gemini_330_series.launch.py)"
    else
      warn "orbbec_camera driver" "built but gemini_330_series.launch.py missing"
    fi
    # the 2052 workaround is a local source patch and is lost on re-clone
    if grep -q 'unreadable on this' "$ROS_WS/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp" 2>/dev/null; then
      info "" "firmware workaround patch present (property 2052)"
    elif [ -d "$ROS_WS/src/OrbbecSDK_ROS2" ]; then
      warn "orbbec patch" "property-2052 patch ABSENT - camera init will fail (SETUP_LOG.md §7)"
    fi
  else
    warn "orbbec_camera driver" "not built - see SETUP_LOG.md §7"
  fi
fi

# ============================== 7. NETWORK ==================================
section "[7] Network"

WIFI_DEV=$(ls /sys/class/net 2>/dev/null | grep -E '^(wl|wlan)' | head -1)
if [ -n "$WIFI_DEV" ]; then
  SSID=$(iw dev "$WIFI_DEV" link 2>/dev/null | awk '/SSID/{print $2}')
  if [ -n "$SSID" ]; then
    SIG=$(iw dev "$WIFI_DEV" link 2>/dev/null | awk '/signal/{print $2" "$3}')
    RATE=$(iw dev "$WIFI_DEV" link 2>/dev/null | awk '/tx bitrate/{print $3" "$4}')
    FREQ=$(iw dev "$WIFI_DEV" link 2>/dev/null | awk '/freq/{print $2}')
    WIP=$(ip -4 -brief addr show "$WIFI_DEV" 2>/dev/null | awk '{print $3}')
    pass "wifi" "$SSID  $WIP  ${SIG}  ${RATE}${FREQ:+  ${FREQ}MHz}"
    # 5 GHz is ~7x the throughput of 2.4 GHz on this card
    [ -n "$FREQ" ] && [ "${FREQ%%.*}" -lt 3000 ] 2>/dev/null && \
      info "" "on 2.4GHz - force 5GHz with: nmcli c m '$SSID' 802-11-wireless.band a"
    PS=$(iw dev "$WIFI_DEV" get power_save 2>/dev/null | awk '{print $3}')
    [ "$PS" = "on" ] && warn "wifi powersave" "ON - causes high SSH latency; nmcli c m '$SSID' wifi.powersave 2"
  else
    warn "wifi" "$WIFI_DEV present but not associated"
  fi
else
  # JetPack 7 ships no Intel/Realtek-USB wireless driver; iwlwifi is built by hand
  if lspci 2>/dev/null | grep -qi 'network controller'; then
    fail "wifi" "card present on PCIe but no interface - iwlwifi not loaded (SETUP_LOG.md §5.2)"
  else
    info "wifi" "no wireless interface"
  fi
fi
lsmod 2>/dev/null | grep -q '^iwlwifi' && info "iwlwifi" "loaded (out-of-tree build - rebuild after kernel updates)"

for i in $(ls /sys/class/net 2>/dev/null | grep -E '^(eth|enP|en[ospx])'); do
  ST=$(cat "/sys/class/net/$i/operstate" 2>/dev/null)
  EIP=$(ip -4 -brief addr show "$i" 2>/dev/null | awk '{print $3}')
  [ "$ST" = "up" ] && pass "ethernet $i" "${EIP:-no IPv4}" || info "ethernet $i" "$ST"
done

if ping -c 1 -W 3 8.8.8.8 >/dev/null 2>&1; then
  RTT=$(ping -c 3 -W 3 8.8.8.8 2>/dev/null | awk -F'/' '/rtt|round-trip/{printf "%.0fms avg", $5}')
  pass "internet" "reachable  $RTT"
else
  warn "internet" "no route to 8.8.8.8"
fi
getent hosts github.com >/dev/null 2>&1 && pass "DNS" "resolving" || warn "DNS" "resolution failing"

# ======================== 8. ROS 2 / ISAAC ROS ==============================
section "[8] ROS 2 & Isaac ROS"

ROS_DIR=$(ls -d /opt/ros/*/ 2>/dev/null | head -1)
if [ -z "$ROS_DIR" ]; then
  warn "ROS 2" "not installed"
else
  RDISTRO=$(basename "$ROS_DIR")
  NPKG=$(ls "$ROS_DIR/share" 2>/dev/null | wc -l)
  pass "ROS 2" "$RDISTRO  ($NPKG packages)"

  missing=""
  for p in rviz2 nav2_bringup robot_localization xacro nav2_mppi_controller \
           controller_manager robot_state_publisher tf2_ros; do
    [ -d "$ROS_DIR/share/$p" ] || missing="$missing $p"
  done
  [ -z "$missing" ] && pass "robot stack" "nav2 + control + mppi + tf2 present" \
                    || warn "robot stack" "missing:$missing"

  imissing=""
  for p in isaac_ros_visual_slam nvblox_ros nvblox_nav2 isaac_ros_nitros \
           isaac_ros_image_proc isaac_ros_tensor_rt; do
    [ -d "$ROS_DIR/share/$p" ] || imissing="$imissing $p"
  done
  if [ -z "$imissing" ]; then
    pass "Isaac ROS" "cuVSLAM + nvblox + NITROS + TensorRT present"
    # A package can be installed yet unloadable if a dependency was pulled out.
    # ldd MUST run with the ROS environment sourced - ROS's own message libraries
    # are found via LD_LIBRARY_PATH, so an unsourced ldd reports ~26 false misses.
    SLIB=$(find "$ROS_DIR/lib" -name 'libisaac_ros_visual_slam_node.so' 2>/dev/null | head -1)
    if [ -n "$SLIB" ]; then
      NMISS=$(bash -c "source '$ROS_DIR/setup.bash' >/dev/null 2>&1; ldd '$SLIB' 2>/dev/null | grep -c 'not found'")
      if [ "${NMISS:-1}" = 0 ]; then
        pass "cuVSLAM linkage" "all shared libs resolve (with ROS env)"
      else
        fail "cuVSLAM linkage" "$NMISS missing shared libs even with ROS sourced"
        bash -c "source '$ROS_DIR/setup.bash' >/dev/null 2>&1; ldd '$SLIB' 2>/dev/null | grep 'not found'" \
          | head -3 | sed 's/^/        /'
      fi
    fi
  else
    warn "Isaac ROS" "missing:$imissing"
  fi
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
  info "workspace" "$ROS_WS ($(ls "$ROS_WS/install" 2>/dev/null | wc -l) packages)"
fi

for m in serial:pyserial smbus2:smbus2 cv2:OpenCV numpy:numpy; do
  mod=${m%%:*}; label=${m##*:}
  V=$(python3 -c "import $mod,sys; print(getattr($mod,'__version__','present'))" 2>/dev/null)
  [ -n "$V" ] && info "python: $label" "$V"
done

systemctl is-active ssh >/dev/null 2>&1 && pass "sshd" "active" || warn "sshd" "not running"

# =============================== SUMMARY ====================================
if [ "$JSON" = 1 ]; then
  printf '{"timestamp":"%s","host":"%s","summary":{"pass":%d,"warn":%d,"fail":%d},"checks":[' \
    "$(date -Iseconds)" "$(hostname)" "$NPASS" "$NWARN" "$NFAIL"
  first=1; for r in "${JSON_ROWS[@]}"; do [ $first = 1 ] || printf ','; printf '%s' "$r"; first=0; done
  printf ']}\n'
else
  printf '\n%s' "$BOLD$B"; echo "=============================================================="; printf '%s' "$N"
  if [ "$NFAIL" -gt 0 ]; then
    printf '  %sRESULT: %d FAILED%s, %d warnings, %d passed\n' "$BOLD$R" "$NFAIL" "$N" "$NWARN" "$NPASS"
  elif [ "$NWARN" -gt 0 ]; then
    printf '  %sRESULT: OK with %d warning(s)%s, %d passed\n' "$BOLD$Y" "$NWARN" "$N" "$NPASS"
  else
    printf '  %sRESULT: ALL SYSTEMS GOOD%s - %d checks passed\n' "$BOLD$G" "$N" "$NPASS"
  fi
  printf '%s' "$BOLD$B"; echo "=============================================================="; printf '%s' "$N"
fi

[ "$NFAIL" -gt 0 ] && exit 1 || exit 0
