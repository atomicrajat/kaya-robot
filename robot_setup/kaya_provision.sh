#!/usr/bin/env bash
# ============================================================================
#  kaya_provision.sh - provision a freshly-flashed Jetson for the Kaya robot
#
#  Takes a clean JetPack 7.2.x (L4T R39) install to a working:
#     JetPack SDK  +  ROS 2 Jazzy  +  Isaac ROS 4.6  +  Orbbec Gemini 335L
#  applying every workaround documented in SETUP_LOG.md.
#
#  PREREQUISITE - not done by this script:
#     The board must already be flashed with the SUPER board config
#     (jetson-orin-nano-devkit-super) from a host PC, otherwise the GPU stays
#     capped at 624.75 MHz. See SETUP_LOG.md section 3-4. This script warns if
#     Super Mode is inactive but continues.
#
#  Usage:
#     ./kaya_provision.sh                    run every stage (skips what's done)
#     ./kaya_provision.sh --list             list stages
#     ./kaya_provision.sh --only ros2,orbbec run just these stages
#     ./kaya_provision.sh --skip wifi        run everything except these
#     ./kaya_provision.sh --dry-run          show what would run
#
#  Safe to re-run: every stage detects completion and skips.
#  Log: /tmp/kaya_provision.log
# ============================================================================

set -uo pipefail

ROS_DISTRO_WANT=${ROS_DISTRO_WANT:-jazzy}
ISAAC_RELEASE=${ISAAC_RELEASE:-release-4.6}
ROS_WS=${ROS_WS:-"$HOME/ros2_ws"}
KSRC_DIR=${KSRC_DIR:-"$HOME/jetson_kernel"}
L4T_SRC_URL=${L4T_SRC_URL:-"https://developer.nvidia.com/downloads/embedded/l4t/r39_release_v2.1/sources/public_sources.tbz2"}
SWAP_TARGET_GB=${SWAP_TARGET_GB:-8}
LOG=/tmp/kaya_provision.log

# On an 8GB board an unconstrained colcon build OOMs the machine hard enough to
# lock out SSH. These caps are not optional. See SETUP_LOG.md section 8.1.
BUILD_JOBS=${BUILD_JOBS:-2}
BUILD_WORKERS=${BUILD_WORKERS:-1}

STAGES=(precheck swap groups usbgadget docker jetpack ros2 isaacros wifi orbbec verify)

# ------------------------------- plumbing -----------------------------------
if [ -t 1 ]; then
  R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'; B=$'\033[34m'; C=$'\033[36m'
  BOLD=$'\033[1m'; DIM=$'\033[2m'; N=$'\033[0m'
else R=""; G=""; Y=""; B=""; C=""; BOLD=""; DIM=""; N=""; fi

ONLY=""; SKIP=""; DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --list) printf 'stages: %s\n' "${STAGES[*]}"; exit 0 ;;
    --only) ONLY="$2"; shift ;;
    --skip) SKIP="$2"; shift ;;
    --dry-run) DRY=1 ;;
    -h|--help) awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
  esac; shift
done

say()  { printf '%s==>%s %s\n' "$BOLD$C" "$N" "$*" | tee -a "$LOG"; }
ok()   { printf '    %s[ok]%s %s\n'   "$G" "$N" "$*" | tee -a "$LOG"; }
skip() { printf '    %s[skip]%s %s\n' "$DIM" "$N" "$*" | tee -a "$LOG"; }
warn() { printf '    %s[warn]%s %s\n' "$Y" "$N" "$*" | tee -a "$LOG"; }
die()  { printf '    %s[FAIL]%s %s\n' "$R" "$N" "$*" | tee -a "$LOG"; exit 1; }

want_stage(){
  local s=$1
  [ -n "$ONLY" ] && { [[ ",$ONLY," == *",$s,"* ]] || return 1; }
  [ -n "$SKIP" ] && { [[ ",$SKIP," == *",$s,"* ]] && return 1; }
  return 0
}

SUDO_OK=0
need_sudo(){
  [ "$SUDO_OK" = 1 ] && return 0
  if sudo -n true 2>/dev/null; then SUDO_OK=1; return 0; fi
  echo "This script needs sudo. You will be prompted once."
  sudo -v || die "sudo required"
  SUDO_OK=1
  # keep the timestamp alive for the length of the run
  ( while true; do sudo -n true 2>/dev/null; sleep 50; done ) & SUDO_KEEPALIVE=$!
  trap 'kill "${SUDO_KEEPALIVE:-}" 2>/dev/null' EXIT
}

apt_q(){ sudo DEBIAN_FRONTEND=noninteractive apt-get -y -q "$@"; }

: > "$LOG"
printf '%s' "$BOLD$B"
echo "=============================================================="
echo "  KAYA PROVISION   $(date '+%Y-%m-%d %H:%M:%S')"
echo "=============================================================="
printf '%s' "$N"
[ "$DRY" = 1 ] && say "DRY RUN - nothing will be changed"

# =============================== PRECHECK ===================================
if want_stage precheck; then
say "precheck"
  L4T_REL=$(sed -n 's/^# R\([0-9]*\).*REVISION: \([0-9.]*\).*/\1.\2/p' /etc/nv_tegra_release 2>/dev/null)
  case "$L4T_REL" in
    39.*) ok "L4T $L4T_REL (JetPack 7.x)" ;;
    "")   die "cannot read /etc/nv_tegra_release - is this a Jetson?" ;;
    *)    warn "L4T $L4T_REL - this script targets R39/JetPack 7.x; continuing anyway" ;;
  esac

  SKU=$(python3 -c "print(hex(open('/sys/bus/nvmem/devices/fuse/nvmem','rb').read()[0x10]))" 2>/dev/null)
  case "$SKU" in
    0xd5) ok "module: Orin Nano 8GB (SKU fuse $SKU)" ;;
    0xd4|0xd3) warn "module SKU $SKU (Orin NX) - tuned for Orin Nano 8GB but should work" ;;
    *)    warn "module SKU unreadable" ;;
  esac

  GPUDEV=$(ls -d /sys/devices/platform/bus@0/*.gpu/devfreq/*.gpu 2>/dev/null | head -1)
  TOP=$(cat "$GPUDEV/available_frequencies" 2>/dev/null | tr ' ' '\n' | sort -n | tail -1)
  if [ "${TOP:-0}" -ge 1020000000 ] 2>/dev/null; then
    ok "Super Mode active (GPU table reaches $((TOP/1000000)) MHz)"
  else
    warn "Super Mode NOT active (GPU tops at $(( ${TOP:-0}/1000000 )) MHz)"
    warn "  reflash with 'jetson-orin-nano-devkit-super' to unlock - see SETUP_LOG.md sect.3"
  fi

  FREE_G=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
  [ "${FREE_G:-0}" -ge 40 ] && ok "disk: ${FREE_G}G free" || die "need >=40G free, have ${FREE_G}G"
  ping -c1 -W4 8.8.8.8 >/dev/null 2>&1 && ok "internet reachable" || die "no internet - connect ethernet first"
fi

# ================================= SWAP =====================================
if want_stage swap; then
say "swap (an 8GB board OOMs during builds without it)"
  HAVE_GB=$(awk '/SwapTotal/{printf "%.0f", $2/1048576}' /proc/meminfo)
  if [ "${HAVE_GB:-0}" -ge "$SWAP_TARGET_GB" ]; then
    skip "already ${HAVE_GB}GB swap"
  elif [ "$DRY" = 1 ]; then
    echo "    would add $((SWAP_TARGET_GB-HAVE_GB))G swapfile"
  else
    need_sudo
    ADD=$((SWAP_TARGET_GB-HAVE_GB)); F=/swapfile_kaya
    if [ ! -f "$F" ]; then
      sudo fallocate -l "${ADD}G" "$F" && sudo chmod 600 "$F" && sudo mkswap "$F" >/dev/null
      sudo swapon "$F"
      grep -q "$F" /etc/fstab || echo "$F none swap sw 0 0" | sudo tee -a /etc/fstab >/dev/null
      ok "added ${ADD}G swap at $F (persistent)"
    else
      sudo swapon "$F" 2>/dev/null; ok "enabled existing $F"
    fi
  fi
fi

# ================================ GROUPS ====================================
if want_stage groups; then
say "group membership (servos, IMU, GPIO, camera, docker)"
  MISSING=""
  for g in dialout i2c gpio video plugdev render docker; do
    getent group "$g" >/dev/null 2>&1 || continue
    id -nG "$USER" | tr ' ' '\n' | grep -qx "$g" || MISSING="$MISSING $g"
  done
  if [ -z "$MISSING" ]; then skip "already in all required groups"
  elif [ "$DRY" = 1 ]; then echo "    would add:$MISSING"
  else
    need_sudo
    sudo usermod -aG "$(echo $MISSING | tr ' ' ',')" "$USER"
    ok "added to:$MISSING"
    warn "log out and back in for group changes to take effect"
  fi
fi

# ============================== USB GADGET ==================================
if want_stage usbgadget; then
say "USB gadget: NCM -> ECM (JetPack 7 default does not bind on macOS)"
  CFG=/opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-config.sh
  if [ ! -f "$CFG" ]; then skip "usb-device-mode not installed"
  elif grep -q '^ecm_ncm=ecm' "$CFG" 2>/dev/null && grep -q '^enable_rndis=0' "$CFG" 2>/dev/null; then
    skip "already ECM-only"
  elif [ "$DRY" = 1 ]; then echo "    would switch $CFG to ECM and disable RNDIS"
  else
    need_sudo
    sudo cp "$CFG" "$CFG.bak.$(date +%s)"
    # Disabling RNDIS matters: with it enabled ECM lands on usb1, which L4T's
    # startup script leaves down and unbridged. ECM-only makes it usb0.
    sudo sed -i -e 's/^ecm_ncm_name=NCM/#ecm_ncm_name=NCM/' \
                -e 's/^ecm_ncm=ncm/#ecm_ncm=ncm/' \
                -e 's/^#ecm_ncm_name=ECM/ecm_ncm_name=ECM/' \
                -e 's/^#ecm_ncm=ecm/ecm_ncm=ecm/' \
                -e 's/^enable_rndis=1/enable_rndis=0/' "$CFG"
    ok "switched to ECM-only (takes effect after reboot)"
    warn "do NOT 'systemctl restart' this service - it tears down the serial console too"
  fi
fi

# ================================ DOCKER ====================================
if want_stage docker; then
say "docker: nvidia default runtime + csv mode (needed by Isaac ROS containers)"
  if ! command -v docker >/dev/null 2>&1; then skip "docker not installed"
  else
    CHANGED=0
    if [ "$(docker info 2>/dev/null | awk -F': ' '/Default Runtime/{print $2}')" != "nvidia" ]; then
      if [ "$DRY" = 1 ]; then echo "    would set default-runtime=nvidia"; else
        need_sudo
        sudo cp /etc/docker/daemon.json /etc/docker/daemon.json.bak 2>/dev/null
        sudo python3 - <<'PY'
import json
p='/etc/docker/daemon.json'
try: d=json.load(open(p))
except Exception: d={}
d.setdefault('runtimes',{})['nvidia']={'path':'nvidia-container-runtime','args':[]}
d['default-runtime']='nvidia'
json.dump(d,open(p,'w'),indent=4)
PY
        CHANGED=1; ok "default-runtime -> nvidia"
      fi
    else skip "default runtime already nvidia"; fi

    # Jetson has no NVML, so the toolkit's auto-detection picks the wrong mode
    RT=/etc/nvidia-container-runtime/config.toml
    if [ -f "$RT" ] && ! grep -q '^mode = "csv"' "$RT"; then
      if [ "$DRY" = 1 ]; then echo "    would set nvidia-container-runtime mode=csv"; else
        need_sudo; sudo sed -i 's/^mode = "auto"/mode = "csv"/' "$RT"; CHANGED=1; ok "runtime mode -> csv"
      fi
    fi
    [ "$CHANGED" = 1 ] && [ "$DRY" = 0 ] && { sudo systemctl restart docker; ok "docker restarted"; }
  fi
fi

# =============================== JETPACK SDK ================================
if want_stage jetpack; then
say "JetPack SDK (CUDA / TensorRT / cuDNN / VPI)"
  if command -v /usr/local/cuda/bin/nvcc >/dev/null 2>&1; then
    skip "CUDA present ($(/usr/local/cuda/bin/nvcc --version | grep -oE 'release [0-9.]+'))"
  elif [ "$DRY" = 1 ]; then echo "    would apt install nvidia-jetpack (~19GB)"
  else
    need_sudo
    apt_q update >>"$LOG" 2>&1
    say "  downloading ~19GB, this takes a while"
    apt_q install nvidia-jetpack >>"$LOG" 2>&1 || warn "nvidia-jetpack install returned non-zero (check $LOG)"
    command -v /usr/local/cuda/bin/nvcc >/dev/null 2>&1 && ok "CUDA installed" || die "CUDA still missing"
  fi
fi

# ================================= ROS 2 ====================================
if want_stage ros2; then
say "ROS 2 $ROS_DISTRO_WANT + robot stack"
  if [ -f "/opt/ros/$ROS_DISTRO_WANT/setup.bash" ]; then
    skip "already installed ($(ls /opt/ros/$ROS_DISTRO_WANT/share | wc -l) packages)"
  elif [ "$DRY" = 1 ]; then echo "    would install ros-$ROS_DISTRO_WANT-desktop + nav2 + control (~600MB)"
  else
    need_sudo
    apt_q install software-properties-common curl >>"$LOG" 2>&1
    sudo add-apt-repository -y universe >>"$LOG" 2>&1
    V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    CODE=$(. /etc/os-release && echo "$VERSION_CODENAME")
    curl -sL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${CODE}_all.deb" \
      || die "could not fetch ros-apt-source"
    apt_q install /tmp/ros2-apt-source.deb >>"$LOG" 2>&1
    apt_q update >>"$LOG" 2>&1
    apt_q install \
      ros-$ROS_DISTRO_WANT-desktop ros-dev-tools python3-colcon-common-extensions \
      ros-$ROS_DISTRO_WANT-navigation2 ros-$ROS_DISTRO_WANT-nav2-bringup \
      ros-$ROS_DISTRO_WANT-robot-localization ros-$ROS_DISTRO_WANT-xacro \
      ros-$ROS_DISTRO_WANT-joint-state-publisher-gui ros-$ROS_DISTRO_WANT-teleop-twist-keyboard \
      ros-$ROS_DISTRO_WANT-ros2-control ros-$ROS_DISTRO_WANT-ros2-controllers \
      ros-$ROS_DISTRO_WANT-rosbag2-storage-mcap ros-$ROS_DISTRO_WANT-rqt-tf-tree \
      ros-$ROS_DISTRO_WANT-image-transport-plugins ros-$ROS_DISTRO_WANT-vision-msgs \
      >>"$LOG" 2>&1 || warn "some ROS packages failed (check $LOG)"
    [ -f "/opt/ros/$ROS_DISTRO_WANT/setup.bash" ] && ok "ROS 2 $ROS_DISTRO_WANT installed" || die "ROS install failed"
    grep -q "source /opt/ros/$ROS_DISTRO_WANT/setup.bash" ~/.bashrc 2>/dev/null || \
      echo "source /opt/ros/$ROS_DISTRO_WANT/setup.bash" >> ~/.bashrc
  fi
fi

# =============================== ISAAC ROS ==================================
if want_stage isaacros; then
say "Isaac ROS $ISAAC_RELEASE (bare-metal)"
  if [ -d "/opt/ros/$ROS_DISTRO_WANT/share/isaac_ros_visual_slam" ] && \
     [ -d "/opt/ros/$ROS_DISTRO_WANT/share/nvblox_ros" ]; then
    skip "cuVSLAM + nvblox already installed"
  elif [ "$DRY" = 1 ]; then echo "    would add isaac repo and install ~1.2GB"
  else
    need_sudo
    K=/usr/share/keyrings/nvidia-isaac-ros.gpg
    if [ ! -f "$K" ]; then
      curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key -o /tmp/isaac.key \
        || die "could not fetch Isaac ROS key"
      sudo gpg --dearmor --yes -o "$K" /tmp/isaac.key
    fi
    echo "deb [signed-by=$K] https://isaac.download.nvidia.com/isaac-ros/$ISAAC_RELEASE noble-jetpack main" \
      | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list >/dev/null
    apt_q update >>"$LOG" 2>&1
    apt_q install isaac-ros-cli >>"$LOG" 2>&1 && ok "isaac-ros-cli installed"
    sudo isaac-ros init baremetal --yes >>"$LOG" 2>&1 && ok "mode: baremetal"

    # NOTE 1: the ros-*-isaac-ros-nvblox METApackage is uninstallable on Jetson -
    #   it pulls nvblox-examples-bringup -> triton-server -> datacenter-only deps.
    #   The six core nvblox packages are clean, so install those directly.
    # NOTE 2: --allow-downgrades and --force-overwrite are both required. NVIDIA's
    #   opencv-licenses and Ubuntu's libopencv-dev ship the same license file, and
    #   the Isaac apt pin steers OpenCV to Ubuntu's build. CUDA/TensorRT/cuDNN all
    #   survive; only the nvidia-jetpack METApackages are dropped. SETUP_LOG.md §6.3
    say "  installing (allow-downgrades + force-overwrite are required, see SETUP_LOG.md)"
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --allow-downgrades \
      -o Dpkg::Options::="--force-overwrite" \
      ros-$ROS_DISTRO_WANT-isaac-ros-visual-slam ros-$ROS_DISTRO_WANT-isaac-ros-nitros \
      ros-$ROS_DISTRO_WANT-isaac-ros-image-proc ros-$ROS_DISTRO_WANT-isaac-ros-depth-image-proc \
      ros-$ROS_DISTRO_WANT-isaac-ros-tensor-rt ros-$ROS_DISTRO_WANT-isaac-ros-dnn-image-encoder \
      ros-$ROS_DISTRO_WANT-isaac-ros-yolov8 ros-$ROS_DISTRO_WANT-isaac-ros-examples \
      ros-$ROS_DISTRO_WANT-nvblox-ros ros-$ROS_DISTRO_WANT-nvblox-msgs \
      ros-$ROS_DISTRO_WANT-nvblox-nav2 ros-$ROS_DISTRO_WANT-nvblox-rviz-plugin \
      ros-$ROS_DISTRO_WANT-nvblox-ros-common ros-$ROS_DISTRO_WANT-nvblox-ros-python-utils \
      >>"$LOG" 2>&1
    # the OpenCV file clash can leave packages half-unpacked; repair in place
    sudo DEBIAN_FRONTEND=noninteractive apt-get -f install -y --allow-downgrades \
      -o Dpkg::Options::="--force-overwrite" >>"$LOG" 2>&1
    sudo dpkg --configure -a --force-overwrite >>"$LOG" 2>&1
    BROKEN=$(dpkg -l | awk '$1 !~ /^(ii|rc)/ && $1 ~ /^[a-z]/' | wc -l)
    [ "$BROKEN" = 0 ] && ok "dpkg clean" || warn "$BROKEN packages not fully configured"
    [ -d "/opt/ros/$ROS_DISTRO_WANT/share/isaac_ros_visual_slam" ] && ok "cuVSLAM present" || warn "cuVSLAM missing"
    [ -d "/opt/ros/$ROS_DISTRO_WANT/share/nvblox_ros" ] && ok "nvblox present" || warn "nvblox missing"
  fi
fi

# ================================= WIFI =====================================
if want_stage wifi; then
say "Intel WiFi (iwlwifi) - JetPack 7 ships no Intel/Realtek-USB driver"
  if ! lspci 2>/dev/null | grep -qi 'Network controller.*Intel'; then
    skip "no Intel wireless card on PCIe"
  elif ls /sys/class/net | grep -qE '^wl'; then
    skip "wireless interface already present ($(ls /sys/class/net | grep -E '^wl'))"
  elif [ "$DRY" = 1 ]; then echo "    would download ~296MB kernel source and build iwlwifi"
  else
    need_sudo
    apt_q install build-essential libncurses-dev bison flex libssl-dev libelf-dev zstd >>"$LOG" 2>&1
    KREL=$(uname -r)
    KDIR="$KSRC_DIR/Linux_for_Tegra/source/kernel/kernel-noble"
    if [ ! -d "$KDIR" ]; then
      mkdir -p "$KSRC_DIR" && cd "$KSRC_DIR"
      [ -f public_sources.tbz2 ] || { say "  downloading kernel source (~296MB)"; wget -q -c -O public_sources.tbz2 "$L4T_SRC_URL" || die "source download failed"; }
      tar -xjf public_sources.tbz2
      cd Linux_for_Tegra/source && tar -xjf kernel_src.tbz2
    fi
    cd "$KDIR" || die "kernel source not found at $KDIR"
    zcat /proc/config.gz > .config 2>/dev/null || die "/proc/config.gz unavailable"
    SUFFIX="-${KREL#*-}"; SUFFIX="${SUFFIX%%-tegra}-tegra"   # e.g. -1021-tegra
    sed -i "s/^CONFIG_LOCALVERSION=.*/CONFIG_LOCALVERSION=\"$SUFFIX\"/" .config
    grep -q '^CONFIG_LOCALVERSION=' .config || echo "CONFIG_LOCALVERSION=\"$SUFFIX\"" >> .config
    sed -i '/^#\? \?CONFIG_IWL/d' .config
    printf 'CONFIG_WLAN_VENDOR_INTEL=y\nCONFIG_IWLWIFI=m\nCONFIG_IWLMVM=m\nCONFIG_IWLDVM=m\n' >> .config
    make olddefconfig >>"$LOG" 2>&1
    GOT=$(make -s kernelrelease 2>/dev/null)
    [ "$GOT" = "$KREL" ] || die "vermagic mismatch: source '$GOT' vs running '$KREL' - module would not load"
    ok "vermagic matches ($GOT)"
    cp "$(readlink -f /lib/modules/$KREL/build)/Module.symvers" . 2>/dev/null
    say "  building (only the iwlwifi subdir - about a minute)"
    make -j$(nproc) modules_prepare >>"$LOG" 2>&1 || die "modules_prepare failed"
    make -j$(nproc) M=drivers/net/wireless/intel/iwlwifi modules >>"$LOG" 2>&1 || die "iwlwifi build failed"
    sudo make M=drivers/net/wireless/intel/iwlwifi modules_install >>"$LOG" 2>&1
    sudo depmod -a
    ok "modules installed"

    # JetPack 7's kernel cannot read .zst-compressed firmware. Without this the
    # module loads cleanly and the card stays dead with no useful error.
    cd /lib/firmware
    NDEC=0
    for f in iwlwifi-*.ucode.zst; do
      [ -e "$f" ] || continue
      [ -f "${f%.zst}" ] && continue
      sudo unzstd -q "$f" 2>/dev/null && NDEC=$((NDEC+1))
    done
    ok "decompressed $NDEC firmware blob(s)"
    sudo modprobe iwlwifi 2>/dev/null; sudo rfkill unblock all 2>/dev/null; sleep 3
    IF=$(ls /sys/class/net | grep -E '^wl' | head -1)
    [ -n "$IF" ] && ok "interface up: $IF" || warn "no wireless interface appeared - check dmesg"
    warn "iwlwifi is out-of-tree: rebuild after every kernel update"
  fi
fi

# ================================ ORBBEC ====================================
if want_stage orbbec; then
say "Orbbec Gemini 335L ROS 2 driver"
  if [ -d "$ROS_WS/install/orbbec_camera" ]; then
    skip "orbbec_camera already built in $ROS_WS"
  elif [ "$DRY" = 1 ]; then echo "    would clone v2-main, patch, and build"
  else
    need_sudo
    apt_q install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libusb-1.0-0-dev \
      libeigen3-dev libdw-dev ros-$ROS_DISTRO_WANT-image-transport \
      ros-$ROS_DISTRO_WANT-image-publisher ros-$ROS_DISTRO_WANT-camera-info-manager \
      ros-$ROS_DISTRO_WANT-diagnostic-updater ros-$ROS_DISTRO_WANT-backward-ros >>"$LOG" 2>&1

    mkdir -p "$ROS_WS/src" && cd "$ROS_WS/src"
    # v2-main is the branch where the Gemini 330-series (incl. 335L) is maintained
    [ -d OrbbecSDK_ROS2 ] || git clone -q --depth 1 -b v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git \
      || die "clone failed"
    ok "cloned OrbbecSDK_ROS2 (v2-main)"

    ( cd OrbbecSDK_ROS2/orbbec_camera/scripts && sudo bash ./install_udev_rules.sh >>"$LOG" 2>&1 )
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ok "udev rules installed (hidraw access for SDK control)"

    # UPSTREAM BUG: the write to OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT (2052) is
    # guarded by TRY_TO_SET_PROPERTY, but the getIntProperty() on the next line is
    # not. Gemini 330-series firmware reports the property as supported and then
    # rejects both ops, so the unguarded read throws and kills device init.
    F="$ROS_WS/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp"
    if grep -q 'unreadable on this firmware' "$F" 2>/dev/null; then
      skip "property-2052 patch already applied"
    else
      cp "$F" "$F.bak"
      python3 - "$F" <<'PY'
import sys
p=sys.argv[1]; L=open(p).read().split('\n')
tgt=next((i for i,l in enumerate(L)
          if 'getIntProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT)' in l), None)
if tgt is None:
    print("PATCH_SKIP"); sys.exit()
s=tgt
while s>0 and 'RCLCPP_INFO_STREAM' not in L[s]: s-=1
blk=L[s:tgt+1]; ind=' '*(len(blk[0])-len(blk[0].lstrip()))
L[s:tgt+1]=([ind+'// Gemini 330-series firmware reports this property as supported but',
             ind+'// rejects read and write. The write is guarded above; guard the read.',
             ind+'try {']
            +['  '+b for b in blk]
            +[ind+'} catch (const ob::Error &) {',
              ind+'  RCLCPP_WARN_STREAM(logger_, "Depth auto-exposure priority unreadable on this "',
              ind+'                              "firmware; continuing.");',
              ind+'}'])
open(p,'w').write('\n'.join(L)); print("PATCH_OK")
PY
      grep -q 'unreadable on this firmware' "$F" && ok "applied property-2052 patch" \
        || warn "patch not applied - upstream source may have changed"
    fi

    # Constrained parallelism is mandatory here: the default (~36 concurrent
    # compilers) exhausts RAM+swap on an 8GB board and locks out SSH entirely.
    say "  building with -j$BUILD_JOBS / $BUILD_WORKERS worker (do not raise this on 8GB)"
    cd "$ROS_WS"
    ( source "/opt/ros/$ROS_DISTRO_WANT/setup.bash"
      MAKEFLAGS="-j$BUILD_JOBS" colcon build --parallel-workers "$BUILD_WORKERS" \
        --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release ) >>"$LOG" 2>&1
    if [ -d "$ROS_WS/install/orbbec_camera" ]; then
      ok "orbbec_camera built"
      grep -q "source $ROS_WS/install/setup.bash" ~/.bashrc 2>/dev/null || \
        echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc
    else
      warn "build failed - check $LOG"
    fi
  fi
fi

# ================================ VERIFY ====================================
if want_stage verify; then
say "verify"
  [ "$DRY" = 1 ] || {
    P=0; F=0
    chk(){ if eval "$2" >/dev/null 2>&1; then ok "$1"; P=$((P+1)); else warn "$1"; F=$((F+1)); fi; }
    chk "CUDA toolchain"      "/usr/local/cuda/bin/nvcc --version"
    chk "ROS 2 $ROS_DISTRO_WANT"  "[ -f /opt/ros/$ROS_DISTRO_WANT/setup.bash ]"
    chk "Isaac ROS cuVSLAM"   "[ -d /opt/ros/$ROS_DISTRO_WANT/share/isaac_ros_visual_slam ]"
    chk "Isaac ROS nvblox"    "[ -d /opt/ros/$ROS_DISTRO_WANT/share/nvblox_ros ]"
    chk "Nav2 + MPPI"         "[ -d /opt/ros/$ROS_DISTRO_WANT/share/nav2_mppi_controller ]"
    chk "Orbbec driver"       "[ -d $ROS_WS/install/orbbec_camera ]"
    chk "swap >= ${SWAP_TARGET_GB}GB"  "[ \$(awk '/SwapTotal/{printf \"%.0f\", \$2/1048576}' /proc/meminfo) -ge $SWAP_TARGET_GB ]"
    echo
    printf '    %s%d ok, %d need attention%s\n' "$BOLD" "$P" "$F" "$N"
  }
  if [ -x "$(dirname "$0")/kaya_check.sh" ]; then
    say "run ./kaya_check.sh for the full hardware report"
  fi
fi

printf '\n%s' "$BOLD$B"
echo "=============================================================="
printf '%s  done - log: %s%s\n' "$N" "$LOG" ""
printf '%s' "$BOLD$B"
echo "=============================================================="
printf '%s' "$N"
echo
echo "  Reminders:"
echo "    - log out/in if groups were added"
echo "    - reboot to apply the USB gadget (ECM) change"
echo "    - rebuild iwlwifi after any kernel update"
echo "    - always build with: MAKEFLAGS=\"-j2\" colcon build --parallel-workers 1"
