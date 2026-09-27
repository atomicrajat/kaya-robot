# Kaya Robot — Jetson Setup Log

Complete record of bringing the Kaya robot's Jetson from JetPack 6.2.1 to a working
JetPack 7.2.1 + ROS 2 Jazzy + Isaac ROS 4.6.0 stack, including every failure and its fix.

**Date:** 2026-08-27
**Result:** All subsystems operational. Super Mode enabled (GPU 624.75 → 1020 MHz).

---

## 1. Final state

| Layer | Version / value |
|---|---|
| Module | **Jetson Orin Nano 8GB** (`p3767-0003`, SKU fuse `0xD5`) |
| Carrier | Orin Nano Developer Kit (`p3768-0000`) |
| JetPack | **7.2.1-b49** (L4T R39.2.1) |
| OS / kernel | Ubuntu 24.04.4 / `6.8.12-1021-tegra` |
| Power mode | **25W**, `MAXN_SUPER` available |
| GPU clock | **1020 MHz** (was 624.75 MHz) |
| CPU clock | **1728 MHz** (was 1510 MHz) |
| CUDA / TensorRT / cuDNN / VPI | 13.2 / 10.16.2 / 9.20.0 / 4.1.4 |
| ROS 2 | **Jazzy**, 454 packages |
| Isaac ROS | **4.6.0** (bare-metal mode) |
| Boot media | NVMe, WD Blue SN550 500GB |
| Hostname / user | `kaya` / `orin-nano` |

### Network

| Interface | Address | Notes |
|---|---|---|
| `wlP1p1s0` | 192.168.1.28 | Intel AC 8265, **780 Mbit/s @ 5 GHz** |
| `enP8p1s0` | 192.168.1.64 | onboard gigabit |
| `l4tbr0` | 192.168.55.1 | USB gadget (ECM) |

### Peripherals

| Device | Interface | Status |
|---|---|---|
| 3× Feetech ST3215 servos | `/dev/ttyACM0` @ 1 Mbaud, IDs 1/2/3 | OK |
| MPU6050 IMU | `/dev/i2c-7` @ `0x68` | OK — **needs calibration** |
| Orbbec Gemini 335L | USB3 SuperSpeed, sn `CP3C641000JS` | OK — colour/depth/IR streaming |

---

## 2. Module identification — it is an Orin Nano, not an Orin NX

The carrier board's device-tree model string said *"Orin NX Engineering Reference Developer
Kit"*, which is misleading — that carrier is shared across both module families.

**Decisive proof: the silicon SKU fuse.**

```bash
python3 -c "print(hex(open('/sys/bus/nvmem/devices/fuse/nvmem','rb').read()[0x10]))"
# 0xd5

sudo dmesg | grep 'Tegra Revision'
# Tegra Revision: A01 SKU: 213 CPU Process: 0 SoC Process: 0
```

`0xD5` (213) = Orin Nano 8GB. `0xD4` would be Orin NX 8GB, `0xD3` Orin NX 16GB.

This is read at **t=0.0038 s** from one-time-programmable fuses — before the EEPROM is read
or any board-ID logic runs. It cannot be reprogrammed.

Corroborated by EEPROM-independent measurements: 8 CUDA SMs, 7.44 GB RAM, 1510 MHz CPU
ceiling (Orin NX 8GB is 1984 MHz), and **zero DLA cores** (`trt.Builder.num_DLA_cores == 0`;
every Orin NX has 2).

> **Why this matters:** ~40 TOPS not 70–100, 8 GB shared RAM, and **no DLA to offload
> inference to**. Everything competes for one GPU. This is the binding constraint on the
> whole project.

---

## 3. Super Mode

### The problem

Only 15W/7W power modes were offered; GPU capped at 624.75 MHz.

### What does NOT work

The widely-recommended workaround is **cosmetic only**:

```bash
# DOES NOT WORK — reports success, changes nothing
sudo ln -sf /etc/nvpmodel/nvpmodel_p3767_0003_super.conf /etc/nvpmodel.conf
sudo nvpmodel -m 2
```

Tested on both JetPack 6.2.1 and 7.2.1. `nvpmodel -q` dutifully reports `MAXN_SUPER`, but
clocks never change. On reboot, `create_nvpmodel_symlink` in `/etc/systemd/nvpower.sh`
regenerates the symlink from the device-tree `compatible` string and the mode resets.

### The test that actually tells the truth

**Do not trust `nvpmodel -q`.** Check the frequency *table*:

```bash
cat /sys/devices/platform/bus@0/*.gpu/devfreq/*.gpu/available_frequencies
```

| | Before | After |
|---|---|---|
| GPU table | 306 … **624.75 MHz** | 306 408 510 612 714 816 918 **1020 MHz** |
| CPU max | 1510 MHz | 1728 MHz |

The higher clock points were **absent, not capped** — they come from the BPMP firmware
device tree, so no userspace change can create them.

### The actual fix — reflash with the super board config

The super config swaps two device trees (from `p3768-0000-p3767-0000-super.conf`):

```
BPFDTB_FILE="tegra234-bpmp-3767-0003-3768-super.dtb"     # BPMP firmware DTB — the clock tables
DTB_FILE="tegra234-p3768-0000+p3767-0003-nv-super.dtb"
```

**Use `jetson-orin-nano-devkit-super`, NOT the `-maxn` variant.** The maxn overlay only
handles SKUs 0000/0001/0002 and hard-exits on anything else:

```bash
else
    echo "Error: module SKU ${board_sku} detected";
    exit 1;
fi
```

The `super-maxn` DTBs for SKU 0003 do not exist. `-maxn` is Orin NX only.

### Verification after flashing

```bash
sudo grep TNSPEC /etc/nv_boot_control.conf
# TNSPEC 3767-300-0003-T.1-1-1-jetson-orin-nano-devkit-super-   <-- note "-super-"

tr '\0' ' ' < /proc/device-tree/compatible
# nvidia,p3768-0000+p3767-0003-super ...
```

---

## 4. Flashing JetPack 7.2.1

### ⚠️ `flash.sh` with the `-nvme` config FAILS on this board

```bash
# FAILED
sudo ./flash.sh jetson-orin-nano-devkit-super-nvme internal
```
```
Erasing nvme: 4 ... E> NV3P_SERVER: Failed to open instance 4 of device 12.
Error: Return value 17
```

This path asks the **bootloader** to write the NVMe directly, and its NV3P server can't
open the device.

### ✅ Use the initrd method instead

It boots a real Linux kernel on the Jetson (with proper NVMe drivers) and streams the image
over USB. This is what SDK Manager uses.

```bash
cd ~/nvidia/nvidia_sdk/JetPack_7.2.1_Linux_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra

# seed the user so it boots straight to a login — no OEM wizard, no monitor needed
sudo ./tools/l4t_create_default_user.sh -u orin-nano -p orin-nano -n kaya --accept-license

sudo ./tools/kernel_flash/l4t_initrd_flash.sh \
  --external-device nvme0n1p1 \
  -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
  -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
  --network usb0 \
  jetson-orin-nano-devkit-super internal
```

Note the target has **no `-nvme` suffix** here — NVMe is handled by `--external-device`/`-c`,
QSPI by `-p`.

### Gotcha: stale recovery mode

After a failed flash the board still enumerates as `0955:7523 APX`, but the tool reports
`SoC: Unknown` and `No Board Spec. and no target connected`. The chip is stuck mid-handshake.

**A USB replug does not fix this.** Power-cycle into recovery mode (jumper FC_REC pin 9 to
GND pin 10, power on, remove jumper). Confirm the USB *device number* changes — that proves a
fresh enumeration rather than the stale one.

### Benign messages during flashing

These appear in every normal flash — not errors:
- `ERROR: failed to read rcm_state`
- `E> NV3P_SERVER: Failed to get address for br bct` (expected after QSPI erase)
- `Error: try getting custinfo fail. Moving on`
- `The backup GPT table is corrupt` (leftover from the failed attempt; rewritten anyway)

---

## 5. Networking

### 5.1 USB gadget — macOS could not get a link

JetPack 7.2 defaults the USB network function to **NCM**, which macOS does not bind.
Symptom: serial console works (`/dev/cu.usbmodem*`) but no network interface activates.

**Fix** — edit `/opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-config.sh`:

```bash
#ecm_ncm_name=NCM
#ecm_ncm=ncm
ecm_ncm_name=ECM      # macOS speaks ECM natively
ecm_ncm=ecm
enable_rndis=0        # RNDIS needs 3rd-party drivers on macOS; removing it makes ECM = usb0
```

Then **reboot** (do not `systemctl restart` — that tears down the composite device including
the serial console, and it may not re-enumerate without a power cycle).

> **Why `enable_rndis=0` matters:** with RNDIS enabled, ECM lands on `usb1`, which L4T's
> startup script leaves down and unbridged — so the host enumerates a device with no L2 path.
> Disabling RNDIS makes ECM `usb0`, which the stock script brings up and bridges
> automatically. No custom systemd unit needed.

Verify: `ls /sys/class/net/l4tbr0/brif/` should list `usb0`.

### 5.2 WiFi — JetPack 7 ships no Intel or Realtek-USB driver

JetPack 5/6 included `iwlwifi` (the Xavier NX devkit shipped with an Intel 8265). **JetPack
7.2 dropped it.** The wireless module set is only:

```
ath  broadcom  marvell  mediatek  rsi  ti   (+ NVIDIA's own rtl8822ce / rtl8852ce)
```

There is **no `linux-modules-extra` for NVIDIA's L4T kernel** — the
`linux-modules-extra-6.8.0-*-nvidia-tegra` packages are Canonical's flavour and their
vermagic will not match. The module must be built from source.

#### Build procedure (repeat after every kernel update)

```bash
sudo apt install -y build-essential libncurses-dev bison flex libssl-dev libelf-dev zstd

# ~296 MB
wget https://developer.nvidia.com/downloads/embedded/l4t/r39_release_v2.1/sources/public_sources.tbz2
mkdir -p ~/jetson_kernel && cd ~/jetson_kernel && tar -xjf ../public_sources.tbz2
cd Linux_for_Tegra/source && tar -xjf kernel_src.tbz2
cd kernel/kernel-noble

zcat /proc/config.gz > .config          # exact running config — do not guess
sed -i 's/^CONFIG_LOCALVERSION=.*/CONFIG_LOCALVERSION="-1021-tegra"/' .config
cat >> .config <<'CFG'
CONFIG_WLAN_VENDOR_INTEL=y
CONFIG_IWLWIFI=m
CONFIG_IWLMVM=m
CONFIG_IWLDVM=m
CFG
make olddefconfig

# CRITICAL: must equal `uname -r` exactly, or the module will not load
make -s kernelrelease        # expect 6.8.12-1021-tegra

cp "$(readlink -f /lib/modules/$(uname -r)/build)/Module.symvers" .   # symbol CRCs

make -j6 modules_prepare
make -j6 M=drivers/net/wireless/intel/iwlwifi modules    # ~1 min (vs an hour for all modules)
sudo make M=drivers/net/wireless/intel/iwlwifi modules_install
sudo depmod -a
```

#### ⚠️ The non-obvious gotcha: `.zst` firmware

**JetPack 7's kernel cannot read compressed firmware.** The module loads fine but the card
stays dead with no clear error:

```bash
cd /lib/firmware
sudo unzstd iwlwifi-8265-34.ucode.zst
sudo unzstd iwlwifi-8265-36.ucode.zst
```

#### Load and connect

```bash
sudo modprobe iwlwifi && sudo rfkill unblock all
sudo nmcli device wifi connect "<SSID>" password "<PASS>" ifname wlP1p1s0
sudo nmcli connection modify "<SSID>" \
  connection.autoconnect yes connection.autoconnect-priority 100 \
  802-11-wireless.powersave 2 \
  802-11-wireless.band a          # force 5 GHz: 780 Mbit/s vs 115 on 2.4 GHz
```

Verified working and auto-loading after reboot.

---

## 6. Software stack

### 6.1 JetPack SDK components

A command-line flash installs **only the L4T BSP** — not CUDA/TensorRT/cuDNN/VPI. Those are
SDK Manager's "Step 2". Install them with apt instead (the repo is already configured):

```bash
sudo apt update && sudo apt install nvidia-jetpack     # ~19 GB
```

### 6.2 ROS 2 Jazzy

```bash
sudo add-apt-repository universe
V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -sL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.noble_all.deb"
sudo apt install /tmp/ros2-apt-source.deb && sudo apt update

sudo apt install ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rqt-tf-tree ros-jazzy-image-transport-plugins ros-jazzy-vision-msgs
```

### 6.3 Isaac ROS 4.6.0

```bash
k=/usr/share/keyrings/nvidia-isaac-ros.gpg
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor -o $k
echo "deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-4.6 noble-jetpack main" \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
sudo apt update && sudo apt install isaac-ros-cli
sudo isaac-ros init baremetal          # docker | venv | baremetal
```

**Bare metal chosen** over NVIDIA's recommended Docker: on an 8 GB board a containerised
duplicate ROS+CUDA stack costs disk and RAM we cannot spare.

#### ⚠️ The `isaac-ros-nvblox` metapackage is uninstallable on Jetson

It depends on `nvblox-examples-bringup` → `triton-server` →
`datacenter-gpu-manager-4-core` + `libnccl2`, which are x86/datacenter packages absent from
the Jetson repo. **Install the core packages directly instead** — only `examples-bringup`
carries that dependency:

```bash
sudo apt install --allow-downgrades \
  ros-jazzy-isaac-ros-visual-slam ros-jazzy-isaac-ros-nitros \
  ros-jazzy-isaac-ros-image-proc ros-jazzy-isaac-ros-depth-image-proc \
  ros-jazzy-isaac-ros-tensor-rt ros-jazzy-isaac-ros-dnn-image-encoder \
  ros-jazzy-isaac-ros-yolov8 ros-jazzy-isaac-ros-examples \
  ros-jazzy-nvblox-ros ros-jazzy-nvblox-msgs ros-jazzy-nvblox-nav2 \
  ros-jazzy-nvblox-rviz-plugin ros-jazzy-nvblox-ros-common ros-jazzy-nvblox-ros-python-utils
```

You lose only NVIDIA's demo launch files, not nvblox functionality.

#### ⚠️ OpenCV file conflict

NVIDIA's `opencv-licenses` and Ubuntu's `libopencv-dev` both ship
`/usr/share/licenses/opencv4/SoftFloat-COPYING.txt`. dpkg refuses, cascading into ~10
half-unpacked packages.

```bash
sudo apt-get -f install -y --allow-downgrades -o Dpkg::Options::="--force-overwrite"
sudo dpkg --configure -a --force-overwrite
```

**Consequence:** OpenCV 4.8.0 → 4.6.0, and `nvidia-jetpack` / `nvidia-jetpack-dev` /
`nvidia-opencv-dev` are removed. The first two are **metapackages only (193 KB)** — CUDA,
TensorRT and cuDNN all survive (verified). NVIDIA's OpenCV had CUDA disabled anyway, so this
costs two minor versions, not acceleration.

### 6.4 Docker for Isaac ROS

```bash
# /etc/docker/daemon.json
{ "runtimes": { "nvidia": { "path": "nvidia-container-runtime", "args": [] } },
  "default-runtime": "nvidia" }

sudo sed -i 's/^mode = "auto"/mode = "csv"/' /etc/nvidia-container-runtime/config.toml
sudo systemctl restart docker
```

`mode = "csv"` is correct for Jetson (auto-detection fails — Jetson has no NVML).

**Testing note:** GPU devices only inject when the container requests them. Plain
`ubuntu:24.04` shows nothing; NVIDIA images set this themselves:

```bash
docker run --rm -e NVIDIA_VISIBLE_DEVICES=all ubuntu:24.04 ls /dev/nvhost-gpu
```

---

## 7. Orbbec Gemini 335L

### Install

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git   # v2-main for 330-series

sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libusb-1.0-0-dev \
  libeigen3-dev ros-jazzy-image-transport ros-jazzy-image-publisher \
  ros-jazzy-camera-info-manager ros-jazzy-diagnostic-updater ros-jazzy-backward-ros libdw-dev

cd OrbbecSDK_ROS2/orbbec_camera/scripts && sudo bash ./install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The udev rules make `/dev/hidraw0` world-accessible — the SDK needs it for camera control and
fails silently without it.

### ⚠️ Build with constrained parallelism — see §8.1

```bash
cd ~/ros2_ws
MAKEFLAGS="-j2" colcon build --parallel-workers 1 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### ⚠️ Upstream bug: unguarded property read aborts device init

The 335L is a **Gemini 330 series** device — the launch file is
`gemini_330_series.launch.py` (there is no `gemini_335L.launch.py`).

Both stock launch files failed with:

```
Failed to set 2052 to 0 ... errorCode: 2, propertyId: 2052 status:1005
Failed to initialize device (Attempt 1 of 3)
```

Property 2052 is `OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT`. This firmware reports it as
supported but rejects both write and read. In `orbbec_camera/src/ob_camera_node.cpp` the
*write* is guarded by `TRY_TO_SET_PROPERTY`, but the `getIntProperty()` on the very next line
is **unguarded** — so it throws and kills device init.

**Fix applied** (original saved as `ob_camera_node.cpp.bak`) — wrap the read:

```cpp
TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT,
                    set_enable_depth_auto_exposure_priority);
try {
  RCLCPP_INFO_STREAM(logger_, "Current depth auto exposure priority: "
      << (device_->getIntProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT) ? "ON" : "OFF"));
} catch (const ob::Error &) {
  RCLCPP_WARN_STREAM(logger_, "Depth auto-exposure priority unreadable on this firmware; continuing.");
}
```

> **This patch is lost if you re-clone the repo.** Worth reporting upstream.

### Launching

**RGB-D (general use):**
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```
→ `/camera/color/image_raw` 28 Hz · `/camera/depth/image_raw` 28 Hz · `/camera/depth/points` 29 Hz

**Stereo IR for cuVSLAM:**
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_left_ir:=true enable_right_ir:=true enable_laser:=false \
  enable_frame_sync:=true time_domain:=global
```

> **`enable_laser:=false` is essential.** The IR projector's dot pattern is fixed to the
> *camera*, not the world. Feature matchers lock onto it and pose estimation silently
> degrades. Default is `true`.

Verified: both IR streams at 30 Hz, `camera_info` on both, **340 synchronised pairs in 12 s
with max stamp delta 0.036 ms** — genuine hardware sync, suitable for cuVSLAM.

---

## 8. Problems encountered

### 8.1 ⚠️ colcon OOM — the most disruptive failure

`colcon build` defaults to one worker per core (6) × unbounded compiler jobs ≈ **36
concurrent C++ compilations**. On 8 GB this consumed all RAM and swap:

```
Mem:  7.4Gi / 7.4Gi   available: 12Mi
Swap: 2.0Gi / 2.0Gi   exhausted
17 OOM kill events
c++: fatal error: Killed signal terminated program cc1plus
```

SSH and serial both stopped responding. The board recovered on its own once the OOM killer
took the compiler — no power cycle needed.

**Always build with:**
```bash
MAKEFLAGS="-j2" colcon build --parallel-workers 1
```
Constrained: **3 min 37 s**. Unconstrained: 37 minutes, then failure.

**Swap added** (fresh install had only 2 GB; the old JetPack 6 install had 10 GB):
```bash
sudo fallocate -l 6G /swapfile2 && sudo chmod 600 /swapfile2
sudo mkswap /swapfile2 && sudo swapon /swapfile2
echo '/swapfile2 none swap sw 0 0' | sudo tee -a /etc/fstab
```
Total now 8 GB, persistent.

### 8.2 Missing group memberships after a fresh flash

The new user was in `video` but not `dialout` or `i2c` — so the servos and IMU appeared
absent when they were merely unreadable.

```bash
sudo usermod -aG dialout,i2c,gpio,plugdev,render,docker $USER   # log out and back in
```

### 8.3 Summary table

| # | Problem | Cause | Fix |
|---|---|---|---|
| 1 | Super Mode unavailable | Flashed with non-super board config | Reflash `jetson-orin-nano-devkit-super` |
| 2 | `nvpmodel` symlink "works" but nothing changes | Clock tables live in BPMP firmware DTB | Only a reflash adds them |
| 3 | `flash.sh` fails on NVMe | Bootloader NV3P can't open NVMe | Use `l4t_initrd_flash.sh` |
| 4 | Board stuck `SoC: Unknown` in recovery | Stale APX state after failed flash | Power-cycle into recovery |
| 5 | No USB network on macOS | JP7 defaults to NCM | Switch to ECM, `enable_rndis=0` |
| 6 | WiFi card dead | JP7 ships no `iwlwifi` | Build from kernel source |
| 7 | WiFi still dead after build | JP7 can't read `.zst` firmware | `unzstd` the `.ucode.zst` files |
| 8 | No CUDA after flash | CLI flash installs BSP only | `apt install nvidia-jetpack` |
| 9 | `isaac-ros-nvblox` uninstallable | Pulls x86-only `triton-server` | Install core nvblox pkgs directly |
| 10 | dpkg conflict on OpenCV | NVIDIA/Ubuntu ship the same license file | `--force-overwrite` |
| 11 | Camera won't initialise | Unguarded property read, upstream bug | Patch `ob_camera_node.cpp` |
| 12 | Build locks up the board | 36 parallel compilers on 8 GB | `-j2 --parallel-workers 1` + swap |
| 13 | Servos/IMU "missing" | User not in `dialout`/`i2c` | `usermod -aG` |

---

## 9. Things that will break again

- **Kernel update → WiFi dies.** `iwlwifi` is out-of-tree; rebuild per §5.2 and re-run
  `unzstd` if firmware is reinstalled.
- **Re-cloning OrbbecSDK_ROS2 → camera breaks.** The §7 patch is lost.
- **Any large build → OOM**, unless you use constrained parallelism.
- **`nvpmodel -q` will keep lying** if anyone applies the symlink workaround. Trust
  `available_frequencies`.

---

## 10. Verification commands

```bash
# module identity (EEPROM-independent)
python3 -c "print(hex(open('/sys/bus/nvmem/devices/fuse/nvmem','rb').read()[0x10]))"   # 0xd5

# Super Mode — the real test
cat /sys/devices/platform/bus@0/*.gpu/devfreq/*.gpu/available_frequencies   # must reach 1020000000
sudo grep TNSPEC /etc/nv_boot_control.conf                                  # must contain -super-

# CUDA actually executing
/usr/local/cuda/bin/nvcc --version

# ROS + Isaac ROS
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep -E 'isaac_ros_visual_slam|nvblox_ros'
ldd /opt/ros/jazzy/lib/*/libisaac_ros_visual_slam_node.so | grep 'not found'   # expect empty

# peripherals
ls /dev/ttyACM0 && i2cget -y 7 0x68 0x75 && lsusb | grep -i orbbec
```

---

## 11. Next steps

1. **Calibrate the MPU6050** — reads `|a| = 1.50 g` at rest (should be 1.00) and 6.1 °/s gyro
   bias. Verified as bias, not scaling (consistent across all four accel ranges). Feeding this
   into an EKF untouched will make heading drift immediately.
2. Update `kaya_check.sh` for JetPack 7 and re-baseline.
3. Begin curriculum Phase 1 (ROS 2 fundamentals) — see `curriculum.html`.
