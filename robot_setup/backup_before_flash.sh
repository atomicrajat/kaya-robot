#!/usr/bin/env bash
# Captures everything on the Kaya Jetson that a reflash would destroy and that
# cannot be recovered from a git clone or a package manager.
# Deliberately does NOT copy: docker images, git clone contents, miniconda,
# IDE server dirs, build/install/log artifacts. Those are all reproducible.

set -uo pipefail
STAMP=$(date +%Y%m%d-%H%M)
OUT="$HOME/kaya-backup-$STAMP"
mkdir -p "$OUT"/{work,diffs,manifests,dotfiles}

say(){ printf '  %-42s %s\n' "$1" "$2"; }
echo "Collecting into $OUT"

# ---- 1. your own ROS 2 package (untracked anywhere else) -------------------
if [ -d ~/ros2_ws/src ]; then
  tar czf "$OUT/work/ros2_ws_src.tgz" -C ~/ros2_ws src \
      --exclude='__pycache__' --exclude='*.pyc' 2>/dev/null
  say "ros2_ws/src (lekiwi_driver)" "$(du -h "$OUT/work/ros2_ws_src.tgz" | cut -f1)"
fi

# ---- 2. loose scripts in home ----------------------------------------------
n=$(ls -1 ~/*.py 2>/dev/null | wc -l)
if [ "$n" -gt 0 ]; then
  tar czf "$OUT/work/home_scripts.tgz" -C ~ $(cd ~ && ls -1 *.py 2>/dev/null) 2>/dev/null
  say "$n loose python scripts" "$(du -h "$OUT/work/home_scripts.tgz" | cut -f1)"
fi

# ---- 3. local edits inside cloned repos ------------------------------------
# Keep the changes, not the repos. Each repo yields a patch + its untracked files.
for d in ~/lerobot ~/FTServo_Python ~/python-st3215 ~/rtl8188eus ~/rtl8188fu; do
  [ -d "$d/.git" ] || continue
  name=$(basename "$d")
  cd "$d" || continue
  {
    echo "# repo:   $(git config --get remote.origin.url 2>/dev/null)"
    echo "# commit: $(git rev-parse HEAD 2>/dev/null)"
    echo "# branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null)"
    echo
  } > "$OUT/diffs/$name.patch"
  git diff HEAD >> "$OUT/diffs/$name.patch" 2>/dev/null

  # untracked files that are real source (skip caches and build junk)
  mapfile -t untracked < <(git ls-files --others --exclude-standard 2>/dev/null \
    | grep -vE '__pycache__|\.pyc$|\.o$|\.ko$|\.mod$|\.cmd$|\.save$|\.bak$')
  if [ ${#untracked[@]} -gt 0 ]; then
    tar czf "$OUT/diffs/$name-untracked.tgz" -C "$d" "${untracked[@]}" 2>/dev/null
  fi
  say "$name" "patch $(wc -l < "$OUT/diffs/$name.patch") lines, ${#untracked[@]} new files"
done

# ---- 4. dotfiles worth keeping ---------------------------------------------
for f in .bash_history .bashrc .profile; do
  [ -f ~/"$f" ] && cp ~/"$f" "$OUT/dotfiles/$f" && say "$f" "$(du -h ~/"$f" | cut -f1)"
done
if [ -d ~/.ssh ]; then
  tar czf "$OUT/dotfiles/ssh.tgz" -C ~ .ssh 2>/dev/null
  chmod 600 "$OUT/dotfiles/ssh.tgz"
  say ".ssh keys" "$(du -h "$OUT/dotfiles/ssh.tgz" | cut -f1)"
fi

# ---- 5. manifests: what to reinstall, not the thing itself -----------------
docker images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}\t{{.ID}}' \
  > "$OUT/manifests/docker-images.txt" 2>/dev/null
say "docker image list" "$(wc -l < "$OUT/manifests/docker-images.txt") images"

apt-mark showmanual > "$OUT/manifests/apt-manual.txt" 2>/dev/null
say "apt manually-installed" "$(wc -l < "$OUT/manifests/apt-manual.txt") packages"

pip3 list --format=freeze > "$OUT/manifests/pip3-freeze.txt" 2>/dev/null
say "pip3 packages" "$(wc -l < "$OUT/manifests/pip3-freeze.txt") packages"

if [ -x ~/miniconda3/bin/conda ]; then
  ~/miniconda3/bin/conda env list > "$OUT/manifests/conda-envs.txt" 2>/dev/null
  for e in $(~/miniconda3/bin/conda env list 2>/dev/null | awk '!/^#/&&NF{print $1}'); do
    ~/miniconda3/bin/conda list -n "$e" --export \
      > "$OUT/manifests/conda-env-$e.txt" 2>/dev/null
  done
  say "conda envs exported" "$(ls "$OUT/manifests"/conda-env-* 2>/dev/null | wc -l)"
fi

# clone list so the repos can be restored before the patches are applied
for d in ~/lerobot ~/jetson-containers ~/FTServo_Python ~/python-st3215 ~/rtl8188eus ~/rtl8188fu; do
  [ -d "$d/.git" ] || continue
  printf '%s\t%s\t%s\n' "$(basename "$d")" \
    "$(cd "$d" && git config --get remote.origin.url)" \
    "$(cd "$d" && git rev-parse HEAD)"
done > "$OUT/manifests/git-clones.txt" 2>/dev/null
say "git clone list" "$(wc -l < "$OUT/manifests/git-clones.txt") repos"

# ---- 6. hardware state that is not in any file -----------------------------
{
  echo "=== captured $(date -Iseconds) from $(hostname) ==="
  echo
  echo "--- module ---"
  echo "board id : $(tr -d '\0' < /proc/device-tree/chosen/ids 2>/dev/null)"
  echo "sku fuse : $(python3 -c "print(hex(open('/sys/bus/nvmem/devices/fuse/nvmem','rb').read()[0x10]))" 2>/dev/null)"
  echo "L4T      : $(head -1 /etc/nv_tegra_release)"
  echo "jetpack  : $(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null)"
  echo
  echo "--- servos (IDs live in servo EEPROM, survive the reflash) ---"
  echo "bus: /dev/ttyACM0 @ 1000000 baud, IDs 1 2 3"
  echo
  echo "--- IMU ---"
  echo "MPU6050 @ 0x68 on /dev/i2c-7 (c250000.i2c), 40-pin header pins 3/5"
  echo "NOTE: uncalibrated. |a|=1.50g at rest (should be 1.00), gyro bias 6.1 deg/s"
  echo
  echo "--- camera ---"
  echo "Orbbec Gemini 335L 2bc5:0804, needs USB3; draws 896mA"
  echo "serial: $(cat /sys/bus/usb/devices/2-1.3/serial 2>/dev/null)"
  echo
  echo "--- wifi dongle (SEE WARNING) ---"
  lsusb -d 0bda:8179 2>/dev/null
  echo "driver: $(modinfo 8188eu 2>/dev/null | awk '/^filename/{print $2}')"
  echo "out-of-tree, built for $(uname -r); NOT in dkms - must be rebuilt for kernel 6.8"
  echo "fallback: USB gadget link at 192.168.55.1 always works"
  echo
  echo "--- wifi credentials ---"
  echo "SSID: $(nmcli -t -f NAME c show 2>/dev/null | grep -iv 'bridge\|docker\|wired' | head -1)"
} > "$OUT/manifests/hardware-state.txt" 2>/dev/null
say "hardware state notes" "written"

# ---- pack ------------------------------------------------------------------
cd "$HOME"
tar czf "kaya-backup-$STAMP.tgz" "kaya-backup-$STAMP"
echo
echo "DONE -> $HOME/kaya-backup-$STAMP.tgz  ($(du -h "$HOME/kaya-backup-$STAMP.tgz" | cut -f1))"
