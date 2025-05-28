#!/usr/bin/env bash
# install.sh – OpenDashboard 3-cam headless dart solution (Pi Zero 2 W)

set -euo pipefail



#############################################################################
# ----------------------------- CLI flags ----------------------------------
#############################################################################
VERSION=0.1.0
UPGRADE=0
FORCE=0
show_help() {
  echo "Usage: $0 [--upgrade] [--force-rebuild] [--version] [--help]"
  exit 0
}
for arg in "$@"; do
  case "$arg" in
    --upgrade)       UPGRADE=1 ;;
    --force-rebuild) FORCE=1   ;;
    --version)       cat /usr/local/share/opendashboard/.build-info 2>/dev/null \
                       || echo "OpenDashboard not installed yet"; exit 0 ;;
    --help|-h)       show_help ;;
    *) echo "Unknown flag $arg"; show_help ;;
  esac
done
echo "==> OpenDashboard install script v${VERSION}, upgrading: ${UPGRADE}, force rebuild: ${FORCE}"

#############################################################################
# -------------------------- system packages -------------------------------
#############################################################################
echo "==> APT update & dependencies"
sudo apt-get update -y
sudo apt-get install -y \
  git build-essential cmake pkg-config \
  libopencv-dev \
  libgstreamer1.0-dev gstreamer1.0-plugins-base gstreamer1.0-libav \
  v4l-utils gettext-base sudo           # envsubst comes from gettext-base

#############################################################################
# ----------------------------- Tunables -----------------------------------
#############################################################################
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-15}

# Where stuff lives once installed
PREFIX=/usr/local                             # bin + share/opendashboard
STATE_DIR=${PREFIX}/share/opendashboard       # models + .build-info

# Git & model sources
OD_REPO=https://github.com/OpenDashboard/OpenDashboard
NCNN_REPO=https://github.com/Tencent/ncnn

# Prep folders
sudo mkdir -p "${PREFIX}"/{bin,src}  "${STATE_DIR}"/{models}
sudo chown "$(id -u):$(id -g)" "${PREFIX}" "${STATE_DIR}" -R
touch "${STATE_DIR}/.build-info" 2>/dev/null || true

#############################################################################
# ----------------------------- helpers ------------------------------------
#############################################################################
record_info() {  # record_info KEY HASH
  grep -v "^$1=" "${STATE_DIR}/.build-info" 2>/dev/null >"${STATE_DIR}/.tmp" || true
  echo "$1=$2" >>"${STATE_DIR}/.tmp"
  mv "${STATE_DIR}/.tmp" "${STATE_DIR}/.build-info"
}
commit_of() { git -C "$1" rev-parse --short HEAD 2>/dev/null || echo "unknown"; }
stored()   { grep "^$1=" "${STATE_DIR}/.build-info" 2>/dev/null | cut -d= -f2 || echo ""; }
sync_repo() {                                  # sync_repo URL DIR
  if [[ ! -d "$2/.git" ]]; then
    git clone --depth 1 "$1" "$2"
  elif [[ $UPGRADE -eq 1 ]]; then
    git -C "$2" fetch --quiet origin && git -C "$2" reset --hard origin/HEAD
  fi
}

#############################################################################
# ---------------------------- build ncnn ----------------------------------
#############################################################################
NCNN_SRC=${PREFIX}/src/ncnn
sync_repo "${NCNN_REPO}" "${NCNN_SRC}"

rebuild_ncnn=0
if [[ $FORCE -eq 1 ]]; then
  rm -rf "${NCNN_SRC}/build"; rebuild_ncnn=1
elif [[ ! -d "${NCNN_SRC}/build" ]]; then
  rebuild_ncnn=1
elif [[ $UPGRADE -eq 1 && $(stored NCNN_COMMIT) != $(commit_of "${NCNN_SRC}") ]]; then
  rebuild_ncnn=1
fi

if [[ $rebuild_ncnn -eq 1 ]]; then
  echo "==> Building ncnn ($(commit_of "${NCNN_SRC}"))"
  cmake -S "${NCNN_SRC}" -B "${NCNN_SRC}/build" \
        -DNCNN_VULKAN=OFF -DNCNN_SHARED_LIB=OFF -DNCNN_INT8=ON \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_C_FLAGS="-O3 -mfpu=neon-vfpv4 -mcpu=cortex-a53" \
        -DCMAKE_CXX_FLAGS="-O3 -mfpu=neon-vfpv4 -mcpu=cortex-a53"
  cmake --build "${NCNN_SRC}/build" -j4
  sudo cmake --install "${NCNN_SRC}/build"
  record_info NCNN_COMMIT "$(commit_of "${NCNN_SRC}")"
else
  echo "==> ncnn up-to-date – skipping build"
fi

#############################################################################
# ------------------------- build OpenDashboard ----------------------------
#############################################################################
OD_SRC=${PREFIX}/src/opendashboard
sync_repo "${OD_REPO}" "${OD_SRC}"

rebuild_od=0
if [[ $FORCE -eq 1 ]]; then
  rm -rf "${OD_SRC}/build"; rebuild_od=1
elif [[ ! -d "${OD_SRC}/build" ]]; then
  rebuild_od=1
elif [[ $UPGRADE -eq 1 && $(stored OD_COMMIT) != $(commit_of "${OD_SRC}") ]]; then
  rebuild_od=1
fi

if [[ $rebuild_od -eq 1 ]]; then
  echo "==> Building OpenDashboard ($(commit_of "${OD_SRC}"))"
  cmake -S "${OD_SRC}" -B "${OD_SRC}/build" \
        -DCMAKE_PREFIX_PATH="${PREFIX}"
  cmake --build "${OD_SRC}/build" -j4
  sudo cp "${OD_SRC}/build/opendashboard" "${PREFIX}/bin/"
  record_info OD_COMMIT "$(commit_of "${OD_SRC}")"
else
  echo "==> OpenDashboard up-to-date – skipping build"
fi

#############################################################################
# ------------------------------- model ------------------------------------
#############################################################################
echo "==> Checking model files"
if [[ ! -f "${STATE_DIR}/models/dart.param" || $FORCE -eq 1 ]]; then
  echo "==> Copying built-in model files"
  sudo cp "${OD_SRC}/models/"*.param "${STATE_DIR}/models/"
  sudo cp "${OD_SRC}/models/"*.bin   "${STATE_DIR}/models/"
else
  echo "==> Model files already present – skipping"
fi

#############################################################################
# -------------------------- camera lock script ----------------------------
#############################################################################
echo "==> Creating camera lock script"
sudo tee /usr/local/bin/lock_cams.sh >/dev/null <<EOF
#!/usr/bin/env bash
for d in /dev/video{0,1,2}; do
  [[ -e \$d ]] || continue
  v4l2-ctl -d \$d --set-fmt-video=width=${WIDTH},height=${HEIGHT},pixelformat=MJPG \
                   --set-parm=${FPS} \
                   --set-ctrl=power_line_frequency=1,exposure_auto_priority=0
done
EOF
sudo chmod +x /usr/local/bin/lock_cams.sh

#############################################################################
# ----------------------------- unit files ---------------------------------
#############################################################################
echo "==> Generating systemd unit files"
TEMPLATES="${OD_SRC}/templates"
export WIDTH HEIGHT FPS STATE_DIR PREFIX     # vars for envsubst

sudo envsubst < "${TEMPLATES}/lock_cams.service.template" \
     | sudo tee /etc/systemd/system/lock_cams.service >/dev/null

sudo envsubst < "${TEMPLATES}/opendashboard.service.template" \
     | sudo tee /etc/systemd/system/opendashboard.service >/dev/null

#############################################################################
# --------------------------- enable & start -------------------------------
#############################################################################
echo "==> Enabling and starting services"
sudo systemctl daemon-reload
sudo systemctl enable --now lock_cams.service opendashboard.service

echo
echo "✅  OpenDashboard install/upgrade complete!"
cat "${STATE_DIR}/.build-info"
echo "  • Live output:  journalctl -fu opendashboard.service"
echo "  • Re-run with  --upgrade  or  --force-rebuild  whenever needed."