#!/usr/bin/env bash
# installer.sh – OpenDartboard relay (Pi Zero 2 W, 3-cam, minimal)

set -euo pipefail

### --- Tunables -----------------------------------------------------------
FPS=10
WIDTH=1280
HEIGHT=720
PORT_BASE=8081                     # cam0→8081, cam1→8082, cam2→8083
USTREAMER_REPO=https://github.com/pikvm/ustreamer
TMP_DIR=/tmp/ustreamer
### -----------------------------------------------------------------------

echo "==> APT update & install deps"
sudo apt-get update -y
sudo apt-get install -y \
  git build-essential pkg-config \
  libevent-dev libjpeg-dev libbsd-dev \
  v4l-utils v4l2loopback-dkms curl    # no ffmpeg / imagemagick now

echo "==> Configure v4l2loopback (dummy devices)"
sudo tee /etc/modprobe.d/v4l2loopback.conf >/dev/null <<'EOF'
options v4l2loopback devices=3 video_nr=10,11,12 \
  card_label="dummy0,dummy1,dummy2" exclusive_caps=1 fmt_list=JPEG
EOF
echo v4l2loopback | sudo tee -a /etc/modules-load.d/v4l2loopback.conf >/dev/null

echo "==> Loading v4l2loopback (wait if DKMS builds)…"
until sudo modprobe v4l2loopback 2>/dev/null; do
  echo "   …module not ready, waiting 10 s"
  sleep 10
done

### --- Build uStreamer only if missing -----------------------------------
if ! command -v ustreamer >/dev/null; then
  echo "==> Building uStreamer (first-time only)"
  sudo rm -rf "${TMP_DIR}"
  git clone --depth 1 "${USTREAMER_REPO}" "${TMP_DIR}"
  make -C "${TMP_DIR}"
  sudo install -m 755 "${TMP_DIR}/ustreamer" /usr/local/bin/
else
  echo "==> uStreamer already installed – skipping build"
fi

### --- Lock real cams to MJPEG 720p10 ------------------------------------
sudo tee /usr/local/bin/lock_cams.sh >/dev/null <<EOF
#!/usr/bin/env bash
for d in /dev/video{0,1,2}; do
  [ -e "\$d" ] || continue
  v4l2-ctl -d "\$d" --set-fmt-video=width=${WIDTH},height=${HEIGHT},pixelformat=MJPG --set-parm=${FPS}
done
EOF
sudo chmod +x /usr/local/bin/lock_cams.sh

sudo tee /etc/systemd/system/lock_cams.service >/dev/null <<'EOF'
[Unit]
Description=Force webcams into MJPEG 720p10
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/lock_cams.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl enable lock_cams.service

### --- uStreamer services (auto-switch real ↔ dummy) ----------------------
for idx in 0 1 2; do
  real=/dev/video$idx
  dummy=/dev/video$((10+idx))
  port=$((PORT_BASE + idx))

sudo tee /etc/systemd/system/ustreamer-cam${idx}.service >/dev/null <<EOF
[Unit]
Description=uStreamer cam ${idx}
After=network.target lock_cams.service

[Service]
ExecStartPre=/bin/sh -c 'test -e ${real} || ln -sf ${dummy} ${real}'
ExecStart=/usr/local/bin/ustreamer -d ${real} \\
  --host 0.0.0.0 --port ${port} --persistent \\
  --resolution ${WIDTH}x${HEIGHT} --desired-fps ${FPS}
Restart=always

[Install]
WantedBy=multi-user.target
EOF
done

### --- Hot-plug udev rule -------------------------------------------------
sudo tee /etc/udev/rules.d/99-opendartboard.rules >/dev/null <<'EOF'
SUBSYSTEM=="video4linux", KERNEL=="video[0-2]", ACTION=="add|remove", \
  RUN+="/bin/systemctl restart ustreamer-cam%k.service"
EOF
sudo udevadm control --reload

### --- Start everything ---------------------------------------------------
sudo systemctl daemon-reload
sudo systemctl enable --now ustreamer-cam0.service ustreamer-cam1.service ustreamer-cam2.service

echo -e "\n==> Streams ready:"
ip=$(hostname -I | awk '{print $1}')
for idx in 0 1 2; do
  echo "   http://${ip}:$((PORT_BASE + idx))/stream"
done
