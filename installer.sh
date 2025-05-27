#!/usr/bin/env bash
# installer.sh  – OpenDartboard relay setup
# Tested on Raspberry Pi OS Lite 64-bit (kernel 6.6)  – Pi Zero 2 W

set -euo pipefail

### --- Tunables -----------------------------------------------------------
FPS=10
WIDTH=1280
HEIGHT=720
PORT_BASE=8081                 # cam0→8081, cam1→8082, cam2→8083
USTREAMER_REPO=https://github.com/pikvm/ustreamer
TMP_DIR=/tmp/ustreamer
### -----------------------------------------------------------------------

echo "==> Updating apt & installing build/runtime deps"
sudo apt-get update -y
sudo apt-get install -y \
  git build-essential pkg-config \
  libevent-dev libjpeg-dev **libbsd-dev** \
  v4l-utils

echo "==> Cleaning any stale clone"
sudo rm -rf "${TMP_DIR}"

echo "==> Cloning & building uStreamer"
git clone --depth 1 "${USTREAMER_REPO}" "${TMP_DIR}"
make -C "${TMP_DIR}"
sudo install -m 755 "${TMP_DIR}/ustreamer" /usr/local/bin/

echo "==> Creating v4l2 mode-lock helper"
sudo tee /usr/local/bin/lock_cams.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
for d in /dev/video{0,1,2}; do
  [ -e "$d" ] || continue
  v4l2-ctl -d "$d" --set-fmt-video=width=1280,height=720,pixelformat=MJPG --set-parm=10
done
EOF
sudo chmod +x /usr/local/bin/lock_cams.sh

echo "==> Creating lock-cams systemd unit"
sudo tee /etc/systemd/system/lock_cams.service >/dev/null <<'EOF'
[Unit]
Description=Fix USB webcams to MJPEG 720p10
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/lock_cams.sh

[Install]
WantedBy=multi-user.target
EOF

echo "==> Creating uStreamer service files"
for idx in 0 1 2; do
  port=$((PORT_BASE + idx))
  sudo tee /etc/systemd/system/ustreamer-cam${idx}.service >/dev/null <<EOF
[Unit]
Description=uStreamer camera $idx
After=network.target

[Service]
ExecStart=/usr/local/bin/ustreamer -d /dev/video$idx \\
  --host 0.0.0.0 --port ${port} --persistent \\
  --resolution ${WIDTH}x${HEIGHT} --fps ${FPS}
Restart=always

[Install]
WantedBy=multi-user.target
EOF
done

echo "==> Enabling & starting services"
sudo systemctl daemon-reload
sudo systemctl enable --now lock_cams.service
for idx in 0 1 2; do
  sudo systemctl enable --now ustreamer-cam${idx}.service
done

echo "==> Success!  Streams available at:"
ip=$(hostname -I | awk '{print $1}')
for idx in 0 1 2; do
  echo "   http://${ip}:$((PORT_BASE + idx))/stream"
done
