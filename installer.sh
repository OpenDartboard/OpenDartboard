#!/usr/bin/env bash
# install_dart_streamer.sh
# One-shot setup for 3-camera MJPEG relay on Raspberry Pi Zero 2 W
# Tested Pi OS Lite 2025-04-02, kernel 6.6

set -euo pipefail

### --- Tunables -----------------------------------------------------------
FPS=10
WIDTH=1280
HEIGHT=720
PORT_BASE=8081                 # cam0→8081, cam1→8082, cam2→8083
USTREAMER_REPO=https://github.com/pikvm/ustreamer
### -----------------------------------------------------------------------

echo "==> Updating apt & installing deps"
sudo apt-get update -y
sudo apt-get install -y git build-essential libevent-dev libjpeg-dev v4l-utils

echo "==> Cloning & building uStreamer"
git clone --depth 1 "$USTREAMER_REPO" /tmp/ustreamer
make -C /tmp/ustreamer
sudo install -m 755 /tmp/ustreamer/ustreamer /usr/local/bin/

echo "==> Creating v4l2 mode-lock script"
cat <<'EOF' | sudo tee /usr/local/bin/lock_cams.sh >/dev/null
#!/usr/bin/env bash
for d in /dev/video{0,1,2}; do
  v4l2-ctl -d "$d" --set-fmt-video=width=1280,height=720,pixelformat=MJPG --set-parm=10
done
EOF
sudo chmod +x /usr/local/bin/lock_cams.sh
sudo /usr/local/bin/lock_cams.sh   # run once now

echo "==> Creating uStreamer service files"
for idx in 0 1 2; do
  port=$((PORT_BASE + idx))
  cat <<EOF | sudo tee /etc/systemd/system/ustreamer-cam${idx}.service >/dev/null
[Unit]
Description=uStreamer camera $idx
After=network.target

[Service]
ExecStart=/usr/local/bin/ustreamer -d /dev/video$idx \
  --host 0.0.0.0 --port $port --persistent \
  --resolution ${WIDTH}x${HEIGHT} --fps ${FPS}
Restart=always

[Install]
WantedBy=multi-user.target
EOF
done

echo "==> Enabling services"
sudo systemctl daemon-reload
sudo systemctl enable --now lock_cams.service 2>/dev/null || true   # optional
for idx in 0 1 2; do
  sudo systemctl enable --now ustreamer-cam${idx}.service
done

echo "==> All done!  Streams at:"
hostname=$(hostname -I | awk '{print $1}')
for idx in 0 1 2; do
  port=$((PORT_BASE + idx))
  echo "   http://${hostname}:${port}/stream"
done