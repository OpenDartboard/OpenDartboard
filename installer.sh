#!/usr/bin/env bash
# installer.sh – OpenDartboard relay (Pi Zero 2 W, 3-cam)
# Tested on Raspberry Pi OS Lite (64-bit) kernel 6.6  (May 2025)

set -euo pipefail

### ---------- Tunables ----------------------------------------------------
FPS=10
WIDTH=1280
HEIGHT=720
PORT_BASE=8081                     # cam0→8081, cam1→8082, cam2→8083
USTREAMER_REPO=https://github.com/pikvm/ustreamer
TMP_DIR=/tmp/ustreamer
DUMMY_LABEL=("dummy0" "dummy1" "dummy2")
### -----------------------------------------------------------------------

echo "==> APT update & install deps"
sudo apt-get update -y
sudo apt-get install -y \
  git build-essential pkg-config \
  libevent-dev libjpeg-dev libbsd-dev \
  v4l-utils v4l2loopback-dkms ffmpeg imagemagick \
  curl

echo "==> Ensure v4l2loopback module autoloads"
sudo tee /etc/modprobe.d/v4l2loopback.conf >/dev/null <<'EOF'
options v4l2loopback devices=3 video_nr=10,11,12 card_label="dummy0,dummy1,dummy2" exclusive_caps=1
EOF
echo v4l2loopback | sudo tee -a /etc/modules-load.d/v4l2loopback.conf >/dev/null
sudo modprobe v4l2loopback || true     # may already be loaded

### --- Build uStreamer if not present ------------------------------------
if ! command -v ustreamer >/dev/null; then
  echo "==> Building uStreamer from source (first-time only)"
  sudo rm -rf "$TMP_DIR"
  git clone --depth 1 "$USTREAMER_REPO" "$TMP_DIR"
  make -C "$TMP_DIR"
  sudo install -m 755 "$TMP_DIR/ustreamer" /usr/local/bin/
else
  echo "==> uStreamer binary already present – skipping build"
fi

### --- Dummy placeholder frame & feeder ----------------------------------
echo "==> Creating dummy-frame feeder"
sudo tee /usr/local/bin/feed_dummy.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
FRAME=/tmp/nocam.jpg
if [ ! -f "$FRAME" ]; then
  convert -size 1280x720 xc:black -fill white -gravity center -pointsize 72 \
    -annotate 0 'NO CAMERA' "$FRAME"
fi
for d in /dev/video{10,11,12}; do
  # feed forever (-stream_loop -1) at 10 fps MJPEG
  ffmpeg -loglevel quiet -re -stream_loop -1 -i "$FRAME" \
         -vf fps=10 -f v4l2 -vcodec mjpeg "$d" &
done
EOF
sudo chmod +x /usr/local/bin/feed_dummy.sh

sudo tee /etc/systemd/system/dummyfeed.service >/dev/null <<'EOF'
[Unit]
Description=Feed placeholder frames to v4l2loopback
After=multi-user.target
ConditionPathExists=/dev/video10

[Service]
Type=oneshot
ExecStart=/usr/local/bin/feed_dummy.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl enable dummyfeed.service

### --- Script to lock real cams to MJPEG 720p10 ---------------------------
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

### --- uStreamer services (smart device switch) --------------------------
echo "==> Creating uStreamer service files"
for idx in 0 1 2; do
  real=/dev/video$idx
  dummy=/dev/video$((10+idx))
  port=$((PORT_BASE + idx))

  sudo tee /etc/systemd/system/ustreamer-cam${idx}.service >/dev/null <<EOF
[Unit]
Description=uStreamer cam ${idx}
After=network.target dummyfeed.service lock_cams.service
Requires=dummyfeed.service

[Service]
# Symlink to dummy if real cam not present
ExecStartPre=/bin/sh -c 'test -e ${real} || ln -sf ${dummy} ${real}'
ExecStart=/usr/local/bin/ustreamer -d ${real} \\
  --host 0.0.0.0 --port ${port} --persistent \\
  --resolution ${WIDTH}x${HEIGHT} --desired-fps ${FPS}
Restart=always

[Install]
WantedBy=multi-user.target
EOF
done

### --- udev rule: restart on hot-plug ------------------------------------
echo '==> Adding udev hot-plug rule'
sudo tee /etc/udev/rules.d/99-opendartboard.rules >/dev/null <<'EOF'
# Restart matching uStreamer unit when a real cam is added or removed
SUBSYSTEM=="video4linux", KERNEL=="video[0-2]", ACTION=="add|remove", \
  RUN+="/bin/systemctl restart ustreamer-cam%k.service"
EOF
sudo udevadm control --reload

### --- Enable & start everything -----------------------------------------
sudo systemctl daemon-reload
sudo systemctl enable --now ustreamer-cam0.service ustreamer-cam1.service ustreamer-cam2.service

echo -e "\n==> Success!  Streams available at:"
ip=$(hostname -I | awk '{print $1}')
for idx in 0 1 2; do
  echo "   http://${ip}:$((PORT_BASE+idx))/stream"
done
