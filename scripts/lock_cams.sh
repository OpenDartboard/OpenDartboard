#!/usr/bin/env bash
# lock_cams.sh – robust camera setup

WIDTH=${WIDTH:-1280}
HEIGHT=${HEIGHT:-720}
FPS=${FPS:-10}
PIXELFORMAT=${PIXELFORMAT:-MJPG}

echo "Starting lock_cams daemon..."

while true; do
  FOUND=0
  for d in /dev/video{0,1,2}; do
    if [ -e "$d" ]; then
      FOUND=1
      echo "Configuring $d ..."
      if ! v4l2-ctl -d "$d" --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$PIXELFORMAT --set-parm=$FPS; then
        echo "Warning: Failed to configure $d (maybe wrong format or cam missing)"
      fi
    fi
  done
  if [[ $FOUND -eq 0 ]]; then
    echo "No cameras found. Will check again in 10s..."
  fi
  sleep 10
done