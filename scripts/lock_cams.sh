#!/usr/bin/env bash
# lock_cams.sh – keep all webcams in the requested MJPEG mode forever.

WIDTH=${WIDTH:-1280}
HEIGHT=${HEIGHT:-720}
FPS=${FPS:-10}
PIXELFORMAT=${PIXELFORMAT:-MJPG}

log() { echo "[lock_cams] $*"; }

# Ensure v4l2-ctl exists (package dep covers this, but be nice at runtime)
command -v v4l2-ctl >/dev/null || { log "v4l2-ctl missing"; sleep 60; exit 0; }

log "Daemon started – target ${WIDTH}x${HEIGHT}@${FPS} ${PIXELFORMAT}"

while true; do
  for cam in /dev/video{0,1,2}; do
    [[ -e "$cam" ]] || continue

    # Check current mode; only change if necessary to minimise USB churn
    if ! v4l2-ctl -d "$cam" --get-fmt-video --get-parm 2>/dev/null | \
         grep -q "pixelformat ${PIXELFORMAT}"; then
      log "Configuring $cam"
      v4l2-ctl -d "$cam" \
        --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$PIXELFORMAT \
        --set-parm=$FPS \
        --set-ctrl=power_line_frequency=1,exposure_auto_priority=0 \
        || log "⚠️  Could not set mode on $cam (maybe unsupported)"
    fi
  done
  sleep 10          # re-check every 10 s for hot-plug/unplug
done