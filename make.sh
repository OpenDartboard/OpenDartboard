#!/usr/bin/env bash
# make.sh – OpenDartboard 3-cam scorer (Pi Zero 2 W)

export PKG_VERSION=0.1.0
export PKG_NAME=opendartboard
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-15}

# setup the build environment
mkdir -p ${PKG_NAME}_${PKG_VERSION}/usr/local/{bin,share/opendartboard/models}
cp opendartboard           ${PKG_NAME}_${PKG_VERSION}/usr/local/bin/
cp yolov8-darts-int8.param *.bin  ${PKG_NAME}_${PKG_VERSION}/usr/local/share/opendartboard/models/

# Minimal DEBIAN/control file
mkdir -p ${PKG_NAME}_${PKG_VERSION}/DEBIAN
cat >${PKG_NAME}_${PKG_VERSION}/DEBIAN/control <<EOF
Package: ${PKG_NAME}
Version: ${PKG_VERSION}-1
Section: misc
Priority: optional
Architecture: armhf
Maintainer: opendartboard community <many>
Depends: libopencv-core4.8, gstreamer1.0-libav, libc6 (>=2.31)
Description: Headless 3-camera dart scorer for Pi Zero 2 W
 Prints D20|T20|MISS to stdout using YOLOv8-nano + ncnn.
EOF

# Generate service files from templates
envsubst < templates/opendartboard.service.template > opendartboard.service
envsubst < templates/lock_cams.service.template > lock_cams.service

# (Optional) systemd unit in /lib/systemd/system
install -Dm644 opendartboard.service \
     ${PKG_NAME}_${PKG_VERSION}/lib/systemd/system/opendartboard.service
install -Dm644 lock_cams.service \
     ${PKG_NAME}_${PKG_VERSION}/lib/systemd/system/lock_cams.service

# 4. Build the .deb
dpkg-deb --build ${PKG_NAME}_${PKG_VERSION}