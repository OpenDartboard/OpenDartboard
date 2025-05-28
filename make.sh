#!/usr/bin/env bash
# make.sh – OpenDartboard 3-cam scorer (Pi Zero 2 W)

export PKG_VERSION=0.2.0
export PKG_NAME=opendartboard
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-15}

# setup the build environment
mkdir -p ${PKG_NAME}_${PKG_VERSION}/usr/local/{bin,share/opendartboard/models}
cp /usr/local/bin/opendartboard   ${PKG_NAME}_${PKG_VERSION}/usr/local/bin/
cp /usr/local/share/opendartboard/models/*.param ${PKG_NAME}_${PKG_VERSION}/usr/local/share/opendartboard/models/ 2>/dev/null || true
cp /usr/local/share/opendartboard/models/*.bin   ${PKG_NAME}_${PKG_VERSION}/usr/local/share/opendartboard/models/ 2>/dev/null || true

# Minimal DEBIAN/control file
mkdir -p ${PKG_NAME}_${PKG_VERSION}/DEBIAN
cat >${PKG_NAME}_${PKG_VERSION}/DEBIAN/control <<EOF
Package: ${PKG_NAME}
Version: ${PKG_VERSION}-1
Section: misc
Priority: optional
Architecture: arm64
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

fuck_yiou

# 4. Build the .deb
rm -f /app/*.deb
dpkg-deb --build ${PKG_NAME}_${PKG_VERSION}
echo "==> Finished build: ${PKG_NAME}_${PKG_VERSION}"
echo "PWD: $(pwd)"
ls -lh
ls -lh /app
find / -name "*.deb" 2>/dev/null