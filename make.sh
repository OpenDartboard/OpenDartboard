#!/usr/bin/env bash
# make.sh – OpenDartboard 3-cam scorer (Pi Zero 2 W)

export PKG_VERSION=0.11.0
export PKG_NAME=opendartboard
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-15}

# output config and build info
echo "==> OpenDartboard build script v${PKG_VERSION}"
echo "==> Config: [width: ${WIDTH}, height: ${HEIGHT}, fps: ${FPS}]"

# setup the build environment
mkdir -p ${PKG_NAME}_${PKG_VERSION}/usr/local/{bin,share/opendartboard/models}
cp /usr/local/bin/opendartboard   ${PKG_NAME}_${PKG_VERSION}/usr/local/bin/
cp /usr/local/share/opendartboard/models/*.param ${PKG_NAME}_${PKG_VERSION}/usr/local/share/opendartboard/models/ 2>/dev/null || true
cp /usr/local/share/opendartboard/models/*.bin   ${PKG_NAME}_${PKG_VERSION}/usr/local/share/opendartboard/models/ 2>/dev/null || true

# Minimal DEBIAN/control file
mkdir -p ${PKG_NAME}_${PKG_VERSION}/DEBIAN
envsubst < distributions/debian_arm64/control > ${PKG_NAME}_${PKG_VERSION}/DEBIAN/control
envsubst < distributions/debian_arm64/postinst > ${PKG_NAME}_${PKG_VERSION}/DEBIAN/postinst
chmod +x ${PKG_NAME}_${PKG_VERSION}/DEBIAN/postinst
envsubst < distributions/debian_arm64/postrm > ${PKG_NAME}_${PKG_VERSION}/DEBIAN/postrm
chmod +x ${PKG_NAME}_${PKG_VERSION}/DEBIAN/postrm

# Generate service files from templates
envsubst < templates/opendartboard.service.template > opendartboard.service
envsubst < templates/lock_cams.service.template > lock_cams.service

install -Dm755 scripts/lock_cams.sh ${PKG_NAME}_${PKG_VERSION}/usr/local/bin/lock_cams.sh

# (Optional) systemd unit in /lib/systemd/system
install -Dm644 opendartboard.service \
     ${PKG_NAME}_${PKG_VERSION}/lib/systemd/system/opendartboard.service
install -Dm644 lock_cams.service \
     ${PKG_NAME}_${PKG_VERSION}/lib/systemd/system/lock_cams.service

# 4. Build the .deb
rm -f /app/*.deb
dpkg-deb --build ${PKG_NAME}_${PKG_VERSION}
echo "==> Finished build: ${PKG_NAME}_${PKG_VERSION}"