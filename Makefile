.PHONY: build run deb release

PROJECT_VERSION_VAL := $(if $(VERSION),$(VERSION),0.0.0-dev)
CMAKE_FLAGS = -DCMAKE_PREFIX_PATH=/usr/local \
              -DCMAKE_CXX_FLAGS="-DDEBUG_SEEK_VIDEO -DDEBUG_VIA_VIDEO_INPUT" \
              -DAPP_VERSION=$(PROJECT_VERSION_VAL)

build:
	mkdir -p build
	cmake -S . -B build $(CMAKE_FLAGS)
	cmake --build build -- -j4 --no-print-directory
	@cp build/opendartboard /usr/local/bin/opendartboard
	@echo "\033[32mBuild completed successfully!\033[0m"

run:
	opendartboard --debug \
		--cams mocks/cam_1.mp4,mocks/cam_2.mp4,mocks/cam_3.mp4 \
		--width 1280 --height 720

deb:
ifndef VERSION
	$(error VERSION is not set, usage: make deb VERSION=X.X.X)
endif
	rm -rf dist/staging
	mkdir -p dist/staging/opendartboard_$(VERSION)/DEBIAN
	mkdir -p dist/staging/opendartboard_$(VERSION)/usr/local/bin

	# Use envsubst to inject version/name into the control file
	env PKG_NAME=opendartboard PKG_VERSION=$(VERSION) \
		envsubst < distributions/debian_arm64/control \
		> dist/staging/opendartboard_$(VERSION)/DEBIAN/control

	# Copy binary into correct path
	cp /usr/local/bin/opendartboard dist/staging/opendartboard_$(VERSION)/usr/local/bin/

	# Build the .deb
	dpkg-deb --build dist/staging/opendartboard_$(VERSION) dist/

	@echo "\033[32mBuilt .deb package: dist/opendartboard_$(VERSION).deb\033[0m"


release:
    # just run the ./scripts/release.sh script
	@echo "\033[32mRunning release script...\033[0m"
	@./scripts/release.sh