# Uncomment the above line if you are building for ARM architecture
# FROM arm32v7/debian:bullseye

# Use this line for x86_64 architecture (testing on x86_64 only and more readily available)
FROM debian:bullseye 

ENV WIDTH=640 \
  HEIGHT=480 \
  FPS=15 \
  DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  git build-essential cmake libopencv-dev libgstreamer1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-libav gettext-base v4l-utils sudo && \
  dpkg-query -W -f='${Package}=${Version}\n' git build-essential cmake libopencv-dev libgstreamer1.0-dev gstreamer1.0-plugins-base gstreamer1.0-libav gettext-base v4l-utils > /versions.txt && \
  cat /versions.txt && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

# Build step
RUN ./install.sh --force-rebuild && ./make.sh