# Use this line for building for ARM architecture (e.g zero 2)
FROM arm64v8/debian:bullseye

# Use this line for x86_64 architecture (testing on x86_64 only and more readily available)
# FROM debian:bullseye 

ENV WIDTH=640 \
  HEIGHT=480 \
  FPS=15 \
  DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  git \
  build-essential \ 
  cmake \
  libopencv-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-0 \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-libav \
  gettext-base \
  v4l-utils \
  sudo \
  # Grab versions for debugging
  && dpkg-query -W -f='${Package}=${Version}\n' \
  git \
  build-essential \ 
  cmake \
  libopencv-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-libav \
  gettext-base \
  v4l-utils \
  sudo \
  # output
  > /versions.txt && cat /versions.txt && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

# Install system deps, Python, git, etc. (this layer will be cached unless dependencies change) add --force-rebuild for a fresh rebuild
RUN ./install.sh

# Build everything (this only reruns if source changes or you touch build.sh)
RUN ./make.sh