# Use this line for building for ARM architecture (e.g zero 2)
FROM arm64v8/debian:bullseye

# Use this line for x86_64 architecture (testing on x86_64 only and more readily available)
# FROM debian:bullseye 

ENV WIDTH=640 \
  HEIGHT=360 \
  FPS=15 \
  DEBIAN_FRONTEND=noninteractive

COPY apt-packages.txt /tmp/
RUN apt-get update -y && xargs apt-get install -y < /tmp/apt-packages.txt

WORKDIR /app
COPY . .

# Install system deps, Python, git, etc. (this layer will be cached unless dependencies change) add --force-rebuild for a fresh rebuild
# RUN ./install.sh

# Build everything (this only reruns if source changes or you touch build.sh)
# RUN ./make.sh