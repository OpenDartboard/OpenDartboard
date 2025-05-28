#!/bin/bash
# build.deb.sh – Build Docker image, then extract .deb into ./dist

set -euo pipefail

dist="./dist"
image="opendartboard:latest"    # Set this to your image (see 'docker images')

echo "==> Building image: $image"
docker build -t "$image" . | tee docker-build.log

# Ensure output folder exists
mkdir -p "$dist"

# Create a container from the image (but don't run it)
containerId=$(docker create "$image")

# Find the .deb file inside /app (allow for any versioned name)
debInContainer=$(docker cp "$containerId:/app" - | tar -tvf - | grep '\.deb$' | head -n1 | awk '{print $NF}')
if [[ -z "$debInContainer" ]]; then
  echo "No .deb file found in /app inside the container."
  docker rm "$containerId" >/dev/null
  exit 1
fi

debFilename=$(basename "$debInContainer")

# Copy it to ./dist
echo "==> Copying $debInContainer out to $dist/$debFilename"
docker cp "$containerId:$debInContainer" "$dist/$debFilename"

# Clean up
docker rm "$containerId" >/dev/null

if [[ -f "$dist/$debFilename" ]]; then
  echo "Success! $debFilename extracted to ./dist"
else
  echo "Something went wrong copying $debFilename."
  exit 1
fi