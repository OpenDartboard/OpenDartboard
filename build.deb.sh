#!/bin/bash
# build.deb.sh – Build Docker image with docker-compose, then extract .deb into ./dist

set -euo pipefail

service="opendartboard"   # Your docker-compose service name
dist="./dist"

echo "==> Running docker-compose build $service..."
docker-compose build "$service"

# Ensure output folder exists
mkdir -p "$dist"

# Get the image ID from docker-compose
imageId=$(docker-compose images "$service" --quiet | head -n1)
if [[ -z "$imageId" ]]; then
  echo "Could not find image for $service. Build may have failed."
  exit 1
fi

# Create a container from the image (but don't run it)
containerId=$(docker create "$imageId")

# Find the .deb file inside /app (we assume only one .deb exists)
debInContainer=$(docker container exec "$containerId" sh -c "ls /app/*.deb 2>/dev/null | head -n1")
if [[ -z "$debInContainer" ]]; then
  echo "No .deb file found in /app inside the container."
  docker rm "$containerId" >/dev/null
  exit 1
fi

debFilename=$(basename "$debInContainer")

# Copy it to ./dist
docker cp "$containerId:$debInContainer" "$dist/$debFilename"

# Clean up
docker rm "$containerId" >/dev/null

if [[ -f "$dist/$debFilename" ]]; then
  echo "Success! $debFilename extracted to ./dist"
else
  echo "Something went wrong copying $debFilename."
  exit 1
fi