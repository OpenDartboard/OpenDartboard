# extract-deb.ps1 – Build Docker image with docker-compose, then extract .deb into ./dist

# 0. Run docker-compose build for your service
$service = "opendartboard"   # Your docker-compose service name
Write-Host "==> Running docker-compose build $service..."
docker-compose build $service

# 1. Ensure output folder exists
$dist = Join-Path $PSScriptRoot "dist"
if (-not (Test-Path $dist)) {
    New-Item -ItemType Directory -Path $dist | Out-Null
}

# 2. Get the image ID from docker-compose
$imageId = docker-compose images $service --quiet | Select-Object -First 1

if (-not $imageId) {
    Write-Host "Could not find image for $service. Build may have failed."
    exit 1
}

# 3. Create a container from the image (but don't run it)
$containerId = docker create $imageId

# 4. Find the .deb file inside /app (we assume only one .deb exists)
$debInContainer = docker container exec $containerId sh -c "ls /app/*.deb 2>/dev/null | head -n1"
if (-not $debInContainer) {
    Write-Host "No .deb file found in /app inside the container."
    docker rm $containerId | Out-Null
    exit 1
}

# 5. Copy it to ./dist
$debFilename = Split-Path $debInContainer -Leaf
docker cp "$($containerId):$debInContainer" "$dist\$debFilename"

# 6. Clean up
docker rm $containerId | Out-Null

if (Test-Path "$dist\$debFilename") {
    Write-Host "Success! $debFilename extracted to .\dist"
} else {
    Write-Host "Something went wrong copying $debFilename."
    exit 1
}