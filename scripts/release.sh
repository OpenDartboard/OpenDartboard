#!/usr/bin/env bash
set -euo pipefail

# Step 1: Get latest tag (fallback to v0.1.0)
last_tag=$(git describe --tags --abbrev=0 2>/dev/null || echo "v0.1.0")

# Step 2: Bump version (patch bump only for now)
IFS='.' read -r major minor patch <<<"${last_tag#v}"
new_tag="v${major}.${minor}.$((patch + 1))"

# Step 3: Generate changelog from last tag
echo "==> Generating changelog from ${last_tag} to HEAD..."
changelog=$(git log "${last_tag}..HEAD" --pretty=format:"- %s" 2>/dev/null || git log --pretty=format:"- %s")

# Step 4: Tag with new version
git tag -a "$new_tag" -m "Release $new_tag"$'\n\n'"$changelog"

# Step 5: Push the tag (triggers GitHub Actions)
git push origin "$new_tag"

# Show summary
echo "Tagged $new_tag and pushed to origin"
echo "---------- CHANGELOG ----------"
echo "$changelog"


