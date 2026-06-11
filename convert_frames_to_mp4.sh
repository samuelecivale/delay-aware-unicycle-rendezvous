#!/usr/bin/env bash
set -e

VIDEO_DIR="${1:-p1_outputs_matlab/videos}"
FPS="${2:-15}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg not found. Install it first."
  echo "macOS Homebrew: brew install ffmpeg"
  exit 1
fi

for folder in "$VIDEO_DIR"/*_frames; do
  [ -d "$folder" ] || continue
  base="$(basename "$folder" _frames)"
  out="$VIDEO_DIR/$base.mp4"
  echo "Converting $folder -> $out"
  ffmpeg -y -framerate "$FPS" -i "$folder/frame_%04d.png" -pix_fmt yuv420p "$out"
done

echo "Done."
