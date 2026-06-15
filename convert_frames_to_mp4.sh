#!/bin/bash

VIDEO_DIR="$1"
FPS="${2:-15}"

for D in "$VIDEO_DIR"/*_frames; do
    [ -d "$D" ] || continue

    BASE=$(basename "$D" _frames)
    OUT="$VIDEO_DIR/$BASE.mp4"

    echo "Converting $D -> $OUT"

    rm -f "$OUT"

    ffmpeg -y \
      -framerate "$FPS" \
      -i "$D/frame_%04d.png" \
      -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2,format=yuv420p" \
      -c:v libx264 \
      -movflags +faststart \
      "$OUT"
done
