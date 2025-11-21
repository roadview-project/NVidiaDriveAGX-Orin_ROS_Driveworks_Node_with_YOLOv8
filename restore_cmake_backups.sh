#!/bin/bash
# restore_cmake_backups.sh

SYSROOT="/home/poledna/nvidia_ws/target_fs"

echo "🔄 Restoring CMake config files from backups..."

find "$SYSROOT/opt/ros/noetic" -name "*.backup" | while read backup; do
    original="${backup%.backup}"
    mv "$backup" "$original"
    echo "Restored: ${original#$SYSROOT}"
done

echo "✅ Restoration complete!"
