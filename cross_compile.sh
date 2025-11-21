#!/bin/bash
set -e

WORKSPACE_ROOT=$(pwd)
BUILD_DIR="$WORKSPACE_ROOT/build_isolated"
INSTALL_DIR="$WORKSPACE_ROOT/install_isolated"
DEVEL_DIR="$WORKSPACE_ROOT/devel_isolated"

echo "=== Cross-Compiling ROS Package ==="

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf "$BUILD_DIR" "$INSTALL_DIR" "$DEVEL_DIR"

# Cross-compile
echo "Starting cross-compilation..."
catkin_make_isolated -DCMAKE_TOOLCHAIN_FILE="$WORKSPACE_ROOT/Toolchain-V5L.cmake" --install

echo "=== Cross-compilation complete  ==="
