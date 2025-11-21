# NVIDIA DRIVE AGX Orin ROS Integration with YOLO and LiDAR Fusion

A comprehensive ROS Noetic package for NVIDIA DRIVE AGX Orin that integrates YOLO object detection, LiDAR-camera fusion, and 3D object tracking with real-time visualization.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
  - [1. Install NVIDIA DRIVE OS Docker](#1-install-nvidia-drive-os-docker)
  - [2. Extract Target Filesystem](#2-extract-target-filesystem)
  - [3. Install ROS Noetic in Target Filesystem](#3-install-ros-noetic-in-target-filesystem)
  - [4. Cross-Compile ROS Packages](#4-cross-compile-ros-packages)
  - [5. Setup on Target Device](#5-setup-on-target-device)
- [Architecture](#architecture)
- [ROS Nodes](#ros-nodes)
- [Usage](#usage)
- [Launch Files](#launch-files)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project provides a complete perception pipeline for autonomous driving applications on NVIDIA DRIVE AGX Orin hardware. It combines:

- **YOLO Object Detection**: Real-time object detection using TensorRT-optimized YOLO models
- **LiDAR-Camera Fusion**: Sensor fusion to enrich 2D detections with accurate 3D distance information
- **3D Object Tracking**: Multi-object tracking with Kalman filtering and motion prediction
- **RViz Visualization**: Comprehensive visualization tools for debugging and analysis

## Features

### Object Detection
- TensorRT-accelerated YOLO inference (supports YOLOv8, YOLO11, etc.)
- DriveWorks camera integration for low-latency image acquisition
- Per-class confidence thresholds
- Hood filtering for vehicle-mounted cameras
- Quiet mode for production deployments

### LiDAR-Camera Fusion
- Camera-LiDAR calibration support
- Point cloud projection onto camera images
- Temporal synchronization diagnostics
- ROI filtering and ground plane removal
- Point clustering to filter noise and rain
- Distance confidence metrics based on LiDAR point density

### 3D Tracking
- Multi-object tracking with unique IDs
- Kalman filter-based motion prediction
- Velocity and acceleration estimation
- 1-second position prediction for path planning
- Track quality metrics (stability, confidence)
- Speed monitoring with configurable thresholds

### Visualization
- RViz integration with pre-configured views
- Velocity vector arrows
- Track ID labels with class and speed
- Position markers with class-based coloring
- Predicted future positions
- Pointcloud overlay on camera images

## System Requirements

### Hardware
- **NVIDIA DRIVE AGX Orin** (or compatible DRIVE platform)
- **LiDAR sensor** (tested with InnovizOne, supports any sensor with PointCloud2 output)
- **GMSL Camera** (tested with Entron F008A120RM0AV2)

### Software
- **NVIDIA DRIVE OS 6.0.6** or later
- **DriveWorks SDK 5.10** or later
- **CUDA 11.4** or later
- **TensorRT 8.x** or later
- **ROS Noetic** (cross-compiled for ARM64)
- **Ubuntu 20.04** (for cross-compilation host)

### Python Dependencies
```
motpy>=0.0.10
numpy>=1.19.0
scipy>=1.5.0
scikit-learn>=0.23.0
filterpy>=1.4.5
```

## Installation
Note as my objective ATM is just to crosscompile the system this doesnt need an NVIDIA GPU on host, im quite sure that also normally you wouldnt need it but just in case you were curious.

### 1. Install NVIDIA DRIVE OS Docker

Download and install NVIDIA DRIVE OS 6.0.6 and DriveWorks 5.10 from the [NVIDIA Developer portal](https://developer.nvidia.com/docs/drive/drive-os/6.0.6/public/drive-os-linux-installation/common/topics/installation/docker-ngc/download-install-docker-linux-devzone.html).
I am using the DRIVE OS 6.0.6 but the workflow should be compatible with 6.0.10. 
Perhaps 7 (to be tested by someone who has a Thor) I only have an Orin saddly.

Follow the NVIDIA installation guide to:
- Pull the DRIVE OS Docker container
```bash
mkdir nvidia_ws
export WORKSPACE=~/nvidia_ws
sudo docker run -it --privileged --name cross_compile --net=host -v /dev/bus/usb:/dev/bus/usb -v ${WORKSPACE}:/home/nvidia/ nvcr.io/drive/driveos-sdk/drive-agx-orin-linux-aarch64-sdk-build-x86:6.0.6.0-0004
```
Just be aware this is one line not 2!
- Extract DriveWorks SDK
Essentially inside the docker theres is two OS, the one that will be installed (not really but as a method to convey info it is), and the one that compiles and flahes it
The docker has literally everything you need.
The thing is, well, depending on how you want to work the data is in an annoying place.
So Logically this work now becomes a somewhat redundant work about re doing what the really cool guys at NVidia did, but in a way i understand and works... the issue inside the docker is to do some chroots and overlays, because the system is overlayed in an overlay doenst work really nicely, so what i do next is to remove the system from the system.
Inside the docker with a lot of snooping around you will find a folder with the targetfs. we want that.
```bash
docker cp cross_compile:/drive/drive-linux/filesystem/targetfs nvidia_ws/target_fs/
```
OK! now we have the crosscompilation environment at the nvidia_ws/target_fs folder!
Now we need to install inside this work inside this target_fs, Use `chroot` to install ROS Noetic inside the target filesystem. You will need `aptitude` for proper dependency resolution:
```bash
# Enter the target filesystem
export SYSROOT=~/nvidia_ws/target_fs
cd ${SYSROOT}

# Mount necessary filesystems
sudo mount --bind /dev ${SYSROOT}/dev
sudo mount --bind /proc ${SYSROOT}/proc
sudo mount --bind /sys ${SYSROOT}/sys

# Enter chroot environment
sudo chroot ${SYSROOT} /bin/bash
```
ok this is now kinda wild, because if you are inside the container you may feel like youre outside (?) 
but if you did correctly you should be super user in the / of the folder
NOTE: the chroot has no well DNS nameservers youll need to add a DNS in the /etc/resolv.conf (add 'nameserver 1.1.1.1' if in doubt, if cloudfare decided to die try google or any other ofc)

```bash
apt update
apt install -y aptitude software-properties-common
add-apt-repository universe
apt upgrade
```
Ok basic stuff, before we continue we need to install some libraries with aptitude.
the thing is you need to deny the solution a couple times

first thing
```bash
aptitude install libboost-all-dev    
```
you want the solution that does this
```bash
The following actions will resolve these dependencies:

     Downgrade the following packages:
1)     ibverbs-providers [58mlnx43-1.58101 (now) -> 28.0-1ubuntu1 (focal)]
2)     libibverbs1 [58mlnx43-1.58101 (now) -> 28.0-1ubuntu1 (focal)]
3)     librdmacm1 [58mlnx43-1.58101 (now) -> 28.0-1ubuntu1 (focal)]
4)     perftest [4.5-0.18.gfcddfe0.58101 (now) -> 4.4+0.5-1 (focal)]
```
OK that was the "hard" part of the moment!

now we need to install ros noetic as usual
```bash

# Add ROS Noetic repository
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic using aptitude
apt update
apt install ros-noetic-desktop-full
```
you definetly dont need ALL these package but well its easy also notice how the apt is pulling from the arm repo^^
```bash
# Install build dependencies
aptitude install -y build-essential cmake python3-rosdep python3-rosinstall \
  python3-rosinstall-generator python3-wstool \
  libboost-all-dev libtinyxml-dev libtinyxml2-dev liblz4-dev libbz2-dev \
  libapr1 libaprutil1 libconsole-bridge-dev libpoco-dev libgpgme-dev \
  python3-defusedxml python3-rospkg python3-catkin-pkg python3-netifaces \
  liblog4cxx-dev libopencv-dev

# Exit chroot
exit

# Unmount filesystems
sudo umount ${SYSROOT}/dev
sudo umount ${SYSROOT}/proc
sudo umount ${SYSROOT}/sys
```
as a suggestion as you didmake long impactful changes make a copy from the target_fs folder and save it for a minute until everything is ready just for safety

### 4. Cross-Compile ROS Packages

#### Setup Cross-Compilation Toolchain

```bash
# Install ARM64 cross-compiler on host
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Clone this repository
cd ~/nvidia_ws
git clone git@github.com:roadview-project/NVidiaDriveAGX-Orin_ROS_Driveworks_Node_with_YOLOv8.git DRIVE_AGX_ORIN_ONNX_ROS_integration
cd DRIVE_AGX_ORIN_ONNX_ROS_integration
```

#### Cross-Compile
OK we are now ready to try and break stuff!!! 
QUITE Literally most of the symlinks were broken when we copied we need to fix it so do
```bash
python3 fix_symlinks.py

```
you have to source THE RIGHT ROS.... think like a bootstrap problem. soooo 

```bash
source $SYSROOT/opt/ros/noetic/setup.bash
```
but ofc this itself would be just too easy... so you also need to change the line 96 from $SYSROOT/target_fs/opt/ros/noetic/share/tf2_geometry_msgs/cmake/tf2_geometry_msgsConfig.cmake
to be this:
```cmake
set(_include_dirs "include;${CMAKE_SYSROOT}/usr/share/orocos_kdl/cmake/../../../include;${CMAKE_SYSROOT}/usr/include/eigen3")
```
if this ROS1 wasnt EOL id complain but thats life ig...

ok so now you must be wondering why the hell did we do this like this
well because we need even more libraries but now from the docker, but from outside the QEMU!

basically, $SYSROOT is missing the tensorRT libs that can be found in /usr/include/aarch64-linux-gnu/ on the docker so we need them!
lets copy them!

```bash
docker cp cross_compile:/usr/include/aarch64-linux-gnu/ $SYSROOT/usr/include/
docker cp cross_compile:/usr/lib/aarch64-linux-gnu/ $SYSROOT/usr/lib/
docker cp cross_compile:/usr/local/driveworks/targets^Carch64-Linux $SYSROOT/usr/local/driveworks/targets/ #
docker cp cross_compile:/drive/drive-linux/include/. $SYSROOT/usr/include/
docker cp cross_compile:/drive/drive-linux/include/nvmedia_6x $SYSROOT/usr/include
docker cp cross_compile:/drive/drive-linux/include/nvmedia_6x  $SYSROOT/usr/include/

docker cp cross_compile:/drive/drive-linux/include/nvmedia_dla.h $SYSROOT/usr/include/
docker cp cross_compile:/drive/drive-linux/include/nvmedia_tensor.h $SYSROOT/usr/include/
docker cp cross_compile:/drive/drive-linux/include/nvmedia_tensormetadata.h $SYSROOT/usr/include/


#we also need to make some symlinks so
cd  ~/nvidia_ws/target_fs/usr/local/driveworks
ln -s targets/aarch64-Linux/include/ .
ln -s targets/aarch64-Linux/lib/ .
cd $SYSROOT/usr/include
ln -sf nvmedia_6x/nvmedia_core.h nvmedia_image.h

```
i probably would have a smarter way to do this but this works, the thing is bc we installed ros inside the qemu hes looking for the boost files in the wrong place, even though i changed that in the CMAKE (see in drive_yolo CMAKE) so this is the workaround bc im tired bossman
```bash
sudo cp -r $SYSROOT/usr/lib/aarch64-linux-gnu/ /usr/lib/
sudo ln -s $SYSROOT/usr/include/opencv4 /usr/include/opencv4 # OMG CROSSCOMPILATIOn AHHHH
```
because i was very smart and added some python package to the whole package you need now to run (~Smort++~)
```bash

pip3 install -r  ~/nvidia_ws/DRIVE_AGX_ORIN_ONNX_ROS_integration/src/drive_yolo/requirements.txt
```

FINALLY this should work now

```bash
./cross_compile

```

### 5. Setup on Target Device

#### Copy Files to Target

```bash
# Copy built packages to target
scp -r ~/drive-ros/catkin_ws/install_isolated nvidia@<target-ip>:~

# Copy YOLO models (prepare models separately using TensorRT)
scp models/*.engine nvidia@<target-ip>:~/models/
```

#### Install Dependencies on Target

```bash
# SSH into target
ssh nvidia@<target-ip>

# Install ROS packages
sudo apt update
sudo apt install -y ros-noetic-ros-base ros-noetic-cv-bridge \
  ros-noetic-image-view ros-noetic-rviz

# Install Python dependencies for 3D tracking
pip3 install motpy numpy scipy scikit-learn filterpy

# Install additional libraries
sudo apt install -y libboost-all-dev libtinyxml-dev libtinyxml2-dev \
  liblz4-dev libbz2-dev libapr1 libaprutil1 libconsole-bridge-dev \
  libpoco-dev liblog4cxx-dev

# Remove conflicting library (if needed - see NVIDIA forums)
sudo rm /usr/lib/libxerces-c*.so || true

# Setup ROS environment
echo "source ~/install_isolated/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Data Sources                             │
├────────────────────┬────────────────────────────────────────────┤
│ DriveWorks Camera  │  LiDAR (PointCloud2)                       │
│  (GMSL/CSI)        │                                             │
└─────────┬──────────┴────────────────┬───────────────────────────┘
          │                           │
          v                           v
┌─────────────────────┐    ┌─────────────────────────────────────┐
│  YOLO Detection     │    │  Camera-LiDAR Calibration           │
│  Node               │    │  (camera_info_publisher)            │
│  (drive_yolo_node)  │    └─────────────────────────────────────┘
└─────────┬───────────┘                           │
          │                                       │
          │ /detections                           │ /camera_info
          v                                       v
┌─────────────────────────────────────────────────────────────────┐
│                    LiDAR-Camera Fusion                          │
│              (lidar_camera_fusion_node)                         │
│  - Projects LiDAR points onto image                             │
│  - Associates points with detections                            │
│  - Computes 3D distances                                        │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      │ /detections_with_distance
                      v
┌─────────────────────────────────────────────────────────────────┐
│                    3D Object Tracking                           │
│                 (tracking_3d_node.py)                           │
│  - Kalman filtering                                             │
│  - Data association                                             │
│  - Motion prediction                                            │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      │ /tracked_objects_3d
                      v
┌─────────────────────────────────────────────────────────────────┐
│                      Visualization                              │
│         (tracking_rviz_visualizer.py + RViz)                    │
│  - Velocity arrows                                              │
│  - Track IDs and labels                                         │
│  - Position markers                                             │
│  - Predicted positions                                          │
└─────────────────────────────────────────────────────────────────┘
```

## ROS Nodes

### Core Nodes

#### 1. `drive_yolo_node` (C++)
**Package**: `drive_yolo`
**Executable**: `drive_yolo_node_node`
**Source**: `drive_yolo/src/yolo_node.cpp`

YOLO object detection node with TensorRT acceleration.

**Subscribed Topics**:
- `/camera/image_input` (sensor_msgs/Image) - Camera images when not using DriveWorks

**Published Topics**:
- `/detections` (drive_yolo/Detections) - Detected objects with bounding boxes
- `/camera/image_raw` (sensor_msgs/Image) - Raw camera images

**Parameters**:
- `model_path` (string): Path to TensorRT engine file
- `use_driveworks_camera` (bool): Enable DriveWorks camera interface
- `camera_params` (string): DriveWorks camera parameters (e.g., "camera-name=...,interface=csi-ab,link=1")
- `confidence_threshold` (double, default: 0.5): Publishing threshold
- `detection_threshold` (double, default: 0.5): Detection threshold
- `vehicle_confidence_threshold` (double): Vehicle-specific threshold
- `pedestrian_confidence_threshold` (double): Pedestrian-specific threshold
- `ignore_hood_pixels` (int, default: 400): Pixels to ignore from bottom
- `enable_hood_filtering` (bool, default: true): Enable hood filtering
- `quiet_mode` (bool, default: false): Reduce log output

**Example**:
```bash
rosrun drive_yolo drive_yolo_node_node \
  _model_path:=/models/yolo11n.engine \
  _use_driveworks_camera:=true \
  _camera_params:="camera-name=F008A120RM0AV2,interface=csi-ab,link=1,output-format=processed"
```

#### 2. `lidar_camera_fusion_node` (C++)
**Package**: `drive_yolo`
**Executable**: `drive_yolo_lidar_fusion_node`
**Source**: `drive_yolo/src/lidar_camera_fusion_node.cpp`

Fuses LiDAR point clouds with camera detections.

**Subscribed Topics**:
- `/detections` (drive_yolo/Detections) - 2D detections from YOLO
- `/lidar/points` (sensor_msgs/PointCloud2) - LiDAR point cloud
- `/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

**Published Topics**:
- `/detections_with_distance` (drive_yolo/DetectionsWithDistance) - Detections enriched with 3D distance
- `/lidar_camera_fusion/debug_cloud` (sensor_msgs/PointCloud2) - Debug visualization

**Parameters**:
- `camera_matrix` (array[9]): 3x3 camera intrinsic matrix
- `distortion_coefficients` (array[5]): Distortion coefficients [k1, k2, p1, p2, k3]
- `image_width` (int): Image width in pixels
- `image_height` (int): Image height in pixels
- `use_tf_transforms` (bool, default: true): Use TF for coordinate transforms
- `lidar_frame` (string): LiDAR frame ID
- `camera_frame` (string): Camera frame ID
- `base_frame` (string): Base/vehicle frame ID
- `max_distance` (double, default: 100.0): Maximum LiDAR range (meters)
- `min_distance` (double, default: 0.5): Minimum LiDAR range (meters)
- `sync_time_slop` (double, default: 0.1): Time tolerance for synchronization (seconds)
- `sync_queue_size` (int, default: 20): Message buffer size
- `time_offset_ms` (double, default: 0.0): Manual time offset between sensors (milliseconds)
- `enable_ground_filtering` (bool, default: false): Remove ground points
- `enable_point_clustering` (bool, default: true): Filter noise with clustering
- `min_points_for_object` (int, default: 3): Minimum points to consider valid object
- `enable_roi_filtering` (bool, default: false): Enable region of interest filtering

**Example**:
```bash
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch
```

#### 3. `tracking_3d_node` (Python)
**Package**: `drive_yolo`
**Executable**: `tracking_3d_node.py`
**Source**: `drive_yolo/scripts/tracking_3d_node.py`

Multi-object tracker with Kalman filtering and motion prediction.

**Subscribed Topics**:
- `/detections_with_distance` (drive_yolo/DetectionsWithDistance) - 3D detections from fusion

**Published Topics**:
- `/tracked_objects_3d` (drive_yolo/TrackedObjects3D) - Tracked objects with IDs and motion

**Parameters**:
- `distance_threshold_3d` (double, default: 2.0): Association distance threshold (meters)
- `max_missed_frames` (int, default: 8): Frames before track deletion
- `min_detections_for_tracking` (int, default: 3): Detections required to publish track
- `min_lidar_points` (int, default: 5): Minimum LiDAR points for valid track
- `min_speed_threshold` (double, default: 0.1): Speed threshold for "moving" classification (m/s)
- `max_tracking_distance` (double, default: 30.0): Maximum tracking range (meters)
- `publish_all_tracks` (bool, default: true): Publish all tracks vs. high-quality only
- `publish_high_quality_only` (bool, default: false): Only publish stable tracks
- `quiet_mode` (bool, default: false): Reduce log output

**Example**:
```bash
rosrun drive_yolo tracking_3d_node.py \
  _distance_threshold_3d:=2.0 \
  _max_missed_frames:=8
```

#### 4. `tracking_rviz_visualizer` (Python)
**Package**: `drive_yolo`
**Executable**: `tracking_rviz_visualizer.py`
**Source**: `drive_yolo/scripts/tracking_rviz_visualizer.py`

RViz visualization node for tracked objects.

**Subscribed Topics**:
- `/tracked_objects_3d` (drive_yolo/TrackedObjects3D) - Tracked objects

**Published Topics**:
- `/tracking/velocity_markers` (visualization_msgs/MarkerArray) - Velocity arrows
- `/tracking/position_markers` (visualization_msgs/MarkerArray) - Position markers
- `/tracking/label_markers` (visualization_msgs/MarkerArray) - Track ID labels
- `/tracking/prediction_markers` (visualization_msgs/MarkerArray) - Predicted positions

**Parameters**:
- `show_velocity_vectors` (bool, default: true): Display velocity arrows
- `show_track_ids` (bool, default: true): Display track ID labels
- `show_positions` (bool, default: true): Display position markers
- `show_predicted_positions` (bool, default: true): Display 1-second predictions
- `velocity_scale` (double, default: 2.0): Arrow size scale factor
- `min_velocity_display` (double, default: 0.5): Minimum speed to display arrow (m/s)
- `frame_id` (string, default: "camera"): Reference frame
- `use_class_colors` (bool, default: true): Use class-specific colors

### Color Scheme

When `use_class_colors` is enabled:
- **Car**: Cyan
- **Truck**: Orange
- **Person**: Red
- **Bicycle**: Yellow
- **Motorcycle**: Magenta
- **Bus**: Purple

**Example**:
```bash
roslaunch drive_yolo tracking_with_rviz.launch launch_rviz:=true
```

### Utility Nodes

#### 5. `camera_info_publisher` (C++)
Publishes camera calibration parameters for RViz pointcloud overlay.

**Published Topics**:
- `/camera_info` (sensor_msgs/CameraInfo)

**Parameters**:
- `camera_matrix` (array[9]): Camera intrinsics
- `distortion_coefficients` (array[5]): Distortion parameters
- `image_width`, `image_height` (int): Image dimensions
- `frame_id` (string): Camera frame ID
- `publish_rate` (double, default: 30.0): Publishing frequency (Hz)

#### 6. `sync_diagnostics_node` (C++)
Diagnoses timing synchronization between camera and LiDAR.

**Subscribed Topics**:
- `/detections` (drive_yolo/Detections)
- `/lidar/points` (sensor_msgs/PointCloud2)

**Output**: Console statistics showing time offsets and synchronization quality.

#### 7. `pointcloud_delay_node` (C++)
Adds configurable delay to pointcloud for testing synchronization.

**Parameters**:
- `delay_ms` (int, default: 0): Delay in milliseconds

#### 8. `speed_monitor_node` (C++)
Monitors object speeds and triggers warnings.

**Subscribed Topics**:
- `/tracked_objects_3d` (drive_yolo/TrackedObjects3D)

**Parameters**:
- `speed_limit_ms` (double): Speed limit (m/s)
- `warn_threshold_ms` (double): Warning threshold (m/s)
- `monitor_classes` (string): Comma-separated class IDs to monitor

## Usage

### Quick Start

#### 1. Basic YOLO Detection with DriveWorks Camera

```bash
# Start roscore
roscore &

# Launch YOLO detection
roslaunch drive_yolo yolo_driveworks.launch \
  model_path:=/models/yolo11n.engine \
  camera_params:="camera-name=F008A120RM0AV2,interface=csi-ab,link=1,output-format=processed"
```

#### 2. Complete Detection + Fusion + Tracking Pipeline

```bash
# Launch full pipeline
roslaunch drive_yolo complete_detection_pipeline.launch \
  model_path:=/models/yolo11n.engine \
  use_driveworks:=true
```

#### 3. 3D Tracking with RViz Visualization

```bash
# Launch tracking with automatic RViz
roslaunch drive_yolo tracking_with_rviz.launch launch_rviz:=true

# Or manually launch RViz with pre-configured view
rviz -d $(rospack find drive_yolo)/rviz/tracking_visualization.rviz
```

#### 4. Pointcloud Overlay in RViz

```bash
# Publish camera info and visualize pointcloud overlay
roslaunch drive_yolo pointcloud_camera_overlay.launch
```

### Advanced Usage

#### Synchronization Diagnostics

If you notice fusion issues:

```bash
# 1. Run diagnostics to measure timing offset
roslaunch drive_yolo sync_diagnostics.launch

# 2. Update time_offset_ms in lidar_camera_fusion_with_sync.launch
# 3. Re-launch fusion with corrected offset
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch
```

#### Custom TF Frames

Set up TF transforms for your sensor configuration:

```bash
# Example: Publish static transforms
roslaunch drive_yolo M8VIL_TFs.launch

# Or define your own in a launch file:
<node pkg="tf2_ros" type="static_transform_publisher" name="lidar_to_base"
      args="0 0 1.383 0 0 0 car_rear_axle lidar" />
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_to_base"
      args="1.5 0 1.2 0 0.1 0 car_rear_axle camera_entron" />
```

## Launch Files

### Main Pipelines

| Launch File | Description |
|------------|-------------|
| `complete_system.launch` | YOLO detection + 2D tracking |
| `complete_detection_pipeline.launch` | Full pipeline: YOLO + LiDAR fusion + 3D tracking |
| `LidarandCamerapipeline.launch` | LiDAR-camera fusion pipeline |

### Individual Components

| Launch File | Description |
|------------|-------------|
| `yolo_driveworks.launch` | YOLO detection only (DriveWorks camera) |
| `yolo_ros_camera.launch` | YOLO detection with ROS camera input |
| `lidar_camera_fusion.launch` | LiDAR-camera fusion node |
| `lidar_camera_fusion_with_sync.launch` | Fusion with synchronization parameters |
| `motpy_tracking.launch` | Python-based 3D tracking |
| `3d_tracking_pipeline.launch` | C++ 3D tracking pipeline |

### Visualization

| Launch File | Description |
|------------|-------------|
| `tracking_with_rviz.launch` | 3D tracking with RViz visualization |
| `tracking_visualizer_only.launch` | Visualizer node only |
| `detection_visualizer.launch` | 2D detection visualization |
| `pointcloud_camera_overlay.launch` | Pointcloud overlay in RViz |

### Diagnostics

| Launch File | Description |
|------------|-------------|
| `sync_diagnostics.launch` | Camera-LiDAR synchronization diagnostics |
| `pointcloud_delay_demo.launch` | Pointcloud delay testing |

### Utilities

| Launch File | Description |
|------------|-------------|
| `M8VIL_TFs.launch` | TF transforms for M8VIL vehicle configuration |
| `osi2ros_lidar.launch` | OSI to ROS LiDAR converter |

## Troubleshooting

### Common Issues

#### 1. **DriveWorks camera fails to initialize**

**Symptoms**: `DriveWorks camera init failed` error

**Solutions**:
- Verify camera is connected: `v4l2-ctl --list-devices`
- Check camera parameters match your hardware
- Ensure DriveWorks SDK is properly installed
- Try different output formats: `output-format=processed` or `output-format=raw`

#### 2. **No LiDAR points in fusion**

**Symptoms**: All detections show distance = 0

**Solutions**:
- Check LiDAR topic is publishing: `rostopic echo /lidar/points`
- Verify TF frames are correct: `rosrun tf tf_echo camera_frame lidar_frame`
- Run sync diagnostics: `roslaunch drive_yolo sync_diagnostics.launch`
- Adjust `sync_time_slop` parameter (increase if messages are being dropped)
- Check `time_offset_ms` parameter

#### 3. **Tracking IDs keep changing**

**Symptoms**: Objects get new track IDs frequently

**Solutions**:
- Increase `max_missed_frames` parameter
- Decrease `distance_threshold_3d` for tighter association
- Increase `min_detections_for_tracking` for more stable tracks
- Check detection consistency (YOLO confidence thresholds)

#### 4. **RViz markers don't appear**

**Symptoms**: No visualization in RViz

**Solutions**:
- Verify Fixed Frame matches `frame_id` parameter (default: "camera")
- Check topics are publishing: `rostopic list | grep tracking`
- Ensure MarkerArray displays are added in RViz
- Load pre-configured RViz file: `rviz -d $(rospack find drive_yolo)/rviz/tracking_visualization.rviz`

#### 5. **Cross-compilation errors**

**Symptoms**: CMake or linker errors during build

**Solutions**:
- Update paths in toolchain files to match your installation
- Verify DriveWorks libraries exist: `ls $DRIVEWORKS_ROOT/targets/aarch64-Linux/lib`
- Check CUDA version matches (11.4 in default config)
- Ensure sysroot contains ROS installation

#### 6. **Python dependencies missing on target**

**Symptoms**: `ImportError: No module named motpy`

**Solutions**:
```bash
pip3 install motpy numpy scipy scikit-learn filterpy
```

### Debugging Tools

```bash
# View all topics
rostopic list

# Monitor specific topic
rostopic echo /tracked_objects_3d

# Check message rate
rostopic hz /detections

# View TF tree
rosrun rqt_tf_tree rqt_tf_tree

# Record data for offline analysis
rosbag record /detections /lidar/points /tracked_objects_3d

# Play back recorded data
rosbag play your_recording.bag
```

## Project Structure

```
.
├── drive_yolo/              # Main package
│   ├── src/                 # C++ source files
│   │   ├── yolo_node.cpp
│   │   ├── yolo_infer.cpp
│   │   ├── lidar_camera_fusion_node.cpp
│   │   ├── tracking_3d_node.cpp
│   │   └── ...
│   ├── scripts/             # Python nodes
│   │   ├── tracking_3d_node.py
│   │   ├── tracking_rviz_visualizer.py
│   │   └── ...
│   ├── include/             # Header files
│   ├── launch/              # Launch files
│   ├── msg/                 # Custom message definitions
│   ├── rviz/                # RViz configuration files
│   ├── docs/                # Documentation
│   ├── models/              # YOLO model files (user-provided)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── Toolchain-V5L.cmake  # Cross-compilation toolchain
├── nv_sensors/              # NVIDIA sensor drivers
│   ├── src/
│   ├── include/
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md
```

## Message Definitions

### Detection.msg
```
int32 class_id
string class_name
float32 confidence
float32 x1
float32 y1
float32 x2
float32 y2
```

### DetectionWithDistance.msg
```
Detection detection
float32 distance_m
float32 distance_confidence
int32 lidar_points_count
geometry_msgs/Point position_3d
```

### TrackedObject3D.msg
```
int32 id
int32 class_id
string class_name
geometry_msgs/Point position_3d
geometry_msgs/Vector3 velocity_3d
geometry_msgs/Vector3 acceleration_3d
float32 speed_3d
float32 confidence
float32 distance_confidence
int32 lidar_points_count
float32 track_stability
float32 track_duration
int32 total_detections
bool is_moving
geometry_msgs/Point predicted_position_1s
```

## Known Limitations

- DriveWorks camera interface is DRIVE AGX-specific
- Cross-compilation requires NVIDIA DRIVE OS SDK
- LiDAR fusion assumes calibrated sensors with known TF transforms
- 3D tracking works best with consistent detections (>10 Hz)

## Future Enhancements

- [ ] Radar sensor fusion
- [ ] ROS 2 migration
- [ ] Docker containerization
- [ ] Pre-built ARM64 binaries
- [ ] Automated calibration tools
- [ ] Real-time performance benchmarking
- [ ] Integration with path planning frameworks

## References

- [NVIDIA DriveWorks Documentation](https://developer.nvidia.com/drive/driveworks)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)

## Acknowledgments

- NVIDIA Corporation for DriveWorks SDK
- Open Robotics for ROS framework
- Ultralytics for YOLO models



