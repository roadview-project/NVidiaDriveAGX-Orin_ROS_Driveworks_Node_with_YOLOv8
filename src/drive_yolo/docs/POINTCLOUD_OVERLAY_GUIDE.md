# Pointcloud Camera Overlay Guide

This guide explains how to overlay LiDAR pointcloud data onto the camera view in RViz using the camera calibration parameters.

## Overview

The pointcloud overlay system uses:
- **Camera intrinsic parameters** from `lidar_camera_fusion.launch` (camera matrix and distortion coefficients)
- **Extrinsic calibration** (TF transformations) from `M8VIL_TFs.launch`
- **Camera info publisher** that publishes `sensor_msgs/CameraInfo` messages
- **RViz Camera display** to visualize the overlay

## Quick Start

### 1. Launch the System

```bash
roslaunch drive_yolo pointcloud_camera_overlay.launch
```

This will:
- Set up all TF transformations (car frame, camera, lidar, etc.)
- Start the camera_info publisher on `/camera/camera_info`

### 2. Launch RViz

```bash
rosrun rviz rviz -d $(rospack find drive_yolo)/rviz/pointcloud_overlay.rviz
```

Or manually open RViz and configure it (see Manual Configuration section below).

### 3. Start Your Camera and LiDAR

Make sure your camera and LiDAR data sources are running:
- Camera images should be published to `/camera/image_raw` (or configure accordingly)
- LiDAR pointcloud should be published to `/invz_summation_reflection` (or configure accordingly)

## Camera Calibration Parameters

The calibration parameters used are from `lidar_camera_fusion.launch`:

### Intrinsic Parameters
- **Camera Matrix (K)**:
  ```
  [3145.24132, 0.0,        1887.68485]
  [0.0,        3166.08502, 990.62304 ]
  [0.0,        0.0,        1.0       ]
  ```
  - fx = 3145.24132 (focal length in x)
  - fy = 3166.08502 (focal length in y)
  - cx = 1887.68485 (principal point x)
  - cy = 990.62304 (principal point y)

- **Distortion Coefficients**: `[-0.298671, 0.087807, 0.000383, 0.000551, 0.000000]`
  - Model: plumb_bob (standard OpenCV model)
  - [k1, k2, p1, p2, k3]

- **Image Size**: 3848 x 2168 pixels

### Extrinsic Parameters (from M8VIL_TFs.launch)

- **Camera (camera_entron) position relative to car_rear_axle**:
  - Translation: [1.706, -0.016, 0.752] meters
  - Rotation: [-1.55, 0.015, -1.45] radians (roll, pitch, yaw)

- **LiDAR position relative to car_rear_axle**:
  - Translation: [1.539, 0.006, 1.041] meters
  - Rotation: [0, 0, 0] radians

## Manual RViz Configuration

If you prefer to configure RViz manually:

### 1. Set Fixed Frame
- In RViz, set **Fixed Frame** to `camera_entron`

### 2. Add Camera Display
- Click "Add" → "Camera"
- Set **Image Topic** to `/camera/image_raw`
- Set **Image Rendering** to "background and overlay"
- Enable the display

### 3. Add PointCloud2 Display
- Click "Add" → "PointCloud2"
- Set **Topic** to `/invz_summation_reflection`
- Set **Style** to "Points" or "Spheres"
- Set **Size (m)** to 0.05 (adjust as needed)
- Set **Color Transformer** to "Intensity" or "AxisColor"
- Enable the display

### 4. Add TF Display (Optional)
- Click "Add" → "TF"
- This helps visualize the coordinate frame relationships
- Enable the display

## Architecture

### Components

1. **camera_info_publisher** (`drive_yolo_camera_info_publisher`)
   - Source: `src/camera_info_publisher.cpp`
   - Publishes `sensor_msgs/CameraInfo` messages
   - Topic: `/camera/camera_info`
   - Rate: 30 Hz (configurable)

2. **M8VIL_TFs.launch**
   - Publishes static TF transformations for all sensors
   - Defines the spatial relationships between:
     - Car rear axle (base reference frame)
     - Camera (camera_entron)
     - LiDAR (lidar)
     - Other sensors

3. **RViz Camera Display**
   - Uses camera_info to understand camera projection
   - Uses TF tree to transform pointcloud into camera frame
   - Renders pointcloud overlaid on camera image

### Data Flow

```
┌─────────────────┐
│  Camera Source  │ ──────────────┐
└─────────────────┘                │
                                   ▼
┌─────────────────┐         ┌──────────┐
│  LiDAR Source   │ ───────►│   RViz   │
└─────────────────┘         │  Camera  │
                            │ Display  │
┌─────────────────┐         │          │
│ camera_info_    │ ───────►│          │
│   publisher     │         └──────────┘
└─────────────────┘                │
                                   ▲
┌─────────────────┐                │
│ TF Tree (from   │ ───────────────┘
│ M8VIL_TFs.launch)│
└─────────────────┘
```

## Customization

### Change Camera Topic

Edit `pointcloud_camera_overlay.launch` or the RViz config to change the camera image topic.

### Change LiDAR Topic

Edit the RViz config or change the topic in RViz directly.

### Adjust Publishing Rate

Modify the `publish_rate` parameter in `pointcloud_camera_overlay.launch`:

```xml
<param name="publish_rate" value="10.0" />  <!-- Change to desired Hz -->
```

### Use Different Calibration

If you have different calibration parameters, update them in `pointcloud_camera_overlay.launch`:

```xml
<rosparam param="camera_matrix">[fx, 0, cx, 0, fy, cy, 0, 0, 1]</rosparam>
<rosparam param="distortion_coefficients">[k1, k2, p1, p2, k3]</rosparam>
<param name="image_width" value="your_width" />
<param name="image_height" value="your_height" />
```

## Troubleshooting

### Pointcloud Not Visible

1. **Check TF tree**: Run `rosrun tf view_frames` to verify all transforms exist
2. **Check topics**: Verify data is being published:
   ```bash
   rostopic hz /camera/camera_info
   rostopic hz /camera/image_raw
   rostopic hz /invz_summation_reflection
   ```
3. **Check frame IDs**: Ensure the pointcloud header.frame_id is in the TF tree

### Pointcloud Not Aligned

1. **Verify extrinsic calibration**: Check the TF transformations in `M8VIL_TFs.launch`
2. **Check camera calibration**: Verify the camera matrix and distortion coefficients
3. **Time synchronization**: Ensure timestamps are properly synchronized between camera and LiDAR

### Performance Issues

1. **Reduce pointcloud density**: Use a voxel grid filter
2. **Reduce publishing rate**: Lower the `publish_rate` parameter
3. **Reduce display size**: Change "Size (m)" in RViz PointCloud2 display

## References

- Camera calibration: `launch/lidar_camera_fusion.launch`
- Extrinsic calibration: `launch/M8VIL_TFs.launch`
- Camera info publisher: `src/camera_info_publisher.cpp`
- RViz config: `rviz/pointcloud_overlay.rviz`
