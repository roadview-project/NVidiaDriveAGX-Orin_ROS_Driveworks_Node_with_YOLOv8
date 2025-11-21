# Camera-LiDAR Synchronization Guide

This guide explains how to diagnose and fix synchronization issues between camera detections and LiDAR pointcloud data in the fusion system.

## Problem Description

In simulation or real-world scenarios, camera and LiDAR sensors may have different latencies, causing timestamp mismatches. This can lead to:
- **Poor association**: Objects detected by camera don't match LiDAR points
- **Dropped messages**: Synchronizer discards messages that don't match in time
- **Inaccurate distance estimation**: Points associated with wrong detections

## Solution Overview

The fusion system now provides:
1. **Diagnostic tool** to measure timing offsets
2. **Configurable synchronization** parameters
3. **Debug logging** to monitor timing in real-time

## Step 1: Measure the Timing Offset

First, run the synchronization diagnostics node to measure the actual delay:

```bash
roslaunch drive_yolo sync_diagnostics.launch
```

This will monitor both camera (`/detections`) and LiDAR (`/invz_summation_reflection`) topics and report statistics every 2 seconds.

### Understanding the Output

```
=== SYNC DIAGNOSTICS REPORT ===
Messages received: Camera=150, LiDAR=150
Queue size: 50/50 samples
Camera-LiDAR timestamp offset (positive = camera ahead):
  Mean:   +45.3 ms
  Median: +44.8 ms
  StdDev:   5.2 ms
  Min:    +35.1 ms
  Max:    +58.7 ms

Recommendation: Set time_offset_ms = 45.3 in fusion config
Or use frame_offset = 1 (at 30 Hz)
```

**Interpretation:**
- **Positive offset**: Camera timestamps are ahead of LiDAR (camera has more delay)
- **Negative offset**: LiDAR timestamps are ahead of camera (LiDAR has more delay)
- **Low StdDev (<10ms)**: Consistent timing, good synchronization possible
- **High StdDev (>20ms)**: Inconsistent timing, may need larger time_slop

### Recent Samples

The diagnostics also show recent individual samples:
```
Recent samples (last 5):
  [45] Camera: 1234.567, LiDAR: 1234.522, Delay: +45.0 ms
  [46] Camera: 1234.600, LiDAR: 1234.555, Delay: +45.0 ms
  [47] Camera: 1234.633, LiDAR: 1234.588, Delay: +45.0 ms
  [48] Camera: 1234.667, LiDAR: 1234.622, Delay: +45.0 ms
  [49] Camera: 1234.700, LiDAR: 1234.655, Delay: +45.0 ms
```

## Step 2: Configure Fusion Parameters

Based on the diagnostic output, update your fusion configuration. You have two options:

### Option A: Update Existing Launch File

Edit `launch/lidar_camera_fusion.launch` and add:

```xml
<!-- Synchronization parameters -->
<param name="lidar_camera_fusion/sync_time_slop" value="0.15" />
<param name="lidar_camera_fusion/sync_queue_size" value="30" />
<param name="lidar_camera_fusion/time_offset_ms" value="45.3" />
```

### Option B: Use Enhanced Launch File

Use the pre-configured launch file with sync parameters:

```bash
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch
```

Then edit `launch/lidar_camera_fusion_with_sync.launch` and update the `time_offset_ms` parameter.

## Parameter Reference

### `sync_time_slop` (seconds, default: 0.1)

Maximum time difference allowed for matching camera and LiDAR messages.

- **Too small**: Messages will be dropped if timing varies
- **Too large**: May match wrong messages together
- **Recommended**: 1.5x the StdDev from diagnostics (e.g., if StdDev=50ms, use 0.075s)

```xml
<param name="lidar_camera_fusion/sync_time_slop" value="0.15" />
```

### `sync_queue_size` (integer, default: 20)

Size of message buffer for each sensor.

- **Too small**: May drop messages before finding matches
- **Too large**: Uses more memory, may increase latency
- **Recommended**: 20-50 for typical scenarios, increase if you see "dropping message" warnings

```xml
<param name="lidar_camera_fusion/sync_queue_size" value="30" />
```

### `time_offset_ms` (milliseconds, default: 0.0)

Manual time offset to compensate for sensor latency differences.

- **Positive value**: Shifts camera timestamps forward (use when camera is ahead)
- **Negative value**: Shifts camera timestamps backward (use when LiDAR is ahead)
- **Recommended**: Use the Mean value from sync diagnostics

```xml
<param name="lidar_camera_fusion/time_offset_ms" value="45.3" />
```

**Note**: Currently this parameter is logged but not actively applied to timestamps. The `sync_time_slop` handles the matching. This parameter is reserved for future frame offset implementation.

## Step 3: Verify Synchronization

After configuring parameters, monitor the fusion output:

```bash
# Terminal 1: Run fusion with new parameters
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch

# Terminal 2: Monitor timing
rostopic echo /sync_diagnostics/camera_lidar_delay
```

### Check Fusion Logs

Look for timing information in the fusion node output:

```
Sync: Camera-LiDAR offset = +45.2 ms (Camera: 1234.567, LiDAR: 1234.522)
Association SUCCESS for person: 3.5 m from 28 LiDAR points
```

### Enable Debug Logging

For more detailed timing information:

```bash
rosservice call /lidar_camera_fusion/set_logger_level ros.drive_yolo DEBUG
```

You'll see messages like:
```
Sync: Camera-LiDAR offset = +45.1 ms (Camera: 1234.567, LiDAR: 1234.522)
Applying time offset: +45.3 ms
```

## Troubleshooting

### Problem: "Dropping message" warnings

**Symptoms:**
```
[WARN] Dropping message (queue full)
```

**Solution:**
- Increase `sync_queue_size` (try 50 or 100)
- Reduce `sync_time_slop` if it's too large

### Problem: No associations despite valid detections

**Symptoms:**
```
Fusion: 0/5 detections matched with LiDAR
```

**Solution:**
1. Run sync diagnostics to check if messages are being received
2. Check if timing offset is very large (>200ms)
3. Increase `sync_time_slop` to accommodate timing variance
4. Verify sensor data is being published with correct timestamps

### Problem: Wrong objects being associated

**Symptoms:**
- Distance estimates seem incorrect
- Pedestrian gets car's distance

**Solution:**
- Timing mismatch is likely too large for the slop parameter
- Use sync diagnostics to measure actual offset
- Increase `sync_queue_size` to buffer more messages
- Consider reducing frame rate if possible

### Problem: High timing variance (StdDev >50ms)

**Symptoms:**
```
StdDev: 75.3 ms
```

**Solution:**
- This indicates inconsistent timing, common in simulation
- Increase `sync_time_slop` to 0.2 or higher
- Increase `sync_queue_size` to 50+
- Consider if sensor simulation timing can be improved

## Frame Offset (Future Feature)

The `time_offset_ms` parameter is designed for future frame offset implementation, where you could specify:

```xml
<!-- Use LiDAR frame N with camera frame N+1 -->
<param name="lidar_camera_fusion/time_offset_ms" value="33.3" />  <!-- ~1 frame at 30Hz -->

<!-- Use LiDAR frame N with camera frame N+2 -->
<param name="lidar_camera_fusion/time_offset_ms" value="66.6" />  <!-- ~2 frames at 30Hz -->
```

Currently, the ApproximateTime synchronizer handles timing mismatches automatically within the `sync_time_slop` tolerance.

## Example Configurations

### Fast Sensors (Low latency, ~5ms variance)
```xml
<param name="lidar_camera_fusion/sync_time_slop" value="0.02" />   <!-- 20ms -->
<param name="lidar_camera_fusion/sync_queue_size" value="10" />
<param name="lidar_camera_fusion/time_offset_ms" value="0.0" />
```

### Simulation (Medium variance, ~30ms offset)
```xml
<param name="lidar_camera_fusion/sync_time_slop" value="0.1" />    <!-- 100ms -->
<param name="lidar_camera_fusion/sync_queue_size" value="30" />
<param name="lidar_camera_fusion/time_offset_ms" value="30.0" />
```

### High Latency (Large variance, >100ms offset)
```xml
<param name="lidar_camera_fusion/sync_time_slop" value="0.2" />    <!-- 200ms -->
<param name="lidar_camera_fusion/sync_queue_size" value="50" />
<param name="lidar_camera_fusion/time_offset_ms" value="120.0" />
```

## Monitoring Tools

### Real-time Delay Visualization

```bash
# Plot delay over time
rqt_plot /sync_diagnostics/camera_lidar_delay
```

### Message Timestamps

```bash
# Check camera timestamps
rostopic echo /detections/header/stamp

# Check LiDAR timestamps
rostopic echo /invz_summation_reflection/header/stamp
```

### TF Tree Timing

```bash
# View TF timing issues
rosrun tf tf_monitor
```

## Best Practices

1. **Always run diagnostics first** before configuring parameters
2. **Start conservative** with larger slop values, then tune down
3. **Monitor association rates** after changes
4. **Log timing data** during testing for post-analysis
5. **Document your configuration** for different scenarios (sim vs real)

## References

- Fusion node: `src/lidar_camera_fusion.cpp`
- Diagnostics node: `src/sync_diagnostics_node.cpp`
- Launch files: `launch/sync_diagnostics.launch`, `launch/lidar_camera_fusion_with_sync.launch`
- ROS message_filters documentation: http://wiki.ros.org/message_filters
