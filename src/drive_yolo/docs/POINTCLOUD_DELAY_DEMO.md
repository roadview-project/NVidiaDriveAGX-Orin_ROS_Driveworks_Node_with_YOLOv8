# Pointcloud Delay Node - Demo and Testing Guide

This guide explains how to use the pointcloud delay node to demonstrate synchronization issues and test synchronization solutions.

## Purpose

The pointcloud delay node adds artificial delay to pointcloud data, allowing you to:

1. **Demonstrate sync problems**: Show what happens when camera and LiDAR timing don't match
2. **Test sync solutions**: Verify that your synchronization parameters work correctly
3. **Create visualizations**: Make before/after comparison videos
4. **Debug timing issues**: Isolate and understand timing-related problems

## Quick Start

### Basic Usage

```bash
# Terminal 1: Add 50ms delay to pointcloud
roslaunch drive_yolo pointcloud_delay_demo.launch

# Terminal 2: Run your system with delayed pointcloud
# Make sure to remap to /invz_summation_reflection_delayed
```

The node will:
- Subscribe to `/invz_summation_reflection`
- Buffer messages for 50ms
- Republish to `/invz_summation_reflection_delayed`

### Monitoring

```bash
# Watch queue size in real-time
rqt_plot /pointcloud_delay/queue_size

# Check node statistics (every 5 seconds in terminal)
# Look for: messages received/published/dropped, average delay
```

## Configuration Parameters

### Delay Mode

#### Time-Based Delay (milliseconds)

Use this for precise time-based delays:

```xml
<param name="delay_ms" value="50.0" />   <!-- 50ms delay -->
<param name="frame_delay" value="0" />    <!-- 0 = use time delay -->
```

**Examples:**
- `delay_ms="33.3"` - One frame at 30 Hz
- `delay_ms="66.6"` - Two frames at 30 Hz
- `delay_ms="100.0"` - Large delay for dramatic effect

#### Frame-Based Delay

Use this to delay by exact frame count:

```xml
<param name="delay_ms" value="0.0" />     <!-- Ignored when frame_delay > 0 -->
<param name="frame_delay" value="2" />    <!-- Wait for 2 frames -->
```

**How it works:**
- Queues messages
- Only publishes when at least N frames are buffered
- More predictable for constant frame rate scenarios

**Frame-to-time conversions (at 30 Hz):**
- 1 frame = ~33ms
- 2 frames = ~66ms
- 3 frames = ~100ms
- 5 frames = ~166ms

### Timestamp Handling

#### Option 1: Shift Timestamps (Recommended for Testing)

```xml
<param name="preserve_timestamps" value="false" />
```

- Adds delay to the message timestamp
- Makes fusion think the data is actually newer
- **Use this** when testing sync compensation

#### Option 2: Preserve Original Timestamps

```xml
<param name="preserve_timestamps" value="true" />
```

- Keeps original timestamps
- Creates actual timing mismatch
- **Use this** to demonstrate the problem

### Other Parameters

```xml
<!-- Buffer size (increase if seeing dropped messages) -->
<param name="queue_size" value="100" />

<!-- Quick enable/disable without relaunching -->
<param name="passthrough" value="false" />  <!-- true = no delay -->

<!-- Statistics reporting interval -->
<param name="report_interval" value="5.0" />  <!-- seconds -->
```

## Complete Demo Workflow

### Step 1: Establish Baseline (No Delay)

```bash
# Terminal 1: Run system normally
roslaunch drive_yolo LidarandCamerapipeline.launch

# Terminal 2: Run sync diagnostics
roslaunch drive_yolo sync_diagnostics.launch

# Record the baseline timing offset (should be small)
```

### Step 2: Add Artificial Delay

```bash
# Terminal 1: Launch delay node
roslaunch drive_yolo pointcloud_delay_demo.launch

# Terminal 2: Update your fusion launch to use delayed cloud
# Edit lidar_camera_fusion.launch, change:
<remap from="/lidar/points" to="/invz_summation_reflection_delayed" />

# Terminal 3: Run fusion with delayed cloud
roslaunch drive_yolo lidar_camera_fusion.launch
```

**Expected behavior:**
- Poor associations (pedestrians get wrong distances)
- Objects don't match properly
- Many "Association FAILED" warnings

### Step 3: Measure the Induced Delay

```bash
# Update sync diagnostics to monitor delayed cloud
# Edit sync_diagnostics.launch:
<param name="lidar_topic" value="/invz_summation_reflection_delayed" />

roslaunch drive_yolo sync_diagnostics.launch
```

**Example output:**
```
Camera-LiDAR timestamp offset:
  Mean:   +50.2 ms  (matches our 50ms delay!)
  Median: +50.1 ms
  StdDev:   2.1 ms
```

### Step 4: Configure Fusion Compensation

```bash
# Edit lidar_camera_fusion_with_sync.launch
<param name="lidar_camera_fusion/time_offset_ms" value="50.0" />
<param name="lidar_camera_fusion/sync_time_slop" value="0.1" />

# Relaunch fusion
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch
```

**Expected behavior:**
- Associations succeed again
- Correct distances
- "Association SUCCESS" messages

### Step 5: Create Comparison

Record or screenshot:
1. **No delay**: Baseline, everything works
2. **With delay, no compensation**: Broken associations
3. **With delay + compensation**: Fixed associations

## Use Cases

### 1. Demonstrating Sync Problems

**Goal**: Show what happens when sensors aren't synchronized

```xml
<!-- Large delay to make problem obvious -->
<param name="delay_ms" value="100.0" />
<param name="preserve_timestamps" value="true" />
```

**What to show:**
- Pedestrian detected but gets wrong distance
- Objects don't match between camera and LiDAR
- Diagnostic shows large timing offset

### 2. Testing Sync Parameters

**Goal**: Verify your sync configuration works for different delays

```xml
<!-- Test with various delays -->
<!-- Try 30ms, 50ms, 100ms, 150ms -->
<param name="delay_ms" value="50.0" />
```

**What to test:**
- Does fusion still associate correctly?
- Are sync parameters large enough?
- Queue size sufficient?

### 3. Creating Training Videos

**Goal**: Educational material showing before/after

**Script:**
1. Show normal operation
2. Add delay, show problems
3. Show diagnostic measuring offset
4. Configure compensation
5. Show fixed operation

### 4. Debugging Real Timing Issues

**Goal**: Isolate whether problem is timing-related

```xml
<!-- If you suspect timing issues, try compensating -->
<param name="delay_ms" value="50.0" />
<param name="frame_delay" value="0" />
```

**Process:**
- Add known delay with this node
- If problems get worse → timing issue
- If problems same → different issue
- Helps isolate root cause

## Monitoring and Verification

### Real-time Queue Visualization

```bash
# Plot queue size
rqt_plot /pointcloud_delay/queue_size

# Should see:
# - Gradual rise as queue fills
# - Plateau at frame_delay value (frame mode)
# - Steady state around expected buffer size (time mode)
```

### Statistics Output

Every 5 seconds, the node reports:

```
=== POINTCLOUD DELAY STATISTICS ===
Uptime: 30.2 seconds
Messages: Received=905, Published=890, Dropped=0
Current queue size: 2 / 100
Average rate: 30.0 Hz
Average actual delay: 50.1 ms
Configured delay: 50.0 ms
===================================
```

**What to check:**
- **Dropped messages**: Should be 0 (increase queue_size if not)
- **Average actual delay**: Should match configured delay
- **Average rate**: Should match your sensor rate (~30Hz)
- **Queue size**: Should be small in time mode, equals frame_delay in frame mode

### Verify Timestamps

```bash
# Check original cloud
rostopic echo /invz_summation_reflection/header/stamp

# Check delayed cloud
rostopic echo /invz_summation_reflection_delayed/header/stamp

# Difference should equal your delay
```

## Advanced Configurations

### Variable Delay Testing

Test how fusion handles inconsistent timing:

```bash
# Run multiple delay nodes with different delays
roslaunch drive_yolo pointcloud_delay_demo.launch delay_ms:=30.0
# Then adjust delay_ms parameter dynamically
```

### Passthrough Toggle

Quickly switch delay on/off without restarting:

```xml
<!-- Start with delay enabled -->
<param name="passthrough" value="false" />

<!-- Then use rosparam to toggle -->
<!-- rosparam set /pointcloud_delay/passthrough true -->
```

### Large Buffer for Variable Rate

If your pointcloud rate varies:

```xml
<param name="queue_size" value="200" />  <!-- Larger buffer -->
<param name="delay_ms" value="100.0" />   <!-- Longer delay -->
```

## Troubleshooting

### Problem: Queue Keeps Filling (Dropped Messages)

**Symptoms:**
```
Queue full (100), dropping oldest message
```

**Solutions:**
- Increase `queue_size` parameter
- Check if input rate is very high
- Verify publish_timer is running (should be 100Hz)

### Problem: Actual Delay Doesn't Match Configured

**Symptoms:**
```
Average actual delay: 75.3 ms
Configured delay: 50.0 ms
```

**Causes:**
- System overload (CPU/memory)
- Queue size too small
- Timer resolution issues

**Solutions:**
- Close other processes
- Increase queue_size
- Check system load with `top` or `htop`

### Problem: Frame Delay Not Publishing

**Symptoms:**
- Queue fills up but no output
- "Current queue size: 100 / 100"

**Cause:**
- Not enough frames buffered yet
- Queue size < frame_delay

**Solution:**
```xml
<!-- If using frame_delay=5, need queue_size >= 5 -->
<param name="frame_delay" value="5" />
<param name="queue_size" value="100" />  <!-- Plenty of room -->
```

### Problem: Timestamps Look Wrong

**Check settings:**
```xml
<!-- For testing compensation -->
<param name="preserve_timestamps" value="false" />  <!-- Shift by delay -->

<!-- For demonstrating problem -->
<param name="preserve_timestamps" value="true" />   <!-- Keep original -->
```

## Example Scenarios

### Scenario 1: Camera Lags by 50ms

```xml
<param name="delay_ms" value="50.0" />
<param name="preserve_timestamps" value="false" />
```

Use this to simulate camera having 50ms more latency than LiDAR.

### Scenario 2: Test 2-Frame Buffering

```xml
<param name="frame_delay" value="2" />
<param name="queue_size" value="50" />
```

Use this to test "match LiDAR frame N with camera frame N+2" scenario.

### Scenario 3: Extreme Delay for Visualization

```xml
<param name="delay_ms" value="200.0" />
<param name="preserve_timestamps" value="true" />
```

Use this to make timing problems very obvious in videos/demos.

### Scenario 4: Quick A/B Comparison

```bash
# Start with delay
roslaunch drive_yolo pointcloud_delay_demo.launch

# Record results with delay

# Disable delay without restarting
rosparam set /pointcloud_delay/passthrough true

# Record results without delay
```

## Best Practices

1. **Always measure first**: Use sync_diagnostics to measure actual delays before configuring
2. **Start small**: Test with small delays (30-50ms) before going larger
3. **Monitor statistics**: Watch for dropped messages or queue overflow
4. **Document configurations**: Note which delay values work for different scenarios
5. **Create baselines**: Always record no-delay behavior for comparison

## Integration with Other Tools

### With Sync Diagnostics

```bash
# Terminal 1: Delay node
roslaunch drive_yolo pointcloud_delay_demo.launch

# Terminal 2: Diagnostics (measuring delayed cloud)
roslaunch drive_yolo sync_diagnostics.launch

# Terminal 3: Fusion (using delayed cloud)
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch
```

### With RViz Visualization

```bash
# Add delay to make timing mismatch visible
roslaunch drive_yolo pointcloud_delay_demo.launch

# Visualize both original and delayed clouds
rosrun rviz rviz

# Add two PointCloud2 displays:
# - /invz_summation_reflection (original)
# - /invz_summation_reflection_delayed (delayed)
```

### Recording for Analysis

```bash
# Record all timing-related topics
rosbag record /invz_summation_reflection \
              /invz_summation_reflection_delayed \
              /detections \
              /sync_diagnostics/camera_lidar_delay \
              /pointcloud_delay/queue_size
```

## References

- Sync diagnostics: `docs/SYNCHRONIZATION_GUIDE.md`
- Delay node source: `src/pointcloud_delay_node.cpp`
- Demo launch file: `launch/pointcloud_delay_demo.launch`
