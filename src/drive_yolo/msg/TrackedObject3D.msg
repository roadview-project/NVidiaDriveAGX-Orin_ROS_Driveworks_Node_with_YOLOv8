int32 id
int32 class_id  
string class_name

# 3D position and motion in meters and m/s
geometry_msgs/Point position_3d         # [x, y, z] in meters
geometry_msgs/Vector3 velocity_3d        # [vx, vy, vz] in m/s  
geometry_msgs/Vector3 acceleration_3d    # [ax, ay, az] in m/s²
float32 speed_3d                         # magnitude in m/s

# 2D bounding box info (still useful for visualization)
float32 x                                # 2D center x in pixels
float32 y                                # 2D center y in pixels  
float32 width                            # 2D bounding box width
float32 height                           # 2D bounding box height

# Quality metrics
float32 confidence                       # Detection confidence [0,1]
float32 distance_confidence              # LiDAR distance confidence [0,1]
int32 lidar_points_count                 # Number of LiDAR points used
float32 track_stability                  # Track quality measure [0,1]

# Tracking info
float32 track_duration                   # Seconds since first detection
int32 total_detections                   # Number of detections
bool is_moving                           # True if speed > threshold
geometry_msgs/Point predicted_position_1s # Where object will be in 1 second
