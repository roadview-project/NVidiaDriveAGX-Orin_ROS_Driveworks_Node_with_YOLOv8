# Detection with distance information
Detection detection          # Original YOLO detection
float64 distance            # Distance in meters
float64 distance_confidence # Confidence of distance estimate [0,1]
geometry_msgs/Point object_center_3d  # 3D center point in camera coordinates  
int32 lidar_points_count    # Number of LiDAR points used for distance calculation