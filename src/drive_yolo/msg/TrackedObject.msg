# Single tracked object with motion information
int32 id                # Unique tracking ID
int32 class_id          # Object class ID
string class_name       # Human readable class name
float32 x               # Bounding box center x
float32 y               # Bounding box center y  
float32 width           # Bounding box width
float32 height          # Bounding box height
float32 confidence      # Detection confidence [0,1]
float32 velocity_x      # Velocity in pixels/second (x component)
float32 velocity_y      # Velocity in pixels/second (y component)
float32 speed           # Speed magnitude in pixels/second
float32 track_duration  # Seconds since first detection
int32 total_detections  # Total number of detections for this track