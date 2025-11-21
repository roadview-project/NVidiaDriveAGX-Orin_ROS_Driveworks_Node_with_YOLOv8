# Multiple detections result
std_msgs/Header header
sensor_msgs/Image source_image  # Original image (optional)
Detection[] detections          # Array of detections
float32 inference_time_ms       # Inference time in milliseconds
int32 total_detections         # Total detections before filtering