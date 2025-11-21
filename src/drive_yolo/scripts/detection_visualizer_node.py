#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from drive_yolo.msg import Detections, Detection
import message_filters

class DetectionVisualizer:
    """
    ROS node that visualizes YOLO detections on camera images
    Subscribes to camera/image_raw and /detections, publishes annotated images
    """

    def __init__(self):
        rospy.init_node('detection_visualizer', anonymous=True)

        self.bridge = CvBridge()

        # Parameters
        self.bbox_thickness = rospy.get_param('~bbox_thickness', 2)
        self.text_scale = rospy.get_param('~text_scale', 0.6)
        self.text_thickness = rospy.get_param('~text_thickness', 2)
        self.show_confidence = rospy.get_param('~show_confidence', True)
        self.show_class_name = rospy.get_param('~show_class_name', True)

        # Class colors (BGR format for OpenCV)
        self.class_colors = {
            0: (255, 0, 0),      # Blue for class 0 (person)
            1: (0, 255, 0),      # Green for class 1 (bicycle)
            2: (0, 0, 255),      # Red for class 2 (car)
            3: (255, 255, 0),    # Cyan for class 3 (motorcycle)
            5: (255, 0, 255),    # Magenta for class 5 (bus)
            7: (0, 255, 255),    # Yellow for class 7 (truck)
        }
        self.default_color = (128, 128, 128)  # Gray for unknown classes

        # Synchronized subscribers for image and detections
        image_sub = message_filters.Subscriber('/camera/image_raw', Image)
        detection_sub = message_filters.Subscriber('/detections', Detections)

        # Synchronize messages with 100ms tolerance
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detection_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.callback)

        # Publisher for annotated images
        self.image_pub = rospy.Publisher('/camera/image_with_detections', Image, queue_size=1)

        # Statistics
        self.frame_count = 0
        self.last_stats_time = rospy.Time.now()

        rospy.loginfo("Detection Visualizer initialized")
        rospy.loginfo("Subscribing to: /camera/image_raw, /detections")
        rospy.loginfo("Publishing to: /camera/image_with_detections")

    def get_color_for_class(self, class_id):
        """Get color for a specific class ID"""
        return self.class_colors.get(class_id, self.default_color)

    def draw_detection(self, image, detection):
        """Draw a single detection on the image"""
        # Get bounding box coordinates (center format)
        x_center = int(detection.x)
        y_center = int(detection.y)
        width = int(detection.width)
        height = int(detection.height)

        # Convert to corner format
        x1 = x_center - width // 2
        y1 = y_center - height // 2
        x2 = x_center + width // 2
        y2 = y_center + height // 2

        # Ensure coordinates are within image bounds
        h, w = image.shape[:2]
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        x2 = max(0, min(x2, w - 1))
        y2 = max(0, min(y2, h - 1))

        # Get color for this class
        color = self.get_color_for_class(detection.class_id)

        # Draw bounding box
        cv2.rectangle(image, (x1, y1), (x2, y2), color, self.bbox_thickness)

        # Prepare label text
        label_parts = []
        if self.show_class_name:
            label_parts.append(detection.class_name)
        if self.show_confidence:
            label_parts.append(f"{detection.confidence:.2f}")

        if label_parts:
            label = " ".join(label_parts)

            # Get text size for background
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, self.text_scale, self.text_thickness
            )

            # Draw background rectangle for text
            cv2.rectangle(
                image,
                (x1, y1 - text_height - baseline - 5),
                (x1 + text_width, y1),
                color,
                -1  # Filled rectangle
            )

            # Draw text
            cv2.putText(
                image,
                label,
                (x1, y1 - baseline - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.text_scale,
                (255, 255, 255),  # White text
                self.text_thickness
            )

    def callback(self, image_msg, detections_msg):
        """Synchronized callback for image and detections"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Draw each detection
            for detection in detections_msg.detections:
                self.draw_detection(cv_image, detection)

            # Add detection count overlay
            detection_count = len(detections_msg.detections)
            info_text = f"Detections: {detection_count}"
            cv2.putText(
                cv_image,
                info_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            # Convert back to ROS Image and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            annotated_msg.header = image_msg.header  # Preserve original header
            self.image_pub.publish(annotated_msg)

            # Update statistics
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                current_time = rospy.Time.now()
                elapsed = (current_time - self.last_stats_time).to_sec()
                fps = 30.0 / elapsed if elapsed > 0 else 0
                rospy.loginfo(f"Visualizer: {self.frame_count} frames, {fps:.1f} FPS, {detection_count} detections")
                self.last_stats_time = current_time

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in visualization: {e}")

    def run(self):
        """Run the node"""
        rospy.loginfo("Detection Visualizer running...")
        rospy.spin()

def main():
    try:
        visualizer = DetectionVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Detection Visualizer shutting down")

if __name__ == '__main__':
    main()
