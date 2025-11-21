#!/usr/bin/env python3

import time
import rospy
import threading
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from http.server import HTTPServer, BaseHTTPRequestHandler
import math
import struct

# OSI imports
try:
    from osi3.osi_sensordata_pb2 import SensorData
except ImportError:
    print("OSI3 not found. Please install with: pip install open-simulation-interface")
    exit(1)


class OSIHTTPHandler(BaseHTTPRequestHandler):
    def __init__(self, lidar_publisher, *args, **kwargs):
        self.lidar_publisher = lidar_publisher
        super().__init__(*args, **kwargs)

    def do_POST(self):
        try:
            # Get content length
            content_length = int(self.headers.get('Content-Length', 0))

            # Read the request body
            post_data = self.rfile.read(content_length)
            #print(time.time())
            # Parse OSI message
            osi_lidar = SensorData()
            if not osi_lidar.ParseFromString(post_data):
                rospy.logerr("Failed to parse OSI Lidar message")
                self.send_error(400, "Invalid OSI data")
                return

            # Convert to PointCloud2 and publish
            self.convert_and_publish_lidar(osi_lidar)

            # Send success response
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Data received')

        except Exception as e:
            rospy.logerr(f"Error processing request: {str(e)}")
            self.send_error(500, str(e))

    def convert_and_publish_lidar(self, osi_lidar):
        """Convert OSI SensorData to ROS PointCloud2 message"""

        # Create header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "lidar_frame"

        # Check if we have lidar sensor data
        if not osi_lidar.feature_data.lidar_sensor:
            rospy.logwarn("No lidar sensor data in OSI message")
            return

        lidar_sensor = osi_lidar.feature_data.lidar_sensor[0]

        if not lidar_sensor.detection:
            rospy.logwarn("No detections in lidar sensor data")
            return

        # Convert OSI detections to point cloud points
        points = []
        for detection in lidar_sensor.detection:
            # Extract spherical coordinates
            distance = detection.position.distance
            elevation = detection.position.elevation
            azimuth = detection.position.azimuth
            intensity = detection.intensity

            # Convert spherical to Cartesian coordinates
            x = distance * math.cos(elevation) * math.cos(azimuth)
            y = - distance * math.cos(elevation) * math.sin(azimuth)
            z = distance * math.sin(elevation)

            # Add point with intensity
            points.append([x, y, z, intensity])

        # Define point cloud fields
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]

        # Create PointCloud2 message
        point_cloud = pc2.create_cloud(header, fields, points)

        # Publish the point cloud
        self.lidar_publisher.publish(point_cloud)
        rospy.loginfo(f"Published lidar data with {len(points)} points")

    def log_message(self, format, *args):
        """Override to use ROS logging instead of print"""
        rospy.logdebug(format % args)


class LidarOsiHttpReceiver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('lidar_osi_http_receiver', anonymous=True)

        # Get parameters
        self.port = rospy.get_param('~port', 5002)
        self.topic_name = rospy.get_param('~topic_name', '/invz_summation_reflection')

        # Create publisher
        self.lidar_pub = rospy.Publisher(self.topic_name, PointCloud2, queue_size=10)

        # Create HTTP server
        def handler_factory(*args, **kwargs):
            return OSIHTTPHandler(self.lidar_pub, *args, **kwargs)

        self.server = HTTPServer(('', self.port), handler_factory)
        rospy.loginfo(f"Starting HTTP server on port {self.port}")
        rospy.loginfo(f"Publishing lidar data to topic: {self.topic_name}")

        # Start server in separate thread
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down HTTP server...")
        self.server.shutdown()
        self.server_thread.join()


def main():
    try:
        # Create the receiver
        receiver = LidarOsiHttpReceiver()

        # Register shutdown hook
        rospy.on_shutdown(receiver.shutdown)

        # Keep the node running
        rospy.loginfo("Lidar OSI HTTP Receiver is running...")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")
    finally:
        rospy.loginfo("Node shutting down")


if __name__ == '__main__':
    main()
