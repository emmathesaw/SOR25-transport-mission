import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import UInt16
import numpy as np
import cv2
from cv_bridge import CvBridge
import math

RAD2DEG = 180 / math.pi

class RedObjectApproach(Node):
    def __init__(self):
        super().__init__('red_object_approach')
        self.bridge = CvBridge()
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_beep = self.create_publisher(UInt16, '/beep', 1)
        self.sub_image = self.create_subscription(Image, '/image_raw', self.process_camera_image, 1)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.process_laser_scan, 1)

        self.red_detected = False
        self.approaching = False
        self.stop_distance = 0.05
        self.front_distance = float('inf')
        self.state = 'SEARCHING'

    def process_camera_image(self, image_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)

            red_pixels = cv2.countNonZero(mask)
            if red_pixels > 500:
                if not self.red_detected:
                    print("Red object detected.")
                self.red_detected = True
                beep = UInt16()
                beep.data = 1
                self.pub_beep.publish(beep)
            else:
                self.red_detected = False
                beep = UInt16()
                beep.data = 0
                self.pub_beep.publish(beep)
        except Exception as e:
            self.get_logger().error(f"Camera processing error: {e}")

    def process_laser_scan(self, scan_data):
        ranges = np.array(scan_data.ranges)
        front_angles = []
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + i * scan_data.angle_increment) * RAD2DEG
            if abs(angle) < 15:
                front_angles.append(ranges[i])
        if front_angles:
            self.front_distance = min(front_angles)

        self.behavior_logic()

    def behavior_logic(self):
        twist = Twist()

        if self.state == 'SEARCHING':
            if self.red_detected:
                self.get_logger().info("Approaching red object...")
                self.state = 'APPROACHING'

        elif self.state == 'APPROACHING':
            if self.front_distance > self.stop_distance + 0.05:
                twist.linear.x = 0.2
            else:
                twist.linear.x = 0.0
                self.state = 'TURNING'
                self.turn_start_time = self.get_clock().now()
                self.get_logger().info("Stopping in front of red object. Turning 180°...")

        elif self.state == 'TURNING':
            duration = 4.0  # seconds to complete 180 turn; tune this for your robot
            if (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9 < duration:
                twist.angular.z = 0.8
            else:
                twist.angular.z = 0.0
                self.state = 'BACKING'
                self.back_start_time = self.get_clock().now()
                self.get_logger().info("Backing into red object...")

        elif self.state == 'BACKING':
            duration = 1.0  # back into the object for 2 seconds
            if (self.get_clock().now() - self.back_start_time).nanoseconds / 1e9 < duration:
                twist.linear.x = -0.2
            else:
                twist.linear.x = 0.0
                self.state = 'DONE'
                self.get_logger().info("Pickup position reached.")

        elif self.state == 'DONE':
            pass  # do nothing

        self.pub_vel.publish(twist)

    def stop_all(self):
        twist = Twist()
        self.pub_vel.publish(twist)
        beep = UInt16()
        beep.data = 0
        self.pub_beep.publish(beep)

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down cleanly...')
        node.stop_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()
