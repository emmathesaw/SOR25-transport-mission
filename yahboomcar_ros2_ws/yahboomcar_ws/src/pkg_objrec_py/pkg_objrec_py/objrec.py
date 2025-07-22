import rclpy
from rclpy.node import Node
from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObjectFinder(Node):
    def __init__(self):
        super().__init__('object_finder')

        # Subscribe to object detection
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/objectsStamped',
            self.listener_callback,
            10
        )

        # Subscribe to laser scan to determine distance
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Publisher to control robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State tracking
        self.target_id = 6
        self.detection_chunk_size = 12
        self.object_detected = False
        self.safe_distance = 0.18  # meters (adjust as needed)

        # Last known horizontal position of the object (None if not detected)
        self.last_obj_x = None


    def listener_callback(self, msg):
        data = msg.objects.data
        found = False

        for i in range(0, len(data), self.detection_chunk_size):
            object_id = int(data[i])
            if object_id == self.target_id:
                found = True
                self.object_detected = True

                chunk = data[i:i+self.detection_chunk_size]
                self.get_logger().info(f'Object chunk data: {chunk}')

                obj_x = chunk[9]  # Try this as horizontal pixel coordinate

                self.get_logger().info(f'Object {self.target_id} detected at x={obj_x:.1f}. Centering...')
                self.move_towards_object(obj_x)
                break

        if not found:
            if self.object_detected:
                self.get_logger().info(f'Lost sight of object {self.target_id}. Stopping.')
            self.object_detected = False
            self.stop_robot()


    def laser_callback(self, msg):
        if not self.object_detected:
            return

        # Check distance to obstacle directly in front (±15 degrees)
        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angle_range = 15  # degrees

        center_indices = [
            i for i in range(len(ranges))
            if abs(math.degrees(angle_min + i * angle_increment)) <= angle_range
        ]

        distances = [ranges[i] for i in center_indices if not math.isnan(ranges[i])]
        if distances and min(distances) < self.safe_distance:
            self.get_logger().info(f'Close to object (distance: {min(distances):.2f} m). Stopping.')
            self.stop_robot()
            self.object_detected = False
            self.last_obj_x = None

    def move_towards_object(self, obj_x):
        twist = Twist()

        image_center = 320  # Half of 640 px width
        error_x = obj_x - image_center  # Positive if object is right of center

        # Proportional control gain for steering — tune this
        Kp = 0.0025

        # Angular velocity to turn robot toward the object center
        twist.angular.z = -Kp * error_x

        # Move forward slowly while steering
        twist.linear.x = 0.12

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()  # Zero velocities to stop robot
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    object_finder = ObjectFinder()
    rclpy.spin(object_finder)
    object_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
