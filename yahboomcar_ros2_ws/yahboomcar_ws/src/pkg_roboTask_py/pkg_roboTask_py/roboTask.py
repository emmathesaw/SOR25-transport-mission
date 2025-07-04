# PROGRAM TASK: robot will navigate through a series of obstacles and beep when
# it comes across a certain color obstacle within a certain distance.
# Referenced colorTracker.py, laser_Avoidance.py, laser_Warning.py
# maybe user can input a color instead of the robot calibrating it??????????? 
# or have a set color in the progam

#figure out how to use the scanning ability, also disect the avoidance program

# ROS Library
import rclpy
import os
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist #for velocity publisher
from sensor_msgs.msg import LaserScan, Image #for the laser subscription
from std_msgs.msg import Bool,UInt16 #for the buzzer publisher
import cv2
from cv_bridge import CvBridge  #to convert ROS Image messages to OpenCV images

import numpy as np

# Common Library
import math
import time
from time import sleep #i think sleep() used to stall the robot for specified time
from yahboomcar_laser.common import * #something with PID (Proportional-Integral-Derivative)
print("import done")
RAD2DEG = 180 / math.pi

# IMPORT MORE THINGS???

class obstacleMaze(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a subscription
        self.sub_camera = self.create_subscription(Image, "/image_raw", self.process_camera_image, 2)
        self.sub_laser = self.create_subscription(LaserScan,"/scan", self.registerScan, 1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback, 1)
        
        #create a publisher
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_Buzzer = self.create_publisher(UInt16, '/beep', 1)

        #declare parameters
        self.declare_parameter("ResponseDist", 0.3)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.declare_parameter("linear", 0.3)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular", 1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle", 45.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("TargetColor", "white")  #default color to detect

        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.Moving = False
        self.ros_ctrl = SinglePID()
        self.bridge = CvBridge()
        self.target_color_hsv = None
        
        #self.Buzzer_state = False

        self.timer = self.create_timer(0.01,self.on_timer)
        self.set_target_color()

    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value


    def set_target_color(self):
        '''target_color = self.get_parameter("TargetColor").get_parameter_value().string_value
        if target_color.lower() == "red":
            self.target_color_hsv = ((0, 100, 100), (10, 255, 255))  # HSV range for red
        elif target_color.lower() == "blue":
            self.target_color_hsv = ((110, 100, 100), (130, 255, 255))  # HSV range for blue
        elif target_color.lower() == "green":
            self.target_color_hsv = ((50, 100, 100), (70, 255, 255))  # HSV range for green
        else:'''
        print("Color not supported, using white as default.")
        self.target_color_hsv = ((0, 0, 0), (0, 0, 255))  # Default to white


    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data



    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return

        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180: angle -= 360
            if 20 < angle < self.LaserAngle:
                if ranges[i] < self.ResponseDist * 1.5:
                    self.Left_warning += 1
            if -self.LaserAngle < angle < -20:
                if ranges[i] < self.ResponseDist * 1.5:
                    self.Right_warning += 1
            if abs(angle) <= 20:
                if ranges[i] <= self.ResponseDist * 1.5: 
                    self.front_warning += 1

        if self.Joy_active or self.Switch:
            if self.Moving:
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return

        self.Moving = True
        twist = Twist()
        if self.front_warning > 10:
            if self.Left_warning > 10 and self.Right_warning > 10:
                print('Obstacle ahead, turning right')
                twist.angular.z = -self.angular
            elif self.Right_warning > 10:
                print('Obstacle ahead, turning left')
                twist.angular.z = self.angular
            else:
                print('Obstacle ahead, turning right')
                twist.angular.z = -self.angular
        elif self.Left_warning > 10:
            print('Obstacle on the left, turning right')
            twist.angular.z = -self.angular
        elif self.Right_warning > 10:
            print('Obstacle on the right, turning left')
            twist.angular.z = self.angular
        else:
            print('No obstacles, moving forward')
            twist.linear.x = self.linear

        self.pub_vel.publish(twist)

        

        
        '''minDist = 0.2 #should beep
        #minDist = 0.4 #shouldn't beep

        print("minDist: ",minDist)
        #print("minDistID: ",minDistID)
        if minDist <= self.ResponseDist:
            print("BEEP")
            beep = UInt16()
            beep.data = 1
            self.pub_Buzzer.publish(beep)

        else:
            print("no obstacles")
            beep = UInt16()
            beep.data = 0
            self.pub_Buzzer.publish(beep)'''


        #TASK: get robot to beep if an object is a certain distance in front of it 
        #(not moving camera, just if it is in frame head on using object recognition) 

    def process_camera_image(self, image_data):
        #self.capture = cv2.VideoCapture(0, apiPreference=cv2.CAP_AVFOUNDATION)
        #if not isinstance(image_data, Image):
        print('CAMERA IMAGE')
        if not isinstance(image_data, Image): return

        '''cap = cv2.VideoCapture(0)
        while True: 
            ret, frame = cap.read()
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            mask = cv2.inRange(hsv_frame, lower_red, upper_red)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.imshow('Color Detection', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        cap.release()
        cv2.destroyAllWindows()'''

        #convert ROS Image message to OpenCV image
        '''np_img = np.frombuffer(image_data.data, np.uint8).reshape(image_data.height, image_data.width, -1)
        hsv_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2HSV)

        #get color boundaries for target color
        lower_bound, upper_bound = self.color_boundaries.get(self.target_color, ([0, 0, 0], [0, 0, 0]))
        mask = cv2.inRange(hsv_img, np.array(lower_bound), np.array(upper_bound))
        detected_color = cv2.countNonZero(mask)

        if detected_color > 0:
            print(f"{self.target_color_hsv} color detected!")
            beep = UInt16()
            beep.data = 1
            self.pub_Buzzer.publish(beep)
        else:
            beep = UInt16()
            beep.data = 0
            self.pub_Buzzer.publish(beep)'''



    # keyboard command to stop car: ctrl + c
    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 + cmd2
        os.system(cmd)
        cmd3 = "ros2 topic pub --once /beep std_msgs/msg/UInt16 "
        cmd4 = '''"data: 0"'''
        cmd5 = cmd3 + cmd4
        os.system(cmd5)


def main():
    rclpy.init()
    obstacle_maze = obstacleMaze("obstacle_maze")
    print("3...2...1...Begin!")
    try:
        rclpy.spin(obstacle_maze)
    # if ctrl + c, shutdown program
    except KeyboardInterrupt:
        obstacle_maze.exit_pro()
        obstacle_maze.destroy_node()
        rclpy.shutdown()
