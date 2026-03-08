#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from ebot_docking.srv import DockSw  # Custom docking service
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class EbotDockingService(Node):

    def __init__(self):
        super().__init__('ebot_docking_service')

        # Ultrasonic sensor readings
        self.ultrasonic_left = float('inf')
        self.ultrasonic_right = float('inf')
        self.callback_group = ReentrantCallbackGroup()

        # Docking service
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)


        # Velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriptions for ultrasonic sensors
        self.ultrasonic_left_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_left_callback, 10)
        self.ultrasonic_right_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_right_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        
        # targets and current robo pose
        self.target_yaw = None
        self.target_distance = None
        self.robot_pose = [0.0, 0.0, 0.0]

        # P-controller gains
        self.kp_angular = 1.5 # prev 0.8
        self.kp_linear = 0.8 # prev 0.5
        self.threshold_yaw = 0.010  # Angular threshold in radians prev 0.019
        self.threshold_distance = 0.05  # Linear distance threshold in meters) prev 0.1
        
        # self.target_yaw = -math.pi/2 - 0.1

        # Flag to enable docking
        self.is_docking = False
        self.dock_aligned = False

        self.controller_timer = self.create_timer(0.3, self.controller_loop)

    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    def ultrasonic_left_callback(self, msg):
        self.ultrasonic_left = msg.range

    def ultrasonic_right_callback(self, msg):
        self.ultrasonic_right = msg.range

   

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def controller_loop(self):
        """Controller loop for docking using P-controller logic."""
        if not self.is_docking:
            return
        print("Hi from control loop")
        twist_msg = Twist()
        yaw_error = self.normalize_angle(self.target_yaw - self.robot_pose[2])
        current_distance = (self.ultrasonic_left + self.ultrasonic_right) / 2
        distance_error = current_distance

        # Angular correction for alignment
        if abs(yaw_error) > self.threshold_yaw:
            twist_msg.angular.z = self.kp_angular * yaw_error
            # print(f"yaw_error: {yaw_error}")
        else:
            twist_msg.angular.z = 0.0  # Stop rotating once aligned
            if abs(distance_error) > self.threshold_distance :
                twist_msg.linear.x = -self.kp_linear * distance_error
            else:
                twist_msg.linear.x = 0.0  # Stop moving once close enough

        self.cmd_vel_pub.publish(twist_msg)

        # Check if docking is complete
        if abs(yaw_error) < self.threshold_yaw and abs(distance_error) < self.threshold_distance:
            self.dock_aligned = True
            self.is_docking = False
            self.get_logger().info("Docking alignment and distance achieved. Docking complete!")

    def dock_control_callback(self, request, response):
        self.is_docking = request.linear_dock or request.orientation_dock
        self.target_yaw = request.orientation - 0.1
        #

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        self.dock_aligned = False
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EbotDockingService()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    executor = MultiThreadedExecutor(num_threads=2)
    
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()