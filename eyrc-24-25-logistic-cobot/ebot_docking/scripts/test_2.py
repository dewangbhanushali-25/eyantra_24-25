#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from threading import Event, Thread, Lock
from ebot_docking.srv import DockSw, PassingSw
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create callback groups for each component
        self.nav_callback_group = MutuallyExclusiveCallbackGroup()
        self.dock_callback_group = MutuallyExclusiveCallbackGroup()
        self.pass_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create events to control thread activation
        self.dock_event = Event()
        self.pass_event = Event()
        
        # Create locks for thread safety
        self.dock_lock = Lock()
        self.pass_lock = Lock()
        
        # Initialize navigation
        self.navigator = BasicNavigator()
        
        # Initialize service clients
        self.dock_client = self.create_client(
            DockSw, 
            'dock_control',
            callback_group=self.dock_callback_group
        )
        
        self.pass_client = self.create_client(
            PassingSw,
            'passing_srv',
            callback_group=self.pass_callback_group
        )
        
        # Status flags
        self.is_docking = False
        self.is_passing = False
        self.navigation_complete = False
        
        # Create threads
        self.nav_thread = Thread(target=self.navigation_loop)
        self.dock_thread = Thread(target=self.docking_loop)
        self.pass_thread = Thread(target=self.passing_loop)
        
    def start_threads(self):
        """Start all threads"""
        self.nav_thread.start()
        self.dock_thread.start()
        self.pass_thread.start()
        
    def stop_threads(self):
        """Stop all threads safely"""
        self.navigation_complete = True
        self.dock_event.set()  # Wake up docking thread to exit
        self.pass_event.set()  # Wake up passing thread to exit
        
        self.nav_thread.join()
        self.dock_thread.join()
        self.pass_thread.join()
        
    def navigation_loop(self):
        """Main navigation loop that runs continuously"""
        self.navigator.waitUntilNav2Active()
        
        while not self.navigation_complete:
            try:
                # Define waypoints
                waypoints = self.get_waypoints()
                
                for waypoint in waypoints:
                    self.get_logger().info(f'Navigating to waypoint: {waypoint}')
                    
                    # Navigate to waypoint
                    self.navigator.goToPose(waypoint)
                    
                    # Check if we need to dock or pass at this waypoint
                    if self.is_docking_position(waypoint):
                        with self.dock_lock:
                            self.is_docking = True
                            self.dock_event.set()  # Wake up docking thread
                            while self.is_docking:
                                time.sleep(0.1)  # Wait for docking to complete
                                
                    if self.is_passing_position(waypoint):
                        with self.pass_lock:
                            self.is_passing = True
                            self.pass_event.set()  # Wake up passing thread
                            while self.is_passing:
                                time.sleep(0.1)  # Wait for passing to complete
                    
                    # Wait for navigation to complete
                    while not self.navigator.isTaskComplete():
                        feedback = self.navigator.getFeedback()
                        if feedback:
                            self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
                        time.sleep(0.1)
                        
            except Exception as e:
                self.get_logger().error(f'Navigation error: {str(e)}')
                time.sleep(1)
                
    def docking_loop(self):
        """Docking thread that sleeps until needed"""
        while not self.navigation_complete:
            self.dock_event.wait()  # Sleep until docking is needed
            
            if self.navigation_complete:
                break
                
            try:
                with self.dock_lock:
                    if self.is_docking:
                        self.get_logger().info('Starting docking procedure')
                        
                        # Create and send docking request
                        request = DockSw.Request()
                        request.linear_dock = True
                        request.orientation_dock = True
                        request.orientation = -math.pi/2  # Example orientation
                        
                        # Call docking service
                        future = self.dock_client.call_async(request)
                        rclpy.spin_until_future_complete(self, future)
                        
                        if future.result().success:
                            self.get_logger().info('Docking successful')
                        else:
                            self.get_logger().error('Docking failed')
                            
                        self.is_docking = False
                        
            except Exception as e:
                self.get_logger().error(f'Docking error: {str(e)}')
                self.is_docking = False
                
            finally:
                self.dock_event.clear()  # Reset event
                
    def passing_loop(self):
        """Passing thread that sleeps until needed"""
        while not self.navigation_complete:
            self.pass_event.wait()  # Sleep until passing is needed
            
            if self.navigation_complete:
                break
                
            try:
                with self.pass_lock:
                    if self.is_passing:
                        self.get_logger().info('Starting passing procedure')
                        
                        # Create and send passing request
                        request = PassingSw.Request()
                        request.get_box = True
                        
                        # Call passing service
                        future = self.pass_client.call_async(request)
                        rclpy.spin_until_future_complete(self, future)
                        
                        if future.result().success:
                            self.get_logger().info('Passing successful')
                        else:
                            self.get_logger().error('Passing failed')
                            
                        self.is_passing = False
                        
            except Exception as e:
                self.get_logger().error(f'Passing error: {str(e)}')
                self.is_passing = False
                
            finally:
                self.pass_event.clear()  # Reset event
    
    def get_waypoints(self):
        """Define navigation waypoints"""
        waypoints = []
        
        # Example waypoints - modify as needed
        start_point = self.create_pose([1.13, -2.25, 3.30])
        arm_position = self.create_pose([2.32, 2.75, -1.57])
        docking_position = self.create_pose([-4.7, 3.11, -1.80])
        
        waypoints.extend([start_point, arm_position, docking_position])
        return waypoints
        
    def create_pose(self, pos_list):
        """Create PoseStamped message from position list"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = pos_list[0]
        pose.pose.position.y = pos_list[1]
        pose.pose.orientation.w = math.cos(pos_list[2] / 2)
        pose.pose.orientation.z = math.sin(pos_list[2] / 2)
        return pose
        
    def is_docking_position(self, pose):
        """Check if current position is a docking position"""
        # Example implementation - modify based on your requirements
        return abs(pose.pose.position.x + 4.7) < 0.1 and abs(pose.pose.position.y - 3.11) < 0.1
        
    def is_passing_position(self, pose):
        """Check if current position is a passing position"""
        # Example implementation - modify based on your requirements
        return abs(pose.pose.position.x - 2.32) < 0.1 and abs(pose.pose.position.y - 2.75) < 0.1

def main(args=None):
    rclpy.init(args=args)
    
    controller = RobotController()
    executor = MultiThreadedExecutor(num_threads=4)  # One thread for each component plus one for the main node
    executor.add_node(controller)
    
    try:
        controller.start_threads()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_threads()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()