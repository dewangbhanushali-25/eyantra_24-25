#!/usr/bin/env python3
# Copyright 2021 Samsung Research America
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
"""
Optimized navigation demo to reach specified poses.
"""

def create_pose(x, y, yaw, frame_id="map"):
    """Helper function to create a PoseStamped with given position and orientation."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = math.cos(yaw / 2)
    pose.pose.orientation.z = math.sin(yaw / 2)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = create_pose(1.839959, -9.049986, 3.140001)
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate
    navigator.waitUntilNav2Active()
    start_pt_after_dock = [0.430323, -2.530024, 3.000075]
    # 2.32, 2.70, -1.57 cb2
    # -4.4, 3.2, -1.64 cb1
    # Define goal poses
    goal_poses = [
        create_pose(0.360323, -2.200024, 3.340075),
        create_pose(2.32, 2.70, -1.57),
        create_pose(0.430323, -2.580024, 3.000075),
        create_pose(-4.4, 3.2, -1.64),
        create_pose(0.430323, -2.630024, 3.000075)
    ]

    # Start navigation to waypoints
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    # Monitor task progress
    feedback_interval = 5
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and feedback.current_waypoint % feedback_interval == 0:
            print(
                f"Executing waypoint: {feedback.current_waypoint + 1}/{len(goal_poses)}"
            )
            if navigator.get_clock().now() - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

    # Handle result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
