#!/usr/bin/env python3
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink


def move_to_pose(moveit2, position, quat_xyzw, cartesian=False):
    """Moves the arm to the specified pose and waits for 3 seconds."""
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    time.sleep(3)  # Wait for 3 seconds at each position


def move_to_joint(moveit2, joint_positions):
    """Moves the arm to the specified joint configuration and waits for 3 seconds."""
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()
    time.sleep(3)  # Wait for 3 seconds after joint movement


def gripper_magnet_on(node, box_name):
    """Activates the gripper magnet to attach a box."""
    client = node.create_client(AttachLink, '/GripperMagnetON')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF service not available, waiting again...')

    request = AttachLink.Request()
    request.model1_name = box_name
    request.link1_name = 'link'
    request.model2_name = 'ur5'
    request.link2_name = 'wrist_3_link'
    
    client.call_async(request)

def gripper_magnet_off(node, box_name):
    """Deactivates the gripper magnet to release a box."""
    client = node.create_client(DetachLink, '/GripperMagnetOFF')
    
    while not client.wait_for_service(timeout_sec=1.0):
         node.get_logger().info('EEF service not available, waiting again...')
    
    request = DetachLink.Request()
    request.model1_name = box_name
    request.link1_name = 'link'
    request.model2_name = 'ur5'
    request.link2_name = 'wrist_3_link'
    
    client.call_async(request)

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Define positions and quaternion orientations for box one
    P1 = [0.20, -0.47, 0.65]  # Box one pickup position
    Q1 = [0.5, -0.5, 0.5, 0.5]

    # Drop-off position
    D = [-0.69, 0.10, 0.44]
    Dq = [-1.0, 0.0, 0.0, 0.0]

    # Define joint positions
    joint_positions = [
        -0.0013444190041891346,
        -2.3929405742185463,
        2.4050544980583943,
        -3.1518947976324028,
        -1.5786568168126147,
        3.1500405026756044
    ]
    
    joint_positions1 = [
        -1.5708,
        -2.3929405742185463,
        2.4050544980583943,
        -3.1518947976324028,
        -1.5786568168126147,
        3.1500405026756044
    ] 
    
    joint_positionsd = [
        0.0000,
        -2.3911,
        0.680678,
        -3.15905,
        -1.5708,
        3.14159
    ]

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Box name for gripper control
    box_name = "box1"  # Replace with your actual box name
    move_to_joint(moveit2, joint_positions)
    move_to_joint(moveit2, joint_positions1)
    move_to_pose(moveit2, P1, Q1)
    gripper_magnet_on(node, box_name)
    move_to_joint(moveit2, joint_positions1)
    move_to_joint(moveit2, joint_positions)
    move_to_joint(moveit2, joint_positionsd)
    move_to_pose(moveit2, D, Dq)
    gripper_magnet_off(node, box_name)
    
    #shutdown
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
