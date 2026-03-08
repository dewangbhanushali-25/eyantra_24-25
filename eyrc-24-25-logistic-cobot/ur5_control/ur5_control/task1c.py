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


def move_to_pose(moveit2, position, quat_xyzw, cartesian=True):
    """Moves the arm to the specified pose and waits for 3 seconds."""
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    #time.sleep(0.1)  # Wait for 3 seconds at each position


def move_to_joint(moveit2, joint_positions):
    """Moves the arm to the specified joint configuration and waits for 3 seconds."""
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()
    #time.sleep(0.1)  # Wait for 3 seconds after joint movement


def gripper_magnet_on(node, box_name):
    """Activates the gripper magnet to attach a box."""
    client = node.create_client(AttachLink, '/GripperMagnetON')
    
    #while not client.wait_for_service(timeout_sec=0.1):
        #node.get_logger().info('EEF service not available, waiting again...')

    request = AttachLink.Request()
    request.model1_name = box_name
    request.link1_name = 'link'
    request.model2_name = 'ur5'
    request.link2_name = 'wrist_3_link'
    
    client.call_async(request)

def gripper_magnet_off(node, box_name):
    """Deactivates the gripper magnet to release a box."""
    client = node.create_client(DetachLink, '/GripperMagnetOFF')
    
    # while not client.wait_for_service(timeout_sec=0.1):
    #      node.get_logger().info('EEF service not available, waiting again...')
    
    request = DetachLink.Request()
    request.model1_name = box_name
    request.link1_name = 'link'
    request.model2_name = 'ur5'
    request.link2_name = 'wrist_3_link'
    
    client.call_async(request)



"""
Start: Position(-0.369374, -0.985672, 0), Orientation(0, 0, 0, 1) = Angle: 0
Destination: Position(-0.638407, -1.54282, 0), Orientation(0, 0, 0, 1) = Angle: 0

box49: Position(0.137998, -0.908352, 0), Orientation(0, 0, 0, 1) = Angle: 0


"""
def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Define positions and quaternion orientations for box one
    P3 = [ 0.75,-0.23,-0.05]  # Box one pickup position
    Q3 = [0, 0, 0, 1]

    P2 = [ 0.75,-0.49,-0.05 ]
    Q2 = [0.5, -0.5, 0.5, 0.5]

    # Drop-off position
    D = [-0.69, 0.10, 0.44]
    Dq = [-1.0, 0.0, 0.0, 0.0]

    # Define joint positions

    og_pose = [0.0, -2.3911, 2.4086, -3.1590, -1.5708, 3.1416]
    # destination = [0.0, -1.9373, -0.8203, 4.3459, 1.5533, -3.1930]
    destination = [2.89725, -1.09956, 0.872665, -1.37881, -1.62316, 2.5529]
    destination1 = [0.087266, -1.97222, -0.994838, -3.29867, -1.50098, 3.14159]
    intermediate_position_box49_box1_predestination = [0.9076, -1.6406, 1.5359, -1.5359, -1.5359, 2.5307]
    intermediate_positions_box49_box1= [0.0, -0.9774, 0.6632, -1.1694, -1.5708, 3.1416]

    #for box1
    joint_positions_box1_int_pos1 = [
        -1.5708,
        -2.3929405742185463,
        2.4050544980583943,
        -3.1518947976324028,
        -1.5786568168126147,
        3.1500405026756044
    ] 
    
    joint_positions_box1_pos = [
        -1.39626,
        -1.50098,
        1.39626,
        -3.12414,
        -1.6057,
        3.21141
        ]

    #for box49
    # intermediate_positions_box49= [0.3142, -1.117, 0.9076, -1.2741, -1.5882, 2.8449] 
    joint_positions = [-0.4712, -0.5236, 1.2392, -2.3213, -1.5359, 2.6878]

    #for box3
    joint_positions2 = [0.3316, -0.4363, 0.9948, -2.1293, -1.5708, 3.4732]





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
    executor = rclpy.executors.MultiThreadedExecutor(6)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Box name for gripper control
    box_name = "box1"
    #move_to_joint(moveit2, joint_positions_box1_int_pos1)
    move_to_joint(moveit2, joint_positions_box1_pos)
    gripper_magnet_on(node, box_name)
    #move_to_joint(moveit2, joint_positions_box1_int_pos1)
    move_to_joint(moveit2, destination1)
    gripper_magnet_off(node, box_name)
    #move_to_joint(moveit2, og_pose)
    

    box_name = "box49"  # Replace with your actual box name
    #move_to_joint(moveit2, intermediate_positions_box49_box1)
    move_to_joint(moveit2, joint_positions)
    gripper_magnet_on(node, box_name)
    #move_to_joint(moveit2, intermediate_positions_box49_box1)
    #move_to_joint(moveit2, intermediate_position_box49_box1_predestination)
    move_to_joint(moveit2, destination1)
    gripper_magnet_off(node, box_name)
    #move_to_joint(moveit2, og_pose)

    box_name = 'box3'
    #move_to_joint(moveit2, intermediate_positions_box49_box1)
    move_to_joint(moveit2, joint_positions2)
    gripper_magnet_on(node, box_name)
    #move_to_joint(moveit2, intermediate_positions_box49_box1)
    #move_to_joint(moveit2, intermediate_position_box49_box1_predestination)
    move_to_joint(moveit2, destination1)
    gripper_magnet_off(node, box_name)
    #move_to_joint(moveit2, og_pose)
    
    #shutdown
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()