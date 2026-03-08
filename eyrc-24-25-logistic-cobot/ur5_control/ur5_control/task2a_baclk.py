#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
import sys
import cv2
import math
import rclpy.executors
import rclpy.time
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Pose, TwistStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, JointState
import tf_transformations as tf
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from threading import Thread
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from servo_msgs.srv import ServoLink
from functools import partial
from control_msgs.msg import JointJog
# from task1c import move_to_pose


def calculate_rectangle_area(coordinates):
    '''
    Function to calculate area or detected ArUco marker
    '''
    (top_l, top_r, bot_r, bot_l) = coordinates
    width = np.linalg.norm(np.array(top_r) - np.array(top_l))
    height = np.linalg.norm(np.array(bot_l) - np.array(top_l))
    area = width * height
    return area, width

def detect_aruco(image):
    '''
    Function to perform ArUco detection and return details such as marker ID, 
    distance, angle, width, and center point location.
    Args:
        image (Image): Input image frame received from respective camera topic
    Returns:
        center_aruco_list (list): Center points of all ArUco markers detected
        distance_from_rgb_list (list): Distance value of each ArUco marker detected from RGB camera
        angle_aruco_list (list): Angle of all pose estimated for ArUco marker (yaw)
        width_aruco_list (list): Width of all detected ArUco markers
        ids (list): List of all ArUco marker IDs detected in a single frame
    '''

    # Camera matrix and distortion coefficients
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    size_of_aruco_m = 0.15  # Size of ArUco marker in meters (150mm)
    aruco_area_threshold = 1500  # Threshold to filter small/far markers

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Load the ArUco dictionary and set up the detector parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    
    # Detect markers in the image
    corners, detected_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # Initialize lists to store results
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    pose = []
    
    if detected_ids is not None:
        # Estimate pose for each detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_aruco_m, cam_mat, dist_mat)
        
        for i, marker_id in enumerate(detected_ids):
            # Calculate the area of the marker
            area, width = calculate_rectangle_area(corners[i][0])
            
            if area > aruco_area_threshold:  # Threshold to filter out far markers
                # Get the center of the marker
                center_x = np.mean(corners[i][0][:, 0])
                center_y = np.mean(corners[i][0][:, 1])
                center_aruco_list.append((center_x, center_y))
                
                # Get the distance (depth) of the marker
                tvec = tvecs[i]
                distance = np.linalg.norm(tvec)
                distance_from_rgb_list.append(distance)
          

                rvec = rvecs[i]
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)
                angle_aruco_list.append(euler_angles)
               
                # Add the marker width and ID
                width_aruco_list.append(width)
                ids.append(marker_id[0])
                
                cv2.aruco.drawDetectedMarkers(image, corners, detected_ids)
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvecs[i], tvecs[i], 0.1)
    
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

def get_pose(rvec, tvec):
    pose = Pose()

    pose.position.x = tvec[0]
    pose.position.y = tvec[1]
    pose.position.z = tvec[2]

     # Convert rvec to a rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Convert the rotation matrix to a homogeneous transformation matrix
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = rotation_matrix

    # Use tf.quaternion_from_matrix to get the quaternion from the transformation matrix
    quaternion = tf.quaternion_from_matrix(transformation_matrix)

    # Set orientation from the quaternion
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose

class aruco_tf(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog,'/servo_node/delta_joint_cmds', 10)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.08, self.process_image)
<<<<<<< HEAD
        self.ee_pos_timer = self.create_timer(0.0001, self.get_pose_ee)
=======
        self.ee_pos_timer = self.create_timer(0.08, self.get_pose_ee)
>>>>>>> 0106fe2 (New changes)

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
        )

        self.aruco_poses = {}
        self.arm_pose = None
        self.cv_image = None
        self.depth_image = None
        self.magnet_on = False
        self.reached_box = False
        self.reached_int = False
        self.reached_drop = False
        self.reached_initial = False
        self.reached_start = False
        self.current_box = None
        self.current_state = "start_point"
        self.box_dropped = False
        self.joint_names = []
        self.joint_angles = []

        self.joint_limits = {
            "shoulder_pan_joint": (-300, 300),
            "shoulder_lift_joint": (-300, 300),
            "elbow_joint": (-100, 100),
            "wrist_1_joint": (-300, 300),
            "wrist_2_joint": (-300, 300),  # Avoid ±90 to reduce singularity risk
            "wrist_3_joint": (-300, 300)
        }

    """
    We try and get the pose of the marker first in order to publish that pose the arm.
    This pose will be used to find the difference between the positon of the eef and the box.
    """
    def get_pose_ee(self):
        try:
            transform = self.tf_buffer.lookup_transform('base_link','wrist_3_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation.x = transform.transform.rotation.x
            pose.orientation.y = transform.transform.rotation.y
            pose.orientation.z = transform.transform.rotation.z
            pose.orientation.w = transform.transform.rotation.w

            return pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Could not get transform: {e}")
            return None

    def depthimagecb(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"Depth image conversion error: {e}")

    def colorimagecb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Color image conversion error: {e}")

    def process_image(self):
        '''
        Description: Timer function used to detect ArUco markers and publish tf on estimated poses.
                    This ensures the Z-axis always points inward.
        '''

        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
<<<<<<< HEAD
=======
        
        if self.cv_image is None:
            self.get_logger().info("No color image received yet, waiting...")
            return  # Skip processing until a valid image is received
        if self.depth_image is None:
            self.get_logger().info("No depth image received yet, waiting...")
            return
>>>>>>> 0106fe2 (New changes)

        # Detect ArUco markers and get relevant details
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)

        for i, marker_id in enumerate(ids):
            # Ensure the length of center_aruco_list and distance_from_rgb_list are as expected
            if len(center_aruco_list[i]) >= 2 and len(distance_from_rgb_list) > i:
                # Calculate the quaternion from the corrected yaw angle

                quat = R.from_euler('xyz',angle_aruco_list[i],degrees=True).as_quat()
                rot_quat = R.from_euler('xyz', (-np.pi/2,np.pi,np.pi/2)).as_quat()
                quat = tf.quaternion_multiply(quat,rot_quat)
                # Get the depth from the RealSense depth image (in meters)
                cX, cY = map(int, center_aruco_list[i])
                depth = self.depth_image[cY][cX] / 1000.0  # Convert depth from mm to meters

                cv2.circle(self.cv_image, (cX, cY), 5, (0, 255, 0), -1)

                # Calculate the x, y, z coordinates
                x = depth * (sizeCamX - cX - centerCamX) / focalX
                y = depth * (sizeCamY - cY - centerCamY) / focalY
                z = depth

                # Publish the transform from camera_link to cam_<marker_id>
                t_cam_to_marker = TransformStamped()
                t_cam_to_marker.header.stamp = self.get_clock().now().to_msg()
                t_cam_to_marker.header.frame_id = 'camera_link'
                t_cam_to_marker.child_frame_id = f'cam_{marker_id}'
                t_cam_to_marker.transform.translation.x = z
                t_cam_to_marker.transform.translation.y = x
                t_cam_to_marker.transform.translation.z = y
                t_cam_to_marker.transform.rotation.x = quat[2]
                t_cam_to_marker.transform.rotation.y = -quat[0]
                t_cam_to_marker.transform.rotation.z = -quat[1]
                t_cam_to_marker.transform.rotation.w = quat[3]

                # Send the transform
                self.br.sendTransform(t_cam_to_marker)

                # Step 5: Look up transform from base_link to camera_link
                try:
                    t_base_to_camera = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
                except Exception as e:
                    self.get_logger().error(f"Could not lookup transform: {e}")
                    continue

                # Publish the final transform from base_link to obj_<marker_id>
                t_base_to_marker = TransformStamped()
                t_base_to_marker.header.stamp = self.get_clock().now().to_msg()
                t_base_to_marker.header.frame_id = 'base_link'
                t_base_to_marker.child_frame_id = f'obj_{marker_id}'


                 # Combine translation and rotations
                translation_camera_to_marker = np.array([t_cam_to_marker.transform.translation.x,
                                                         t_cam_to_marker.transform.translation.y,
                                                         t_cam_to_marker.transform.translation.z])

                base_to_camera_quat = [t_base_to_camera.transform.rotation.x,
                                       t_base_to_camera.transform.rotation.y,
                                       t_base_to_camera.transform.rotation.z,
                                       t_base_to_camera.transform.rotation.w]
                rotation_base_to_camera = tf.quaternion_matrix(base_to_camera_quat)[:3, :3]  # 3x3 rotation matrix
                translation_base_to_marker = np.dot(rotation_base_to_camera, translation_camera_to_marker)

                # Set the translation in the final transform
                t_base_to_marker.transform.translation.x = t_base_to_camera.transform.translation.x + translation_base_to_marker[0]
                t_base_to_marker.transform.translation.y = t_base_to_camera.transform.translation.y + translation_base_to_marker[1]
                t_base_to_marker.transform.translation.z = t_base_to_camera.transform.translation.z + translation_base_to_marker[2]


              
                to_cam_to_marker_quat = [t_cam_to_marker.transform.rotation.x,
                                                                       t_cam_to_marker.transform.rotation.y,
                                                                       t_cam_to_marker.transform.rotation.z,
                                                                       t_cam_to_marker.transform.rotation.w]
                
                final_quat = tf.quaternion_multiply(base_to_camera_quat,to_cam_to_marker_quat)
                
                t_base_to_marker.transform.rotation.x = final_quat[0]
                t_base_to_marker.transform.rotation.y = final_quat[1]
                t_base_to_marker.transform.rotation.z = final_quat[2]
                t_base_to_marker.transform.rotation.w = final_quat[3]
            

                # Send the final transform
                self.br.sendTransform(t_base_to_marker)

                # Mark the center points on the image
                cv2.circle(self.cv_image, (cX, cY), 5, (0, 255, 0), -1)

                # Generating the pose of the marker
                marker_pose = Pose()

                marker_pose.position.x = t_base_to_marker.transform.translation.x
                marker_pose.position.y = t_base_to_marker.transform.translation.y
                marker_pose.position.z = t_base_to_marker.transform.translation.z

                marker_pose.orientation.x = final_quat[0]
                marker_pose.orientation.y = final_quat[1]
                marker_pose.orientation.z = final_quat[2]
                marker_pose.orientation.w = final_quat[3]

                key = f"box{marker_id}"
                # the transfor for the box should only be added to the dictionary once and should not be updated
                if not key in self.aruco_poses:
                    self.aruco_poses[key] = marker_pose
               
               
                    
                


        # Display the detected markers
        cv2.imshow('Aruco Detection', self.cv_image)
        cv2.waitKey(1)
 
        
        """
        Object to left of the arm has position.x < 0 and to the right has position.x > 0
        self.current_box.positon.x < 0 then go to left otherwise go to right
        """
       
        box_list = []
        for key,value in self.aruco_poses.items():
            if key != 'box12':
                box_list.append(key)
        
        if self.current_state == 'start_point':
            self.move_to_marker(self.make_pose([0.16324, 0.11132, 0.59002],[-0.52517, 0.85084, 0.0081286, 0.014236]))


            print("Moving to start!")

            if self.reached_start:
                self.current_state = 'moving_to_box'
                self.reached_start = False
        elif self.current_state == 'moving_to_box':
            if len(box_list) > 0:

                self.current_box = box_list[0]
                current_box_pose = self.aruco_poses[self.current_box]

                print(f"movving tp {self.current_box} box array len {len(box_list)}") # CHECK THIS, THE BOX AFTER DROPPING THE FIRST BOX AINT BEING POPPED 
                self.move_to_marker(self.aruco_poses[self.current_box])
                


                if self.reached_box:
                    self.gripper_magnet_on(self.current_box)
                    self.current_state = 'move_to_intermediate'
                    self.reached_box = False
        elif self.current_state == 'move_to_intermediate':
            # self.move_to_marker(self.make_pose([0.16194, 0.10803, 0.46418] , [0.50503, 0.49614, 0.4992, 0.49959] ))
            # self.move_to_marker(self.make_pose([0.31786, 0.10561, 0.62377] , [0.71136, 0.70483, 0.0037214, -0.0022702] ))
            # self.move_to_marker(self.make_pose([0.16463, 0.10763, 0.54032],[0.71039, 0.70376, 0.0075831, 0.0016888]))
            
            # self.move_to_marker(self.make_pose([0.16324, 0.11132, 0.59002],[-0.52517, 0.85084, 0.0081286, 0.014236]))
            self.move_to_marker(self.make_pose([0.32421, 0.10564, 0.59955], [0.71032, 0.70372, 0.012966, 0.0071012])) # Anees

            
            if self.reached_int:
                self.current_state = 'move_to_drop'
                self.reached_int = False
        elif self.current_state == 'move_to_drop':
            print("Moving to drop")

            self.move_to_marker(self.aruco_poses['box12'])
            if self.reached_drop:
                self.gripper_magnet_off(self.current_box)
                self.servo_link_client(self.current_box)
                if self.reached_drop:
                    self.current_state = 'start_point'
                    self.reached_drop = False
                     

    def make_pose(self, position, orientation):
        pose = Pose()

        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        return pose

    
    def move_to_marker(self, target_pose):
        """
        Moves the end-effector to the specified target pose while checking for singularities and collisions.
        """

        # Get the current end-effector pose
        self.arm_pose = self.get_pose_ee()
        
        # Calculate position error
        position_error = [
            target_pose.position.x - self.arm_pose.position.x,
            target_pose.position.y - self.arm_pose.position.y,
            target_pose.position.z - self.arm_pose.position.z
        ]

        # Set thresholds based on the current state
        if self.current_state == 'move_to_drop':
            position_threshold = 0.7 # Larger threshold for linear movement in the 'move_to_drop' state
            orientation_threshold = 1.2  # Larger threshold for orientation
        elif self.current_state == 'start_point':
            position_threshold = 0.25 # 3 cm threshold for linear movement
            orientation_threshold = 0.35  # Smaller threshold for orientation
        elif self.current_state == 'moving_to_box':
            position_threshold = 0.09 # 3 cm threshold for linear movement
            orientation_threshold = 1.2  # Larger threshold for orientation
        elif self.current_state == 'move_to_intermediate':
            position_threshold = 0.1 # 3 cm threshold for linear movement
            orientation_threshold = 1.0  

        # Calculate orientation error
        current_orientation = [
            self.arm_pose.orientation.x,
            self.arm_pose.orientation.y,
            self.arm_pose.orientation.z,
            self.arm_pose.orientation.w
        ]
        target_orientation = [
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ]

        # Calculate relative rotation between target and current orientations
        relative_rotation = tf.quaternion_multiply(target_orientation, tf.quaternion_inverse(current_orientation))
        
        # Define a parameter to control the interpolation step, e.g., between 0 and 1
      
        angle = 2 * math.acos(relative_rotation[3])  # angle in radians

        # Check if both position and orientation errors are within thresholds
        if all(abs(e) < position_threshold for e in position_error) and abs(angle) < orientation_threshold:
            self.get_logger().info("Reached the target position and orientation.")
            self.stop_movement()

            # Update the state based on the current state
            if self.current_state == 'start_point':
                self.reached_start = True
            elif self.current_state == 'moving_to_box':
                print("Moved to box")
                self.reached_box = True
            elif self.current_state == 'move_to_intermediate':
                print("Moved to int pos")
                self.reached_int = True
            elif self.current_state == 'move_to_drop':
                self.reached_drop = True

            # Reset position error to prevent further movement
            position_error = [float('inf'), float('inf'), float('inf')]
            return

        # Define velocity control parameters
<<<<<<< HEAD
        linear_velocity_gain = 10.0  # Gain for linear control
        max_linear_velocity = 1.8 # Maximum linear speed limit
=======
        linear_velocity_gain = 25.0  # Gain for linear control
        max_linear_velocity = 75.00  # Maximum linear speed limit
>>>>>>> 0106fe2 (New changes)

        # Calculate linear velocities
        linear_velocity = [
            min(max(vel * linear_velocity_gain, -max_linear_velocity), max_linear_velocity)
            for vel in position_error
        ]

        # Calculate angular velocity to align end-effector orientation
<<<<<<< HEAD
        max_angular_velocity = 1.8 # Maximum angular speed limit
        if abs(angle) > orientation_threshold:  # Threshold for alignment
            angular_velocity_gain = 10.0 * (angle / math.pi)  # Scales down as angle decreases
=======
        max_angular_velocity = 75.0 # Maximum angular speed limit
        if abs(angle) > orientation_threshold:  # Threshold for alignment
            angular_velocity_gain = 175.0 * (angle / math.pi)  # Scales down as angle decreases
>>>>>>> 0106fe2 (New changes)
            axis = [
                relative_rotation[0] / math.sin(angle / 2),
                relative_rotation[1] / math.sin(angle / 2),
                relative_rotation[2] / math.sin(angle / 2)
            ]
            angular_velocity = [
                min(max(axis[i] * angle * angular_velocity_gain, -max_angular_velocity), max_angular_velocity)
                for i in range(3)
            ]
        else:
            angular_velocity = [0.0, 0.0, 0.0]

        # Publish the calculated linear and angular velocities
        self.twist_it(linear_velocity, angular_velocity)


        
    """
    Publish delta twist linear and angular coordinates
    """
    def twist_it(self,linear_velocity, angular_velocity):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = linear_velocity[0]
        twist.twist.linear.y = linear_velocity[1]
        twist.twist.linear.z = linear_velocity[2]
        twist.twist.angular.x = angular_velocity[0]
        twist.twist.angular.y = angular_velocity[1]
        twist.twist.angular.z = angular_velocity[2]
        self.twist_pub.publish(twist)

    """
    Publish delta twist linear and angular coordinates to stop the motion after reaching destination
    """
    def stop_movement(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = ur5.base_link_name()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.twist_pub.publish(twist)
        self.get_logger().info("Movement stopped.")

    """Activates the gripper magnet to attach a box."""

    def gripper_magnet_on(self, box_name):
        client = self.create_client(AttachLink, '/GripperMagnetON')
        
        while not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('EEF service not available, waiting again...')

        request = AttachLink.Request()
        request.model1_name = box_name
        request.link1_name = 'link'
        request.model2_name = 'ur5'
        request.link2_name = 'wrist_3_link'
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.magnet_on_callback, box_name=box_name))

    def magnet_on_callback(self,future, box_name):
        try:
            response = future.result()
            self.magnet_on = response.success
        except Exception as e:
            self.get_logger().info(e)

    def gripper_magnet_off(self, box_name):
        """Deactivates the gripper magnet to release a box."""
        client = self.create_client(DetachLink, '/GripperMagnetOFF')
                
        request = DetachLink.Request()
        request.model1_name = box_name
        request.link1_name = 'link'
        request.model2_name = 'ur5'
        request.link2_name = 'wrist_3_link'
        
        client.call_async(request)
    def servo_link_client(self,box_name):
        client = self.create_client(ServoLink,'/SERVOLINK')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servolink service")

        req = ServoLink.Request()
        req.box_name = box_name
        req.box_link = 'link'

        future = client.call_async(req)
        future.add_done_callback(partial(self.servo_callback,box_name=box_name))

    def servo_callback(self, future, box_name):
        try:
            response = future.result()
            self.box_dropped = response.success
            self.get_logger().info("Box dropped successfully!")
            if box_name in self.aruco_poses:
                del self.aruco_poses[box_name]  # Remove the dropped box
        except Exception as e:
            self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)
    aruco_tf_node = aruco_tf()
    rclpy.spin(aruco_tf_node)
    aruco_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
<<<<<<< HEAD
    main()
=======
    main()
>>>>>>> 0106fe2 (New changes)
