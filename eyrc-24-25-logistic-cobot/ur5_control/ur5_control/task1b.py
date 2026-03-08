#!/usr/bin/python3
# -*- coding: utf-8 -*-

<<<<<<< HEAD
=======

'''
# Team ID:          3152
# Theme:            Logistic Bot
# Author List:      Anees Alwani, Anish Oturkar, Dewang Bhanushali
# Filename:         task1b
# Functions:        
# Global variables: None
'''

>>>>>>> 0106fe2 (New changes)
import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
import tf_transformations as tf

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
                print(marker_id, tvec)

                rvec = rvecs[i]
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)
                angle_aruco_list.append(euler_angles)
                # print(marker_id, euler_angles)
                
                # Add the marker width and ID
                width_aruco_list.append(width)
                ids.append(marker_id[0])
                
                # Draw the detected marker and its axis
                cv2.aruco.drawDetectedMarkers(image, corners, detected_ids)
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvecs[i], tvecs[i], 0.1)
    
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

class aruco_tf(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.process_image)

        self.cv_image = None
        self.depth_image = None

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


                roll_base_to_camera, pitch_base_to_camera, yaw_base_to_camera = tf.euler_from_quaternion(base_to_camera_quat)
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

        # Display the detected markers
        cv2.imshow('Aruco Detection', self.cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_tf_node = aruco_tf()
    rclpy.spin(aruco_tf_node)
    aruco_tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
