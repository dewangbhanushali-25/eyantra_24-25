import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ebot_docking.srv import DockSw
from payload_service.srv import PayloadSW
from functools import partial
from ebot_docking.srv import PassingSw
import math

class EbotNavigationAndDocking(Node):

    def __init__(self):
        super().__init__('ebot_navigation_and_docking')
        self.navigator = BasicNavigator()
        self.docked = False
        self.drop = False
        self.box_name = None
        self.received = False
        self.ready_to_drop = False  # New flag to control payload drop

        # Set up docking and payload clients
        self.docking_client_ = self.create_client(DockSw, 'dock_control')
        self.payload_client_ = self.create_client(PayloadSW, 'payload_sw') # not to be used anymore
        self.passing_client_ = self.create_client(PassingSw,'passing_srv') # use this service to get the box

    def create_pose(self, pos_list, frame_id="map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = pos_list[0]
        pose.pose.position.y = pos_list[1]
        pose.pose.orientation.w = math.cos(pos_list[2] / 2)
        pose.pose.orientation.z = math.sin(pos_list[2] / 2)
        return pose
    
    def navigate_to(self, pose):
        """Navigate to a specific location."""
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                pass  # Can add logging if needed

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation succeeded")
            return True
        else:
            self.get_logger().warn("Navigation failed")
            return False
    
    def passing_client(self,get_box):
        req = PassingSw.Request()
        req.get_box = True
        print("Sending request to arm")
        future = self.passing_client_.call_async(req)
        future.add_done_callback(partial(self.passing_callback,get_box=get_box))


    def passing_callback(self, future, get_box):
        try:
            response = future.result()
            print(f"Got the response {response}")
            self.received = response.success
            if self.received:
                self.docked = False
                print("Passing the box!")
        except Exception as e:
            self.get_logger().error(f"Passing service call failed: {e}")
            
    def payload_client(self, receive, drop):
        """Client function to control payload operations."""
        self.received = False  # Reset the received flag
        req = PayloadSW.Request()
        req.receive = receive
        req.drop = drop

        future = self.payload_client_.call_async(req)
        future.add_done_callback(partial(self.payload_callback, receive=receive, drop=drop))

    def payload_callback(self, future, receive, drop):
        """Callback to handle payload service response."""
        try:
            response = future.result()
            self.box_name = response.message
            self.received = response.success  # Set the flag based on response
            print(f"Payload operation successful: {self.box_name}")
        except Exception as e:
            self.get_logger().error(f"Payload service call failed: {e}")
            
    def docking_client(self, angle):
        """Initiate docking procedure."""
        self.docked = False  # Reset docking flag before starting
        self.ready_to_drop = False  # Reset ready_to_drop flag before docking
        request = DockSw.Request()
        request.linear_dock = True
        request.orientation_dock = True
        request.orientation = angle  # Target orientation for alignment

        future = self.docking_client_.call_async(request)
        future.add_done_callback(self.docking_callback)

    def docking_callback(self, future):
        """Callback to handle docking service response."""
        try:
            response = future.result()
            # self.docked = response.success
            if response.success:
                self.docked = True
                self.get_logger().info(f"Docking completed successfully. docked: {self.docked}")
                self.ready_to_drop = True  # Set flag to allow payload drop after docking
            else:
                self.get_logger().warn("Docking failed.")
        except Exception as e:
            self.get_logger().error(f"Docking service call failed: {e}")

    def wait_for_docking(self):
        """Wait until the docking process completes and triggers payload drop after docking."""
        self.get_logger().info("Waiting for docking to complete and drop payload...")
        while not (self.docked and self.ready_to_drop):
            rclpy.spin_once(self, timeout_sec=0.1)  # Allows other callbacks to be processed

        # Drop the payload only after docking is fully completed
        if self.ready_to_drop and self.received:
            # print("Hii from conveyor")
            self.payload_client(False, True)  # Trigger payload drop
            self.wait_for_payload()  # Ensure payload is dropped before continuing
            self.ready_to_drop = False  # Reset ready_to_drop for next operation
            self.docked = False

    def wait_for_payload(self):
        """Wait until payload operation is complete."""
        self.get_logger().info("Waiting for payload operation to c,omplete...")
        rate = self.create_rate(2, self.get_clock())

        while not self.received:
            self.get_logger().info(f"Waiting for box... Box received: {self.received}" )
            rclpy.spin_once(self, timeout_sec=0.1)
            # rate.sleep()
        # while not self.received:
        #     rclpy.spin_once(self, timeout_sec=0.1)  # Allows other callbacks to be processed

    def navigate_to_waypoints(self):
        self.navigator.waitUntilNav2Active()
        # start_pt = [0.370323, -2.250024, 3.140075]
        start_pt = [0.709031, -2.408234, 3.117550]
        # start_pt = [0.46002, -2.508678, 3.14] # good for coming back from CBs
        """
        1. 0.30002 -2.508678
        2. 0.30002, -2.488678
        3 0.28002, -2.508678
        4. 0.31502, -2.510678 best so far
        """
        # start_pt = [0.31502, -2.510678, 3.14] 
        # start_pt = [0.436524, -2.200024, 3.340075]

        avg = [0.245323,-2.405700,3.160360]
        start_pt_after_dock = [0.435323, -2.605700, 3.160360] # jigaad pt
        
        # Dont touch these
        # cb_2 = [2.34, 2.80, -1.57]
        # cb_1 = [-4.4, 2.65, -1.57]

        # cb_1 = [-4.65, 3.10, -1.57]
        cb_2 = [2.32, 2.75, -1.57]

        # cb_2 = [2.32, 2.70, -1.57]
        cb_1 = [-4.7, 3.11, -1.80]

        goal_poses = [
            self.create_pose(start_pt),
            self.create_pose(cb_2),
            self.create_pose(start_pt_after_dock),
            self.create_pose(cb_1)
        ]

        # Navigate to the first goal and receive the payload

        """
        Change this part of the code and make other changes as required. First move to the arm, once the 
        motion is successfull then call the passing service to get the box, after getting the box move 
        to the docking station
        """
        if self.navigate_to(goal_poses[0]):
            # self.payload_client(True, False)
            self.docking_client(3.16)
            self.wait_for_docking()
            # print(self.docked)
            if self.docked:
                self.passing_client(True)
                self.wait_for_payload()  # Wait until payload is confirmed
               

            if self.received:
                if self.navigate_to(goal_poses[1]):
                    self.docking_client(-1.57)
                
                self.wait_for_docking()
                self.docked = False
            # if self.received:
            #     print(self.box_name)  # Print the received box name
            #     self.received = False  # Reset received flag

                # # Navigate based on box type
                # if self.box_name == 'box1':
                #     if self.navigate_to(goal_poses[1]):
                # elif self.box_name == 'box2':
                #     if self.navigate_to(goal_poses[3]):
                #         self.docking_client(-1.47)
                # self.wait_for_docking()  # Wait for docking completion and payload drop
                # self.docked = False  # Reset docked flag for the next docking

        # Navigate to the next goal and repeat payload operation
        # if self.navigate_to(goal_poses[2]):
        #     self.payload_client(True, False)
        #     self.wait_for_payload()  # Wait until payload is confirmed

        #     if self.received:
        #         print(self.box_name)  # Print the received box name
        #         self.received = False  # Reset received flag

        #         # Navigate based on box type
        #         if self.box_name == 'box1':
        #             if self.navigate_to(goal_poses[1]):
        #                 self.docking_client(-1.57)
        #         elif self.box_name == 'box2':
        #             if self.navigate_to(goal_poses[3]):
        #                 self.docking_client(-1.47)
        #         self.wait_for_docking()  # Wait for docking completion and payload drop
        #         # self.docked = False  # Reset docked flag for the next docking

        self.navigator.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    node = EbotNavigationAndDocking()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Start navigation to waypoints
          # Start navigation to waypoints
        node.navigate_to_waypoints()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()