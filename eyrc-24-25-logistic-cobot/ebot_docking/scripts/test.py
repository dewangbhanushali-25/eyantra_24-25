import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from ebot_nav2_cmd_task2b import EbotNavigationAndDocking
from ebot_passing_service import aruco_tf
from ebot_docking_service_task2b import EbotDockingService
import time

def main():
    rclpy.init()

    # Events to synchronize node execution
    nav2_ready = threading.Event()
    passing_ready = threading.Event()
    docking_ready = threading.Event()
    shutdown_flag = threading.Event()  # Flag to signal threads to terminate

    # Initialize the nodes
    nav2_node = EbotNavigationAndDocking()
    passing_node = aruco_tf()
    docking_node = EbotDockingService()

    # Synchronization logic
    def nav2_task():
        while not shutdown_flag.is_set():
            nav2_ready.wait()
            nav2_node.get_logger().info("Starting navigation to position...")
            nav2_node.navigate_to_waypoints()
            nav2_ready.clear()
            passing_ready.set()
            nav2_node.get_logger().info("Navigation task complete.")

    def passing_task():
        while not shutdown_flag.is_set():
            passing_ready.wait()
            passing_node.get_logger().info("Passing Service starting...")
            rclpy.spin_once(passing_node)
            passing_ready.clear()
            docking_ready.set()
            passing_node.get_logger().info("Passing Service task complete.")

    def docking_task():
        while not shutdown_flag.is_set():
            docking_ready.wait()
            docking_node.get_logger().info("Docking Service starting...")
            docking_node.dock_control_callback()
            docking_ready.clear()
            nav2_ready.set()
            docking_node.get_logger().info("Docking Service task complete.")

    # MultiThreadedExecutor to handle all nodes
    executor = MultiThreadedExecutor()

    # Add nodes to the executor
    executor.add_node(nav2_node)
    executor.add_node(passing_node)
    executor.add_node(docking_node)

    # Run tasks in separate threads
    t1 = threading.Thread(target=nav2_task, name="Nav2Thread")
    t2 = threading.Thread(target=passing_task, name="PassingServiceThread")
    t3 = threading.Thread(target=docking_task, name="DockingServiceThread")

    t1.start()
    t2.start()
    t3.start()

    # Signal the first task (navigation) to start
    nav2_ready.set()

    try:
        # Spin the executor
        executor.spin()
    except KeyboardInterrupt:
        shutdown_flag.set()
        t1.join()
        t2.join()
        t3.join()
    finally:
        # Signal threads to terminate
        # shutdown_flag.set()
        # t1.join()
        # t2.join()
        # t3.join()

        # Clean up nodes and shutdown
        nav2_node.destroy_node()
        passing_node.destroy_node()
        docking_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        time.sleep(2)  # Allow time for cleanup
        print("Program terminated gracefully.")

if __name__ == '__main__':
    main()
