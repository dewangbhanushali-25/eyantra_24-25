from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ls = LaunchDescription()

    car_node = Node(
        package="ebot_docking",
        exec_name="ebot_nav2_cmd_task2b"
        
    )

    docking_node = Node(
        package="ebot_docking",
        exec_name="ebot_docking_service_task2b"
    )


    passing_node = Node(
        package="ebot_docking",
        exec_name="ebot_passing_service"
    )

    ls.add_action(car_node)
    ls.add_action(docking_node)
    ls.add_action(passing_node)


    return ls