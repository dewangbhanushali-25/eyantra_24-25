from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ls = LaunchDescription()

    car_node = Node(
        package="task4",
        executable="navigation"
        
    )

    docking_node = Node(
        package="task4",
        executable="docking"
    )


    passing_node = Node(
        package="task4",
        executable="passing"
    )

    ls.add_action(car_node)
    ls.add_action(docking_node)
    ls.add_action(passing_node)


    return ls