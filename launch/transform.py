from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    dataForward_node = Node(
        package="transform_data",
        executable="data_forward",
        )
    
    ld.add_action(dataForward_node)

    return ld