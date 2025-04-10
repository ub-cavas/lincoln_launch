from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def load_parameters_from_yaml(absolute_path):
    with open(absolute_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    package_name = 'universe_dbw2_bridge'
    executable_name = 'Lincoln_MKZ_Bridge'

    # Get actual path to the YAML file using FindPackageShare and os.path.join
    package_share_path = FindPackageShare(package=package_name).find(package_name)
    yaml_file_path = os.path.join(package_share_path, 'config', 'vehicle_params.yaml')

    # Load the YAML config
    parameters = load_parameters_from_yaml(yaml_file_path)

    return LaunchDescription([
        Node(
            package=package_name,
            executable=executable_name,
            name=executable_name,
            output='screen',
            parameters=[parameters]
        )
    ])