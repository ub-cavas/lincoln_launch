
import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "novatel_oem7_driver"

def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory(PKG), "config", cfg )
    

def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)

def get_params(cfg):
    return load_yaml(get_cfg_path(cfg))

def get_override_params():    
    try:
        oem7_param_override_path = os.environ['NOVATEL_OEM7_DRIVER_PARAM_OVERRIDES_PATH']
    except KeyError: # No overrides specified.
        return {}
        
    return load_yaml(oem7_param_override_path)

      
def arg(name, default_value, description):
    return DeclareLaunchArgument(name = name, description = description, default_value = default_value)
    

    
def generate_launch_description():

    node = Node(
        package=PKG,
        namespace='novatel/oem7',
        name='main',
        executable='novatel_oem7_driver_exe',
        
        parameters=[
                    get_params("std_msg_handlers.yaml"    ),
                    get_params("std_oem7_raw_msgs.yaml"   ),
                    get_params("std_msg_topics.yaml"      ),
                    get_params("oem7_supported_imus.yaml" ),
                    get_params("std_init_commands.yaml"   ),
                    {
                    # Refer to README for more information on the configuration parameters
                    # Receiver IO Parameters
                    'oem7_max_io_errors' : 10,
                    'oem7_msg_decoder': 'Oem7MessageDecoder',
                    'oem7_if'         : LaunchConfiguration('oem7_if'),
                    'oem7_ip_addr'    : LaunchConfiguration('oem7_ip_addr'),
                    'oem7_port'       : LaunchConfiguration('oem7_port'),
                    
                    # Topic Parameters
                    'oem7_position_source' : '', # If not set, then uses INSPVA or BESTPOS based on quality
                    'oem7_imu_rate'   : 0,
                    'oem7_odometry_zero_origin' : False,
                    'oem7_odometry_transform' : False,

                    # Debug/Other Parameters
                    'oem7_receiver_log_file' : '',
                    'oem7_decoder_log_file' : '',
                    'oem7_strict_receiver_init' : True,
                    'oem7_publish_unknown_oem7raw' : False,
                    'oem7_publish_delay' : 0.0
                    
                    },
                    get_override_params() # Must be last to override
                    ],
    
        output='screen',
    )
    
    ip_arg   = arg('oem7_ip_addr', '192.168.100.201',  'IP Address of Oem7 Receiver, e.g. 192.168.1.2')
    port_arg = arg('oem7_port',    '3005',             'TCP or UDP port, e.g. 3002')
    if_arg   = arg('oem7_if',      'Oem7ReceiverTcp',  'Interface Type: Oem7ReceiverTcp or Oem7ReceiverUdp')
    
    return LaunchDescription([ip_arg, port_arg, if_arg, 
                              node])