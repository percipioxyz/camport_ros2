import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess

def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('percipio_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'cam_base_cfg.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_0',
            'serial_number': '"207000113334"',
            'device_ip' : '192.168.120.112',
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'cam_base_cfg.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_1',
            'serial_number': '"207000148668"',
            'device_ip' : '192.168.120.187',
        }.items()
    )

    #add more launch_include here
    #launch3_include = ...

    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld