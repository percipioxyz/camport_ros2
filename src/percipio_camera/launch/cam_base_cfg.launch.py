import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value='""'),
        DeclareLaunchArgument('device_ip', default_value=''),
        DeclareLaunchArgument('depth_registration_enable', default_value='true'),

        DeclareLaunchArgument('point_cloud_enable', default_value='true'),
        DeclareLaunchArgument('color_point_cloud_enable', default_value='true'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),

        DeclareLaunchArgument('color_enable', default_value='true'),
        DeclareLaunchArgument('color_resolution', default_value='"1280x960"'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),

        DeclareLaunchArgument('depth_enable', default_value='true'),
        DeclareLaunchArgument('depth_resolution', default_value='"640x480"'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),

        DeclareLaunchArgument('left_ir_enable', default_value='true'),
        DeclareLaunchArgument('left_ir_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),
    ]

    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]
    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "foxy":
        return LaunchDescription(
            args
            + [
                Node(
                    package="percipio_camera",
                    executable="percipio_camera_node",
                    name="percipio_camera_node",
                    namespace=LaunchConfiguration("camera_name"),
                    parameters=parameters,
                    output="screen",
                )
            ]
        )
    # Define the ComposableNode
    else:
        # Define the ComposableNode
        compose_node = ComposableNode(
            package="percipio_camera",
            plugin="percipio_camera::PercipioCameraNodeDriver",
            name=LaunchConfiguration("camera_name"),
            namespace="",
            parameters=parameters,
        )
        # Define the ComposableNodeContainer
        container = ComposableNodeContainer(
            name="camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                compose_node,
            ],
            output="screen",
        )
        # Launch description
        ld = LaunchDescription(
            args
            + [
                GroupAction(
                    [PushRosNamespace(LaunchConfiguration("camera_name")), container]
                )
            ]
        )
        return ld
