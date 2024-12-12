from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value='""'),
        DeclareLaunchArgument('device_ip', default_value=''),

        # Setup device work mode
        DeclareLaunchArgument('device_workmode', default_value='trigger_off'),#trigger_off / trigger_soft / trigger_hard

        #Enable device auto try reconnect while offline
        DeclareLaunchArgument('device_auto_reconnect', default_value='false'),

        # Enable rgb stream output
        DeclareLaunchArgument('color_enable', default_value='true'),
        DeclareLaunchArgument('color_resolution', default_value='"1280x960"'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),

        # Enable depth stream output
        DeclareLaunchArgument('depth_enable', default_value='true'),
        DeclareLaunchArgument('depth_resolution', default_value='"640x480"'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),

        # Map depth image to color coordinate
        DeclareLaunchArgument('depth_registration_enable', default_value='true'),

        # Enable point cloud stream output
        DeclareLaunchArgument('point_cloud_enable', default_value='false'),

        # Enable color point cloud stream,  
        # depth_registration_enable will be automatically set to true
        # point_cloud_enable will be automatically set to false
        DeclareLaunchArgument('color_point_cloud_enable', default_value='true'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),

        DeclareLaunchArgument('left_ir_enable', default_value='true'),
        DeclareLaunchArgument('left_ir_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),

        #Tof camera features
        DeclareLaunchArgument('tof_depth_quality', default_value='medium'),  #basic / medium / high
        DeclareLaunchArgument('tof_modulation_threshold', default_value='-1'),
        DeclareLaunchArgument('tof_jitter_threshold', default_value='-1'),
		DeclareLaunchArgument('tof_filter_threshold', default_value='-1'),
        DeclareLaunchArgument('tof_channel', default_value='-1'),
        DeclareLaunchArgument('tof_HDR_ratio', default_value='-1'),

    ]

    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]

    compose_node = ComposableNode(
        package='percipio_camera',
        plugin='percipio_camera::PercipioCameraNodeDriver',
        name=LaunchConfiguration('camera_name'),
        namespace='',
        parameters=parameters,
    )

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            compose_node,
        ],
        output='screen',
    )

    ld = LaunchDescription(
        args +
        [
            GroupAction([
                PushRosNamespace(LaunchConfiguration('camera_name')),
                container
            ])
        ]
    )
    return ld