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
        # If using trigger_stoft mode, you can refer to the example file "send_trigger.py" to send soft trigger signal
        DeclareLaunchArgument('device_workmode', default_value='trigger_off'),#trigger_off / trigger_soft / trigger_hard

        # Enable the network packet retransmission function
        # If device workmode is in trigger mode, this feature will be automatically forced on
        DeclareLaunchArgument('gvsp_resend', default_value='false'),

        # Enable device auto try reconnect while offline
        # You can refer to the example file "offline_detect.py" to detect camera offline events, regardless of whether this switch is turned on or off.
        DeclareLaunchArgument('device_auto_reconnect', default_value='false'),

        # Enable rgb stream output
        DeclareLaunchArgument('color_enable', default_value='true'),
        DeclareLaunchArgument('color_resolution', default_value='"1280x960"'),
        #format list:yuv / jpeg / bayer / mono...
        #DeclareLaunchArgument('color_format', default_value='"yuv"'),

        
        # Used to set the area of interest for color camera auto exposure
        # This setting requires that the camera itself supports this feature, otherwise the setting is invalid
        #DeclareLaunchArgument('color_aec_roi', default_value='0.0.1280.960'),
        
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),

        # Enable depth stream output
        DeclareLaunchArgument('depth_enable', default_value='true'),
        DeclareLaunchArgument('depth_resolution', default_value='"640x480"'),

        #format list:depth16/xyz48...
        #DeclareLaunchArgument('depth_format', default_value='"xyz48"'),

        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),

        # Map depth image to color coordinate
        DeclareLaunchArgument('depth_registration_enable', default_value='true'),

        #Speckle filtering enable/disable switch
        DeclareLaunchArgument('depth_speckle_filter', default_value='false'),
        #Blob size smaller than this will be removed
        DeclareLaunchArgument('max_speckle_size', default_value='150'),
        #Maximum difference between neighbor disparity pixels
        DeclareLaunchArgument('max_speckle_diff', default_value='64'),
  
        #depth stream Time-domain filtering enable/disable switch
        DeclareLaunchArgument('depth_time_domain_filter', default_value='false'),
        #Time-domain filtering frame count：2 - 10
        DeclareLaunchArgument('depth_time_domain_num', default_value='3'),

        # Enable point cloud stream output
        DeclareLaunchArgument('point_cloud_enable', default_value='false'),

        # Enable color point cloud stream,  
        # depth_registration_enable will be automatically set to true
        # point_cloud_enable will be automatically set to false
        DeclareLaunchArgument('color_point_cloud_enable', default_value='true'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),

        DeclareLaunchArgument('left_ir_enable', default_value='false'),
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
