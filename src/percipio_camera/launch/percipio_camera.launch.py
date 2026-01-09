from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    param_file = PathJoinSubstitution([
        FindPackageShare('percipio_camera'),
        'launch/parameters.xml'
    ]).perform(context)

    # read xml
    if os.path.exists(param_file):
        with open(param_file, 'r') as f:
            xml_content = f.read()
    else:
        xml_content = ""  # clear
        print(f"Warning: Parameter file not found: {param_file}")
    
    parameters = {}
    
    bool_params = [
        'device_log_enable',
        'frame_rate_control',
        'device_auto_reconnect',
        'color_enable',
        'depth_enable',
        'depth_registration_enable',
        'depth_speckle_filter',
        'depth_time_domain_filter',
        'point_cloud_enable',
        'color_point_cloud_enable',
        'left_ir_enable'
    ]
    
    int_params = [
        'device_log_server_port',
        'max_speckle_size',
        'max_speckle_diff',
        'depth_time_domain_num',
        'tof_modulation_threshold',
        'tof_jitter_threshold',
        'tof_filter_threshold',
        'tof_channel',
        'tof_HDR_ratio'
    ]
    
    float_params = [
        'frame_rate',
        'max_physical_size'
    ]
    
    for arg in args_list:
        param_name = arg.name
        param_value = LaunchConfiguration(param_name).perform(context)
        
        if isinstance(param_value, str) and param_value.startswith('"') and param_value.endswith('"'):
            param_value = param_value[1:-1]
        
        if param_name in bool_params:
            if param_value.lower() in ['true', '1', 'yes', 'on']:
                parameters[param_name] = True
            elif param_value.lower() in ['false', '0', 'no', 'off']:
                parameters[param_name] = False
            else:
                default_value = arg.default_value[0].perform(context) if hasattr(arg.default_value, '__getitem__') else str(arg.default_value)
                if default_value.lower() in ['true', '1', 'yes', 'on']:
                    parameters[param_name] = True
                else:
                    parameters[param_name] = False
                print(f"Warning: Could not convert {param_name}='{param_value}' to boolean, using default: {parameters[param_name]}")
        
        elif param_name in int_params:
            try:
                parameters[param_name] = int(param_value)
            except ValueError:
                try:
                    default_value = arg.default_value[0].perform(context) if hasattr(arg.default_value, '__getitem__') else str(arg.default_value)
                    parameters[param_name] = int(default_value)
                except:
                    parameters[param_name] = 0
                print(f"Warning: Could not convert {param_name}='{param_value}' to integer, using default: {parameters[param_name]}")
        
        elif param_name in float_params:
            try:
                parameters[param_name] = float(param_value)
            except ValueError:
                try:
                    default_value = arg.default_value[0].perform(context) if hasattr(arg.default_value, '__getitem__') else str(arg.default_value)
                    parameters[param_name] = float(default_value)
                except:
                    parameters[param_name] = 0.0
                print(f"Warning: Could not convert {param_name}='{param_value}' to float, using default: {parameters[param_name]}")
        
        else:
            parameters[param_name] = param_value
    
    parameters['camera_parameter'] = xml_content
    
    compose_node = ComposableNode(
        package='percipio_camera',
        plugin='percipio_camera::PercipioCameraNodeDriver',
        name=parameters.get('camera_name', 'camera'),
        namespace='',
        parameters=[parameters],
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
    
    return [
        GroupAction([
            PushRosNamespace(parameters.get('camera_name', 'camera')),
            container
        ])
    ]

args_list = []

def generate_launch_description():
    global args_list
    
    # Declare arguments
    args_list = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('device_ip', default_value=''),

        # Device log configuration
        DeclareLaunchArgument('device_log_enable', default_value='false'),
        # Device log level VERBOSE / DEBUG / INFO / WARNING / ERROR / NEVER
        DeclareLaunchArgument('device_log_level', default_value='WARNING'),
        DeclareLaunchArgument('device_log_server_port', default_value='9001'),

        # Whether to enable frame rate control for device output images
        DeclareLaunchArgument('frame_rate_control', default_value='false'),
        #  Frame rate parameter for device output images (floating point number)
        DeclareLaunchArgument('frame_rate', default_value='5.0'),

        # Setup device work mode
        # If using trigger_stoft mode, you can refer to the example file "send_trigger.py" to send soft trigger signal
        # If frame_rate_control is enabled, then trigger_mode will be deactivated.-->
        DeclareLaunchArgument('device_workmode', default_value='trigger_off'),#trigger_off / trigger_soft / trigger_hard

        # Enable the network packet retransmission function
        # DeclareLaunchArgument('gvsp_resend', default_value='false'),

        # Enable device auto try reconnect while offline
        # You can refer to the example file "offline_detect.py" to detect camera offline events, regardless of whether this switch is turned on or off.
        DeclareLaunchArgument('device_auto_reconnect', default_value='true'),

        # Enable rgb stream output
        DeclareLaunchArgument('color_enable', default_value='true'),
        DeclareLaunchArgument('color_resolution', default_value='640x480'),
        #format list:yuv / jpeg / bayer / mono...
        #DeclareLaunchArgument('color_format', default_value='yuv'),

        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),

        # Enable depth stream output
        DeclareLaunchArgument('depth_enable', default_value='true'),
        DeclareLaunchArgument('depth_resolution', default_value='640x400'),

        #format list:depth16/xyz48...
        #DeclareLaunchArgument('depth_format', default_value='xyz48'),

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
        #Maximum Speckle Physical Size to be Filtered-Out, uint is mm^2
        DeclareLaunchArgument('max_physical_size', default_value='20.0'),
  
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
    ]

    ld = LaunchDescription(args_list + [OpaqueFunction(function=launch_setup)])
    return ld