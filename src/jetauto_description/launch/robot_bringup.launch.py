import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    lidar_frame = LaunchConfiguration('lidar_frame', default='lidar_frame')
    astra_launch = LaunchConfiguration('astra_launch', default='astra_pro.launch.xml')
    enable_point_cloud = LaunchConfiguration('enable_point_cloud', default='true')
    display = LaunchConfiguration('display', default=os.environ.get('DISPLAY', ':0'))
    default_rviz_config = os.path.join(
        get_package_share_directory('jetauto_description'),
        'rviz',
        'sensors.rviz',
    )
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config)

    # Environment variables required by jetauto_description xacro
    env = [
        SetEnvironmentVariable('LIDAR_TYPE', 'A1'),
        SetEnvironmentVariable('MACHINE_TYPE', 'JetAuto'),
        SetEnvironmentVariable('DEPTH_CAMERA_TYPE', 'Astra'),
        SetEnvironmentVariable('need_compile', 'True'),
        SetEnvironmentVariable('DISPLAY', display),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            '/home/jcallano/ros2_ws/libuvc_install/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
        ),
    ]

    # robot_description (TF tree + static camera_link bridge)
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('jetauto_description'),
                'launch',
                'robot_description.launch.py',
            )
        ),
        launch_arguments={
            'use_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': 'false',
        }.items(),
    )

    # RPLidar A1
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py',
            )
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'frame_id': lidar_frame,
        }.items(),
    )

    # STM32 bridge (ROS robot controller)
    controller_port = LaunchConfiguration('controller_port', default='/dev/ttyACM0')
    controller_baudrate = LaunchConfiguration('controller_baudrate', default='1000000')
    controller_timeout = LaunchConfiguration('controller_timeout', default='5.0')
    controller_reset_dtr_rts = LaunchConfiguration('controller_reset_dtr_rts', default='true')
    controller_reset_pulse_ms = LaunchConfiguration('controller_reset_pulse_ms', default='100')
    controller_reset_post_ms = LaunchConfiguration('controller_reset_post_ms', default='500')
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_robot_controller'),
                'launch',
                'ros_robot_controller.launch.py',
            )
        ),
        launch_arguments={
            'port': controller_port,
            'baudrate': controller_baudrate,
            'timeout': controller_timeout,
            'reset_dtr_rts': controller_reset_dtr_rts,
            'reset_pulse_ms': controller_reset_pulse_ms,
            'reset_post_ms': controller_reset_post_ms,
        }.items(),
    )

    # Astra camera (depth + IR + pointcloud via OpenNI, color disabled - handled by v4l2_camera)
    astra = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('astra_camera'),
                'launch',
                astra_launch,
            ])
        ),
        launch_arguments={
            'enable_color': 'false',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '15',
            'enable_ir': 'true',
            'ir_width': '640',
            'ir_height': '480',
            'ir_fps': '30',
            'enable_point_cloud': enable_point_cloud,
            'use_uvc_camera': 'false',
        }.items(),
    )

    # Astra RGB color via kernel uvcvideo (/dev/video20) - avoids libuvc conflict
    color_camera = Node(
        package='jetauto_description',
        executable='astra_color_node.py',
        name='color',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0',
            'width': 640,
            'height': 480,
            'fps': 30,
            'frame_id': 'camera_color_optical_frame',
        }],
    )

    # Mecanum odometry publisher
    odom_publisher = Node(
        package='controller',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('controller'),
                'config',
                'calibrate_params.yaml',
            )
        ],
    )

    # RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value=use_rviz),
        DeclareLaunchArgument('lidar_port', default_value=lidar_port),
        DeclareLaunchArgument('lidar_frame', default_value=lidar_frame),
        DeclareLaunchArgument('astra_launch', default_value=astra_launch),
        DeclareLaunchArgument('enable_point_cloud', default_value=enable_point_cloud),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('display', default_value=display),
        DeclareLaunchArgument('controller_port', default_value=controller_port),
        DeclareLaunchArgument('controller_baudrate', default_value=controller_baudrate),
        DeclareLaunchArgument('controller_timeout', default_value=controller_timeout),
        DeclareLaunchArgument('controller_reset_dtr_rts', default_value=controller_reset_dtr_rts),
        DeclareLaunchArgument('controller_reset_pulse_ms', default_value=controller_reset_pulse_ms),
        DeclareLaunchArgument('controller_reset_post_ms', default_value=controller_reset_post_ms),
        *env,
        robot_description,
        rplidar,
        controller_launch,
        astra,
        color_camera,
        odom_publisher,
        rviz,
    ])
