from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baudrate = LaunchConfiguration('baudrate', default='1000000')
    timeout = LaunchConfiguration('timeout', default='5.0')
    reset_dtr_rts = LaunchConfiguration('reset_dtr_rts', default='true')
    reset_pulse_ms = LaunchConfiguration('reset_pulse_ms', default='100')
    reset_post_ms = LaunchConfiguration('reset_post_ms', default='500')
    port_arg = DeclareLaunchArgument('port', default_value=port)
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value=baudrate)
    timeout_arg = DeclareLaunchArgument('timeout', default_value=timeout)
    reset_dtr_rts_arg = DeclareLaunchArgument('reset_dtr_rts', default_value=reset_dtr_rts)
    reset_pulse_ms_arg = DeclareLaunchArgument('reset_pulse_ms', default_value=reset_pulse_ms)
    reset_post_ms_arg = DeclareLaunchArgument('reset_post_ms', default_value=reset_post_ms)

    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',
        parameters=[{
            'imu_frame': imu_frame,
            'port': port,
            'baudrate': baudrate,
            'timeout': timeout,
            'reset_dtr_rts': reset_dtr_rts,
            'reset_pulse_ms': reset_pulse_ms,
            'reset_post_ms': reset_post_ms,
        }]
    )

    return LaunchDescription([
        imu_frame_arg,
        port_arg,
        baudrate_arg,
        timeout_arg,
        reset_dtr_rts_arg,
        reset_pulse_ms_arg,
        reset_post_ms_arg,
        ros_robot_controller_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
