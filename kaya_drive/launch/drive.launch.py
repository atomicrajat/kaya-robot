"""Phase 2 bringup: cmd_vel -> wheels, plus wheel odometry.

    ros2 launch kaya_drive drive.launch.py
    ros2 launch kaya_drive drive.launch.py wheel_speed_scale:=17500.0

Run alongside kaya_bringup's sensor/TF bringup for a live TF tree:

    ros2 launch kaya_bringup imu.launch.py real_joint_states:=true
    ros2 launch kaya_drive drive.launch.py

Then drive it:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.15',
                              description='m/s clamp -- raise once the calibration in '
                                          'Sec. 02 of curriculum.html is done'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.0'),
        # 0.0 => drive_node uses the analytical counts/mps derived from wheel
        # radius and the 4096-count/rev encoder. Override once the Phase 2
        # measured-2m-roll calibration gives a corrected value.
        DeclareLaunchArgument('wheel_speed_scale', default_value='0.0'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='0.5'),

        Node(
            package='kaya_drive', executable='drive_node', name='drive_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'wheel_speed_scale': LaunchConfiguration('wheel_speed_scale'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            }],
        ),
    ])
