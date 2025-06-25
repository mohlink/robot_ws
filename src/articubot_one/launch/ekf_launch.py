from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_se_node',
            output='screen',
            parameters=['/home/ubuntu/robot_ws/src/articubot_one/config/ekf.yaml'],
            remappings=[('diff_cont/odom', 'diff_cont/odom'), ('imu/data', 'imu/data')]
        ),
    ])
