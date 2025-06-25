from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'params_file': '/home/ubuntu/robot_ws/src/articubot_one/config/nav2_params.yaml'},
                {'map': '/home/ubuntu/robot_ws/map_lab1.yaml'}
            ]
        ),
    ])
