from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='galactic_chase_object',
            executable='find_object',
            name='find_object'
        ),
        Node(
            package='galactic_chase_object',
            executable='get_object_range',
            name='get_object_range'
        ),
        Node(
            package='galactic_chase_object',
            executable='chase_object',
            name='chase_object',
            parameters=[
                {'kp_r' : 0.7},
                {'ki_r' : 0.0},
                {'kd_r' : 0.0},
                {'kp_t' : 0.01},
                {'ki_t' : 0.0},
                {'kd_t' : 0.0},
            ]
        ),
    ])