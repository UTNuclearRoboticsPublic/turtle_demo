from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'),
        Node(
            package='turtle_demo',
            executable='turtle_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename' : 'turtle1'}
            ]
        ),
        Node(
            package='turtle_demo',
            executable='turtle_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename' : 'turtle2'}
            ]
        ),
        Node(
            package='turtle_demo',
            executable='turtle_listener',
            name='listener',
            parameters=[
                {'target_frame' : 'turtle1'}
            ]
        )
    ])