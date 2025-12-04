from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_ds',
            executable='turtlesim_node',
            output='screen'
        ),
        Node(
            package='turtle_demo',
            executable='turtle_tf2_frame_publisher',
            name='broadcaster1',
            parameters=[
                {'turtlename' : 'turtle1'}
            ]
        ),
        Node(
            package='turtle_demo',
            executable='turtle_tf2_frame_publisher',
            name='broadcaster2',
            parameters=[
                {'turtlename' : 'turtle2'}
            ]
        ),
        Node(
            package='turtle_demo',
            executable='turtle_spawner',
            name='turtle_spawner'
        ),
        Node(
            package='turtle_demo',
            executable='turtle_BT',
            output='screen',
            arguments=['/home/sheneman/thesis/turtle_ws/src/turtle_demo/behavior_trees/control_tree.xml'],
            ros_arguments=['--log-level', 'warn']
        ),
        Node(
            package='turtle_demo',
            executable='turtle_service_handler',
            name='turtle_service_handler'
        ),
        Node(
            package='turtle_demo',
            executable='turtle_monitor',
            name='turtle_monitor'
        )
    ])