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
        ),
        Node(
            package='turtle_demo',
            executable='turtle_BT',
            name='turtle_BT',
            output='screen',
            arguments=['/home/sheneman/thesis/turtle_ws/src/turtle_demo/behavior_trees/turtle_tree.xml'],
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
            name='turtle_monitor',
            parameters=[
                {'csv_path' : '/tmp/turtle_auto.csv'}
            ]
        ),
        Node(
            package='turtle_demo',
            executable='turtle_autopilot',
            name='turtle_autopilot'
        )
    ])