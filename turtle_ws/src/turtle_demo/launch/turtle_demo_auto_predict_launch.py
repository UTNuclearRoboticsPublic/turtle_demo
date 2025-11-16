from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def namespace_launch(context):
    num_groups = LaunchConfiguration("num_groups").perform(context)

    launch_groups = []
    for i in range(int(num_groups)):
        namespace = f"group{i}"
        turtle_BT = Node(
            package='turtle_demo',
            executable='turtle_BT_predict',
            name='turtle_BT',
            namespace=namespace,
            output='screen',
            arguments=['/home/sheenan/thesis/turtle_ws/src/turtle_demo/behavior_trees/turtle_tree_predictive.xml'],
            ros_arguments=['--log-level', 'warn']
        )

        kill_turtle1 = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                namespace,
                '/kill ',
                'turtlesim_ds/srv/Kill ',
                '"{name: turtle1}"'
            ]],
            shell=True
        )

        launch_groups += [
            turtle_BT,
            Node(
                package='turtlesim_ds',
                executable='turtlesim_node',
                namespace=namespace,
                output='screen'
            ),
            Node(
                package='turtle_demo',
                executable='turtle_tf2_frame_publisher',
                name='broadcaster1',
                namespace=namespace,
                parameters=[
                    {'turtlename' : 'turtle1'}
                ]
            ),
            Node(
                package='turtle_demo',
                executable='turtle_tf2_frame_publisher',
                name='broadcaster2',
                namespace=namespace,
                parameters=[
                    {'turtlename' : 'turtle2'}
                ]
            ),
            Node(
                package='turtle_demo',
                executable='turtle_spawner',
                name='turtle_spawner',
                namespace=namespace,
            ),
            Node(
                package='turtle_demo',
                executable='turtle_service_handler',
                name='turtle_service_handler',
                namespace=namespace
            ),
            Node(
                package='turtle_demo',
                executable='turtle_monitor',
                name='turtle_monitor',
                namespace=namespace,
                parameters=[
                    {'csv_path' : '/home/sheenan/thesis/turtle_auto_predict.csv'}
                ]
            ),
            Node(
                package='turtle_demo',
                executable='turtle_autopilot',
                name='turtle_autopilot',
                namespace=namespace
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=turtle_BT,
                    on_exit=kill_turtle1
                )
            )
        ]

    return launch_groups

def generate_launch_description():
    num_groups_launch_arg = DeclareLaunchArgument("num_groups", default_value='1')

    opaque_launch_groups = OpaqueFunction(function=namespace_launch)

    ld = LaunchDescription([num_groups_launch_arg])
    ld.add_action(opaque_launch_groups)

    return ld