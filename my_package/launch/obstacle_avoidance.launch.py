from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{
                'yaml_filename': '/home/jun/map.yaml'
            }]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['controller_server', 'planner_server', 'recoveries_server', 'amcl']
            }]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[
                {'use_sim_time': True},
                '/home/jun/robot_ws/src/my_package/param/nav2_params.yaml'            
            ]
        ),
        Node(
            package='my_package',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen'
        )
    ])
