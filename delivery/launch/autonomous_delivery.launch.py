from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_drone_control_package',
            executable='your_drone_control_node',
            name='drone_control_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='your_path_planning_package',
            executable='your_path_planning_node',
            name='path_planning_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='your_drone_state_monitoring_package',
            executable='your_drone_state_monitoring_node',
            name='drone_state_monitoring_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='your_marker_recognition_package',
            executable='your_marker_recognition_node',
            name='marker_recognition_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='your_drone_marker_interaction_package',
            executable='your_drone_marker_interaction_node',
            name='drone_marker_interaction_node',
            output='screen',
            parameters=[]
        ),
    ])

