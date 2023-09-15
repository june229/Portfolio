from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  param_dir = LaunchConfiguration(      #변수 설정 param위치 설정.
      'param_dir',
      default=os.path.join(
          get_package_share_directory('my_package'),
          'param',
          'turtlesim.yaml'))

  return LaunchDescription([
    DeclareLaunchArgument(  #parameter를 쓰기 위해 선언. 3개 쓸려면 3개 선언.
      'param_dir',
      default_value=param_dir,
    ),

    Node(
      package='turtlesim',
      executable='turtlesim_node',      #node 생성하여 node 실행 및 parameter 불러와 적용
      parameters=[param_dir]
    ),

		Node(
    package='my_package',
    executable='moveturtle2',
    ),

    ExecuteProcess(
        cmd=[[ #명령어
            'ros2 service call ',
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 3, y: 2, theta: 0.2, name: ''turtle2''}"']],
            shell=True),    #cmd 라인에서 '하나를 쓰면 앞에 것과 연동되기에 이를 구분하기 위해 '를 두개 입력한다. ''

    ExecuteProcess(
        cmd=[[ #명령어
            'ros2 service call ',
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 4, y: 3, theta: 0.2, name: ''turtle3''}"']],
            shell=True),

    ExecuteProcess(
        cmd=[[ #명령어
            'ros2 service call ',
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 5, y: 4, theta: 0.2, name: ''turtle4''}"']],
            shell=True),

    ExecuteProcess(
        cmd=[[ #명령어
            'ros2 service call ',
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 6, y: 5, theta: 0.2, name: ''turtle5''}"'
        ]],
        shell=True)   #터미널을 shell이라고 부르기도 한다. 터미널에서 나오는 내용을 보기 싫으면 False라고 하면 된다.
	])
