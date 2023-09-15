from setuptools import setup
import os
import glob

package_name = 'my_package'
#package_name = 'topic_service_action_rclpy_example'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))), #share dir = share/my_package,    //빌드를 할 때마다 디렉토리 위치를 바꾸지 않기 위해 자동으로 불러오는 기능
        # ('share/my_package/launch/test.launch.py')
        (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='mswkdh3790@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mp = my_package.mpub:main', # 명령어 mp는 my_package폴더 안의 mpub코드를 하고 사요할 함수(main) 입력
            'ms = my_package.msub:main',
            'mt = my_package.mtim:main',
            'moveturtle = my_package.move_turtle:main',
            'moveturtle2 = my_package.move_turtle2:main',
            'real_turtle = my_package.real_turtle:main',
            'moveturtle_copy = my_package.copy:main',
            'movearm = my_package.move_arm:main',
            'talker = my_package.publisher_member_function:main',
            'listener = my_package.subscriber_member_function:main',
            'lidar_avoidance = my_package.lidar_avoidance:main',
            'depth_avoidance = my_package.depth_avoidance:main',
            'complex_lidar_avoidance = my_package.complex_lidar_avoidance:main',
            'mission = my_package.mission:main',
            'goal_publisher = my_package.goal_publisher:main',
            'turtlebot_controller = my_package.turtlebot_controller:main',
            'navigate_to_pose = my_package.navigate_to_pose:main',
            'lidar_turtlesim = my_package.lidar_turtlesim:main',
            'move_circle = my_package.move_circle:main',
            'lidar = my_package.lidar:main',
            'givantake = my_package.give_and_take:main',


        ],
    },
)
