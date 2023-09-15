from setuptools import setup
import os
import glob

package_name = 'delivery'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))), #share dir = share/my_package,    //빌드를 할 때마다 디렉토리 위치를 바꾸지 않기 위해 자동으로 불러오는 기능
        # ('share/my_package/launch/test.launch.py')
        (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='jun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward = delivery.forward:main',
            'offboard = delivery.offboard:main',
            'standing = delivery.standing:main',
            'realsense = delivery.realsense:main',
            'lidar = delivery.lidar:main',
            'forward_realworld = delivery.forward_realworld:main',
            'circle_realworld = delivery.circle_realworld:main',
            'waypoint = delivery.waypoint:main',
            'test = delivery.test:main',
            'standing_pid = delivery.standing_pid:main',
            'waypoint_pid = delivery.waypoint_pid:main',
            'speed = delivery.speed:main',
            'multy = delivery.multy:main',
            'multy_waypoint = delivery.multy_waypoint:main',
            'multy_square = delivery.multy_square:main',
            'lidar_key = delivery.lidar_key:main',
            'trans = delivery.trans:main',
            'bridge = delivery.bridge:main',
            'outside = delivery.outside:main',
            'get_gps = delivery.get_gps:main',
            'get_gps_sub = delivery.get_gps_sub:main',
            'test_developing = delivery.test_developing:main',
            'test_gps_point = delivery.test_gps_point:main',
        ],
    },
)
