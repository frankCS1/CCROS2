from setuptools import setup

package_name = 'my_robot_vision_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/vision_and_traffic_launch.py',
            'launch/rtabmap_slam_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frank Motsi',
    maintainer_email='frank.motsi@example.com',
    description='ROS2 package for object detection, traffic handling, navigation, goal detection and landmark logging',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = my_robot_vision_pkg.vision_node:main',
            'traffic_node = my_robot_vision_pkg.traffic_node:main',
            'control_node = my_robot_vision_pkg.control_node:main',
            'nav_node = my_robot_vision_pkg.nav_node:main',
            'goal_node = my_robot_vision_pkg.goal_node:main',
            'landmark_logger = my_robot_vision_pkg.landmark_logger:main',
        'wall_follow_node = my_robot_vision_pkg.wall_follow_node:main'
        ],
    },
)
