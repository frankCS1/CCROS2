from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start RTAB-Map SLAM node (subscribing to camera and odometry topics)
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'approx_sync': True,
                'queue_size': 10,
                'subscribe_stereo': False,
                'Rtabmap/TimeThr': 700,
                'Rtabmap/DetectionRate': 1.0,
                'RGBD/LinearUpdate': 0.5,
                'RGBD/AngularUpdate': 0.05,
                'Reg/Force3DoF': True,
                'Optimizer/GravitySigma': 0.0
            }],
            remappings=[
                ('rgb/image', '/atlas/rgbd_camera/image'),
                ('depth/image', '/atlas/rgbd_camera/depth/image'),
                ('rgb/camera_info', '/atlas/rgbd_camera/camera_info'),
                ('odom', '/odom')
            ]
        )
    ])
