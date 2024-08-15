import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'rtcrobot_hins_camera'
    package_share_directory = get_package_share_directory(package_name)

    param_file = os.path.join(package_share_directory, 'config', 'param.yaml')

    hins_camera_node = Node(
        package='rtcrobot_hins_camera',
        executable='hins_camera_node',
        name='hins_camera_node',
        parameters=[param_file],
        output='screen'
    )

    nav2_life_cycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['hins_camera_node']}]
    )

    return LaunchDescription([hins_camera_node, nav2_life_cycle_manager])
