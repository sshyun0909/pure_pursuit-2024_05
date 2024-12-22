from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    param_file = os.path.join(get_package_share_directory('pure_pursuit_pkg'), 'config', 'pure_pursuit_params.yaml')

    pure_pursuit_node = Node(
        package='pure_pursuit_pkg',
        executable='pure_pursuit',
        name='pure_pursuit_node',
        parameters=[param_file]
    )

    # emergency_brake_node = Node(
    #     package='pure_pursuit_pkg',
    #     executable='emergency_brake',
    #     parameters=[
    #         {"break_threshold": brake_threshold}
    #     ]
    # )

    ld = [
        pure_pursuit_node,
    ]

    return LaunchDescription(ld)
