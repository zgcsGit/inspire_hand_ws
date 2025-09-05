from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    hand_dir = get_package_share_directory('hand')
    json_path = os.path.join(hand_dir, 'pickle', 'inspire_joints3.json')
    urdf_path = os.path.join(hand_dir, 'urdf', 'inspire_hand_left_dae.urdf')
    rviz_config_path = os.path.join(hand_dir, 'config', 'default2.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        Node(
            package='hand',
            executable='json_joint_publisher.py',  
            name='json_joint_publisher',
            output='screen',
            arguments=[json_path]  
        )
    ])
