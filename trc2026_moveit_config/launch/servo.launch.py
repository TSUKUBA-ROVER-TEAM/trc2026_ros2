import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_pkg_dir = get_package_share_directory('trc2026_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder("trc2026", package_name="trc2026_moveit_config")
        .robot_description(file_path="config/trc2026.urdf.xacro")
        .robot_description_semantic(file_path="config/trc2026.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # ロボットの姿勢をTFに反映
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'publish_frequency': 50.0}]
    )

    # 実機ドライバ（全関節のJointStateを直接 /joint_states に出す）
    bridge_node = Node(
        package="trc2026_driver",
        executable="robstride_bridge_node",
        output="screen",
    )

    # MoveIt Servo 本体
    servo_yaml = os.path.join(moveit_pkg_dir, "config", "servo_config.yaml")
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_yaml,
            {'use_sim_time': False}
        ],
        output="screen"
    )

    # RViz2
    rviz_config_file = os.path.join(moveit_pkg_dir, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False}
        ],
        output="screen"
    )

    # 固定TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint']
    )

    return LaunchDescription([
        static_tf_node,
        start_robot_state_publisher_cmd,
        bridge_node,
        servo_node,
        rviz_node
    ])