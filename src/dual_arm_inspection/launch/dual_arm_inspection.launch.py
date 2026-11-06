"""双腕の外観検査セルを起動する: robot_state_publisher（xacro）+ セルの動き + 疑似カメラ + 検査 + RViz。

    ros2 launch dual_arm_inspection dual_arm_inspection.launch.py
    ros2 launch dual_arm_inspection dual_arm_inspection.launch.py rviz:=false speed:=2.0 seed:=3

xacro は Python の xacro モジュールでここで展開する（Windows で `xacro` コマンドを外から呼ばなくてよい）。
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("dual_arm_inspection")
    robot_description = xacro.process_file(os.path.join(share, "urdf", "dual_arm_cell.urdf.xacro")).toxml()
    rviz_config = os.path.join(share, "rviz", "dual_arm_inspection.rviz")

    args = [
        DeclareLaunchArgument("rviz", default_value="true", description="RViz を開く"),
        DeclareLaunchArgument("speed", default_value="1.0", description="動きの速さの倍率"),
        DeclareLaunchArgument("seed", default_value="1", description="疑似カメラの乱数の種"),
        DeclareLaunchArgument("p_defect", default_value="0.3", description="欠陥を入れる確率"),
        DeclareLaunchArgument("p_ood", default_value="0.05", description="範囲外の照明にする確率"),
        DeclareLaunchArgument("parts", default_value="", description="検査する部品（カンマ区切り、空なら全部）"),
    ]
    num = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)  # noqa: E731
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", emulate_tty=True,
             parameters=[{"robot_description": ParameterValue(robot_description, value_type=str)}]),
        Node(package="dual_arm_inspection", executable="cell_node", output="screen", emulate_tty=True,
             parameters=[{"speed": num("speed"),
                          "parts": ParameterValue(LaunchConfiguration("parts"), value_type=str)}]),
        Node(package="dual_arm_inspection", executable="camera_node", output="screen", emulate_tty=True,
             parameters=[{"seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                          "p_defect": num("p_defect"), "p_ood": num("p_ood")}]),
        Node(package="dual_arm_inspection", executable="inspector_node", output="screen", emulate_tty=True),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config], output="screen", emulate_tty=True,
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
    return LaunchDescription(args + nodes)
