"""launch の練習: パラメータ付きの talker と listener を1回のコマンドで起動する。

例: ros2 launch py_pubsub pubsub.launch.py greeting:=Hi period:=1.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    greeting = LaunchConfiguration("greeting")
    period = LaunchConfiguration("period")
    return LaunchDescription([
        DeclareLaunchArgument("greeting", default_value="Hello from launch"),
        DeclareLaunchArgument("period", default_value="0.5"),
        Node(
            package="py_pubsub",
            executable="param_talker",
            # launch の引数は文字列なので、period は小数として渡す
            parameters=[{"greeting": greeting, "period": ParameterValue(period, value_type=float)}],
            output="screen",
        ),
        Node(package="py_pubsub", executable="listener", output="screen"),
    ])
