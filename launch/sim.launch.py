"""
Gazebo(Fortress) + ROS 2ブリッジ をまとめて起動する launch ファイル

  ros2 launch pai_robot sim.launch.py
  （仮想マシンで画面が真っ黒/落ちる場合）
  ros2 launch pai_robot sim.launch.py render_engine:=ogre
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(get_package_share_directory("pai_robot"), "worlds", "pai_world.sdf")
    render_engine = LaunchConfiguration("render_engine")

    gazebo = ExecuteProcess(
        # -r : 起動と同時にシミュレーション開始（▶を押し忘れて動かない問題を防ぐ）
        cmd=["ign", "gazebo", "-r", "-v", "3", "--render-engine", render_engine, world],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # ROS 2 -> Gazebo : 速度指令
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            # Gazebo -> ROS 2 : オドメトリ
            "/model/vehicle/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
        remappings=[("/model/vehicle/odometry", "/odom")],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("render_engine", default_value="ogre2",
                              description="ogre2(標準) / ogre(VM向け・軽い)"),
        gazebo,
        bridge,
    ])
