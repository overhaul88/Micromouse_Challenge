"""
Quick geometry/TF sanity check for the micromouse description, with no
Gazebo involved. Spawns robot_state_publisher off the xacro and a
joint_state_publisher_gui so you can spin the wheel joints with sliders
and visually confirm wheel/caster/IR-sensor placement and TF frames
before bringing physics and sensors into the picture.

Run:
  ros2 launch micromouse_simulation view_robot.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory("micromouse_description")
    xacro_path = os.path.join(description_pkg, "urdf", "micromouse.urdf.xacro")
    rviz_config = os.path.join(description_pkg, "rviz", "view_robot.rviz")

    # controllers_yaml is left blank here on purpose - the ros2_control /
    # Gazebo plugin block in the xacro is inert without Gazebo running, so
    # robot_state_publisher just ignores it for this geometry-only check.
    robot_description = ParameterValue(
        Command(["xacro ", xacro_path, " controllers_yaml:=", ""]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
