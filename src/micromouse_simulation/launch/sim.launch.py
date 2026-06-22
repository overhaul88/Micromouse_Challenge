"""
Full Gazebo bring-up for the micromouse: starts gz sim with the maze
world, publishes robot_description, spawns the robot at the maze start
cell, brings up the IR sensor bridge, and starts the joint_state_broadcaster
and wheel_velocity_controller once the robot actually exists in the sim.

Run:
  ros2 launch micromouse_simulation sim.launch.py
  ros2 launch micromouse_simulation sim.launch.py mode:=auto record:=true
  ros2 launch micromouse_simulation sim.launch.py mode:=auto viz:=true
"""
import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Topics worth capturing to reconstruct and debug a run offline: the full
# control loop (setpoint -> wheels), the pose/heading estimate, and the raw
# perception inputs. /clock is required so the bag carries simulated time.
BAG_TOPICS = [
    "/clock",
    "/odom",
    "/odom_corrected",
    "/imu",
    "/ir_front",
    "/ir_left",
    "/ir_right",
    "/joint_states",
    "/motion_setpoint",
    "/heading_error",
    "/wheel_velocity_controller/commands",
    "/ground_truth/tf",
]


def generate_launch_description():
    # solver_node phase: explore | run | auto (explore -> return -> speed run).
    mode = LaunchConfiguration("mode")
    map_path = LaunchConfiguration("map_path")
    record = LaunchConfiguration("record")
    bag_dir = LaunchConfiguration("bag_dir")
    viz = LaunchConfiguration("viz")
    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="auto",
        description="solver mode: explore (map only), run (speed run from saved map), "
        "or auto (explore, return to start, then speed run)",
    )
    declare_map_path = DeclareLaunchArgument(
        "map_path",
        default_value="/tmp/micromouse_map.txt",
        description="where the discovered wall map is saved (explore/auto) and loaded (run)",
    )

    description_pkg = get_package_share_directory("micromouse_description")
    control_pkg = get_package_share_directory("micromouse_control")
    simulation_pkg = get_package_share_directory("micromouse_simulation")

    # Workspace root = four levels up from this package's install/share dir
    # (<ws>/install/micromouse_simulation/share/micromouse_simulation). Bags
    # land in <ws>/bags/ by default so they're easy to find and analyze.
    ws_root = os.path.abspath(os.path.join(simulation_pkg, "..", "..", "..", ".."))
    default_bag_dir = os.path.join(ws_root, "bags")
    declare_record = DeclareLaunchArgument(
        "record",
        default_value="false",
        description="record a rosbag of the run for offline analysis",
    )
    declare_viz = DeclareLaunchArgument(
        "viz",
        default_value="false",
        description="launch RViz2 with the floodfill visualizer (viz:=true)",
    )
    declare_bag_dir = DeclareLaunchArgument(
        "bag_dir",
        default_value=default_bag_dir,
        description="directory to write recorded rosbags into",
    )
    os.makedirs(default_bag_dir, exist_ok=True)
    bag_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_output = PathJoinSubstitution([bag_dir, f"run_{bag_stamp}"])

    record_bag = ExecuteProcess(
        condition=IfCondition(record),
        cmd=["ros2", "bag", "record", "-o", bag_output, *BAG_TOPICS],
        output="screen",
    )

    xacro_path = os.path.join(description_pkg, "urdf", "micromouse.urdf.xacro")
    controllers_yaml = os.path.join(control_pkg, "config", "controllers.yaml")
    world_path = os.path.join(simulation_pkg, "worlds", "maze_16x16.sdf")
    bridge_config = os.path.join(simulation_pkg, "config", "bridge.yaml")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path, " controllers_yaml:=", controllers_yaml]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # Maze start cell (0,0) center is (0.09, 0.09); z lands base_link's
    # wheel-axle-height origin just above the floor plane (floor top at
    # z=0.005, wheel radius 0.016).
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_micromouse",
        arguments=[
            "-topic", "robot_description",
            "-name", "micromouse",
            "-x", "0.09", "-y", "0.09", "-z", "0.021",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller"],
        output="screen",
    )

    # controller_manager only exists once gz_ros2_control loads the robot,
    # which happens when spawn_robot finishes - chain off its exit instead
    # of guessing a fixed delay.
    delayed_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, wheel_velocity_controller_spawner],
        )
    )

    pid_controller = Node(
        package="micromouse_control",
        executable="pid_controller_node",
        name="pid_controller_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    solver = Node(
        package="micromouse_solver",
        executable="solver_node",
        name="solver_node",
        parameters=[
            {"use_sim_time": True},
            {"mode": mode},
            {"map_path": map_path},
        ],
        output="screen",
    )

    rviz_config = os.path.join(simulation_pkg, "config", "rviz", "maze_viz.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(viz),
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        declare_mode,
        declare_map_path,
        declare_record,
        declare_viz,
        declare_bag_dir,
        record_bag,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        delayed_controllers,
        pid_controller,
        solver,
        rviz,
    ])
