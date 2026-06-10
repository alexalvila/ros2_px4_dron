import glob
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _first_existing(candidates):
    for candidate in candidates:
        matches = glob.glob(os.path.expanduser(candidate))
        for match in matches:
            if os.path.exists(match):
                return os.path.abspath(os.path.expanduser(match))
    return ""


def _workspace_setup():
    try:
        pkg_prefix = Path(get_package_prefix("dron_bringup_pkg")).resolve()
        install_dir = pkg_prefix.parent
        setup = install_dir / "setup.bash"
        if setup.exists():
            return str(setup)
    except Exception:
        pass

    return _first_existing([
        "~/ros2_px4_WorkSpace/install/setup.bash",
        "~/dron_ws/install/setup.bash",
        "~/ros2_ws/install/setup.bash",
    ])


def _source_if_exists(path):
    if not path:
        return ""
    return f'if [ -f "{path}" ]; then source "{path}"; fi; '


def _terminal(title, command):
    return ExecuteProcess(
        cmd=[
            "gnome-terminal",
            f"--title={title}",
            "--",
            "bash",
            "-c",
            command + "; exec bash",
        ],
        output="screen",
    )


def generate_launch_description():
    launch_qgc_arg = DeclareLaunchArgument(
        "launch_qgc",
        default_value="true",
        description="Lanzar QGroundControl",
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Lanzar RViz2",
    )

    px4_dir = os.environ.get("PX4_DIR", "") or _first_existing([
        "~/PX4-Autopilot",
        "~/PX4_Autopilot",
        "~/px4/PX4-Autopilot",
    ])

    dependencies_setup = os.environ.get("DEPENDENCIAS_SETUP", "") or _first_existing([
        "~/dependencias_ros2_px4/install/setup.bash",
        "~/Micro-XRCE-DDS-Agent/install/setup.bash",
    ])

    workspace_setup = os.environ.get("WORKSPACE_SETUP", "") or _workspace_setup()

    qgc_path = os.environ.get("QGC_PATH", "") or _first_existing([
        "~/QGroundControl-x86_64.AppImage",
        "~/QGroundControl*.AppImage",
        "~/Descargas/QGroundControl*.AppImage",
        "~/Downloads/QGroundControl*.AppImage",
    ])

    px4_sys_autostart = os.environ.get("PX4_SYS_AUTOSTART", "4001")
    px4_gz_world = os.environ.get("PX4_GZ_WORLD", "default")
    px4_sim_model = os.environ.get("PX4_SIM_MODEL", "gz_x500")
    xrce_agent_port = os.environ.get("XRCE_AGENT_PORT", "8888")

    pkg_share = get_package_share_directory("dron_bringup_pkg")

    params_file = os.path.join(pkg_share, "config", "manual_params.yaml")
    urdf_file = os.path.join(pkg_share, "urdf", "x500_base.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "dron_tf.rviz")

    ros_sources = (
        "source /opt/ros/humble/setup.bash; "
        + _source_if_exists(dependencies_setup)
        + _source_if_exists(workspace_setup)
    )

    colorized_output = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    px4_process = _terminal(
        "PX4_SITL",
        (
            "clear; "
            "echo \"============================\"; "
            "echo \"      PX4 SITL + GAZEBO     \"; "
            "echo \"============================\"; "
            f"if [ ! -d \"{px4_dir}\" ]; then "
            f"echo \"No encuentro PX4-Autopilot. Exporta PX4_DIR=/ruta/PX4-Autopilot\"; "
            "else "
            f"cd \"{px4_dir}\" && "
            f"PX4_SYS_AUTOSTART={px4_sys_autostart} "
            f"PX4_GZ_WORLD={px4_gz_world} "
            f"PX4_SIM_MODEL={px4_sim_model} "
            f"make px4_sitl {px4_sim_model}; "
            "fi"
        ),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    microxrce_agent = TimerAction(
        period=3.0,
        actions=[
            _terminal(
                "MicroXRCEAgent",
                (
                    "clear; "
                    "echo \"============================\"; "
                    "echo \"       MICRO XRCE AGENT     \"; "
                    "echo \"============================\"; "
                    f"{ros_sources}"
                    f"MicroXRCEAgent udp4 -p {xrce_agent_port}"
                ),
            )
        ],
    )

    robot_state_publisher = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["cat ", urdf_file]),
                            value_type=str,
                        ),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            )
        ],
    )

    manual_nodes = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="dron_bringup_pkg",
                executable="manual_safety_node",
                name="manual_safety_node",
                parameters=[params_file, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="dron_bringup_pkg",
                executable="manual_control_node",
                name="manual_control_node",
                parameters=[params_file, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="dron_bringup_pkg",
                executable="telemetry_node",
                name="telemetry_node",
                parameters=[params_file, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="dron_bringup_pkg",
                executable="tf_odometry_node",
                name="tf_odometry_node",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
        ],
    )

    keyboard_control = TimerAction(
        period=8.0,
        actions=[
            _terminal(
                "Keyboard_Control",
                (
                    "clear; "
                    "echo \"============================\"; "
                    "echo \"      KEYBOARD CONTROL      \"; "
                    "echo \"============================\"; "
                    f"{ros_sources}"
                    f"ros2 run dron_bringup_pkg keyboard_node "
                    f"--ros-args --params-file \"{params_file}\" -p use_sim_time:=true"
                ),
            )
        ],
    )

    qgroundcontrol = TimerAction(
        period=5.0,
        actions=[
            _terminal(
                "QGroundControl",
                (
                    "clear; "
                    "echo \"============================\"; "
                    "echo \"       QGROUNDCONTROL       \"; "
                    "echo \"============================\"; "
                    f"if [ -f \"{qgc_path}\" ]; then "
                    f"chmod +x \"{qgc_path}\" && \"{qgc_path}\"; "
                    "else "
                    "echo \"No encuentro QGroundControl. Exporta QGC_PATH=/ruta/QGroundControl-x86_64.AppImage\"; "
                    "fi"
                ),
            )
        ],
        condition=IfCondition(LaunchConfiguration("launch_qgc")),
    )

    rviz = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": False}],
                output="screen",
            )
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription([
        launch_qgc_arg,
        launch_rviz_arg,

        colorized_output,

        px4_process,
        clock_bridge,
        microxrce_agent,

        robot_state_publisher,
        manual_nodes,
        keyboard_control,

        qgroundcontrol,
        rviz,
    ])