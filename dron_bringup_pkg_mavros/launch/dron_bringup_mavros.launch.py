import glob
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
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
        pkg_prefix = Path(get_package_prefix("dron_bringup_pkg_mavros")).resolve()
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

    launch_keyboard_arg = DeclareLaunchArgument(
        "launch_keyboard",
        default_value="true",
        description="Lanzar nodo de control por teclado",
    )

    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros",
        default_value="true",
        description="Lanzar MAVROS para leer MAVLink por la radio de telemetría",
    )

    mavros_fcu_url_arg = DeclareLaunchArgument(
        "mavros_fcu_url",
        default_value="/dev/ttyUSB0:57600",
        description="Puerto serie MAVLink de la radio de telemetría",
    )

    mavros_gcs_url_arg = DeclareLaunchArgument(
        "mavros_gcs_url",
        default_value="udp://127.0.0.1:14551@127.0.0.1:14550",
        description="Salida UDP de MAVROS hacia QGroundControl",
    )

    dependencies_setup = os.environ.get("DEPENDENCIAS_SETUP", "") or _first_existing([
        "~/dependencias_ros2_px4/install/setup.bash",
    ])

    workspace_setup = os.environ.get("WORKSPACE_SETUP", "") or _workspace_setup()

    qgc_path = os.environ.get("QGC_PATH", "") or _first_existing([
        "~/QGroundControl-x86_64.AppImage",
        "~/QGroundControl*.AppImage",
        "~/Descargas/QGroundControl*.AppImage",
        "~/Downloads/QGroundControl*.AppImage",
    ])

    pkg_share = get_package_share_directory("dron_bringup_pkg_mavros")

    params_file = os.path.join(pkg_share, "config", "manual_params.yaml")
    urdf_file = os.path.join(pkg_share, "urdf", "x500_base.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "dron_tf.rviz")

    ros_sources = (
        "source /opt/ros/humble/setup.bash; "
        + _source_if_exists(dependencies_setup)
        + _source_if_exists(workspace_setup)
    )

    colorized_output = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mavros"),
                "launch",
                "px4.launch",
            )
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("mavros_fcu_url"),
            "gcs_url": LaunchConfiguration("mavros_gcs_url"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
    )

    robot_state_publisher = TimerAction(
        period=4.0,
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
                        "use_sim_time": False,
                    }
                ],
                output="screen",
            )
        ],
    )

    manual_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="dron_bringup_pkg_mavros",
                executable="manual_safety_node_mavros",
                name="manual_safety_node_mavros",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
            Node(
                package="dron_bringup_pkg_mavros",
                executable="manual_control_node_mavros",
                name="manual_control_node_mavros",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
            Node(
                package="dron_bringup_pkg_mavros",
                executable="telemetry_node_mavros",
                name="telemetry_node_mavros",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
            Node(
                package="dron_bringup_pkg_mavros",
                executable="tf_odometry_node_mavros",
                name="tf_odometry_node_mavros",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
        ],
    )

    keyboard_control = TimerAction(
        period=6.0,
        actions=[
            _terminal(
                "Keyboard_Control",
                (
                    "clear; "
                    "echo \"============================\"; "
                    "echo \"      KEYBOARD CONTROL      \"; "
                    "echo \"============================\"; "
                    f"{ros_sources}"
                    f"ros2 run dron_bringup_pkg_mavros keyboard_node_mavros "
                    f"--ros-args --params-file \"{params_file}\" -p use_sim_time:=false"
                ),
            )
        ],
        condition=IfCondition(LaunchConfiguration("launch_keyboard")),
    )

    qgroundcontrol = TimerAction(
        period=15.0,
        actions=[
            _terminal(
                "QGroundControl",
                (
                    "clear; "
                    "echo \"============================\"; "
                    "echo \"       QGROUNDCONTROL       \"; "
                    "echo \"============================\"; "
                    "echo \"IMPORTANTE:\"; "
                    "echo \"QGroundControl debe conectarse por UDP 14550.\"; "
                    "echo \"No debe abrir /dev/ttyUSB0 directamente.\"; "
                    f"if [ -f \"{qgc_path}\" ]; then "
                    f"chmod +x \"{qgc_path}\" && \"{qgc_path}\"; "
                    "else "
                    "echo \"No encuentro QGroundControl.\"; "
                    "echo \"Exporta QGC_PATH=/ruta/QGroundControl-x86_64.AppImage\"; "
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
        launch_keyboard_arg,
        launch_mavros_arg,
        mavros_fcu_url_arg,
        mavros_gcs_url_arg,
        colorized_output,
        mavros,
        robot_state_publisher,
        manual_nodes,
        keyboard_control,
        qgroundcontrol,
        rviz,
    ])
