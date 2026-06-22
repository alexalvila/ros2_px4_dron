"""
dron_bringup_mavros.launch.py
=============================
Lanza todos los nodos del sistema de control del dron físico mediante MAVROS.

Arquitectura desplegada:
  - MAVROS          : puente ROS 2 ↔ MAVLink (radio de telemetría)
  - manual_control_node_mavros  : lógica central, backend manual/offboard
  - keyboard_node_mavros        : entrada de teclado (terminal separada)
  - manual_safety_node_mavros   : watchdog de batería y conexión
  - telemetry_node_mavros       : telemetría JSON periódica
  - tf_odometry_node_mavros     : transformaciones TF desde odometría MAVROS
  - robot_state_publisher       : URDF para RViz
  - QGroundControl              : supervisión / parámetros (opcional)
  - RViz2                       : visualización 3D (opcional)

Argumentos de lanzamiento:
  launch_mavros   (true)   Lanzar MAVROS
  mavros_fcu_url  (/dev/ttyUSB0:57600)  Puerto serie de la radio
  mavros_gcs_url  (udp://127.0.0.1:14551@127.0.0.1:14550)  Puerto QGC
  launch_keyboard (true)   Abrir terminal de teclado
  launch_qgc      (true)   Abrir QGroundControl
  launch_rviz     (true)   Abrir RViz2

Variables de entorno (opcionales):
  QGC_PATH          – ruta al AppImage de QGroundControl
  WORKSPACE_SETUP   – ruta al setup.bash del workspace
  DEPENDENCIAS_SETUP – ruta al setup.bash de dependencias externas
"""

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


# ── Utilidades de rutas ──────────────────────────────────────────────────────

def _first_existing(candidates: list) -> str:
    """Devuelve la primera ruta que exista de la lista de candidatos (con glob)."""
    for candidate in candidates:
        for match in glob.glob(os.path.expanduser(candidate)):
            if os.path.exists(match):
                return os.path.abspath(match)
    return ""


def _workspace_setup() -> str:
    """Intenta localizar el setup.bash del workspace actual."""
    try:
        pkg_prefix = Path(get_package_prefix("dron_bringup_pkg_mavros")).resolve()
        setup = pkg_prefix.parent / "setup.bash"
        if setup.exists():
            return str(setup)
    except Exception:
        pass
    return _first_existing([
        "~/ros2_px4_WorkSpace/install/setup.bash",
        "~/dron_ws/install/setup.bash",
        "~/ros2_ws/install/setup.bash",
    ])


def _source_if_exists(path: str) -> str:
    if not path:
        return ""
    return f'if [ -f "{path}" ]; then source "{path}"; fi; '


def _terminal(title: str, command: str) -> ExecuteProcess:
    """Abre una nueva terminal gnome-terminal con el comando dado."""
    return ExecuteProcess(
        cmd=[
            "gnome-terminal",
            f"--title={title}",
            "--",
            "bash", "-c",
            command + "; exec bash",
        ],
        output="screen",
    )


# ── Generador de descripción de lanzamiento ───────────────────────────────────

def generate_launch_description():

    # ── Argumentos ────────────────────────────────────────────────────────────

    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros",
        default_value="true",
        description="Lanzar MAVROS para comunicación MAVLink por radio",
    )
    mavros_fcu_url_arg = DeclareLaunchArgument(
        "mavros_fcu_url",
        default_value="/dev/ttyUSB0:57600",
        description="Puerto serie y baudios de la radio de telemetría",
    )
    mavros_gcs_url_arg = DeclareLaunchArgument(
        "mavros_gcs_url",
        default_value="udp://127.0.0.1:14551@127.0.0.1:14550",
        description="URL UDP de reenvío hacia QGroundControl",
    )
    launch_keyboard_arg = DeclareLaunchArgument(
        "launch_keyboard",
        default_value="true",
        description="Abrir terminal con keyboard_node_mavros",
    )
    launch_qgc_arg = DeclareLaunchArgument(
        "launch_qgc",
        default_value="true",
        description="Lanzar QGroundControl",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Lanzar RViz2 para visualización 3D",
    )

    # ── Rutas ────────────────────────────────────────────────────────────────

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

    pkg_share   = get_package_share_directory("dron_bringup_pkg_mavros")
    params_file = os.path.join(pkg_share, "config", "manual_params.yaml")
    urdf_file   = os.path.join(pkg_share, "urdf",   "x500_base.urdf")
    rviz_config = os.path.join(pkg_share, "rviz",   "dron_tf.rviz")

    ros_sources = (
        "source /opt/ros/humble/setup.bash; "
        + _source_if_exists(dependencies_setup)
        + _source_if_exists(workspace_setup)
    )

    # ── Variables de entorno ─────────────────────────────────────────────────

    colorized_output = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # ── MAVROS ───────────────────────────────────────────────────────────────

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

    # Configurar frecuencias de mensajes MAVLink después de que MAVROS arranque
    mavros_message_rates = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    (
                        "source /opt/ros/humble/setup.bash; "
                        "echo 'Esperando /mavros/set_message_interval…'; "
                        "for i in $(seq 1 60); do "
                        "  if ros2 service list | grep -q /mavros/set_message_interval; then break; fi; "
                        "  sleep 1; "
                        "done; "
                        "if ! ros2 service list | grep -q /mavros/set_message_interval; then "
                        "  echo 'ERROR: /mavros/set_message_interval no disponible'; exit 1; "
                        "fi; "
                        # LOCAL_POSITION_NED → 30 Hz (odometría local)
                        "echo 'Configurando LOCAL_POSITION_NED a 30 Hz'; "
                        "ros2 service call /mavros/set_message_interval "
                        "mavros_msgs/srv/MessageInterval '{message_id: 32, message_rate: 30.0}'; "
                        "sleep 1; "
                        # ATTITUDE → 30 Hz (IMU / actitud)
                        "echo 'Configurando ATTITUDE a 30 Hz'; "
                        "ros2 service call /mavros/set_message_interval "
                        "mavros_msgs/srv/MessageInterval '{message_id: 30, message_rate: 30.0}'; "
                        "sleep 1; "
                        # BATTERY_STATUS → 1 Hz
                        "echo 'Configurando BATTERY_STATUS a 1 Hz'; "
                        "ros2 service call /mavros/set_message_interval "
                        "mavros_msgs/srv/MessageInterval '{message_id: 147, message_rate: 1.0}'"
                    ),
                ],
                output="screen",
            )
        ],
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
    )

    # ── robot_state_publisher ────────────────────────────────────────────────

    robot_state_publisher = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{
                    "robot_description": ParameterValue(
                        Command(["cat ", urdf_file]),
                        value_type=str,
                    ),
                    "use_sim_time": False,
                }],
                output="screen",
            )
        ],
    )

    # ── Nodos de control (arrancan a los 5 s para dar tiempo a MAVROS) ───────

    control_nodes = TimerAction(
        period=5.0,
        actions=[
            # Control central: backend manual/offboard
            Node(
                package="dron_bringup_pkg_mavros",
                executable="manual_control_node_mavros",
                name="manual_control_node_mavros",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
            # Watchdog de safety: batería, conexión, estado en tierra
            Node(
                package="dron_bringup_pkg_mavros",
                executable="manual_safety_node_mavros",
                name="manual_safety_node_mavros",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
            # Telemetría JSON
            Node(
                package="dron_bringup_pkg_mavros",
                executable="telemetry_node_mavros",
                name="telemetry_node_mavros",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
            # TF map → odom → base_link
            Node(
                package="dron_bringup_pkg_mavros",
                executable="tf_odometry_node_mavros",
                name="tf_odometry_node_mavros",
                parameters=[
                    params_file,
                    {
                        "odom_topic": "/mavros/local_position/odom",
                        "odom_msg_topic": "/odom",
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "publish_map_to_odom": True,
                        "publish_odom_msg": True,
                        "publish_identity_until_odom": True,
                        "use_msg_timestamp": False,
                        "zero_initial_pose": True,
                        "base_z_offset": 0.0,
                        "use_sim_time": False,
                    },
                ],
                output="screen",
            ),
        ],
    )

    # ── Teclado (terminal separada, 6 s para que arranquen los nodos) ────────

    keyboard_control = TimerAction(
        period=6.0,
        actions=[
            _terminal(
                "Keyboard_Control",
                (
                    "clear; "
                    'echo "============================"; '
                    'echo "      KEYBOARD CONTROL      "; '
                    'echo "============================"; '
                    f"{ros_sources}"
                    f"ros2 run dron_bringup_pkg_mavros keyboard_node_mavros "
                    f'--ros-args --params-file "{params_file}" -p use_sim_time:=false'
                ),
            )
        ],
        condition=IfCondition(LaunchConfiguration("launch_keyboard")),
    )

    # ── QGroundControl (15 s para que MAVROS esté listo) ─────────────────────

    qgroundcontrol = TimerAction(
        period=15.0,
        actions=[
            _terminal(
                "QGroundControl",
                (
                    "clear; "
                    'echo "============================"; '
                    'echo "       QGROUNDCONTROL       "; '
                    'echo "============================"; '
                    'echo "NOTA: QGC debe conectarse por UDP 14550."; '
                    'echo "      No debe abrir /dev/ttyUSB0 directamente."; '
                    f'if [ -f "{qgc_path}" ]; then '
                    f'chmod +x "{qgc_path}" && "{qgc_path}"; '
                    "else "
                    'echo "No encuentro QGroundControl."; '
                    'echo "Exporta QGC_PATH=/ruta/QGroundControl-x86_64.AppImage"; '
                    "fi"
                ),
            )
        ],
        condition=IfCondition(LaunchConfiguration("launch_qgc")),
    )

    # ── RViz2 ────────────────────────────────────────────────────────────────

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

    # ── Descripción final ─────────────────────────────────────────────────────

    return LaunchDescription([
        # Argumentos
        launch_mavros_arg,
        mavros_fcu_url_arg,
        mavros_gcs_url_arg,
        launch_keyboard_arg,
        launch_qgc_arg,
        launch_rviz_arg,
        # Entorno
        colorized_output,
        # Nodos / acciones (en orden de arranque)
        mavros,
        mavros_message_rates,
        robot_state_publisher,
        control_nodes,
        keyboard_control,
        qgroundcontrol,
        rviz,
    ])
