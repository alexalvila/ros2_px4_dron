# dron_bringup_pkg

Paquete ROS 2 en C++ para la primera fase del dron: bringup básico con teclado, comunicación PX4 mediante uXRCE-DDS, telemetría y bloqueo de seguridad. Incluye control incremental de velocidad y TF para visualización en RViz.

Este paquete está diseñado para funcionar tanto en simulación (SITL + Gazebo) como en un dron real. Por defecto, está preparado para dron real (`use_sim_time: false`).

## Nodos

- `keyboard_node`: lee el teclado y publica:
  - `/manual/cmd_vel` (`geometry_msgs/msg/Twist`)
  - `/manual/key` (`std_msgs/msg/String`)

- `manual_control_node`: recibe velocidades y teclas, mantiene Offboard activo y publica hacia PX4:
  - `/fmu/in/offboard_control_mode`
  - `/fmu/in/trajectory_setpoint`
  - `/fmu/in/vehicle_command`

- `manual_safety_node`: escucha batería, failsafe, estado y detección de tierra. Publica:
  - `/manual/safety_enable`
  - `/manual/safety_reason`

- `telemetry_node`: agrupa telemetría PX4 en JSON y publica:
  - `/manual/telemetry_state`

- `tf_odometry_node`: transforma datos de odometría PX4 a marco ENU para ROS y publica TF.

## Teclas

```text
w/s : avanzar / retroceder
a/d : izquierda / derecha
r/f : subir / bajar
q/e : guiñada izquierda / derecha
espacio o z : parar movimiento
m : armar
n : desarmar
b : activar Offboard
t : despegar
l : aterrizar
+ : aumentar velocidad incremental
- : disminuir velocidad incremental
h : ayuda
Ctrl+C : salir
```

## Instalación

```bash
mkdir -p ~/ros2_px4_Workspace/src
cd ~/ros2_px4_Workspace/src
unzip dron_bringup_pkg_cpp.zip
cd ~/ros2_px4_Workspace
colcon build --packages-select dron_bringup_pkg
source install/setup.bash
```

## Agente uXRCE-DDS

Ejemplo por USB/serial:

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

Ajusta `/dev/ttyUSB0` y el baudrate según tu enlace TELEM/RF.

## Lanzamiento

```bash
ros2 launch dron_bringup_pkg dron_bringup.launch.py
```

El launch abre `keyboard_node` en una terminal interactiva. Por defecto usa `gnome-terminal`. Si no tienes `gnome-terminal`, edita `launch/dron_bringup.launch.py` y cambia el prefijo.

## Avisos importantes

1. PX4 usa marco NED: `z` positiva significa bajar. El nodo invierte `cmd_vel.linear.z` automáticamente.
2. El nodo publica continuamente `OffboardControlMode` y `TrajectorySetpoint` porque PX4 necesita un flujo sostenido de setpoints.
3. Este paquete es para pruebas progresivas. Primero úsalo en SITL o con el dron sin hélices. Después prueba en vuelo real con límites bajos y failsafes revisados.
4. Los tópicos de PX4 están parametrizados en YAML (`config/manual_params.yaml`). Ajusta si tu versión de `px4_msgs` tiene nombres diferentes.
5. Para seguridad, el nodo `manual_safety_node` puede desactivar Offboard si no hay batería, GPS o failsafe activo. Ajusta parámetros en YAML según tu uso real o simulación.

## Launch SITL automático

Detecta automáticamente:

- `~/PX4-Autopilot`
- el `install/setup.bash` del workspace donde está `dron_bringup_pkg`
- `~/dependencias_ros2_px4/install/setup.bash`
- `~/QGroundControl-x86_64.AppImage`

Si alguna ruta no coincide con tu sistema, sobrescríbela con variables de entorno:

```bash
export PX4_DIR=~/PX4-Autopilot
export WORKSPACE_SETUP=~/ros2_px4_WorkSpace/install/setup.bash
export DEPENDENCIAS_SETUP=~/dependencias_ros2_px4/install/setup.bash
export QGC_PATH=~/QGroundControl-x86_64.AppImage
```

Variables adicionales:

```bash
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_WORLD=default
export PX4_SIM_MODEL=gz_x500
export XRCE_AGENT_PORT=8888
```