# dron_bringup_pkg_mavros

Paquete ROS 2 para control de un dron real mediante **MAVROS** y **PX4** (Pixhawk 6C).

---

## Arquitectura de nodos

```
QGroundControl
 └── UDP 14550 ◄── MAVROS ──► Pixhawk/PX4 (radio serial)
                      ▲
                      │
           ┌──────────┴────────────┐
           │  manual_control_node  │  ← nodo central
           └──────────┬────────────┘
                      │ /manual/cmd_vel, /manual/key
           ┌──────────┴────────────┐
           │   keyboard_node       │  ← stdin (terminal propia)
           └───────────────────────┘
                      │ /manual/safety_enable
           ┌──────────┴────────────┐
           │  manual_safety_node   │  ← watchdog batería/conexión
           └───────────────────────┘
```

Nodos adicionales:

| Nodo | Función |
|---|---|
| `telemetry_node_mavros` | Publica telemetría JSON en `/manual/telemetry_state` |
| `tf_odometry_node_mavros` | Publica TF `map→odom→base_link` desde `/mavros/local_position/odom` |
| `robot_state_publisher` | Publica URDF del dron para RViz |

---

## Dos modos de control (backend)

### `manual_mavlink` (defecto)

- Teclado actúa como **joystick MAVLink** → `MANUAL_CONTROL`.
- PX4 en **STABILIZED** (o ALTCTL).
- No requiere GPS ni VIO. El piloto controla actitud directamente.
- Ideal para vuelo manual con radio MAVLink.

### `offboard_velocity`

- Publica **setpoints de velocidad** (`/mavros/setpoint_raw/local`, `FRAME_BODY_NED`).
- PX4 en **OFFBOARD**.
- Requiere estimación de posición válida (GPS o cámara VIO).
- Permite comportamiento **autónomo** (modos futuros de detección con cámara).

Cambio de backend en vuelo:

| Tecla | Acción |
|---|---|
| `b` / `g` | → modo teclado MAVLink (STABILIZED) |
| `o` | → modo Offboard autónomo |

---

## Mapa de teclas

| Tecla | Acción |
|---|---|
| `b` / `g` | Modo teclado MAVLink → PX4 STABILIZED |
| `o` | Modo autónomo Offboard |
| `m` | Armar |
| `n` | Desarmar |
| `l` | Aterrizar (AUTO.LAND) |
| `t` | Despegar automático (solo Offboard) |
| `x` | Throttle a cero (emergencia suave) |
| `w`/`s` | Pitch adelante / atrás |
| `a`/`d` | Roll izquierda / derecha |
| `r`/`f` | Subir / bajar |
| `q`/`e` | Yaw izquierda / derecha |
| `SPACE`/`z` | Nivelar ejes |
| `+`/`-` | Aumentar / reducir sensibilidad |
| `p` | Imprimir estado del nodo de control |
| `h` | Mostrar ayuda |

---

## Lanzamiento

```bash
# Lanzamiento completo (MAVROS + todos los nodos + QGC + RViz)
ros2 launch dron_bringup_pkg_mavros dron_bringup_mavros.launch.py

# Puerto serie personalizado
ros2 launch dron_bringup_pkg_mavros dron_bringup_mavros.launch.py \
  mavros_fcu_url:=/dev/ttyUSB1:115200

# Sin RViz ni QGC (solo control)
ros2 launch dron_bringup_pkg_mavros dron_bringup_mavros.launch.py \
  launch_rviz:=false launch_qgc:=false

# Cambiar a backend Offboard desde el inicio
ros2 param set /manual_control_node_mavros control_backend offboard_velocity
```

---

## Procedimiento de vuelo – modo teclado (STABILIZED)

1. Conectar la radio de telemetría (USB).
2. Lanzar: `ros2 launch dron_bringup_pkg_mavros dron_bringup_mavros.launch.py`
3. Verificar en la terminal de teclado que MAVROS aparece conectado (`connected=Y`).
4. Pulsar `b` → PX4 cambia a STABILIZED.
5. Pulsar `m` → armar.
6. Subir throttle con `r`.
7. Para aterrizar: `l` (AUTO.LAND) o bajar con `f` y `n` para desarmar.

## Procedimiento de vuelo – modo Offboard (autónomo)

1. Asegurarse de tener estimación de posición válida (GPS fix o VIO activo).
2. Pulsar `o` → arranca calentamiento de setpoints (30 ciclos) y pide OFFBOARD.
3. Pulsar `m` → armar.
4. Pulsar `t` → despegue automático a `takeoff_altitude_m` (defecto 2 m).
5. Controlar posición con `w/s/a/d/r/f/q/e`.
6. Para modo autónomo: conectar tu nodo de navegación que publique en `/manual/cmd_vel`.

---

## Parámetros clave (config/manual_params.yaml)

| Parámetro | Defecto | Descripción |
|---|---|---|
| `control_backend` | `manual_mavlink` | Backend de control activo |
| `manual_mode` | `STABILIZED` | Modo PX4 para backend manual |
| `takeoff_altitude_m` | `2.0` | Altura de despegue en Offboard (m) |
| `max_xy_speed` | `1.0` | Velocidad horizontal máxima (m/s) |
| `max_z_speed` | `0.5` | Velocidad vertical máxima (m/s) |
| `require_safety_enable` | `false` | Activar watchdog de safety |
| `manual_deadman_timeout_s` | `0.30` | Timeout deadman de teclado (s) |

---

## Estructura del paquete

```
dron_bringup_pkg_mavros/
├── src/
│   ├── keyboard_node_mavros.cpp        # Lectura de teclado
│   ├── manual_control_node_mavros.cpp  # Control central (manual + offboard)
│   ├── manual_safety_node_mavros.cpp   # Watchdog de safety
│   ├── telemetry_node_mavros.cpp       # Telemetría JSON
│   └── tf_odometry_node_mavros.cpp     # TF desde odometría
├── launch/
│   └── dron_bringup_mavros.launch.py
├── config/
│   └── manual_params.yaml
├── urdf/
│   └── x500_base.urdf
└── rviz/
    └── dron_tf.rviz
```

---

## Roadmap

- [ ] Integración VIO (cámara → estimación de posición → EKF2)
- [ ] Nodo de navegación autónoma (detección de objetos con cámara)
- [ ] Soporte de mando de joystick USB (publicar en `/manual/cmd_vel`)
- [ ] Modo de seguimiento de trayectoria waypoints
