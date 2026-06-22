# Configuración PX4/QGroundControl para este paquete físico con MAVROS

Este paquete tiene dos formas de control:

1. Teclado manual, sin GPS ni VIO:
   - ROS 2 publica en `/mavros/manual_control/send`.
   - PX4 debe estar en `STABILIZED` o `ALTCTL`.
   - No usa `/mavros/setpoint_raw/local` salvo que se active el modo autónomo.

2. Autonomía/Offboard:
   - ROS 2 publica setpoints en `/mavros/setpoint_raw/local`.
   - PX4 debe estar en `OFFBOARD`.
   - Si el setpoint es local de posición/velocidad, PX4 necesita estimación válida: GPS, VIO, optical flow, mocap o external vision.

## QGroundControl para teclado sin GPS/VIO

Parámetros recomendados para pruebas controladas:

- `COM_ARM_WO_GPS = Warning only`
- `SYS_HAS_GPS = Disabled`
- `EKF2_GPS_CTRL = 0`
- `COM_RC_IN_MODE = MAVLink only` o `RC or MAVLink with fallback`
- Modo de vuelo: `STABILIZED`

No uses `OFFBOARD` para teclado visual sin GPS/VIO. Ese modo con setpoints locales necesita estimación válida.

## QGroundControl para futuro GPS + cámaras/VIO

- `SYS_HAS_GPS = Enabled`
- `EKF2_GPS_CTRL = 7`
- `EKF2_EV_CTRL = activo según tu VIO/cámara`
- `COM_ARM_WO_GPS = Warning only`
- `COM_POSCTL_NAVL = Altitude mode`
- `COM_POS_FS_DELAY = 5-10 s`

Cuando uses VIO/cámaras, publica odometría externa hacia PX4 mediante MAVROS y entonces Offboard podrá funcionar sin GPS.

## Teclas principales

- `b` o `g`: modo teclado MAVLink, pide `STABILIZED`.
- `m`: armar.
- `n`: desarmar.
- `r/f`: subir/bajar throttle.
- `w/s/a/d`: pitch/roll.
- `q/e`: yaw.
- `z` o espacio: neutraliza roll/pitch/yaw, mantiene throttle.
- `x`: throttle a cero.
- `o`: modo Offboard/autónomo, requiere estimación válida.
