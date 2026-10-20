# go2w_ws

Workspace ROS 2 Humble para el Unitree Go2-W (ruedas). Convierte los mensajes de Unitree (`unitree_go`, `unitree_api`) en topics estándar de ROS 2 y publica `robot_description` y las TF.

| Paquete | Qué hace |
|---|---|
| `go2w_description` | URDF y mallas del Go2-W (de `unitreerobotics/unitree_ros`), con raíz `base_link` y frame `utlidar_lidar` |
| `go2w_driver` | `sim_go2w` para trabajar sin robot (`sim:=true`). `lowstate` → `/joint_states`, `/imu`, `/battery_state`; `sportmodestate` → `/odom` y TF `odom → base_footprint → base_link`; `/cmd_vel` → `/api/sport/request` (Move) con watchdog; servicio `/mode` |
| `go2w_bringup` | Lanza description + driver + `/utlidar/cloud` → `/scan` + RViz |

Dependencias externas (en `deps.repos`, con commits fijos): `unitree_go`, `unitree_api`, `go2_interfaces` de [Unitree-Go2-Robot](https://github.com/Unitree-Go2-Robot).

## Flujo de trabajo

- **PC**: se desarrolla dentro de Docker (Ubuntu 22.04 + Humble, igual que la NX).
- **NX (Orin NX, JetPack 6)**: compila nativo. `git pull` y `scripts/nx_update.sh`.
- `scripts/install_deps.sh` lo usan el Dockerfile y la NX, así los dos entornos tienen los mismos paquetes.

### PC (Docker)

Una sola vez, para instalar Docker en Ubuntu 24.04:

```bash
sudo apt install docker.io docker-compose-v2
sudo usermod -aG docker $USER   # cerrar sesión y volver a entrar
```

Uso:

```bash
cd ~/go2w_ws
xhost +local:                       # RViz desde el contenedor
export GO2_IFACE=enp3s0             # interfaz con IP 192.168.123.x (vacía = sin robot)
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm go2w

# dentro del contenedor
scripts/install_deps.sh ws
colcon build --symlink-install
source scripts/env.sh
```

### NX

```bash
sudo apt install python3-vcstool python3-rosdep   # una sola vez
git clone <repo> ~/go2w_ws && cd ~/go2w_ws
scripts/install_deps.sh all                        # una sola vez
scripts/nx_update.sh                               # cada vez que haya cambios
GO2_IFACE=eth0 source scripts/env.sh
```

## Lanzar

```bash
ros2 launch go2w_description view.launch.py        # solo el modelo, sin robot
ros2 launch go2w_bringup go2w.launch.py sim:=true rviz:=true  # robot simulado (sim_go2w)
ros2 launch go2w_bringup go2w.launch.py rviz:=true # con el robot
ros2 service call /mode go2_interfaces/srv/Mode "mode: 'stand_up'"
ros2 run plotjuggler plotjuggler                   # gráficas de topics (ROS2 Topic Subscriber)
```

### Simulación (`sim:=true`)

`sim_go2w` hace de robot: publica `/lowstate` y `/sportmodestate` y ejecuta `/api/sport/request` (Move, StopMove, StandUp, StandDown...). El driver corre igual que con el robot real, así que `/cmd_vel`, `/mode`, `/odom`, TF y `/joint_states` funcionan. Es solo cinemático: sin física ni LiDAR (no hay `/scan`). No lanzarlo con el robot conectado.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
ros2 service call /mode go2_interfaces/srv/Mode "mode: 'stand_down'"
```

## Pendiente de verificar en el robot

1. **Índices de las ruedas** (`config/go2w_driver.yaml`, `motor_indices` 12-15): girar cada rueda a mano y comprobar que se mueve el `*_foot_joint` correcto en `/joint_states`. Las patas siguen el orden de Unitree (FR, FL, RR, RL).
2. **Frame de la nube**: `/utlidar/cloud` debe salir alineada con el suelo en RViz. Si no, ajustar `utlidar_lidar_joint` en el URDF.
3. **Odometría**: comprobar que `velocity` de `sportmodestate` está en el frame del mundo y que la altura de `base_link` es razonable (`base_height_offset`).
4. **Move en modo ruedas**: confirmar que el Go2-W acepta la API 1008 igual que el Go2. Probar con el robot en espacio libre y velocidades bajas.
5. **QoS**: si algún topic de Unitree no llega, revisar `ros2 topic info -v`.

## Siguiente paso: EasyNav

`easynav.repos` tiene EasyNav 0.5.0 desde fuente: `vcs import src/external < easynav.repos`.
