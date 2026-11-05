# go2w_ws

Workspace ROS 2 Humble para el Unitree Go2-W (ruedas). Convierte los mensajes de Unitree (`unitree_go`, `unitree_api`) en topics estándar de ROS 2 y publica `robot_description` y las TF.

| Paquete | Qué hace |
|---|---|
| `go2w_description` | URDF y mallas del Go2-W (de `unitreerobotics/unitree_ros`), con raíz `base_link` y frame `utlidar_lidar` |
| `go2w_driver` | `sim_go2w` para trabajar sin robot (`sim:=true`). `lowstate` → `/joint_states`, `/imu`, `/battery_state`; `/odom` y TF `odom → base_footprint → base_link` desde FAST-LIO (`odom_source: lio`, por defecto), las ruedas (`wheels`, en `sim:=true`) o `sportmodestate`; `/cmd_vel` → `/api/sport/request` (Move) con watchdog; servicio `/mode` |
| `go2w_bringup` | Lanza description + driver + FAST-LIO (`lio:=true`) + `/utlidar/cloud` → `/scan` + RViz |
| `go2w_gazebo` | Go2-W en Gazebo Harmonic sobre mundos de `navmap_gis_tool` |
| `worlds/gis_*` | Mundos generados con `navmap_gis_tool` (`gis_mic_recinto_world`: recinto del MIC, León) |
| `go2w_nav_test` | `waypoint_follower`: recorre waypoints con `/odom` → `/cmd_vel` y se para si `/scan` ve algo delante. Para probar el robot |

Dependencias externas, en `deps.repos` con commits fijos. `scripts/install_deps.sh ws` las clona en `src/external/` (no versionado); para actualizar una, cambiar su `version` en `deps.repos` y volver a ejecutarlo:

| Paquete | Origen | Versión | Commit |
|---|---|---|---|
| `unitree_go`, `unitree_api`, `go2_interfaces` | [Unitree-Go2-Robot](https://github.com/Unitree-Go2-Robot) | | ver `deps.repos` |
| `fast_lio_go2` | [aron-assani/fast_lio_go2](https://github.com/aron-assani/fast_lio_go2) (FAST-LIO2 con soporte de `/utlidar/cloud`) | | `540b85b` |
| `EasyNavigation` | [EasyNavigation/EasyNavigation](https://github.com/EasyNavigation/EasyNavigation) | 0.5.0 | `9ca3cca` |
| `easynav_plugins` | [EasyNavigation/easynav_plugins](https://github.com/EasyNavigation/easynav_plugins) | 0.5.0 | `2580d29` |
| `NavMap` | [EasyNavigation/NavMap](https://github.com/EasyNavigation/NavMap) | 0.6.0 | `c96d8c8` |
| `yaets` | [fmrico/yaets](https://github.com/fmrico/yaets) | 1.2.0 | `1452f14` |
| `yolo_ros` | [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros) | 5.0.0 | `7924a8b` |

`src/third_party/livox_ros_driver2` es lo único de terceros versionado aquí: solo los mensajes de Livox que necesita `fast_lio` para compilar (no existe como repo aparte).

## Estructura

```
src/
  go2w_*/         paquetes propios
  worlds/         mundos de Gazebo (navmap_gis_tool)
  third_party/    livox_ros_driver2 recortado (solo mensajes)
  external/       repos de deps.repos (no versionado, lo crea install_deps.sh)
config/           parámetros de EasyNav y slam_toolbox
scripts/          instalación, entorno, red PC/NX
docker/           imagen de desarrollo del PC
```

## Cómo trabajamos (ramas)

- `main` siempre compila y es lo que se lleva al robot. No se hacen commits directos a `main`.
- Cada cambio va en una rama desde `main` actualizado: `feat/...`, `fix/...`, `chore/...`, `docs/...`.
  ```bash
  git switch main && git pull
  git switch -c feat/mi-cambio
  # ... commits ...
  git push -u origin feat/mi-cambio   # y abrir una Pull Request en GitHub
  ```
- La PR se fusiona cuando la CI (build) pasa y la ha mirado la otra persona.
- Para traer cambios nuevos de `main` a tu rama: `git pull --rebase origin main` si la rama es solo tuya, `git merge main` si la compartís. No hacer `push --force` sobre ramas compartidas ni sobre `main`.

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

GPU NVIDIA en el contenedor (OpenGL para RViz y PlotJuggler; el compose usa `gpus: all`):

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

La imagen trae `plotjuggler4` (repo `apt.plotjuggler.io`, clave en `docker/plotjuggler.asc`).

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
GO2_IFACE=enP8p1s0 source scripts/env.sh          # interfaz de la NX en la red del robot
```

### Internet en la NX a través del PC

El PC se conecta por cable a la red del robot (192.168.123.x) y comparte su WiFi. No es persistente: se deshace con `down` o reiniciando.

```bash
# PC (cualquier Linux): detecta solo el cable al robot y la interfaz con internet
scripts/pc_share_internet.sh up            # o: up <cable_robot> <interfaz_internet>
# NX (por ssh desde el PC): IP del PC en la red del robot (mirarla con 'ip -br a' en el PC).
# La interfaz de la NX (enP8p1s0) se detecta sola.
scripts/nx_internet.sh up 192.168.123.99
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

`sim_go2w` hace de robot: publica `/lowstate` y `/lf/sportmodestate` (como el robot) y ejecuta `/api/sport/request` (Move, StopMove, StandUp, StandDown...). El driver corre igual que con el robot real, así que `/cmd_vel`, `/mode`, `/odom`, TF y `/joint_states` funcionan. Es solo cinemático: sin física ni LiDAR (no hay `/scan`). No lanzarlo con el robot conectado.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
ros2 service call /mode go2_interfaces/srv/Mode "mode: 'stand_down'"
```

### Prueba de navegación (`go2w_nav_test`)

Recorre los `waypoints` de `go2w_nav_test/config/waypoint_follower.yaml` (por defecto un cuadrado de 1 m a la izquierda), relativos a donde está el robot al arrancar el nodo. Va a 0.3 m/s y 0.5 rad/s como máximo. Se para mientras haya algo a menos de `stop_distance` (0.5 m) delante en `/scan`, y si dejan de llegar `/odom` o `/scan`. Termina al llegar al último waypoint; Ctrl+C lo para (el watchdog del driver detiene el robot en 0.5 s).

```bash
ros2 launch go2w_bringup go2w.launch.py                         # con el robot (publica /scan)
ros2 service call /mode go2_interfaces/srv/Mode "mode: 'stand_up'"
ros2 launch go2w_nav_test waypoint_follower.launch.py

ros2 launch go2w_bringup go2w.launch.py sim:=true               # en simulación (sin /scan)
ros2 launch go2w_nav_test waypoint_follower.launch.py   # con use_scan: false en el yaml
```

## Pendiente de verificar en el robot

Verificado (2026-10-20): **índices de las ruedas** 12-15 = FR, FL, RR, RL, con las patas en el orden de Unitree.

1. **Frame de la nube**: `/utlidar/cloud` debe salir alineada con el suelo en RViz. Si no, ajustar `utlidar_lidar_joint` en el URDF.
2. **Odometría**: el Go2-W no publica odometría (`sportmodestate` da position/velocity a 0 y `/utlidar/robot_odom` no publica; Unitree aún no la ha sacado para el Go2-W). Se usa FAST-LIO sobre `/utlidar/cloud` + `/utlidar/imu`. Arrancar con el robot quieto ~2 s. Comprobar: 1 m adelante → `/odom` x≈1; giro 90° → yaw≈1.57; con las ruedas girando en el aire `/odom` no se mueve. Si los ejes salen girados, revisar `utlidar_lidar_joint`. Alternativa: `lio:=false` (odometría por ruedas).
3. **Move en modo ruedas**: confirmar que el Go2-W acepta la API 1008 igual que el Go2. Probar con el robot en espacio libre y velocidades bajas.
4. **QoS**: si algún topic de Unitree no llega, revisar `ros2 topic info -v`.

## Siguiente paso: EasyNav

EasyNav 0.5.0 (EasyNavigation, easynav_plugins, NavMap, yaets) se clona en `src/external/` desde `deps.repos`; se compila con el resto del workspace.
