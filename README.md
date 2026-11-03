# go2w_ws

Workspace ROS 2 Humble para el Unitree Go2-W (ruedas). Convierte los mensajes de Unitree (`unitree_go`, `unitree_api`) en topics estándar de ROS 2 y publica `robot_description` y las TF.

| Paquete | Qué hace |
|---|---|
| `go2w_description` | URDF y mallas del Go2-W (de `unitreerobotics/unitree_ros`), con raíz `base_link` y frame `utlidar_lidar` |
| `go2w_driver` | `sim_go2w` para trabajar sin robot (`sim:=true`). `lowstate` → `/joint_states`, `/imu`, `/battery_state`; `sportmodestate` → `/odom` y TF `odom → base_footprint → base_link`; `/cmd_vel` → `/api/sport/request` (Move) con watchdog; servicio `/mode` |
| `go2w_bringup` | Lanza description + driver + `/utlidar/cloud` → `/scan` + RViz |
| `go2w_nav_test` | `waypoint_follower`: recorre waypoints con `/odom` → `/cmd_vel` y se para si `/scan` ve algo delante. Para probar el robot |
| `go2w_perception` | Detección de personas con YOLO26 (`yolo_ros`) en la RealSense: `people_bag.launch.py` alinea la profundidad del rosbag al color, lanza `yolo_node` (detección + tracking + 3D) y `person_filter` publica solo las personas en `/people` |

Paquetes de terceros copiados como código en `src/` (sin `.git`; para actualizarlos, sustituir la carpeta por la nueva versión y anotar aquí el commit):

| Paquete | Origen | Versión | Commit |
|---|---|---|---|
| `EasyNavigation` | [EasyNavigation/EasyNavigation](https://github.com/EasyNavigation/EasyNavigation) | 0.5.0 | `9ca3cca` |
| `easynav_plugins` | [EasyNavigation/easynav_plugins](https://github.com/EasyNavigation/easynav_plugins) | 0.5.0 | `2580d29` |
| `NavMap` | [EasyNavigation/NavMap](https://github.com/EasyNavigation/NavMap) | 0.6.0 | `c96d8c8` |
| `yaets` | [fmrico/yaets](https://github.com/fmrico/yaets) | 1.2.0 | `1452f14` |
| `yolo_ros` | [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros) | 5.0.0 | `7924a8b` |

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

ArduPilot SITL + Mission Planner + NTRIP (contenedor `go2w-ardu`): ver [MISSIONPLANNER.md](MISSIONPLANNER.md).

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

### Detección de personas (YOLO26, `go2w_perception`)

`yolo_ros` es C++ con ONNX Runtime; el modelo (`yolo26s.onnx`) se descarga solo de Hugging Face (`unileon-robotics/YOLO26-ONNX`) la primera vez. La imagen Docker trae CUDA 13 + cuDNN 9 (la RTX 50xx necesita CUDA 13), pero `yolo_onnxruntime_vendor` compila para CPU salvo que se le pida GPU:

```bash
# dentro del contenedor, una vez (borra la compilación CPU anterior)
rm -rf build/yolo_onnxruntime_vendor build/yolo_ros install/yolo_onnxruntime_vendor install/yolo_ros
scripts/install_deps.sh ws        # depth_image_proc
colcon build --symlink-install --packages-up-to go2w_perception --cmake-args -DONNX_GPU=ON -DONNX_CUDA_MAJOR=13
source scripts/env.sh
```

Los rosbags se montan en `/bags` desde `~/rosbags` del host (otra carpeta con `BAGS_DIR=...`). Con el rosbag de la RealSense:

```bash
ros2 launch go2w_perception people_bag.launch.py
ros2 bag play /bags/rosbag2_2026_10_28-10_44_19 --loop \
  --topics /camera/camera/color/image_raw /camera/camera/color/camera_info \
           /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info
ros2 run rqt_image_view rqt_image_view /yolo/cam0/debug_image
ros2 topic echo /people           # yolo_msgs/DetectionArray, bbox3d.center = x y z (m)
```

- El log de `yolo_node` debe decir `Using execution provider: cuda`; si dice `cpu`, falta cuDNN o no se compiló con `-DONNX_GPU=ON`.
- Las posiciones 3D están en `camera_color_optical_frame` (z hacia delante, x derecha, y abajo): los rosbags no tienen TF a `base_link`. Los extrínsecos depth → color están fijos en `people_bag.launch.py` (leídos del topic `extrinsics/depth_to_color` del rosbag).
- Con la cámara real no hace falta alinear: lanzar la RealSense con `align_depth.enable:=true` y usar `aligned_depth_to_color`.
- Los pesos de Ultralytics (YOLO26) son AGPL-3.0, no MIT como `yolo_ros`.

## Pendiente de verificar en el robot

Verificado (2026-10-20): **índices de las ruedas** 12-15 = FR, FL, RR, RL, con las patas en el orden de Unitree.

1. **Frame de la nube**: `/utlidar/cloud` debe salir alineada con el suelo en RViz. Si no, ajustar `utlidar_lidar_joint` en el URDF.
2. **Odometría**: comprobar que `velocity` de `sportmodestate` está en el frame del mundo y que la altura de `base_link` es razonable (`base_height_offset`).
3. **Move en modo ruedas**: confirmar que el Go2-W acepta la API 1008 igual que el Go2. Probar con el robot en espacio libre y velocidades bajas.
4. **QoS**: si algún topic de Unitree no llega, revisar `ros2 topic info -v`.

## Siguiente paso: EasyNav

EasyNav 0.5.0 (EasyNavigation, easynav_plugins, NavMap, yaets) está en `src/` como código; se compila con el resto del workspace.
