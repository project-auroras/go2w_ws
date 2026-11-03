# go2w_perception — detección y seguimiento de personas con YOLO26

Guía paso a paso para detectar personas con YOLO26 (`yolo_ros` + ONNX Runtime en GPU) en un rosbag de la RealSense del Go2-W, en el PC y dentro de Docker.

## Qué hace

```
rosbag (RealSense)
  ├─ color/image_raw + camera_info ───────────────────────────┐
  └─ depth/image_rect_raw + camera_info                       │
        │                                                     ▼
        ▼                                         yolo_node (/yolo)
  depth_image_proc::RegisterNode ──► aligned_depth ──►  detección YOLO26s
  (proyecta la profundidad sobre el color,             + tracking ByteTrack
   usando el TF estático depth→color)                  + 3D con la profundidad
                                                       + debug (imagen y markers)
                                                              │
                                              /yolo/cam0/detections_3d
                                                              ▼
                                                       person_filter
                                                              │
                                                              ▼
                                                  /people (solo "person")
```

| Archivo | Contenido |
|---|---|
| `launch/people_bag.launch.py` | TF estático depth→color, `RegisterNode`, `yolo_bringup/yolo.launch.py` con nuestro yaml y `person_filter` |
| `config/yolo_people.yaml` | `yolo26s.onnx`, `provider: cuda`, plugins detection + tracking + detection3d + debug |
| `go2w_perception/person_filter.py` | Republica solo la clase `person` (desde la versión 5.0, `yolo_ros` no filtra por clase) |
| `go2w_perception/person_follower.py` | `/people` → `/cmd_vel`: sigue a una persona por su ID de tracking |
| `go2w_perception/fake_person.py` | Personas virtuales que publican `/people` para simular sin cámara |
| `launch/person_follower_sim.launch.py` | `sim_go2w` + TF de la cámara + `fake_person` + `person_follower` + RViz |
| `config/person_follower.yaml` | Parámetros de `person_follower` y `fake_person` |

## Requisitos

- PC con GPU NVIDIA y `nvidia-container-toolkit` (ver el README principal). Probado con una RTX 5060 (Blackwell), que necesita **CUDA 13**.
- Imagen `go2w:humble`, construida con el `docker/Dockerfile` del repo, que incluye CUDA 13 y cuDNN 9.
- Rosbags en `~/rosbags` del host. El contenedor los monta en `/bags`; para usar otra carpeta, pon `BAGS_DIR=/otra/carpeta`.
- El rosbag tiene que llevar color y profundidad de la RealSense. El que se usó es `rosbag2_2026_10_28-10_44_19`. El de las `10_39_32` solo tiene lidar y odometría.

## 1. Construir la imagen (una sola vez o cuando cambie el Dockerfile)

```bash
cd ~/go2w_provisional
docker compose -f docker/compose.yaml build
```

## 2. Arrancar el contenedor

```bash
cd ~/go2w_provisional
xhost +local:docker                # para que se abran rqt y RViz desde el contenedor
docker compose -f docker/compose.yaml run --rm --name go2w-yolo go2w
```

`--name` sirve para entrar desde otras terminales con `docker exec`. Como el contenedor usa `network_mode: host`, cualquier contenedor o el propio host ven los topics.

## 3. Compilar con GPU (una sola vez)

`yolo_onnxruntime_vendor` compila ONNX Runtime para CPU por defecto. Dentro del contenedor:

```bash
# borra una compilación CPU anterior, si la hay
rm -rf build/yolo_onnxruntime_vendor build/yolo_ros install/yolo_onnxruntime_vendor install/yolo_ros

scripts/install_deps.sh ws         # rosdep: depth_image_proc, etc.
colcon build --symlink-install --packages-up-to go2w_perception \
  --cmake-args -DONNX_GPU=ON -DONNX_CUDA_MAJOR=13
source scripts/env.sh
```

`build/` e `install/` están en `/ws`, que es la carpeta del repo montada, así que la compilación se mantiene entre contenedores.

## 4. Lanzar la detección (terminal 1, dentro del contenedor)

Cada `docker compose run --rm` crea un contenedor nuevo y pierde lo instalado con apt. Por eso `install_deps.sh ws` hay que ejecutarlo en cada sesión:

```bash
scripts/install_deps.sh ws         # instala depth_image_proc
source scripts/env.sh
ros2 launch go2w_perception people_bag.launch.py
```

La primera vez, el modelo `yolo26s.onnx` se descarga de Hugging Face (`unileon-robotics/YOLO26-ONNX`).

**Comprobación:** el log de `yolo_node` tiene que decir `Using execution provider: cuda`.

## 5. Reproducir el rosbag (terminal 2)

```bash
docker exec -it go2w-yolo bash
source scripts/env.sh
ros2 bag play /bags/rosbag2_2026_10_28-10_44_19 --loop \
  --topics /camera/camera/color/image_raw /camera/camera/color/camera_info \
           /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info
```

## 6. Ver el resultado (terminal 3)

```bash
docker exec -it go2w-yolo bash
source scripts/env.sh

ros2 run rqt_image_view rqt_image_view /yolo/cam0/debug_image   # imagen con cajas, clase e ID
ros2 topic echo /people                                          # solo personas
ros2 topic hz /people
```

En `/people` (`yolo_msgs/DetectionArray`), cada detección trae:
- `id`: el ID del tracking. Se mantiene mientras la persona sigue a la vista.
- `bbox`: la caja 2D en píxeles.
- `bbox3d.center.position`: la posición en metros, en `camera_color_optical_frame`, con **z hacia delante, x a la derecha e y hacia abajo**.

### En RViz

```bash
rviz2
```

- Fixed Frame: `camera_color_optical_frame`.
- Add → Image: `/yolo/cam0/debug_image`.
- Add → MarkerArray: `/yolo/cam0/debug_bb_markers` (cajas 3D) y `/yolo/cam0/debug_kp_markers` (keypoints).

## 7. Grabar un vídeo con las detecciones (opcional)

`image_view` no viene en la imagen. Se instala en cada sesión:

```bash
sudo apt update && sudo apt install -y ros-humble-image-view
mkdir -p /bags/videos
ros2 topic hz /yolo/cam0/debug_image            # anota la frecuencia real
ros2 run image_view video_recorder --ros-args \
  -r image:=/yolo/cam0/debug_image \
  -p filename:=/bags/videos/personas.avi -p fps:=<hz real> -p codec:=MJPG
```

Para grabar, reproduce el bag **sin `--loop`** y pulsa Ctrl+C al terminar para cerrar el fichero. El vídeo queda en `~/rosbags/videos/` del host.

## 8. Sigue-personas simulado (sin cámara ni robot)

Prueba el control del sigue-personas en bucle cerrado con el robot simulado (`sim_go2w`) y personas virtuales:

```
fake_person ──► /people ──► person_follower ──► /cmd_vel ──► go2w_driver ──► sim_go2w ──► /odom + TF
     ▲                                                                                     │
     └──────────────── TF odom → camera_color_optical_frame (dónde está la cámara) ─────────┘
```

- `fake_person`: dos personas recorren `path` y `distractor_path` en el frame `odom`. Solo publica en `/people` las que caen dentro del campo de visión de la RealSense (69° y de 0,3 a 6 m), con ruido y un 10 % de fallos de detección. Usa el mismo formato que `person_filter`, en `camera_color_optical_frame`.
- `person_follower`: escoge la persona más cercana y se queda con su ID. Avanza hasta quedarse a `target_distance` y gira para tenerla centrada. Si no la ve durante `lost_timeout`, se para y suelta el ID. Nunca va marcha atrás.
- En el launch, el TF `base_link → camera_color_optical_frame` es una posición **provisional** de la cámara.
- Los parámetros de los dos nodos están en `config/person_follower.yaml`.

```bash
# una vez, o al cambiar setup.py
colcon build --symlink-install --packages-select go2w_perception
source scripts/env.sh

ros2 launch go2w_perception person_follower_sim.launch.py
```

En RViz, el cilindro verde es la persona 1 y el naranja la 2 (distracción); se ven translúcidas cuando están fuera de vista. Las líneas amarillas marcan el campo de visión y la flecha roja es la odometría del robot.

Variantes:

```bash
# robot y personas sin follower, para lanzarlo aparte con otros parámetros
ros2 launch go2w_perception person_follower_sim.launch.py follower:=false
ros2 run go2w_perception person_follower --ros-args \
  --params-file install/go2w_perception/share/go2w_perception/config/person_follower.yaml \
  -p target_distance:=2.0 -p max_linear:=0.3

# ver lo que manda al robot
ros2 topic echo /cmd_vel
```

Probar `person_follower` con el rosbag real, sin robot: lanza `people_bag.launch.py` + `ros2 bag play ...` (pasos 4 y 5), luego `ros2 run go2w_perception person_follower` y mira `ros2 topic echo /cmd_vel`.

## Problemas habituales

| Síntoma | Solución |
|---|---|
| `Using execution provider: cpu` | Falta cuDNN o no se compiló con `-DONNX_GPU=ON -DONNX_CUDA_MAJOR=13`: repite el paso 3 |
| `package 'depth_image_proc' not found` | Ejecuta `scripts/install_deps.sh ws` en este contenedor |
| No se abre rqt o RViz | En el host: `xhost +local:docker` |
| Aparecen topics de otros sistemas (`go2w-ardu`, `middlewares_container`) | Arranca con otro dominio: `ROS_DOMAIN_ID=5 docker compose -f docker/compose.yaml run ...` y usa el mismo en todas las terminales |
| No hay detecciones 3D | Comprueba que llega `/camera/camera/aligned_depth/image` (`ros2 topic hz`) y que el bag tiene profundidad |

## Notas

- Los rosbags no tienen `/tf_static`. Los extrínsecos depth→color de la RealSense se leyeron del topic `/camera/camera/extrinsics/depth_to_color` del bag y están fijos en `people_bag.launch.py`.
- Con la cámara real no hace falta `RegisterNode`: basta con lanzar `realsense2_camera` con `align_depth.enable:=true` y usar `aligned_depth_to_color/image_raw`.
- Las posiciones 3D están en el frame de la cámara. Para usarlas en el robot falta el TF `base_link → camera_link` en el URDF.
- `src/yolo_ros` es código de terceros (v5.0.0) y no se modifica.
- Los pesos YOLO26 de Ultralytics tienen licencia AGPL-3.0.
