# go2w_perception — detección y seguimiento de personas con YOLO26 y navegación semántica

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
| `launch/people_live.launch.py` | RealSense real (en la NX) → descompresión + YOLO + `person_filter` (+ `person_follower` con `follower:=true`) en el portátil |
| `config/yolo_live.yaml` | `yolo_ros` con la cámara real, posiciones 3D en `base_link` |
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

- `fake_person`: dos personas recorren `path` y `distractor_path` en el frame `odom`. Solo publica en `/people` las que caen dentro del campo de visión de la RealSense (69° y de 0,3 a 6 m), con ruido, un 10 % de fallos de detección y un ID nuevo cada vez que vuelven a entrar en la imagen (como ByteTrack). Usa el mismo formato que `person_filter`, en `camera_color_optical_frame`.
- `person_follower`: escoge la persona más cercana (a menos de 3 m) y se queda con su ID. Gira para tenerla centrada y avanza para mantener la distancia. Si la persona se le acerca a menos de `backoff_distance`, retrocede despacio. Si no la ve, sigue hacia donde predice que está; a los 0,5 s frena, la recuerda 3 s y se reengancha si aparece cerca de donde debería estar, aunque con otro ID.
- **Velocidad de la persona**: `person_follower` pasa cada detección al frame `odom` (con `/odom`) y estima la velocidad de la persona respecto al suelo con un filtro alfa-beta (`track_alpha`, `track_beta`). La usa como base de la orden: `vx = v_persona + k_linear · (distancia − objetivo)`. Así acelera a la vez que ella, en lugar de esperar a que se aleje. La distancia objetivo crece con la velocidad (`target_distance + time_gap · v`) para tener margen de frenada, y los cambios de velocidad van con rampa (`max_accel`, `max_decel`, `max_angular_accel`). Si la persona está andando (`moving_speed`), no se para a girar aunque esté ladeada.
- Para probar varios ritmos: `speed` de `fake_person` (m/s constantes) o `speed_profile` (m/s de cada tramo de `path`, p. ej. `[0.5, 1.4, 0.5, 1.0]` para una persona que acelera de golpe). En simulación sigue sin perderla de 0,4 a 1,5 m/s, a unos 2–2,5 m.
- La posición de la cámara sale del URDF (`base_link → camera_link`, medida en el robot); el launch solo añade `camera_link → camera_color_optical_frame`, que en el robot real publica el driver.
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

## 9. Con la RealSense real (YOLO en el portátil)

La cámara va por USB a la NX y YOLO corre en el portátil, con su GPU. La NX manda la imagen comprimida por WiFi (color en JPEG y profundidad en PNG sin pérdida); `people_live.launch.py` la descomprime, lanza YOLO y publica las personas en `/people`. Las posiciones salen en **`base_link`** (x delante, y izquierda), con la posición e inclinación de la cámara del URDF.

Red: el portátil está en la red del robot por WiFi (`wlo1`, `192.168.123.103`) y la NX por cable (`enP8p1s0`, `192.168.123.18`).

**NX** (cada terminal con `export GO2_IFACE=enP8p1s0 && source scripts/env.sh`):

```bash
ros2 launch go2w_bringup go2w.launch.py
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 depth_module.depth_profile:=640x480x15
```

**Portátil**:

```bash
# host
export GO2_IFACE=wlo1
docker compose -f docker/compose.yaml run --rm --name go2w-cam go2w

# contenedor
scripts/install_deps.sh ws            # image_transport (compressed / compressedDepth) y rqt_image_view
colcon build --symlink-install --packages-select go2w_perception
source scripts/env.sh
ros2 launch go2w_perception people_live.launch.py
```

Comprobar (otra terminal: `docker exec -it go2w-cam bash` y `source scripts/env.sh`):

```bash
ros2 topic hz /camera_local/color/image_raw            # imagen descomprimida (~15 Hz)
ros2 topic hz /camera_local/aligned_depth/image_raw
ros2 run rqt_image_view rqt_image_view /yolo/cam0/debug_image
ros2 topic echo /people --field detections             # bbox3d.center.position en base_link
```

Sigue-personas (**el robot se mueve**: zona despejada y alguien con el mando):

```bash
ros2 launch go2w_perception people_live.launch.py follower:=true            # 0.6 m/s, 0.8 rad/s
ros2 launch go2w_perception people_live.launch.py follower:=true max_linear:=0.2
```

- Antes de `follower:=true`, mira con `follower:=false` y `ros2 run go2w_perception person_follower` + `ros2 topic echo /cmd_vel` que las órdenes tienen sentido (persona a la derecha → `angular.z` negativo).
- `go2w_driver` para el robot si no recibe `/cmd_vel` en 0,5 s: si el WiFi se corta, se para.
- Para seguir a alguien andando a paso normal (1,2–1,4 m/s) hace falta `max_linear` de ~1,6 (el de `person_follower.yaml`; `go2w_driver` también recorta a `max_vx`, 1,6). Por seguridad, `people_live.launch.py` lo deja en 0,6 por defecto: súbelo poco a poco (`max_linear:=1.0`, luego `1.6`) en una zona amplia.
- **No frena ante obstáculos**: solo sigue a la persona. Zona despejada y alguien con el mando.
- Si la persona se acerca a menos de `backoff_distance` (1,3 m del centro, ~1 m del morro), retrocede a `max_backward` (0,3 m/s) sin dejar de mirarla. Detrás no hay cámara: hace falta espacio libre detrás del robot.
- Con la cámara real, ByteTrack cambia a menudo el ID de la persona (de cerca, o cuando llegan frames a saltos por el WiFi). Por eso `person_follower`:
  - se **para** a los `stop_timeout` (0,5 s) sin verla, pero la **recuerda** durante `lost_timeout` (3 s);
  - si mientras tanto aparece una detección a menos de `reacquire_radius` (1 m) de donde estaba, se **reengancha** a ella aunque tenga otro ID (`Persona X reenganchada como Y`);
  - al empezar, solo elige personas a menos de `max_start_distance` (3 m).
- `person_filter` descarta las detecciones con `score` < `min_score` (0,6) o una caja de menos de `min_height_px` (60 px) de alto: falsos positivos como trozos en el borde de la imagen.

## 10. Navegación semántica (modelo propio)

Nodos para navegar con un modelo YOLO26 entrenado con clases propias (`models/yolo26m_datasetFinal.onnx`: Row, Trace, Trunk, Post, Pothole, Puddle, person, animales y frutas). `yolo_ros` lo carga con `model_path`. El modelo se exportó sin NMS, así que lo aplica `yolo_ros`, y las clases las lee de los metadatos del ONNX.

| Nodo | Entrada | Salida |
|---|---|---|
| `semantic_mapper` | `/yolo/cam0/detections_3d` + TF | `/semantic/landmarks` (`DetectionArray` en `map`, `id` = objeto), `/semantic/markers`, servicio `~/save` |
| `semantic_goal` | `/semantic/goal_request` (`"Post"`, `"Post:3"`) | `/goal_pose` para EasyNav, `/semantic/goal_status` |
| `row_follower` | `/yolo/cam0/detections_3d` (Row, Trace), `/semantic/row_follow` (Bool) | `/goal_pose` sobre el eje del pasillo, `/semantic/row_status`, `/semantic/row_markers` |
| `class_stats` | `/yolo/cam0/detections_3d` | Recuento por clase cada 5 s en el log (para ver qué detecta el modelo) |

Parámetros en `config/semantic_nav.yaml`. Cómo probarlo en Gazebo: [go2w_gazebo/README.md](../go2w_gazebo/README.md#navegación-semántica-modelo-propio--easynav).

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
- Las posiciones 3D están en el frame de la cámara: los rosbags de octubre no tienen TF. La posición de la cámara en el robot (`base_link → camera_link`) ya está en el URDF de `go2w_description`, así que con la cámara real se puede poner `target_frame: base_link`.
- `src/yolo_ros` es código de terceros (v5.0.0) y no se modifica.
- Los pesos YOLO26 de Ultralytics tienen licencia AGPL-3.0.
