# go2w_gazebo — simulación del Go2-W en Gazebo (detección de objetos, sigue-personas y navegación semántica)

Simulación en **Gazebo Harmonic** para probar, antes de pasar al robot físico, que el Go2-W detecta objetos con YOLO26, sigue a una persona y navega por el campo con un modelo YOLO propio (navegación semántica, [abajo](#navegación-semántica-modelo-propio--easynav)).

Usa la misma cadena que el robot real. Solo cambia de dónde sale la imagen (Gazebo en lugar de la RealSense) y a dónde va `/cmd_vel` (las ruedas de Gazebo en lugar de `go2w_driver`):

```
Gazebo (RealSense simulada) ──ros_gz_bridge──► /camera/camera/... ──► yolo_node ──► person_filter ──► /people
                                                                                                        │
Gazebo (ruedas, DiffDrive) ◄──ros_gz_bridge── /cmd_vel ◄───────────────── person_follower ◄────────────┘
```

## Qué hay en la escena

`worlds/objetos.sdf`. El robot empieza en (0, 0) mirando a +x. Los modelos se descargan de [Gazebo Fuel](https://app.gazebosim.org/fuel/models) la primera vez.

| Objeto | Posición (x, y) | Clase de YOLO |
|---|---|---|
| Persona de pie | (4, 0) | person |
| Persona andando (quieta) | (6, -2) | person |
| Mujer | (5, 2.5) | person |
| Mesa | (3, -3) | dining table |
| Silla | (2.5, 3) | chair |
| Boca de incendios | (8, 1) | fire hydrant |
| Señal de stop | (9, -1.5) | stop sign |
| Coche | (12, 3) | car |
| Cono de obra | (2, -1) | ninguna (para ver falsos positivos) |
| Caja roja | (3, 1) | ninguna (objeto hecho a mano; YOLO la confunde con "car") |
| **Persona que camina** | rectángulo x 3..9, y 7..10 | person, la que sigue el robot |

## El robot en Gazebo

`urdf/go2w_gazebo.urdf.xacro` incluye el URDF de `go2w_description` y le añade:
- **Patas:** un controlador de posición en cada articulación que las sujeta en la postura de pie. No camina con las patas.
- **Ruedas:** tracción diferencial con las 4 ruedas. Lee `/cmd_vel` y publica `/odom` y la TF `odom → base_link`.
- **RealSense simulada:** cámara RGB-D de 69°, 640x480 y 15 Hz, en el `camera_link` del URDF de `go2w_description`: la posición medida en el robot real (31,8 cm delante y 10 cm arriba de `base_link`, inclinada 20° hacia arriba).
- **Ruedas con poco agarre lateral:** con 4 ruedas fijas, el robot gira derrapando. Con mucho agarre no giraba y llegaba a volcar.

`launch/escena.launch.py` cambia la colisión de las ruedas (que en el URDF original es la malla 3D) por un cilindro. Con la malla, la simulación iba unas 1.000 veces más lenta que el tiempo real.

## Requisitos

- Haber seguido el README de `go2w_perception` (pasos 1 a 3): imagen Docker y `yolo_ros` compilado con GPU.
- Gazebo Harmonic y su `ros_gz`: vienen en la imagen Docker (`docker compose -f docker/compose.yaml build` si la imagen es anterior y `gz` no existe).

## Lanzar

**1. En el host:**

```bash
cd ~/go2w_provisional
xhost +local:docker
docker compose -f docker/compose.yaml run --rm --name go2w-gz go2w
```

**2. En el contenedor.** Con `--rm`, Gazebo se borra al cerrar el contenedor, así que `install_deps.sh` hay que ejecutarlo en cada sesión:

```bash
scripts/install_deps.sh ws
colcon build --symlink-install --packages-select go2w_gazebo   # la primera vez o si hay carpetas nuevas
source scripts/env.sh
```

### Detección de objetos (el robot quieto)

```bash
ros2 launch go2w_gazebo simulacion.launch.py
```

Se abren Gazebo y RViz. El panel **YOLO** de RViz muestra la imagen de la cámara del robot con las cajas, la clase y el ID. Las cajas 3D salen en la vista 3D.

Para ver qué detecta, desde otra terminal (`docker exec -it go2w-gz bash` y `source scripts/env.sh`):

```bash
ros2 topic echo /yolo/cam0/detections_3d --field detections | grep class_name
```

Las posiciones (`bbox3d.center.position`) salen en `base_link`, con x hacia delante e y a la izquierda.

### Sigue-personas

```bash
ros2 launch go2w_gazebo simulacion.launch.py follower:=true y:=7.0
```

- `y:=7.0` pone el robot junto al recorrido de la persona que camina, que está 3 m delante.
- El robot va detrás de ella y se queda a unos 1,5 m. Los parámetros están en `go2w_perception/config/person_follower.yaml`.
- En la terminal salen mensajes como `Siguiendo a la persona N` y `id N: d=... -> vx=..., wz=...`.

### Campo de cultivo (`world:=campo`)

Mundo vacío con un campo: 6 surcos de 18 m (lechugas, maíz y tomates), malas hierbas, árboles alrededor y una persona que camina por los pasillos entre surcos.

```bash
ros2 launch go2w_gazebo simulacion.launch.py world:=campo follower:=true
```

```
y=4.5  ===== tomates ==========================
y=3.75   <---------- persona (vuelta) ----------
y=3.0  ===== tomates ==========================
y=1.5  ===== maíz =============================
y=0.75 R  ---------- persona (ida) ------------>
y=0.0  ===== maíz =============================
y=-1.5 ===== lechugas =========================
y=-3.0 ===== lechugas =========================
       x=2                                  x=20
```

- El robot (R) sale en (-1.5, 0.75), en la cabecera, y la persona empieza 2,5 m delante.
- La persona va por el pasillo y=0.75 a 0,5 m/s, gira en la cabecera de x=23, vuelve por el pasillo y=3.75 y repite (vuelta de 104 s).
- Los surcos miden 50 cm de ancho y 12 cm de alto y tienen colisión: si el robot se sale del pasillo, choca con ellos. Los pasillos miden 1 m. Las plantas no tienen colisión.
- El maíz (1,3–1,8 m) tapa a la persona cuando está en otro pasillo.
- Lo genera `scripts/generar_campo.py` (es un archivo grande y repetitivo, no se edita a mano). Para cambiar los cultivos, los surcos, los árboles o el recorrido de la persona, cambia las constantes del principio del script y ejecútalo: `python3 src/go2w_gazebo/scripts/generar_campo.py`.

### Argumentos

| Argumento | Por defecto | Qué hace |
|---|---|---|
| `follower` | `false` | `true`: lanza `person_follower` |
| `world` | `objetos` | Mundo: `objetos`, `campo` o la ruta a un `.sdf` |
| `x`, `y`, `yaw` | según el mundo | Posición inicial del robot (m, rad). `objetos`: (0, 0); `campo`: (-1.5, 0.75) |
| `rviz` | `true` | Abre RViz |
| `gui` | `true` | `false`: Gazebo sin ventana |

Solo el mundo y el robot, sin YOLO: `ros2 launch go2w_gazebo escena.launch.py`.

Mover el robot a mano (sin `follower:=true`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Navegación semántica (modelo propio + EasyNav)

El robot navega por el campo con lo que ve un modelo YOLO26m entrenado con clases propias (`yolo26m_datasetFinal.onnx`, 47 clases). Las que se usan para navegar son **Row** (surco), **Trace** (rodada), **Trunk** (tronco), **Post** (poste), **Pothole** (bache) y **Puddle** (charco). Usa el robot de `go2w_gis.launch.py`, que tiene lidar, RealSense y los mismos topics que el real, y EasyNav para moverse:

```
RealSense sim ─► yolo_node (modelo propio, 3D en base_link) ─► /yolo/cam0/detections_3d
                       │                                             │
                       ▼                                             ▼
              semantic_mapper ─► /semantic/landmarks         row_follower ◄── /semantic/row_follow (Bool)
                       │                                             │
 /semantic/goal_request ─► semantic_goal ──────────► /goal_pose ◄────┘
                                                        │
                                   EasyNav (NavMap plano del campo, lidar) ─► /cmd_vel ─► Gazebo
```

- `semantic_mapper`: acumula en `map` los objetos estáticos que ve (Row, Trace, Trunk, Post, Pothole, Puddle). Un objeto se confirma cuando lo ha visto 3 veces. En RViz salen como cajas con la etiqueta `Clase:id`.
- `semantic_goal`: con `"Post"` va al poste más cercano del mapa y con `"Post:3"` al poste con id 3. Se para a 1 m del objeto, mirándolo.
- `row_follower`: ajusta una recta a los surcos de cada lado y va mandando metas 3 m por delante sobre el eje del pasillo. Cuando deja de ver surcos (llega a la cabecera), se para.
- Parámetros: `go2w_perception/config/semantic_nav.yaml`. YOLO: `config/yolo_semantic_gazebo.yaml`. EasyNav: `config/easynav_campo_sim.params.yaml` (en la raíz del repo).

### Preparación (una vez)

```bash
# host: el modelo no va en git (82 MB). models/ del repo = /ws/models en el contenedor
cp ~/yolo26m_datasetFinal.onnx ~/go2w_ws/models/
```

Dentro del contenedor, compilar como en el README de `go2w_perception` (yolo_ros con GPU) y:

```bash
colcon build --symlink-install --packages-select go2w_gazebo go2w_perception
source scripts/env.sh
```

### Lanzar

```bash
# terminal 1: Gazebo (campo) + YOLO + nodos semánticos + RViz
ros2 launch go2w_gazebo semantica.launch.py

# terminal 2: EasyNav
ros2 run easynav_system system_main --ros-args --params-file config/easynav_campo_sim.params.yaml

# terminal 3: qué clases detecta el modelo en la simulación (cada 5 s)
ros2 run go2w_perception class_stats
```

**1. Comprobar qué detecta el modelo.** Esto es lo primero: el modelo se entrenó con fotos reales y la simulación no lo es. Mueve el robot por el campo con `ros2 run teleop_twist_keyboard teleop_twist_keyboard` (con EasyNav parado, para que no se peleen por `/cmd_vel`) y mira la salida de `class_stats` y el panel YOLO de RViz. Si una clase no sale, mejora su aspecto en `scripts/generar_campo.py` (texturas y objetos) o baja `threshold` en `yolo_semantic_gazebo.yaml`.

**2. Mapa semántico.** Al recorrer el campo aparecen los objetos en RViz. Para guardarlo y cargarlo después:

```bash
ros2 service call /semantic_mapper/save std_srvs/srv/Trigger     # -> /ws/maps/semantic_<fecha>.yaml
ros2 launch go2w_gazebo semantica.launch.py semantic_map:=/ws/maps/semantic_<fecha>.yaml
```

**3. Ir a un objeto** (con EasyNav en marcha):

```bash
ros2 topic pub --once /semantic/goal_request std_msgs/msg/String "data: Post"     # el más cercano
ros2 topic pub --once /semantic/goal_request std_msgs/msg/String "data: Trunk:12" # uno concreto
ros2 topic echo /semantic/goal_status
```

**4. Seguir el surco.** Con el robot en la cabecera, mirando a un pasillo (el de salida, y=0.75):

```bash
ros2 topic pub --once /semantic/row_follow std_msgs/msg/Bool "data: true"
ros2 topic echo /semantic/row_status
```

En RViz, **Eje del surco** muestra las detecciones de Row que usa (rojo) y el eje que calcula (verde).

### Qué hay en el campo para el modelo

`scripts/generar_campo.py` genera texturas procedurales en `models/campo_texturas/` (tierra, caballón, madera, corteza, hojas, charco, bache y rodada) y estos objetos:

| Objeto | Dónde | Clase |
|---|---|---|
| Surcos con textura de tierra | y = -3 … 4.5, x 2..20 | Row |
| Postes de madera de 1,2 m | 30 cm antes y después de cada surco; valla en y=6.5 | Post |
| Frutales (tronco con corteza y copa) | y=-5.5, cada 3 m | Trunk |
| Charcos | (8, -2.25), (-0.8, 2.6), (21.5, 0) | Puddle |
| Baches | (12, -0.75), (0.2, -1.5), (22, 3) | Pothole |
| Rodadas de tractor | pasillo y=2.25 y camino y=-4.4 | Trace |

También escribe `maps/campo_flat.pcd`: el suelo plano del campo (x -10..32, y -12..14, z 0). Si cambias ese rectángulo, regenera el NavMap:

```bash
ros2 run navmap_tools pointcloud_to_navmap --input src/go2w_gazebo/maps/campo_flat.pcd \
  --output src/go2w_gazebo/maps/campo_flat.navmap --resolution 0.25 --frame-id map
```

### Sin verificar todavía

- Si el modelo detecta en Gazebo las texturas procedurales. Si no, hay que poner texturas más realistas o reentrenar con imágenes de la simulación.
- Ajustes de EasyNav para pasillos de 1 m (`robot_geometry.radius` 0,3, `inflation_radius` 0,35, obstáculos a partir de 0,15 m). El lidar no ve los caballones de 12 cm hasta unos 2 m de distancia.
- Que `map` coincida con el mundo de Gazebo (el NavMap plano está en coordenadas del mundo).

## Añadir objetos a la escena

**Editando `worlds/objetos.sdf`.** Copia un bloque `<include>` y cambia el nombre, la posición y el modelo. No hace falta recompilar.

```xml
<include>
  <name>otra_silla</name>
  <pose>4 -1 0 0 0 0</pose>     <!-- x y z roll pitch yaw (m, rad) -->
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair</uri>
</include>
```

- Busca los modelos en https://app.gazebosim.org/fuel/models.
- Para que YOLO los detecte, elige objetos de las clases COCO: person, chair, car, bottle, dog, stop sign...

**Desde la ventana de Gazebo:** menú ⋮ → **Resource Spawner** → elige el modelo y arrástralo. Para guardarlo: menú → **Save world as...** encima de `worlds/objetos.sdf`.

## Archivos

| Archivo | Contenido |
|---|---|
| `worlds/objetos.sdf` | La escena: suelo, luz, objetos y la persona que camina |
| `worlds/campo.sdf` | Campo de cultivo con surcos, plantas, árboles, postes, frutales, charcos, baches, rodadas y la persona (generado) |
| `scripts/generar_campo.py` | Genera `worlds/campo.sdf`, `models/campo_texturas/*.png` y `maps/campo_flat.pcd` |
| `models/campo_texturas/` | Texturas procedurales del campo (generadas) |
| `maps/campo_flat.pcd`, `maps/campo_flat.navmap` | Suelo plano del campo para EasyNav |
| `urdf/gazebo.xml` | Lo que `go2w_gis.launch.py` añade al URDF: lidar, IMU, GPS, RealSense, ruedas y odometría |
| `config/yolo_semantic_gazebo.yaml` | `yolo_ros` con el modelo propio (`/ws/models/yolo26m_datasetFinal.onnx`) |
| `launch/go2w_gis.launch.py` | Robot con lidar y RealSense en un mundo de `navmap_gis_tool` (`world_package:=`) o de `worlds/` (`world:=campo`) |
| `launch/semantica.launch.py` | Campo + robot + YOLO propio + `semantic_mapper`, `semantic_goal`, `row_follower` + RViz |
| `rviz/semantica.rviz` | NavMap, ruta, imagen de YOLO, mapa semántico y eje del surco |
| `urdf/go2w_gazebo.urdf.xacro` | El Go2-W para Gazebo: patas sujetas, ruedas y RealSense |
| `config/bridge.yaml` | Puente Gazebo ↔ ROS (`/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, cámara) con los mismos topics que la RealSense real |
| `config/yolo_gazebo.yaml` | `yolo_ros` con la cámara simulada, tiempo simulado y posiciones 3D en `base_link` |
| `launch/escena.launch.py` | Gazebo + robot + puente |
| `launch/simulacion.launch.py` | Escena + YOLO + `person_filter` + `person_follower` (opcional) + RViz |
| `rviz/simulacion.rviz` | Imagen de YOLO, cajas 3D, robot y odometría |

## Problemas habituales

| Síntoma | Solución |
|---|---|
| Gazebo tarda mucho la primera vez o la ventana sale vacía | Está descargando los modelos de Fuel; espera. Se guardan en `~/.ignition/fuel` (dentro del contenedor, así que se pierden con `--rm`) |
| `package 'ros_gz_sim' not found` | Ejecuta `scripts/install_deps.sh ws` en este contenedor |
| La simulación va más lenta que el tiempo real | Es normal (0,5–0,8), porque renderiza la cámara y ejecuta YOLO a la vez |
| La caja roja sale como "car" | Falso positivo de YOLO, no es un fallo de la simulación |
| No detecta la silla, la mesa o el stop desde (0, 0) | La silla y la mesa quedan fuera del campo de visión y el stop está de canto. Mueve el robot (`x:=`, `y:=`, `yaw:=` o teleop) |
| Aparecen robots u objetos de más, o topics de Gazebo raros | Hay otro Gazebo en el host (con `network_mode: host` se mezclan). Ciérralo, o usa otra partición en todas las terminales: `export IGN_PARTITION=otra` |
| No abre la ventana | En el host: `xhost +local:docker` |
