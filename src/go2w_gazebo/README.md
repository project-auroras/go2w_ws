# go2w_gazebo — simulación del Go2-W en Gazebo (detección de objetos y sigue-personas)

Simulación en **Gazebo Fortress** para probar, antes de pasar al robot físico, que el Go2-W detecta objetos con YOLO26 y sigue a una persona.

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
- Gazebo Fortress (`ros_gz`). No viene en la imagen: lo instala `scripts/install_deps.sh ws`.

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
| `worlds/campo.sdf` | Campo de cultivo con surcos, plantas, árboles y la persona (generado) |
| `scripts/generar_campo.py` | Genera `worlds/campo.sdf` |
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
