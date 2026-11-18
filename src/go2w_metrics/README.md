# go2w_metrics

Métricas de hardware y de ROS de las pruebas: se graban en el mismo rosbag que las cámaras y luego se pasan a CSV para sacar estadísticas.

| Qué | De dónde | Topic |
|---|---|---|
| CPU (total y por núcleo), carga, RAM, swap, temperatura de CPU, red por interfaz, disco | [`ros2_vitals`](https://github.com/tu-darmstadt-ros-pkg/ros2_vitals) (`collector`), en el portátil y en la NX | `/vitals/status` (`ros2_vitals_msgs/SystemStatus`, `hostname` dice qué máquina) |
| CPU, RAM, hilos, memoria de GPU y red TCP **de cada nodo ROS** | `ros2_vitals` | `/vitals/status` (`processes`) |
| GPU del portátil (carga, VRAM, temperatura, potencia) | `ros2_vitals` con NVML (`python3-pynvml`) | `/vitals/status` (`gpus`) |
| GPU, controlador de memoria (EMC), CPU por núcleo y frecuencia, zonas térmicas y consumo por raíl **de la NX** | `tegrastats` → `jetson_stats` (NVML no existe en Jetson) | `/jetson/stats` (`diagnostic_msgs/DiagnosticArray`) |
| Frecuencia, jitter y latencia de cada topic | el propio bag (hora de llegada vs `header.stamp`) | — |

## Uso

**NX** (además de bringup y cámara):

```bash
ros2 launch go2w_metrics metrics.launch.py jetson:=true
```

**Portátil**: `scripts/record_tests.sh` ya lanza `metrics.launch.py` y graba `/vitals/status` y `/jetson/stats`.

**CSV** de un bag (en el contenedor):

```bash
ros2 run go2w_metrics bag_to_csv /bags/mi_prueba                       # -> /bags/mi_prueba/csv/
ros2 run go2w_metrics bag_to_csv /bags/mi_prueba --clock-offset 1.334  # desfase NX - portátil (palmadas)
ros2 run go2w_metrics bag_to_csv /bags/*                               # todos
```

| Archivo | Una fila por |
|---|---|
| `system.csv` | mensaje de `/vitals/status` y máquina: CPU, RAM, temperatura, GPU del portátil |
| `network.csv` | máquina e interfaz: tx/rx en kbit/s, drops, errores |
| `processes.csv` | máquina y proceso ROS (y cada nodo de un launch, con `parent_pid`): CPU, RAM, GPU, red |
| `jetson.csv` | línea de tegrastats de la NX |
| `messages.csv` | mensaje de cada topic: `t_bag`, `t_stamp`, `latency_ms` |
| `summary.csv` | topic (frecuencia, periodo p95/máx, latencia media/p50/p95/máx) y máquina (media y máximo de cada métrica) |

- `t_bag` (hora a la que llegó al portátil) es el eje de tiempo común de todos los archivos.
- `latency_ms` = `t_bag - header.stamp`. En los topics de la NX incluye el desfase de reloj NX → portátil: mídelo con las palmadas (`scripts/bag_frames.py`) y pásalo con `--clock-offset`. Así, la latencia de `/people` es cámara + WiFi + YOLO.
- `ros2_vitals` se clona en `src/external` desde `deps.repos` (`scripts/install_deps.sh ws`); su GUI `rqml_vitals` (Qt6) no se compila (`COLCON_IGNORE`).

## ros2_tracing (más adelante)

[`ros2_tracing`](https://github.com/ros2/ros2_tracing) mide dentro de ROS: duración de cada callback y latencia de un mensaje de publicación a suscripción, con LTTng. En los paquetes de Humble viene **desactivado** (`ros2 run tracetools status` → `Tracing disabled`): hay que instalar LTTng (`lttng-tools liblttng-ust-dev python3-lttng python3-babeltrace`) y compilar `tracetools` desde el código. Sirve para saber en qué parte de YOLO o del sigue-personas se va el tiempo, no para el hardware.
