# Validación del sigue-personas

Experimentos de detección y seguimiento de personas con tiempos, velocidades y uso de hardware. Se hacen primero **en interior**, con marcas y escenarios repetibles. El día que haga bueno se repiten **las mismas pruebas, con los mismos comandos, en la calle**: solo cambian las notas del bag. El análisis es el mismo que el de FAST-LIO y EasyNav (`ros2 run go2w_metrics validate`), así que todos los resultados del proyecto se leen igual.

## Montaje

```
RealSense ──USB──► NX: realsense2_camera (15 fps, JPEG + profundidad PNG), go2w_bringup (FAST-LIO, /scan),
                       go2w_metrics jetson:=true (/vitals/status + /jetson/stats)
                 ──WiFi──► portátil: people_live.launch.py (descompresión + YOLO + person_filter
                                     [+ person_follower]), go2w_metrics, ros2 bag (record_tests.sh)
```

**NX** (cada terminal con `export GO2_IFACE=enP8p1s0 && source scripts/env.sh`):

```bash
ros2 launch go2w_bringup go2w.launch.py lio:=true          # robot QUIETO ~2 s al arrancar
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 depth_module.depth_profile:=640x480x15
ros2 launch go2w_metrics metrics.launch.py jetson:=true
ros2 service call /mode go2_interfaces/srv/Mode "mode: 'stand_up'"
```

**Portátil** (contenedor, `GO2_IFACE=wlo1`):

```bash
scripts/record_tests.sh                                  # P1: solo detección, el robot NO se mueve
scripts/record_tests.sh --follower --max-linear 0.6      # P2-P8: el robot SE MUEVE (alguien con el mando)
```

`record_tests.sh` comprueba los topics, graba un bag por prueba en `/bags` (`~/rosbags` en el host) y, al parar cada una, pregunta el tipo (`detect` / `follow`), las marcas y las notas, y escribe el `meta.yaml`. Ahí guarda también las velocidades máximas, el commit y el **proveedor con el que corrió YOLO** (`cuda` o `cpu`, leído del log). Entre pruebas, `v 1.0` cambia la velocidad máxima sin salir del script.

**Palmadas**: al empezar y al acabar cada bag, palmada delante de las dos cámaras. Con `scripts/bag_frames.py` se saca el desfase de reloj NX - portátil; se apunta en `clock_offset_s` del `meta.yaml` y el informe añade la latencia cámara + WiFi. Sin el desfase, el informe ya da la latencia de procesado exacta: `/people` lleva el stamp de su imagen, y se compara a qué hora llegan las dos al portátil.

**Análisis** (PC o portátil, contenedor):

```bash
ros2 run go2w_metrics validate /bags/<nombre>     # -> /bags/<nombre>/report/report.md + gráficas + CSV
```

## Pruebas

Repetir cada prueba de seguimiento con `max_linear` 0.6 y 1.0 m/s (`v 0.6` / `v 1.0`). Nombre del bag: `<ID>_<variante>_<repetición>`, por ejemplo `P2_normal_0.6_r1`.

| ID | Tipo | Escenario | Repeticiones | Qué responde |
|---|---|---|---|---|
| P1 | `detect` | Robot quieto, sin follower. Marcas con cinta a 1, 2, 3, 4, 5 y 6 m. La persona está ~10 s en cada marca mirando al robot; luego otra vez de lado | 1 de frente + 1 de lado | ¿A qué distancia detecta? ¿Con qué error mide la distancia? Latencia y consumo solo de percepción |
| P2 | `follow` | Recta de 15-20 m. Andar lento (~0.5 m/s), normal (~1.0) y rápido (~1.4) | 2 por ritmo y velocidad | Error de distancia, velocidad que alcanza el robot, retardo de respuesta |
| P3 | `follow` | Arranca y para: 4 m andando, 5 s quieta, ×4 | 2 | Frenada y distancia mínima a la persona, tirones |
| P4 | `follow` | Giros: recorrido en L y en U con giros de 90°, y un círculo de ~3 m de radio | 2 | Error de ángulo, si la pierde en los giros |
| P5 | `follow` | Pérdida: sale del campo de visión de lado y vuelve a los 5 s; se esconde 3 s tras una columna o puerta | 3 de cada | Tiempo de recuperación, si la búsqueda funciona |
| P6 | `follow` | Distracción: otra persona cruza entre el robot y la seguida a ~1 m; otra persona anda en paralelo | 3 de cada | Cambios de ID, si sigue a la persona correcta |
| P7 | `follow` | Obstáculo: caja o silla en el camino del robot; la persona la rodea | 3 | Frena o esquiva sin tocarla, holgura mínima |
| P8 | `follow` | Resistencia: 10 min siguiendo por un circuito | 1 | Temperatura, estabilidad, pérdidas por minuto |

Exterior (día de buen tiempo): P1, P2, P4, P5, P6 y P8. En las notas: lugar, sol o sombra, suelo y gente alrededor.

## Métricas y criterios

Los umbrales están en `CRITERIA` de `src/go2w_metrics/go2w_metrics/validate_detect.py` y `validate_follow.py`. Son una **propuesta inicial**: se ajustan con los datos de interior, antes de salir a la calle.

| Métrica | Criterio | De dónde sale |
|---|---|---|
| Frecuencia de `/people` | ≥ 10 Hz (cámara a 15) | bag |
| Latencia de procesado p95 (imagen → `/people`) | ≤ 150 ms | llegada al portátil de la imagen y de su `/people` (mismo stamp) |
| Error de distancia en las marcas (P1) | ≤ 0.25 m de sesgo en cada marca | `/people` frente a la cinta |
| Frames con persona en cada marca (P1) | ≥ 90 % | `/people` |
| Error medio de distancia siguiendo | ≤ 0.30 m | `~/status`: distancia − distancia objetivo (1.5 m + 0.5 s × v) |
| Error p95 de distancia | ≤ 0.80 m | ídem |
| Distancia mínima a la persona | ≥ 0.90 m | `~/status` |
| Tiempo siguiendo | ≥ 85 % desde la primera detección | estados de `~/status` |
| Recuperación tras perderla | ≤ 5 s | transiciones perdida/buscando → siguiendo |
| Se documentan (sin umbral) | velocidades de persona y robot, retardo de respuesta, aceleración p95 de `/cmd_vel`, cambios de ID, pérdidas/min, % de ciclos frenado por obstáculo | |
| Hardware | CPU y temperatura de la NX, CPU y GPU del portátil, CPU y RAM de cada nodo | `go2w_metrics` en las dos máquinas |

`person_follower` publica `/person_follower/status` (JSON en cada ciclo de control: estado, ID, distancia, distancia objetivo, ángulo, velocidad estimada de la persona, orden enviada, nota de obstáculo). De ahí salen las métricas de seguimiento sin tener que deducir el estado desde fuera.

## Comprobado en simulación (2026-11-19)

`person_follower_sim.launch.py` (`fake_person`) + `go2w_metrics` + bag, 90 s, analizado con `validate --test follow`:

- siguiendo 100 % del tiempo, sin pérdidas;
- error de distancia medio 0.02 m, p95 0.14 m; distancia mínima 1.43 m;
- persona a 0.32 m/s de media (máx. 0.55), robot a 0.22 m/s de media (máx. 0.54);
- `person_follower` ~22 % de un núcleo y 66 MB.

Solo sirve para comprobar la cadena de medida. Los números válidos son los del robot.

## Problemas conocidos (2026-11-19)

**"Al lanzar las métricas falla la detección"**: medido en el PC (i7-12700F), con YOLO26s sobre una imagen con 4 personas a 30 Hz:

| | Solo YOLO | YOLO + `go2w_metrics` | Métricas paradas |
|---|---|---|---|
| 20 núcleos | 30.0 Hz, 33 ms p50 | 29.8 Hz, 38 ms | 30.0 Hz, 31 ms |
| 4 núcleos (`--cpuset-cpus=0-3`) | 12.7 Hz, 109 ms | 12.7 Hz, 107 ms | 13.0 Hz, 106 ms |

Personas detectadas en el 100 % de los frames en todos los casos. El colector de `ros2_vitals` tarda ~4 ms por ciclo: **por sí solo no rompe la detección**. Falta ver los logs del portátil donde falló (`/bags/logs/<sesión>/yolo.log`, `metrics.log`, `camera.log`) para saber qué pasa allí. Hay dos sospechosos, que vienen a continuación.

**YOLO corre en CPU**: `yolo_onnxruntime_vendor` se compila por defecto sin GPU (`ONNX_GPU=OFF`). Con `provider: cuda`, el log dice `Requested provider "cuda" is unavailable; using cpu`, y con `n_threads: -1` usa todos los núcleos. Así cualquier otra carga del portátil (cámara externa, descompresión, `ros2 bag`) le quita frames. Usar la GPU requiere CUDA + cuDNN en la imagen y compilar con `-DONNX_GPU=ON`: es una decisión de equipo. Mientras tanto, el `meta.yaml` deja apuntado el proveedor de cada prueba.

**El contenedor pierde la GPU**: con Docker en cgroup v2 + systemd, un `systemctl daemon-reload` en el host (lo hace cualquier `apt install`) deja a los contenedores ya arrancados sin GPU: `nvidia-smi` dentro da `Failed to initialize NVML: Unknown Error`. Comprobar antes de cada sesión con `nvidia-smi` dentro del contenedor; si falla, reiniciar el contenedor. Hay dos soluciones permanentes, pendientes de decidir: `"exec-opts": ["native.cgroupdriver=cgroupfs"]` en `/etc/docker/daemon.json` del host, o listar `/dev/nvidia*` en `devices:` del compose (esto falla en máquinas sin GPU NVIDIA).
