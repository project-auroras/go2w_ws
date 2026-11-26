# Validación en el robot: FAST-LIO + EasyNav

Qué se mide, cómo y con qué criterio se da por bueno, antes de montar encima la percepción y el resto de nodos. Cada prueba deja un bag, un `meta.yaml` con la medida real (cinta métrica) y un informe generado. Los resultados van en [`results/`](results/), uno por sesión.

## Herramientas

| Qué | Dónde | Para qué |
|---|---|---|
| `scripts/validation/record.sh <prueba>` | NX | Comprobación previa (frecuencias, TF) + métricas de hardware (`go2w_metrics`) + bag en `bags/<fecha>_<prueba>/` + pregunta la medida real y la guarda en `meta.yaml` |
| `scripts/validation/preflight.py` | NX | Solo la comprobación previa |
| `scripts/validation/drive_line.py [m] --speed v` | NX | Recta para L2: `/cmd_vel` a 50 Hz con rampas, sin corregir el rumbo; para tras `m` metros de `/odom` (10 m a 0.3 m/s por defecto) |
| `scripts/validation/send_goal.py <adelante> [izquierda] [giro]` | NX o PC | Objetivo de EasyNav relativo al robot (`5` = 5 m al frente, `--back` vuelve al inicio). Espera el resultado e imprime tiempo y error |
| `ros2 run go2w_metrics validate <bag>` | PC (contenedor) | Informe: `report/report.md`, `metrics.yaml`, `trajectory.png`, `hardware.png`, CSV de hardware |

El bag se graba **en la NX**: así la latencia sale sin desfase de reloj y no depende del WiFi. Por defecto incluye `/utlidar/cloud` y las IMU, para poder reproducir FAST-LIO en el PC con otra configuración (≈2.3 MB/s; `--no-raw` para quitarlas).

## Preparación de la NX (una vez)

```bash
# NX (con internet: scripts/nx_internet.sh, ver README principal)
cd ~/go2w_provisional && git fetch && git switch test/validation
scripts/nx_update.sh          # clona ros2_vitals (deps.repos) y compila go2w_metrics
```

## Arranque en cada sesión

```bash
# NX, terminal 1: robot + FAST-LIO + lio_scan. Robot QUIETO ~2 s al arrancar (inicializa la IMU)
GO2_IFACE=enP8p1s0 source scripts/env.sh
ros2 launch go2w_bringup go2w.launch.py lio:=true 2>&1 | tee ~/launch.log
#   /scan from lio_scan_cpp (C++); lio_scan_cpp:=false -> lio_scan.py (same /scan, 6x more CPU)
# NX, terminal 2: de pie
ros2 service call /mode go2_interfaces/srv/Mode "mode: 'stand_up'"
# NX, terminal 3: cada prueba
scripts/validation/record.sh static
# PC: traer y analizar
scp -r unitree@192.168.123.18:go2w_provisional/bags/<nombre> ~/go2w_ws/bags/
ros2 run go2w_metrics validate bags/<nombre>       # dentro del contenedor, ws en /ws
```

## Pruebas y criterios

Los umbrales están en `CRITERIA` de `src/go2w_metrics/go2w_metrics/validate.py`: si se cambia uno, cambiarlo en los dos sitios.

### FAST-LIO (odometría)

| ID | Prueba | Cómo | Criterio |
|---|---|---|---|
| L0 | Entradas | En todas las pruebas | nube ≥ 14 Hz, IMU ≥ 230 Hz, `/odom` ≥ 14 Hz sin huecos > 200 ms, latencia nube → `/Odometry` p95 ≤ 100 ms |
| L1 | `static` | Robot de pie, quieto, nadie alrededor, **5 min** | deriva ≤ 5 cm y ≤ 1° en 5 min (pendiente de una recta ajustada a x, y, yaw; el temblor del robot de pie se da aparte) |
| L2 | `line` | Línea de referencia en el suelo (cinta de carrocero, ~12 m) con marcas a 0, 5, 8, 10 y 12 m (un metro de 5 m basta: cada marca se mide desde la anterior). Robot con el punto bajo el LiDAR (frontal de la cabeza, plomada) sobre el 0 y alineado con la línea. `drive_line.py 10` (10 m a velocidad constante sin corregir; espera a que el robot esté quieto 2 s y al final lo mantiene parado 3 s). Sin tocar el mando. Medida: distancia del punto bajo el LiDAR a las **dos marcas más cercanas** y lado (izquierda/derecha); `validate` triangula la posición (no hace falta escuadra). ×2 a 0.3 m/s y ×1 a 1 m/s (`--speed 1.0`) | error de escala ≤ 2 % |
| L3 | `spin` | Marcar en el suelo el punto bajo el LiDAR (delante) y bajo la parte trasera del cuerpo (detrás). `drive_line.py --spin 3` y `--spin -3` (3 vueltas a cada lado, se para por odometría). Medida: vueltas, desvío lateral del punto delantero y del trasero respecto a su marca (+ izquierda) y distancia entre las dos marcas; el giro real es vueltas × 360° + atan((delante − detrás) / distancia) | ≤ 1° de error por vuelta; base_link se mueve ≤ 10 cm |
| L4 | `loop` | Circuito cerrado de 30-50 m en interior, empezar y acabar en una marca, cinta del robot a la marca. ×2 | cota superior de deriva ≤ 0.5 % del recorrido |
| L5 | `loop` exterior | Igual que L4 en el campo del MIC (poca estructura: es donde FAST-LIO puede degenerar) | ≤ 0.5 % (se documenta aunque falle: decide si hace falta GPS/ruedas) |

En `line` las ruedas del Go2-W se desvían en lazo abierto (el robot acaba en diagonal): eso es el robot, no la odometría. La escala compara distancias inicio → fin en línea recta, así que el desvío no le afecta. Se mide el punto bajo el LiDAR porque `/Odometry` es la pose del LiDAR: con el giro de ~15° que acumula el robot, otro punto (centro, ruedas) cambiaría el desvío lateral unos cm, aunque la distancia solo ~0.1 %. Con dos distancias a marcas conocidas de la línea la posición sale por triangulación (`forward_m`/`lateral_m` medidos a mano siguen valiendo en `meta.yaml`). Inicio y fin son la pose media en reposo antes y después del movimiento (el yaw tiembla ±2-4° de muestra a muestra); si el robot se movía en esas ventanas la prueba no vale (`line_start_end_at_rest`). El ángulo lateral de la cinta (respecto a la línea) y el de LIO (respecto al rumbo inicial) solo difieren en lo mal alineado que saliera el robot: se da como comprobación (`line_start_misalignment_deg`), no como error de LIO.

**Escala del LiDAR (pared).** Un error de escala uniforme no se ve en `loop` (el circuito encogido sigue cerrando). Para separar sensor de FAST-LIO: robot quieto frente a una pared plana, `scripts/validation/wall_record.sh <metro>` (metro del punto bajo el LiDAR a la pared, perpendicular) a dos o más distancias separadas ≥ 2 m (p. ej. 1.5 y 4.5 m); en el PC `python3 scripts/validation/wall_range.py bags/*_wall` ajusta la pared en la nube y da `nube = k · metro + c` (el desfase c del centro óptico se cancela; k − 1 es el error de escala).

**Calentamiento del L1.** El −2.6 % de la pared (2026-11-24) se midió con el robot encendido desde hacía horas, y hay un informe de que el L1 mide más corto los primeros 10-20 min encendido. Antes de fijar una corrección: robot apagado ≥ 1 h, frente a la pared a ~2.8 m, encender y lanzar nada más arrancar la NX `scripts/validation/wall_warmup.sh <metro>` (una grabación cada 5 min durante 40 min, sin tocar el robot). `wall_range.py` da la pared y el suelo (~0.47 m) de cada grabación: si cambian los mismos mm es un desfase, si cambian el mismo % es escala. Al terminar, ya caliente, dos posiciones más (~1 m y ~4.5 m) para la escala con más recorrido. Resultado (2026-11-24 a 26): escala sin determinar (−0.6 a −2.6 %), sin corrección; ver [results/2026-11-26_l1_scale.md](results/2026-11-26_l1_scale.md).

En `loop` solo se conoce la **distancia** real a la marca, no la dirección, así que la deriva queda entre `|LIO - cinta|` y `LIO + cinta`. El criterio usa la cota superior (pesimista). Para que la cota sea útil, parar lo más cerca posible de la marca.

### EasyNav (`config/easynav_real.params.yaml`)

Arranque: `ros2 run easynav_system system_main --ros-args --params-file config/easynav_real.params.yaml` en la NX, RViz en el PC con `src/go2w_gazebo/rviz/go2w_sim.rviz` (Fixed Frame `map`).

| ID | Prueba | Cómo | Criterio |
|---|---|---|---|
| N1 | Parado (`free`, 2 min) | Robot quieto con todo lanzado. En RViz: ningún obstáculo bajo el robot; persona a 1, 2 y 3 m aparece en el NavMap | visual (captura) + hardware de la pila completa |
| N2 | Objetivos en libre (`nav`) | `send_goal.py 2 --back`, `5 --back`, `10 --back`, ×3 cada uno | 100 % FINISHED, error final ≤ 0.35 m y ≤ 17°, holgura mínima ≥ 0.25 m |
| N3 | Obstáculo (`nav`) | Caja a 2.5 m en la ruta de `send_goal.py 5`. ×3 | la esquiva sin tocarla, holgura ≥ 0.25 m |
| N4 | Persona (`nav`) | Una persona se cruza a ~2 m durante `send_goal.py 5` | se detiene o la esquiva |
| N5 | Resistencia (`nav`, 10 min) | `for i in $(seq 10); do scripts/validation/send_goal.py 5 --back; done` | 100 % FINISHED, temperatura ≤ 80 °C |

El error final de N2 se mide en `odom` (lo que cree el localizador). El error real lo da L4: es la deriva de FAST-LIO.

### Hardware (NX, todas las pruebas)

| Métrica | Criterio | Motivo |
|---|---|---|
| CPU de la pila de localización (FAST-LIO + lio_scan + go2w_driver, que corre en `go2w_container`) | ≤ 150 % (1.5 núcleos) | dejar sitio a EasyNav, YOLO y lo que venga |
| CPU total de la NX | ≤ 50 % de media | margen para la percepción |
| Temperatura máxima | ≤ 80 °C | lejos del throttling |
| RAM, GPU, potencia (`VDD_IN`) | se documentan | referencia para cuando se añada YOLO |

## Orden recomendado en una sesión (~2 h)

1. Arranque + `preflight.py` → L1 `static` (5 min; también es la medida de hardware base).
2. L2 `line` ×3 → L3 `spin` ×2 → L4 `loop` ×2.
3. EasyNav: N1 → N2 → N3 → N4 → N5.
4. L5 en el exterior cuando se pueda ir al campo.

Después de cada prueba, apuntar en las notas del `record.sh` el lugar, el suelo y cualquier cosa rara (golpe, patinazo, gente cerca).
