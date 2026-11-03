# Resumen 2026-10-29: Mission Planner, SITL, NTRIP, MAVROS y telemetría

Lo que se hizo para usar Mission Planner en el PC (Ubuntu 24.04) con el ArduPilot del contenedor `go2w-ardu`, el NTRIP de ITACyL y la controladora real por radio. Guía de uso paso a paso: [`MISSIONPLANNER.md`](../MISSIONPLANNER.md).

## Estado final

| Pieza | Estado |
|---|---|
| Mission Planner 1.3.83 | Instalado en el host (`~/MissionPlanner`, con Mono) |
| SITL (Rover) | Funciona en el contenedor `go2w-ardu` y conecta a Mission Planner por UDP 14550 |
| NTRIP ITACyL | Conectado (usuario `auroras-leon`, mountpoint `leon3m`) |
| MAVROS | Instalado en el contenedor; datos de GeographicLib pendientes de instalar |
| Controladora real | Llega MAVLink por la radio (`/dev/ttyUSB1`, 57600): Rover con ArduPilot, sysid 1 |
| Archivos nuevos | `scripts/missionplanner.sh`, `MISSIONPLANNER.md`, `config/ntrip.env.example`, `config/ntrip.env` (no se sube a git) |

## 1. Instalar Mission Planner

- Mission Planner es para Windows. En Linux funciona con **Mono**, aunque con alguna limitación (mapa 3D, algunos plugins). Alternativa nativa: QGroundControl.
- Se instala **en el host, no en el contenedor**. El contenedor usa la red del host, así que lo que lanza SITL dentro llega a `127.0.0.1:14550` sin abrir puertos. Además, el contenedor no tiene acceso a los puertos serie.
- El aviso del primer arranque *"Restart Mission Planner to enable Drone ID Tab"* es normal. El plugin se puede desactivar con Ctrl+P.

## 2. SITL en el contenedor y conexión

- ArduPilot y MAVProxy están instalados **solo dentro del contenedor `go2w-ardu`**, no en la imagen. No hay que borrarlo, y `docker compose run --rm` crea uno nuevo sin ArduPilot. Para guardar una copia: `docker commit go2w-ardu go2w:ardupilot`.
- Error `No such file or directory: 'mavproxy.py'`: `~/.local/bin` no estaba en el PATH porque `~/.profile` solo se lee en shells de login, y `docker exec ... bash` no lo es. Se arregló añadiendo `export PATH=$HOME/.local/bin:$PATH` a `~/.bashrc` del contenedor.
- `sim_vehicle.py` ya envía a `udp:127.0.0.1:14550` por defecto, así que no hay que añadir otro `--out` (se duplicarían los mensajes).
- En Mission Planner se conecta con el botón **CONNECT de arriba a la derecha** (UDP → 14550). Esa conexión vale para todas las pestañas: no hay que volver a conectar en SETUP.

## 3. NTRIP (correcciones RTK)

- Los primeros errores (`Bad ntrip Response`, 404 y 401) aparecieron porque se pulsaba Connect en **SETUP → Optional Hardware → RTK/GPS Inject**, que conecta al caster NTRIP, no al vehículo.
- El texto *"if using com0com please rename the ports to COM??"* es genérico en cualquier error de Mission Planner: no tiene nada que ver con Ubuntu.
- Caster de ITACyL: `52.164.213.26:2101` (`ntrip.itacyl.es`). Lista de mountpoints: `curl -s -H 'Ntrip-Version: Ntrip/2.0' http://52.164.213.26:2101/ | grep '^STR'`.
- **Una sola conexión simultánea por usuario** (lo dice la página de Inscripción de ITACyL). La causa real del `401` era que la sesión estaba ocupada, no la contraseña: el caster tarda en liberar una sesión cerrada. Para conectarse varias personas a la vez, cada una necesita su propio usuario (gratis, en https://gnss.itacyl.es/inscripcion).
- Los caracteres especiales en la contraseña se codifican en la URL (`@` → `%40`). Se comprobó en su código que Mission Planner los decodifica antes de enviarlos.
- La web de ITACyL no tiene área de usuario ni lista de sesiones activas. Para eso hay que escribir a `gnss@itacyl.es`. Para cambiar la contraseña: https://gnss.itacyl.es/web/redgnss/registro/resetpassword
- IP pública del laboratorio: `193.146.109.25`. Es la salida común de la red de la ULE, así que no identifica a un equipo concreto.
- Configuración final: usuario `auroras-leon`, mountpoint `leon3m` (estación de León, RTCM 3.2, no necesita enviar la posición). `VRS3M` (red VRS) sí necesita la posición (GGA) dentro de Castilla y León.
- Con SITL las correcciones no cambian nada, porque el GPS simulado ya es perfecto. Tendrán efecto con un GPS RTK real.

## 4. Script y guía

`scripts/missionplanner.sh` se ejecuta en el host:

```bash
scripts/missionplanner.sh install        # Mono + Mission Planner + permisos serie
scripts/missionplanner.sh sitl           # arranca go2w-ardu y lanza SITL Rover en León
scripts/missionplanner.sh ntrip-test     # comprueba la cuenta NTRIP
scripts/missionplanner.sh ntrip-config   # escribe la URL NTRIP en Mission Planner (con Mission Planner cerrado)
scripts/missionplanner.sh start          # ntrip-config + abre Mission Planner
```

Las credenciales van en `config/ntrip.env`, que está en `.gitignore`. Plantilla: `config/ntrip.env.example`. La contraseña se escribe tal cual y el script la codifica.

## 5. MAVROS

- `sudo ros2 run mavros install_geographiclib_datasets.sh` → `ros2: command not found`. `sudo` usa su propio PATH (`secure_path`) y descarta lo que añade `source /opt/ros/humble/setup.bash`.
- Solución, dentro del contenedor:
  ```bash
  sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
  ls /usr/share/GeographicLib/geoids
  ```
- Para comprobar la conexión con la controladora:
  ```bash
  ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyUSB1:57600
  ros2 topic echo /mavros/state      # connected: true
  ```
  **Pendiente:** el contenedor no tiene acceso a `/dev/ttyUSB*`, porque al crearlo solo se le pasó `/dev/dri`. Hay que hacer `docker commit go2w-ardu go2w:ardupilot` y crear un contenedor nuevo con `--device`, o usar las rutas `by-id`.

## 6. Controladora real por telemetría

Dispositivos en el PC:

| Puerto | Chip | Ruta fija | Qué es |
|---|---|---|---|
| `/dev/ttyUSB1` (puede cambiar) | FTDI FT231X `DU0ENGAS` | `/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0ENGAS-if00-port0` | Radio de telemetría SiK (lado del PC) |
| `/dev/ttyUSB0` (ya desconectado) | FTDI FT232R `A5XK3RJT` | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5XK3RJT-if00-port0` | Cable o adaptador FTDI genérico; no enviaba datos |
| `/dev/ttyACM0` | — | — | No aparece: la controladora no está conectada por USB de datos a este portátil |

Problemas y causas:

- **Llegaban 0 bytes por los puertos.** Se comprobó leyendo directamente como root a 9600–1 500 000 baudios. La causa era que **las radios no estaban enlazadas**. No era cosa de permisos, ni de MAVLink "pausado" en el contenedor (el contenedor ni siquiera ve los puertos).
- **LEDs SiK:** verde parpadeando = buscando pareja; verde fijo = enlazadas; rojo parpadeando = hay datos; rojo fijo = bootloader. La radio del robot tenía rojo + verde parpadeando: la controladora le enviaba datos, pero no había enlace.
- Para que se enlacen, las dos radios tienen que tener la misma banda (433/915 MHz) y los mismos `NETID`, frecuencias y `AIR_SPEED` (SETUP → Optional Hardware → SiK Radio).
- **`Sequence contains no elements`** al conectar es como Mission Planner (con Mono) indica que **no ha recibido ningún HEARTBEAT**. Se debía a que el puerto o la velocidad eran incorrectos (tenía guardado `/dev/ttyUSB0` y llegaba basura a 115200), o a que todavía no había enlace.
- **Resultado:** por `/dev/ttyUSB1` a **57600** llega `HEARTBEAT sysid=1 compid=1 type=10 (GROUND_ROVER) autopilot=3 (ArduPilot)`. Para conectar: Mission Planner → `/dev/ttyUSB1` → 57600 → CONNECT.

**Permisos:** el usuario del host no está en el grupo `dialout`. Se hizo `chmod 777` a los puertos a mano, pero eso se pierde al reconectar. Solución permanente: `sudo usermod -aG dialout $USER` y volver a iniciar sesión.

## Pendiente

- [ ] `sudo usermod -aG dialout $USER` en el host y volver a iniciar sesión.
- [ ] Instalar los datos de GeographicLib para MAVROS.
- [ ] Dar acceso a la radio al contenedor (`docker commit` + nuevo contenedor con `--device`) y probar `/mavros/state`.
- [ ] Revisar la calidad del enlace de radio (llegan pocos mensajes por segundo): antenas, distancia y `SR1_*` en la controladora.
- [ ] Probar `scripts/missionplanner.sh install`, `sitl`, `start` y `ntrip-test` en una sesión limpia.
- [ ] Hacer commit de `scripts/missionplanner.sh`, `MISSIONPLANNER.md`, `config/ntrip.env.example` y `.gitignore` (nunca de `config/ntrip.env`).
