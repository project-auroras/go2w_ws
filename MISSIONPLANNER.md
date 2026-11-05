# Mission Planner + SITL + NTRIP

Cómo usar Mission Planner en el PC (Ubuntu) con el ArduPilot simulado (SITL) del contenedor y las correcciones RTK del caster NTRIP de ITACyL.

```
Host (Ubuntu)                                   Contenedor go2w-ardu (red del host)
┌────────────────────────────┐   UDP 14550     ┌──────────────────────────────┐
│ Mission Planner (Mono)     │◄────────────────│ MAVProxy ◄── ardurover (SITL)│
│   └─ RTK/GPS Inject (NTRIP)│                 └──────────────────────────────┘
└─────────────┬──────────────┘
              │ TCP 2101
                       ▼
   Caster NTRIP ITACyL (52.164.213.26:2101/leon3m)
```

- Mission Planner va **en el host**, no en el contenedor. Como el contenedor usa la red del host, lo que sale de SITL llega a `127.0.0.1:14550` sin abrir puertos.
- Todo se hace con `scripts/missionplanner.sh` desde el host:

| Comando | Qué hace |
|---|---|
| `scripts/missionplanner.sh install` | Instala Mono y Mission Planner en `~/MissionPlanner`, y da permisos de puerto serie |
| `scripts/missionplanner.sh sitl` | Arranca el contenedor `go2w-ardu` y lanza SITL (Rover en León) |
| `scripts/missionplanner.sh ntrip-test` | Comprueba usuario, contraseña y mountpoint contra el caster |
| `scripts/missionplanner.sh ntrip-config` | Escribe la URL NTRIP en la configuración de Mission Planner |
| `scripts/missionplanner.sh start` | `ntrip-config` y abre Mission Planner |

## 1. Una sola vez

### Mission Planner

```bash
cd ~/go2w_provisional
scripts/missionplanner.sh install
```

Instala `mono-complete`, descarga `MissionPlanner-latest.zip` en `~/MissionPlanner`, añade tu usuario al grupo `dialout` y desactiva ModemManager. Estos dos últimos pasos solo hacen falta para conectar una controladora por USB o radio. Si te añade a `dialout`, **cierra sesión y vuelve a entrar**.

Ábrelo una vez y ciérralo para que cree su configuración (`~/.local/share/Mission Planner/config.xml`):

```bash
scripts/missionplanner.sh start
```

En el primer arranque sale el aviso *"Restart Mission Planner to enable Drone ID Tab"*. Es normal: pulsa OK. Si no quieres esa pestaña, desactiva el plugin con Ctrl+P.

### Credenciales NTRIP

```bash
cp config/ntrip.env.example config/ntrip.env
nano config/ntrip.env      # NTRIP_USER, NTRIP_PASS, NTRIP_MOUNT
chmod 600 config/ntrip.env
```

`config/ntrip.env` está en `.gitignore`: **la contraseña no se sube al repo**. Escríbela tal cual, aunque lleve `@`. El script la codifica para la URL (`@` → `%40`), y Mission Planner la decodifica antes de enviarla.

Las cuentas se piden en https://gnss.itacyl.es/inscripcion (gratis). **Cada usuario admite una sola conexión a la vez**: si otra persona o dispositivo está usando la misma cuenta, el caster responde `401`. Para conectaros varios a la vez, pedid un usuario por persona (pueden ir con el mismo email). Para cambiar o recuperar la contraseña: https://gnss.itacyl.es/web/redgnss/registro/resetpassword

## 2. Levantar el contenedor de ArduPilot

ArduPilot (`~/ardu_ws/src/ardupilot`, ya compilado) y MAVProxy (`~/.local/bin`) están instalados **dentro del contenedor `go2w-ardu`**, no en la imagen `go2w:humble`. Por eso:

- **Usa siempre ese contenedor**: `docker start` y `docker exec`.
- **No lo borres** (`docker rm go2w-ardu`). `docker compose ... run --rm go2w` crea un contenedor nuevo **sin ArduPilot**.

```bash
xhost +local:                          # ventanas de MAVProxy y SITL (X11)
docker start go2w-ardu                 # si está parado (docker ps -a para verlo)
docker exec -it go2w-ardu bash         # abrir una terminal dentro
```

`scripts/missionplanner.sh sitl` ya hace el `docker start` y el `xhost` por ti.

Para guardar una copia del contenedor tal como está (recomendable):

```bash
docker commit go2w-ardu go2w:ardupilot
```

<details>
<summary>Si hay que crearlo de cero</summary>

```bash
cd ~/go2w_provisional
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --name go2w-ardu go2w   # sin --rm

# dentro del contenedor
mkdir -p ~/ardu_ws/src && cd ~/ardu_ws/src
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc   # mavproxy.py
source ~/.bashrc
./waf configure --board sitl && ./waf rover
```

Al salir con `exit` el contenedor se para pero no se borra: vuelve a él con `docker start go2w-ardu`.
</details>

## 3. Lanzar SITL

En una terminal del host (déjala abierta):

```bash
scripts/missionplanner.sh sitl
```

Equivale a ejecutar dentro del contenedor:

```bash
cd ~/ardu_ws
src/ardupilot/Tools/autotest/sim_vehicle.py -v Rover --console --map -l 42.6135,-5.5605,838,0
```

- `-l` pone el vehículo en el campus de la ULE (León). Si no lo pones, aparece en Canberra (Australia).
- `sim_vehicle.py` ya envía MAVLink a `udp:127.0.0.1:14550`. No añadas otro `--out` a 14550 o Mission Planner recibirá cada mensaje dos veces.
- Puedes pasarle más argumentos, por ejemplo `scripts/missionplanner.sh sitl --speedup 2`. Para otro vehículo o posición: `SITL_VEHICLE=ArduCopter SITL_LOCATION=lat,lon,alt,rumbo scripts/missionplanner.sh sitl`.
- Está listo cuando la consola de MAVProxy muestra que ha recibido los parámetros y el modo (`MANUAL`, `HOLD`…).

## 4. Conectar Mission Planner a SITL

En otra terminal del host:

```bash
scripts/missionplanner.sh start
```

1. Maximiza la ventana. Con Mono, si es estrecha, se corta la barra superior.
2. **Arriba a la derecha** de la ventana principal: desplegable de puerto → **UDP** (la velocidad da igual) → botón **CONNECT** (icono de los dos enchufes).
3. Puerto: **14550** → OK.
4. Se descargan los parámetros y el rover aparece en el mapa de **DATA**. El botón cambia a **DISCONNECT**.

Esa conexión vale para todas las pestañas (DATA, PLAN, SETUP, CONFIG). **No hay que volver a conectar en SETUP.** Los botones *Connect* que hay dentro de algunas pantallas de SETUP son para otras cosas: el de *RTK/GPS Inject* es para el NTRIP, y *Install Firmware* solo vale para una controladora física por USB.

## 5. Conectar el NTRIP (correcciones RTK)

Antes, comprueba la cuenta (opcional):

```bash
scripts/missionplanner.sh ntrip-test     # debe decir OK (200)
```

Esta prueba ocupa la sesión de la cuenta unos segundos. Si justo después Mission Planner da `401`, espera un poco y vuelve a intentarlo.

`scripts/missionplanner.sh start` ya ha escrito la URL en Mission Planner (si lo tenías abierto, ciérralo y usa `ntrip-config`, porque Mission Planner sobrescribe su configuración al cerrarse). Después:

1. **SETUP → Optional Hardware → RTK/GPS Inject**.
2. En el desplegable de puerto de esa pantalla: **NTRIP**.
3. **"Send NTRIP protocol v1.0?"**: desmarcado (usa NTRIP v2).
4. **Connect** (el de esa pantalla). Si pregunta la URL, ya viene rellenada: `http://usuario:contraseña@52.164.213.26:2101/leon3m`.
5. Deben ir subiendo los bytes y los mensajes RTCM (1005, 1074, 1084…) en la tabla de la derecha.

Para comprobarlo desde la terminal: `ss -tnp | grep 52.164.213.26` muestra una línea `ESTAB` mientras Mission Planner está conectado.

**Mountpoint**: `leon3m` es la estación de León (RTCM 3.2, GPS+GLONASS+Galileo+BeiDou) y no necesita enviar posición. `VRS3M` (red VRS) sí necesita la posición del vehículo (GGA), y el vehículo tiene que estar en Castilla y León. La lista completa:

```bash
curl -s -H 'Ntrip-Version: Ntrip/2.0' http://52.164.213.26:2101/ | grep '^STR'
```

**Con SITL, las correcciones no cambian nada**: el GPS simulado ya es perfecto. Solo sirve para comprobar la conexión. Las correcciones tienen efecto con un GPS RTK real (u-blox F9P, etc.) en el robot. En ese caso Mission Planner se conecta a la controladora por USB (`/dev/ttyACM0`) o radio (`/dev/ttyUSB0`) en lugar de UDP, y reenvía las correcciones al vehículo por MAVLink.

## Problemas frecuentes

| Síntoma | Causa | Solución |
|---|---|---|
| Mission Planner se queda "Connecting…" con UDP | SITL no está corriendo | `scripts/missionplanner.sh sitl` y esperar a los parámetros |
| `No such file or directory: 'mavproxy.py'` | `~/.local/bin` no está en el PATH del contenedor | Dentro: `echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc` |
| `Bad ntrip Response` + `401 Unauthorized` | Contraseña mal, o la cuenta ya está conectada en otro sitio (1 conexión por usuario) | `ntrip-test`. Desconectar los otros equipos, esperar 1–2 min, conectar **una** vez |
| `Bad ntrip Response` + `404 Not Found` | Mountpoint inexistente | Revisar `NTRIP_MOUNT` con la lista de arriba |
| "if using com0com please rename the ports to COM??" | Texto genérico de Mission Planner en cualquier error | Ignorarlo y leer la línea `HTTP/1.1 ...` |
| `ntrip-config`: "close Mission Planner first" | Mission Planner abierto sobrescribiría el cambio | Cerrarlo y repetir |
| No se ve el botón CONNECT | Ventana estrecha con Mono | Maximizar |
| No abre el puerto USB | Falta `dialout` o ModemManager lo coge | `scripts/missionplanner.sh install`, cerrar sesión y entrar |
