#!/usr/bin/env python3
"""Genera worlds/campo.sdf: un campo de cultivo con surcos, plantas, árboles y una
persona que camina por los pasillos entre surcos.

Las plantas son geometrías simples (cilindros, esferas y cajas), así que no hace
falta descargar nada de Fuel salvo los árboles y la persona. Para cambiar el
campo, toca las constantes de abajo y vuelve a ejecutar:

    python3 src/go2w_gazebo/scripts/generar_campo.py

Escribe en src/go2w_gazebo/worlds/campo.sdf (con --symlink-install no hace falta
recompilar).

Disposición (vista desde arriba, x hacia la derecha, y hacia arriba):

    y=4.5  ===== tomates ==========================
    y=3.75   <---------- persona (vuelta) ----------
    y=3.0  ===== tomates ==========================
    y=1.5  ===== maíz =============================
    y=0.75 R  ---------- persona (ida) ------------>
    y=0.0  ===== maíz =============================
    y=-1.5 ===== lechugas =========================
    y=-3.0 ===== lechugas =========================
           x=2                                  x=20

    R = robot en (-1.5, 0.75) mirando a +x. Las cabeceras (x < 2 y x > 20) son
    de tierra lisa para que el robot pueda girar.
"""
import math
import os
import random

SEED = 7

# Surcos (caballones): a lo largo de x
FIELD_X0, FIELD_X1 = 2.0, 20.0
RIDGE_YS = [-3.0, -1.5, 0.0, 1.5, 3.0, 4.5]
RIDGE_CROPS = ['lechuga', 'lechuga', 'maiz', 'maiz', 'tomate', 'tomate']
RIDGE_WIDTH = 0.5    # m (deja pasillos de 1 m: el robot mide ~0.45 m de ancho)
RIDGE_HEIGHT = 0.12  # m

# Persona: ida por el pasillo y=0.75, vuelta por y=3.75, giros en las cabeceras
PERSON_SPEED = 0.5   # m/s
PERSON_PATH = [(1.0, 0.75), (23.0, 0.75), (23.0, 3.75), (1.0, 3.75)]

TREES = [  # (nombre, modelo, x, y, yaw)
    ('roble_1', 'Oak tree', -4.0, 9.0, 0.0),
    ('roble_2', 'Oak tree', 10.0, 10.0, 1.2),
    ('roble_3', 'Oak tree', 24.0, 9.5, 2.5),
    ('roble_4', 'Oak tree', 28.0, -2.0, 0.7),
    ('pino_1', 'Pine Tree', 2.0, 9.5, 0.0),
    ('pino_2', 'Pine Tree', 17.0, 9.0, 0.0),
    ('pino_3', 'Pine Tree', 6.0, -8.0, 0.0),
    ('pino_4', 'Pine Tree', 15.0, -8.5, 0.0),
    ('roble_5', 'Oak tree', 22.0, -8.0, 1.9),
    ('pino_5', 'Pine Tree', -5.0, -5.0, 0.0),
]

SOIL = '0.40 0.27 0.16 1'
SOIL_RIDGE = '0.35 0.23 0.13 1'
GRASS = '0.30 0.45 0.20 1'
GREEN = '0.20 0.55 0.15 1'
LIGHT_GREEN = '0.45 0.70 0.25 1'
DARK_GREEN = '0.12 0.38 0.10 1'
STEM = '0.35 0.50 0.20 1'
TASSEL = '0.80 0.70 0.35 1'
RED = '0.80 0.10 0.08 1'

rnd = random.Random(SEED)


def material(color):
    return f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'


DZ = 0.0  # altura que se suma a las visuales (la de encima del surco)


def visual(name, pose, geometry, color):
    pose = list(pose)
    pose[2] += DZ
    pose = ' '.join(f'{v:.3f}' for v in pose)
    return (f'        <visual name="{name}"><pose>{pose}</pose>'
            f'<geometry>{geometry}</geometry>{material(color)}'
            f'<cast_shadows>false</cast_shadows></visual>\n')


def box(sx, sy, sz):
    return f'<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'


def cyl(r, length):
    return f'<cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder>'


def sphere(r):
    return f'<sphere><radius>{r:.3f}</radius></sphere>'


# ---------- plantas (visual solo, sin colisión) ----------
# z = 0 es la parte de arriba del surco

def lechuga(i, x, y):
    s = rnd.uniform(0.8, 1.2)
    out = visual(f'l{i}_centro', (x, y, 0.06 * s, 0, 0, 0), sphere(0.08 * s), LIGHT_GREEN)
    for k in range(4):  # hojas exteriores
        a = k * math.pi / 2 + rnd.uniform(-0.3, 0.3)
        out += visual(f'l{i}_hoja{k}',
                      (x + 0.07 * s * math.cos(a), y + 0.07 * s * math.sin(a), 0.04 * s,
                       0, 0, 0), sphere(0.07 * s), GREEN)
    return out


def maiz(i, x, y):
    h = rnd.uniform(1.3, 1.8)
    out = visual(f'm{i}_tallo', (x, y, h / 2, 0, 0, 0), cyl(0.015, h), STEM)
    for k in range(4):  # hojas largas e inclinadas hacia fuera
        z = h * (0.3 + 0.17 * k)
        yaw = rnd.uniform(0, 2 * math.pi)
        length = 0.45 - 0.05 * k
        out += visual(f'm{i}_hoja{k}',
                      (x + 0.5 * length * math.cos(yaw) * 0.8,
                       y + 0.5 * length * math.sin(yaw) * 0.8, z, 0, 0.5, yaw),
                      box(length, 0.06, 0.004), GREEN)
    out += visual(f'm{i}_penacho', (x, y, h + 0.08, 0, 0, 0), cyl(0.01, 0.16), TASSEL)
    return out


def tomate(i, x, y):
    h = rnd.uniform(0.6, 0.9)
    out = visual(f't{i}_tutor', (x, y, (h + 0.2) / 2, 0, 0, 0), cyl(0.008, h + 0.2),
                 '0.55 0.40 0.25 1')
    out += visual(f't{i}_mata', (x, y, h * 0.6, 0, 0, 0), sphere(0.22), DARK_GREEN)
    out += visual(f't{i}_mata2', (x + 0.05, y - 0.04, h * 0.95, 0, 0, 0), sphere(0.15),
                  DARK_GREEN)
    for k in range(rnd.randint(2, 4)):
        a = rnd.uniform(0, 2 * math.pi)
        out += visual(f't{i}_fruto{k}',
                      (x + 0.2 * math.cos(a), y + 0.2 * math.sin(a),
                       h * rnd.uniform(0.35, 0.8), 0, 0, 0), sphere(0.035), RED)
    return out


CROPS = {'lechuga': (lechuga, 0.35), 'maiz': (maiz, 0.30), 'tomate': (tomate, 0.50)}


def surco(j, y, crop):
    """Un caballón con colisión + sus plantas (un solo link: menos coste)."""
    length = FIELD_X1 - FIELD_X0
    cx = (FIELD_X0 + FIELD_X1) / 2
    geo = box(length, RIDGE_WIDTH, RIDGE_HEIGHT)
    out = f'''    <model name="surco_{j}">
      <static>true</static>
      <pose>{cx:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision"><pose>0 0 {RIDGE_HEIGHT / 2:.3f} 0 0 0</pose>
          <geometry>{geo}</geometry></collision>
        <visual name="tierra"><pose>0 0 {RIDGE_HEIGHT / 2:.3f} 0 0 0</pose>
          <geometry>{geo}</geometry>{material(SOIL_RIDGE)}</visual>
'''
    global DZ
    make, step = CROPS[crop]
    n = int(length / step)
    plants = ''
    DZ = RIDGE_HEIGHT  # las plantas van encima del caballón
    for i in range(n):
        if rnd.random() < 0.06:  # algún hueco, como en un campo real
            continue
        x = -length / 2 + step * (i + 0.5) + rnd.uniform(-0.04, 0.04)
        py = rnd.uniform(-0.05, 0.05)
        plants += make(i, x, py)
    DZ = 0.0
    out += plants
    out += '      </link>\n    </model>\n'
    return out


def malas_hierbas():
    """Matas pequeñas sueltas por los pasillos y las cabeceras (visual solo)."""
    out = '''    <model name="malas_hierbas">
      <static>true</static>
      <link name="link">
'''
    for i in range(120):
        x = rnd.uniform(-3.0, 25.0)
        y = rnd.uniform(-4.5, 6.0)
        # que no caigan en el recorrido de la persona ni del robot (pasillos 0.75 y 3.75)
        if min(abs(y - 0.75), abs(y - 3.75)) < 0.3:
            continue
        r = rnd.uniform(0.03, 0.08)
        out += visual(f'h{i}', (x, y, r * 0.6, 0, 0, 0), sphere(r),
                      rnd.choice([GREEN, LIGHT_GREEN, DARK_GREEN]))
    out += '      </link>\n    </model>\n'
    return out


def arboles():
    out = ''
    for name, model, x, y, yaw in TREES:
        out += f'''    <include>
      <name>{name}</name>
      <pose>{x} {y} 0 0 0 {yaw}</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/{model}</uri>
    </include>
'''
    return out


def persona():
    """Actor que recorre PERSON_PATH en bucle a PERSON_SPEED, girando en cada esquina."""
    pts = PERSON_PATH + [PERSON_PATH[0]]
    t = 0.0
    wps = []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        yaw = math.atan2(y1 - y0, x1 - x0)
        wps.append((t, x0, y0, yaw))           # ya girado hacia el siguiente punto
        t += math.hypot(x1 - x0, y1 - y0) / PERSON_SPEED
        wps.append((t, x1, y1, yaw))           # llega al punto
        t += 1.0                               # 1 s para girar
    first_yaw = wps[0][3]
    wps.append((t, pts[0][0], pts[0][1], first_yaw))
    lines = ''.join(
        f'          <waypoint><time>{tt:.1f}</time>'
        f'<pose>{x:.2f} {y:.2f} 1.0 0 0 {a:.4f}</pose></waypoint>\n'
        for tt, x, y, a in wps)
    skin = 'https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae'
    return f'''    <!-- Persona que camina por los pasillos (vuelta completa en {t:.0f} s).
         Un actor no tiene colisión: el robot no choca con ella. -->
    <actor name="persona_caminando">
      <skin><filename>{skin}</filename><scale>1.0</scale></skin>
      <animation name="walk">
        <filename>{skin}</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.6">
{lines}        </trajectory>
      </script>
    </actor>
'''


def world():
    surcos = ''.join(surco(j, y, c) for j, (y, c) in enumerate(zip(RIDGE_YS, RIDGE_CROPS)))
    return f'''<?xml version="1.0"?>
<!-- GENERADO por scripts/generar_campo.py: no editar a mano, cambia el script.
     Campo de cultivo: surcos de lechuga, maíz y tomate, malas hierbas, árboles
     alrededor y una persona que camina por los pasillos.
     El robot debe empezar en (-1.5, 0.75) mirando a +x (lo hace simulacion.launch.py
     con world:=campo). -->
<sdf version="1.8">
  <world name="campo">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.95 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sol">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <!-- Suelo: hierba alrededor (con colisión) y una capa de tierra encima, solo visual
         para que no haya escalón -->
    <model name="suelo">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="hierba">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          {material(GRASS)}
        </visual>
        <visual name="tierra">
          <pose>11 0.75 0.003 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>32 12</size></plane></geometry>
          {material(SOIL)}
        </visual>
      </link>
    </model>

    <!-- Surcos (caballones de {RIDGE_HEIGHT * 100:.0f} cm con colisión) y sus plantas (sin colisión) -->
{surcos}
{malas_hierbas()}
    <!-- Árboles alrededor del campo (Gazebo Fuel) -->
{arboles()}
{persona()}
  </world>
</sdf>
'''


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, '..', 'worlds', 'campo.sdf')
    with open(path, 'w') as f:
        f.write(world())
    print(f'Escrito {os.path.normpath(path)}')
