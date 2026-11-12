#!/usr/bin/env python3
"""Genera worlds/campo.sdf: un campo de cultivo con surcos, plantas, árboles y una
persona que camina por los pasillos entre surcos.

Las plantas son geometrías simples (cilindros, esferas y cajas), así que no hace
falta descargar nada de Fuel salvo los árboles y la persona.

Para la navegación semántica (modelo YOLO con clases Row, Post, Trunk, Puddle,
Pothole, Trace...) el campo lleva además texturas procedurales (tierra, madera,
corteza, agua) y esos objetos: postes al final de los surcos y en una valla,
una hilera de frutales, charcos, baches y huellas de tractor. Las texturas se
escriben en models/campo_texturas/ y el suelo plano para EasyNav en
maps/campo_flat.pcd (pasarlo a .navmap con pointcloud_to_navmap, ver README).

Para cambiar el campo, toca las constantes de abajo y vuelve a ejecutar:

    python3 src/go2w_gazebo/scripts/generar_campo.py

Escribe src/go2w_gazebo/worlds/campo.sdf, models/campo_texturas/*.png y
maps/campo_flat.pcd (con --symlink-install no hace falta recompilar).

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
import struct
import zlib

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

# Objetos para la navegación semántica (clases del modelo entre paréntesis)
POST_FENCE_Y = 6.5                   # valla de postes (Post) entre los tomates y los árboles
POST_FENCE_XS = [x * 2.5 for x in range(-1, 10)]
POST_HEIGHT = 1.2
ORCHARD_Y = -5.5                     # hilera de frutales (Trunk) al sur de las lechugas
ORCHARD_XS = [x * 3.0 + 2.0 for x in range(7)]
PUDDLES = [(8.0, -2.25, 0.9, 0.6), (-0.8, 2.6, 1.3, 0.8), (21.5, 0.0, 1.0, 1.4)]  # x y sx sy
POTHOLES = [(12.0, -0.75, 0.30), (0.2, -1.5, 0.35), (22.0, 3.0, 0.40)]          # x y r
TRACKS = [  # huellas de tractor (Trace): (x0, x1, y_centro) de una pareja de rodadas
    (2.0, 20.0, 2.25),        # pasillo entre el maíz y los tomates
    (-3.0, 24.0, -4.4),       # camino al sur del campo
]
TRACK_GAP = 0.5      # m entre rodadas
TRACK_WIDTH = 0.18

# Suelo plano para EasyNav (maps/campo_flat.pcd): rectángulo en el frame del mundo
NAVMAP_X = (-10.0, 32.0)
NAVMAP_Y = (-12.0, 14.0)
NAVMAP_STEP = 0.25   # la resolución de pointcloud_to_navmap (pasillos de 1 m)

TEXTURES = 'campo_texturas'          # models/<TEXTURES>/*.png, en GZ_SIM_RESOURCE_PATH

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


def textured(texture, roughness=0.9, tint='1 1 1 1'):
    """Material PBR con una textura de models/campo_texturas (se estira una vez por cara)."""
    return (f'<material><ambient>{tint}</ambient><diffuse>{tint}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular><pbr><metal>'
            f'<albedo_map>model://{TEXTURES}/{texture}.png</albedo_map>'
            f'<roughness>{roughness}</roughness><metalness>0.0</metalness>'
            f'</metal></pbr></material>')


# ---------- texturas procedurales (PNG sin dependencias) ----------

def write_png(path, size, pixel):
    """PNG RGB de size x size; pixel(u, v) -> (r, g, b) en 0..1, u y v en 0..1."""
    rows = []
    for j in range(size):
        row = bytearray([0])
        for i in range(size):
            r, g, b = pixel(i / size, j / size)
            row += bytes(max(0, min(255, int(c * 255))) for c in (r, g, b))
        rows.append(bytes(row))
    raw = zlib.compress(b''.join(rows), 9)

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))
    with open(path, 'wb') as f:
        header = struct.pack('>IIBBBBB', size, size, 8, 2, 0, 0, 0)
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header)
                + chunk(b'IDAT', raw) + chunk(b'IEND', b''))


class Noise:
    """Ruido de valor 2D periódico (las texturas se repiten sin costura)."""

    def __init__(self, seed, cells):
        r = random.Random(seed)
        self.cells = cells
        self.v = [[r.random() for _ in range(cells)] for _ in range(cells)]

    def __call__(self, u, v):
        n = self.cells
        x, y = u * n, v * n
        i, j = int(x) % n, int(y) % n
        fx, fy = x - int(x), y - int(y)
        fx, fy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
        i1, j1 = (i + 1) % n, (j + 1) % n
        a = self.v[j][i] + (self.v[j][i1] - self.v[j][i]) * fx
        b = self.v[j1][i] + (self.v[j1][i1] - self.v[j1][i]) * fx
        return a + (b - a) * fy


def fbm(octaves):
    def f(u, v):
        return sum(o(u, v) * 0.5 ** k for k, o in enumerate(octaves)) / sum(
            0.5 ** k for k in range(len(octaves)))
    return f


def mix(c0, c1, t):
    return tuple(a + (b - a) * t for a, b in zip(c0, c1))


def texturas(folder):
    os.makedirs(folder, exist_ok=True)
    soil = fbm([Noise(1, 8), Noise(2, 16), Noise(3, 32), Noise(4, 64)])
    clods = Noise(5, 48)

    def tierra(u, v):      # tierra labrada con terrones
        t = soil(u, v)
        c = mix((0.27, 0.17, 0.10), (0.50, 0.36, 0.23), t)
        return mix(c, (0.20, 0.13, 0.08), 0.6) if clods(u, v) > 0.8 else c

    def caballon(u, v):    # tierra del surco, más oscura y con surcos finos a lo largo
        t = soil(u, v) * 0.7 + 0.3 * (0.5 + 0.5 * math.sin(v * 2 * math.pi * 12))
        return mix((0.22, 0.14, 0.08), (0.42, 0.29, 0.18), t)

    grain = fbm([Noise(6, 4), Noise(7, 32)])

    def madera(u, v):      # vetas verticales
        t = 0.5 + 0.5 * math.sin((u * 18 + grain(u, v) * 6) * 2 * math.pi)
        return mix((0.36, 0.24, 0.13), (0.58, 0.42, 0.26), t * 0.7 + soil(u, v) * 0.3)

    bark_n = fbm([Noise(8, 16), Noise(9, 64)])

    def corteza(u, v):     # grietas verticales oscuras
        t = abs(math.sin((u * 8 + bark_n(u, v) * 1.5) * math.pi))
        c = mix((0.30, 0.23, 0.17), (0.48, 0.39, 0.30), bark_n(u, v))
        return mix(c, (0.10, 0.07, 0.05), 0.8) if t < 0.22 else c

    leaf = fbm([Noise(10, 16), Noise(11, 32), Noise(12, 64)])

    def hojas(u, v):
        return mix((0.10, 0.28, 0.07), (0.38, 0.62, 0.20), leaf(u, v))

    def charco(u, v):      # agua turbia que refleja el cielo, orilla húmeda oscura
        d = math.hypot(u - 0.5, v - 0.5) * 2 + (soil(u, v) - 0.5) * 0.3
        if d > 0.95:
            return tierra(u, v)
        if d > 0.8:
            return mix(tierra(u, v), (0.18, 0.13, 0.09), 0.7)
        return mix((0.42, 0.47, 0.50), (0.70, 0.76, 0.82), 0.5 * soil(u, v) + 0.5 * (1 - v))

    def bache(u, v):       # agujero oscuro con borde de tierra suelta
        d = math.hypot(u - 0.5, v - 0.5) * 2 + (soil(u, v) - 0.5) * 0.25
        if d > 0.95:
            return tierra(u, v)
        if d > 0.75:
            return mix(tierra(u, v), (0.55, 0.42, 0.30), 0.5)
        return mix((0.08, 0.06, 0.04), (0.22, 0.15, 0.09), d / 0.75 * soil(u, v))

    def rodada(u, v):      # tierra compactada con tacos del neumático en diagonal
        c = mix((0.30, 0.21, 0.13), (0.45, 0.33, 0.22), soil(u, v))
        taco = ((v * 16 + abs(u - 0.5) * 2) % 1.0) < 0.35
        return mix(c, (0.20, 0.14, 0.09), 0.55) if taco else c

    for name, fn in [('tierra', tierra), ('caballon', caballon), ('madera', madera),
                     ('corteza', corteza), ('hojas', hojas), ('charco', charco),
                     ('bache', bache), ('rodada', rodada)]:
        write_png(os.path.join(folder, name + '.png'), 256, fn)


DZ = 0.0  # altura que se suma a las visuales (la de encima del surco)


def visual(name, pose, geometry, color):
    """color: 'r g b a' o un <material> ya hecho (textured())."""
    pose = list(pose)
    pose[2] += DZ
    pose = ' '.join(f'{v:.3f}' for v in pose)
    mat = color if color.startswith('<material>') else material(color)
    return (f'        <visual name="{name}"><pose>{pose}</pose>'
            f'<geometry>{geometry}</geometry>{mat}'
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
'''
    # la textura se estira una vez por cara: tramos de 2 m para que no quede borrosa
    n_seg = int(round(length / 2.0))
    seg = length / n_seg
    for k in range(n_seg):
        out += visual(f'tierra_{k}', (-length / 2 + seg * (k + 0.5), 0, RIDGE_HEIGHT / 2, 0, 0, 0),
                      box(seg, RIDGE_WIDTH, RIDGE_HEIGHT), textured('caballon'))
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


def poste(name, x, y):
    """Poste de madera con colisión (Post)."""
    h = POST_HEIGHT
    geo = cyl(0.05, h)
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {h / 2:.3f} 0 0 {rnd.uniform(0, 6.28):.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geo}</geometry></collision>
        <visual name="visual"><geometry>{geo}</geometry>{textured('madera')}</visual>
      </link>
    </model>
'''


def postes():
    """Postes al principio y al final de cada surco y una valla al norte."""
    out = '    <!-- Postes (Post): cabeceras de los surcos y valla -->\n'
    for j, y in enumerate(RIDGE_YS):
        out += poste(f'poste_ini_{j}', FIELD_X0 - 0.3, y)
        out += poste(f'poste_fin_{j}', FIELD_X1 + 0.3, y)
    for k, x in enumerate(POST_FENCE_XS):
        out += poste(f'valla_{k}', x, POST_FENCE_Y)
    # alambre de la valla (solo visual)
    x0, x1 = POST_FENCE_XS[0], POST_FENCE_XS[-1]
    wires = ''.join(visual(f'alambre_{k}', ((x0 + x1) / 2, POST_FENCE_Y, z, 0, 1.5708, 0),
                           cyl(0.005, x1 - x0), '0.5 0.5 0.5 1')
                    for k, z in enumerate((0.5, 0.95)))
    out += f'''    <model name="valla_alambre">
      <static>true</static>
      <link name="link">
{wires}      </link>
    </model>
'''
    return out


def frutales():
    """Hilera de frutales jóvenes (Trunk): tronco con corteza y copa, colisión en el tronco."""
    out = '    <!-- Frutales (Trunk) -->\n'
    for k, x in enumerate(ORCHARD_XS):
        x += rnd.uniform(-0.2, 0.2)
        h = rnd.uniform(1.1, 1.5)
        r = rnd.uniform(0.08, 0.12)
        crown = rnd.uniform(0.7, 0.9)
        bark = textured('corteza')
        parts = (visual('tronco', (0, 0, h / 2, 0, 0, 0), cyl(r, h), bark)
                 + visual('rama1', (0.15, 0, h + 0.1, 0, 0.6, 0), cyl(r * 0.45, 0.6), bark)
                 + visual('rama2', (-0.12, 0.05, h + 0.1, 0, -0.5, 0.3), cyl(r * 0.45, 0.6),
                          bark)
                 + visual('copa', (0, 0, h + crown * 0.8, 0, 0, 0), sphere(crown),
                          textured('hojas')))
        out += f'''    <model name="frutal_{k}">
      <static>true</static>
      <pose>{x:.3f} {ORCHARD_Y:.3f} 0 0 0 {rnd.uniform(0, 6.28):.3f}</pose>
      <link name="link">
        <collision name="tronco"><pose>0 0 {h / 2:.3f} 0 0 0</pose>
          <geometry>{cyl(r, h)}</geometry></collision>
{parts}      </link>
    </model>
'''
    return out


def suelo_objetos():
    """Charcos (Puddle), baches (Pothole) y huellas de tractor (Trace): solo visuales,
    un poco por encima del suelo para que no parpadeen con él."""
    out = '''    <!-- Charcos (Puddle), baches (Pothole) y rodadas (Trace): visuales sobre el suelo -->
    <model name="suelo_objetos">
      <static>true</static>
      <link name="link">
'''
    for k, (x, y, sx, sy) in enumerate(PUDDLES):
        out += visual(f'charco_{k}', (x, y, 0.006, 0, 0, rnd.uniform(0, 3.14)),
                      box(sx, sy, 0.002), textured('charco', roughness=0.15))
    for k, (x, y, r) in enumerate(POTHOLES):
        out += visual(f'bache_{k}', (x, y, 0.006, 0, 0, 0), box(2 * r, 2 * r, 0.002),
                      textured('bache'))
    for k, (x0, x1, y) in enumerate(TRACKS):
        n = max(1, int(round((x1 - x0) / 1.5)))   # tramos de 1.5 m: textura sin estirar
        seg = (x1 - x0) / n
        for side in (-1, 1):
            for i in range(n):
                out += visual(f'rodada_{k}_{side + 1}_{i}',
                              (x0 + seg * (i + 0.5), y + side * TRACK_GAP / 2, 0.005, 0, 0, 0),
                              box(seg, TRACK_WIDTH, 0.002), textured('rodada'))
    out += '      </link>\n    </model>\n'
    return out


def suelo_tierra():
    """Capa de tierra labrada sobre la hierba en tramos de 4 m (textura sin estirar)."""
    out = ''
    x0, x1, y0, y1 = -5.0, 27.0, -5.25, 6.75
    for i in range(8):
        for j in range(3):
            out += visual(f'tierra_{i}_{j}', (x0 + 4 * (i + 0.5), y0 + 4 * (j + 0.5), 0.003,
                                              0, 0, 0), box(4.0, 4.0, 0.001), textured('tierra'))
    assert abs(x0 + 32 - x1) < 1e-6 and abs(y0 + 12 - y1) < 1e-6
    return out


def navmap_pcd(path):
    """Nube organizada (rejilla) del suelo plano z=0 para pointcloud_to_navmap."""
    nx = int(round((NAVMAP_X[1] - NAVMAP_X[0]) / NAVMAP_STEP)) + 1
    ny = int(round((NAVMAP_Y[1] - NAVMAP_Y[0]) / NAVMAP_STEP)) + 1
    lines = [f'{NAVMAP_X[0] + i * NAVMAP_STEP:.3f} {NAVMAP_Y[0] + j * NAVMAP_STEP:.3f} 0.000'
             for j in range(ny) for i in range(nx)]
    header = (f'# .PCD v0.7 - suelo plano de worlds/campo.sdf (generar_campo.py)\n'
              f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {nx}\nHEIGHT {ny}\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {nx * ny}\n'
              f'DATA ascii\n')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(header + '\n'.join(lines) + '\n')


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
     Objetos para la navegación semántica: postes, frutales, charcos, baches y rodadas.
     El robot debe empezar en (-1.5, 0.75) mirando a +x (lo hacen simulacion.launch.py
     y go2w_gis.launch.py con world:=campo). -->
<sdf version="1.8">
  <world name="campo">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- Gazebo Harmonic. Imu y NavSat: los sensores del robot de go2w_gis.launch.py -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- origen del mundo: junto al MIC (León), para que /gps/fix tenga sentido -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>42.61528</latitude_deg>
      <longitude_deg>-5.56333</longitude_deg>
      <elevation>830</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

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
{suelo_tierra()}      </link>
    </model>

    <!-- Surcos (caballones de {RIDGE_HEIGHT * 100:.0f} cm con colisión) y sus plantas (sin colisión) -->
{surcos}
{malas_hierbas()}
{postes()}
{frutales()}
{suelo_objetos()}
    <!-- Árboles alrededor del campo (Gazebo Fuel) -->
{arboles()}
{persona()}
  </world>
</sdf>
'''


if __name__ == '__main__':
    pkg = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    texturas(os.path.join(pkg, 'models', TEXTURES))
    path = os.path.join(pkg, 'worlds', 'campo.sdf')
    with open(path, 'w') as f:
        f.write(world())
    navmap_pcd(os.path.join(pkg, 'maps', 'campo_flat.pcd'))
    print(f'Escrito {path}, models/{TEXTURES}/*.png y maps/campo_flat.pcd')
