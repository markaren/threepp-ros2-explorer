"""Generates the Blender-made models of the warehouse demo (robot, racks, pallets, crate, signs).

Run it headless from the project root (no window opens):

    "C:\\Program Files\\Blender Foundation\\Blender 5.0\\blender.exe" --background --python tools/blender/make_models.py -- src/explorer_sim/assets/models

Add model names after the folder to build only those, for example `... -- src/explorer_sim/assets/models robot rack_a`.

Conventions (the simulator relies on them):
  * metres, real-world size
  * authored with ROS axes: +X forward, +Y left, +Z up; exported as .glb with the standard
    +Y-up conversion, so a loader sees +X forward, +Y up, and Blender's +Y as -Z
  * origin at the centre of the footprint, on the floor (signs: centre of the back face)
  * colours stay out of the orange band the cone detector looks for (OpenCV hue 3..24):
    blue, teal, green, grey, white, black, and pale desaturated "wood" and "cardboard"

Every model is built from a few primitives (bevelled box, lathe, loft, arc band) that are merged
into one mesh per moving part. A Weighted Normal modifier then makes the bevels shade round.
"""
import math
import os
import random
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

# ----------------------------------------------------------------------------------------------
# materials
# ----------------------------------------------------------------------------------------------


def _lin(c):
    """sRGB 0..255 -> linear 0..1 (Blender colours are linear)."""
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def mat(name, rgb, metallic=0.0, roughness=0.6, emission=None, strength=1.0):
    """A plain Principled BSDF material. `rgb` and `emission` are sRGB 0..255."""
    m = bpy.data.materials.get(name)
    if m:
        return m
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = next((n for n in nt.nodes if n.type == 'BSDF_PRINCIPLED'), None)
    if bsdf is None:
        bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
        out = nt.nodes.new('ShaderNodeOutputMaterial')
        nt.links.new(bsdf.outputs[0], out.inputs[0])
    bsdf.inputs["Base Color"].default_value = (_lin(rgb[0]), _lin(rgb[1]), _lin(rgb[2]), 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if emission:
        bsdf.inputs["Emission Color"].default_value = (_lin(emission[0]), _lin(emission[1]), _lin(emission[2]), 1.0)
        bsdf.inputs["Emission Strength"].default_value = strength
    return m


# ----------------------------------------------------------------------------------------------
# geometry primitives (each returns a bmesh in world coordinates)
# ----------------------------------------------------------------------------------------------


def box(size, center, bevel=0.006, seg=2, rot=None):
    """A box with rounded edges. `rot` is an optional 4x4 rotation applied about its centre."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=Vector(size), verts=bm.verts)
    bevel = min(bevel, 0.45 * min(size))
    if bevel > 0.0:
        bmesh.ops.bevel(bm, geom=bm.edges[:], offset=bevel, segments=seg, profile=0.5, affect='EDGES')
    m = Matrix.Translation(Vector(center))
    if rot is not None:
        m = m @ rot
    bmesh.ops.transform(bm, matrix=m, verts=bm.verts)
    return bm


def lathe(profile, n=48, axis='Z', origin=(0, 0, 0)):
    """Revolves a profile of (radius, height) points about an axis. Radius 0 closes an end."""
    bm = bmesh.new()
    rings = []
    for r, h in profile:
        if r <= 1e-9:
            rings.append([bm.verts.new((0.0, 0.0, h))])
        else:
            rings.append([bm.verts.new((r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), h))
                          for i in range(n)])
    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            if len(a) == 1 and len(b) == 1:
                continue
            if len(a) == 1:
                bm.faces.new((a[0], b[i], b[j]))
            elif len(b) == 1:
                bm.faces.new((a[i], a[j], b[0]))
            else:
                bm.faces.new((a[i], a[j], b[j], b[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    if axis == 'Y':
        rot = Matrix.Rotation(-math.pi / 2, 4, 'X')
    elif axis == 'X':
        rot = Matrix.Rotation(math.pi / 2, 4, 'Y')
    else:
        rot = Matrix.Identity(4)
    bmesh.ops.transform(bm, matrix=Matrix.Translation(Vector(origin)) @ rot, verts=bm.verts)
    return bm


def loft(rings, cap=True):
    """Skins a list of closed rings (each a list of xyz points, same count) and caps both ends."""
    bm = bmesh.new()
    vr = [[bm.verts.new(p) for p in ring] for ring in rings]
    n = len(vr[0])
    for a, b in zip(vr[:-1], vr[1:]):
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((a[i], a[j], b[j], b[i]))
    if cap:
        bm.faces.new(list(reversed(vr[0])))
        bm.faces.new(vr[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


def arc_band(r_in, r_out, z0, z1, th0, th1, n=24, yflat=None):
    """A strip with a rectangular section that follows a circle from angle th0 to th1 (degrees)."""
    rings = []
    for i in range(n + 1):
        t = math.radians(th0 + (th1 - th0) * i / n)
        c, s = math.cos(t), math.sin(t)

        def p(r, z):
            y = r * s
            if yflat is not None:
                y = max(-yflat, min(yflat, y))
            return (r * c, y, z)
        rings.append([p(r_in, z0), p(r_out, z0), p(r_out, z1), p(r_in, z1)])
    bm = bmesh.new()
    vr = [[bm.verts.new(q) for q in ring] for ring in rings]
    for a, b in zip(vr[:-1], vr[1:]):
        for i in range(4):
            j = (i + 1) % 4
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bm.faces.new(list(reversed(vr[0])))
    bm.faces.new(vr[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


# ----------------------------------------------------------------------------------------------
# a model = a few named parts, each one mesh
# ----------------------------------------------------------------------------------------------


class Part:
    """Collects primitives into ONE mesh, one material slot per material used."""

    def __init__(self, name):
        self.name = name
        self.bm = bmesh.new()
        self.mats = []

    def add(self, bm, material):
        if material not in self.mats:
            self.mats.append(material)
        idx = self.mats.index(material)
        for f in bm.faces:
            f.material_index = idx
            f.smooth = True
        tmp = bpy.data.meshes.new("tmp")
        bm.to_mesh(tmp)
        bm.free()
        self.bm.from_mesh(tmp)  # from_mesh appends
        bpy.data.meshes.remove(tmp)

    def finish(self, origin=(0, 0, 0), parent=None):
        """Turns the collected geometry into an object whose origin is at `origin`."""
        me = bpy.data.meshes.new(self.name)
        self.bm.to_mesh(me)
        self.bm.free()
        for m in self.mats:
            me.materials.append(m)
        me.transform(Matrix.Translation(-Vector(origin)))
        # edges sharper than 50 degrees stay crisp, the two-segment bevels (45 degrees) shade round
        me.set_sharp_from_angle(angle=math.radians(50))
        ob = bpy.data.objects.new(self.name, me)
        ob.location = origin
        bpy.context.scene.collection.objects.link(ob)
        wn = ob.modifiers.new("weighted_normals", 'WEIGHTED_NORMAL')
        wn.keep_sharp = True
        wn.weight = 50
        if parent is not None:
            ob.parent = parent
        return ob


def new_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def export(path):
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', export_yup=True, export_apply=True,
                              export_cameras=False, export_lights=False, export_animations=False,
                              export_tangents=False, use_selection=False)
    print(f"[make_models] wrote {path} ({os.path.getsize(path)} bytes)")


# ----------------------------------------------------------------------------------------------
# palette (all checked against the cone detector's orange band)
# ----------------------------------------------------------------------------------------------

CYAN = (25, 211, 255)
TEAL = (42, 157, 143)
# Cardboard and wood are kraft and pine, as they are. What keeps the cone detector from
# taking them for cones is their HUE: a cone is orange (25 degrees on the colour wheel),
# these are yellow-brown, at 38 to 41 degrees. The detector's limit lies at 30 degrees.
# In a shadow they turn dark and strongly coloured, but they keep their hue.
CARDBOARD = [(202, 181, 145), (196, 176, 141), (208, 187, 150), (190, 170, 136), (232, 230, 224), (178, 184, 188)]
WOOD = [(206, 188, 150), (198, 180, 144), (212, 193, 155)]
PLYWOOD = [(204, 187, 152), (188, 171, 138), (124, 112, 90)]     # panel, batten, the gaps between boards
TOTE = [(38, 92, 172), (52, 146, 92), (112, 122, 132), (30, 128, 140), (36, 60, 120)]


def m_cardboard(i):
    return mat(f"cardboard_{i}", CARDBOARD[i % len(CARDBOARD)], roughness=0.85)


def m_wood(i):
    return mat(f"wood_{i}", WOOD[i % len(WOOD)], roughness=0.8)


def m_tote(i):
    return mat(f"tote_{i}", TOTE[i % len(TOTE)], roughness=0.45)


# ----------------------------------------------------------------------------------------------
# robot
# ----------------------------------------------------------------------------------------------


def outline(radius, yflat, z, n=96):
    """A disc outline with two flat sides (where the drive wheels sit)."""
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        pts.append((radius * math.cos(t), max(-yflat, min(yflat, radius * math.sin(t))), z))
    return pts


def build_robot(out_dir):
    new_scene()
    white = mat("shell_white", (236, 239, 242), roughness=0.32)
    dark = mat("bumper_dark", (40, 45, 54), roughness=0.78)
    deck = mat("deck_dark", (46, 52, 64), roughness=0.62)
    glow = mat("light_cyan", (10, 60, 75), roughness=0.3, emission=CYAN, strength=1.0)
    rubber = mat("tyre_rubber", (34, 37, 44), roughness=0.95)
    rim = mat("rim_metal", (188, 194, 202), roughness=0.4, metallic=0.2)
    # satin navy, not mirror black: a mirror shows the warm warehouse photo, and the cone detector
    # reads those dark amber reflections as orange
    glass = mat("glass_navy", (18, 30, 56), roughness=0.45)
    lidar_grey = mat("lidar_grey", (64, 72, 86), roughness=0.55)

    body = Part("body")
    # bumper skirt, 0.03 .. 0.09 m, the widest part: radius 0.22
    body.add(loft([outline(r, f, z) for z, r, f in [
        (0.030, 0.204, 0.166), (0.034, 0.214, 0.174), (0.042, 0.220, 0.178),
        (0.078, 0.220, 0.178), (0.086, 0.216, 0.175), (0.090, 0.206, 0.168)]]), dark)
    # white shell with a rounded shoulder, up to the 0.19 m chassis top
    body.add(loft([outline(r, f, z) for z, r, f in [
        (0.088, 0.204, 0.167), (0.093, 0.212, 0.174), (0.150, 0.212, 0.174), (0.168, 0.207, 0.170),
        (0.180, 0.196, 0.161), (0.187, 0.180, 0.148), (0.190, 0.165, 0.136)]]), white)
    # dark top deck, a slightly raised plate
    body.add(loft([outline(r, f, z) for z, r, f in [
        (0.1890, 0.160, 0.132), (0.1935, 0.160, 0.132), (0.1950, 0.156, 0.129)]]), deck)
    # the front: a glowing strip around the nose, a glossy sensor window above it, an arc on the deck
    body.add(arc_band(0.200, 0.2150, 0.106, 0.124, -50, 50, n=32), glow)
    body.add(arc_band(0.200, 0.2140, 0.132, 0.150, -32, 32, n=24), glass)
    body.add(arc_band(0.128, 0.146, 0.1940, 0.1970, -52, 52, n=32), glow)
    # the back: a dark charging panel with two contacts, and a status light on the deck
    body.add(arc_band(0.200, 0.2135, 0.100, 0.138, 160, 200, n=16), deck)
    for th in (166, 188):
        body.add(arc_band(0.205, 0.2150, 0.108, 0.130, th, th + 6, n=4), rim)
    body.add(lathe([(0.0, 0.1945), (0.008, 0.1945), (0.008, 0.1968), (0.0, 0.1968)], n=16, origin=(-0.118, 0, 0)), glow)
    # drive units that carry the wheels, and the rear caster
    for s in (1, -1):
        body.add(lathe([(0.0, 0.0), (0.034, 0.0), (0.034, 0.014), (0.0, 0.014)], n=24, axis='Y',
                       origin=(0, s * 0.178 - (0.014 if s < 0 else 0.0), 0.07)), dark)
    body.add(lathe([(0.0, -0.008), (0.016, -0.008), (0.020, -0.004), (0.020, 0.004), (0.016, 0.008), (0.0, 0.008)],
                   n=20, axis='Y', origin=(-0.160, 0, 0.020)), rubber)
    body.add(box((0.030, 0.030, 0.016), (-0.160, 0, 0.040), bevel=0.003), dark)
    # lidar mast
    body.add(lathe([(0.046, 0.1945), (0.046, 0.200), (0.034, 0.207), (0.029, 0.214), (0.029, 0.270),
                    (0.034, 0.2765), (0.0, 0.2765)], n=40), deck)
    # camera: pylon, housing, lens. The glass is at x = 0.18, z = 0.25
    body.add(box((0.032, 0.046, 0.046), (0.150, 0, 0.212), bevel=0.006), white)
    body.add(box((0.042, 0.078, 0.046), (0.151, 0, 0.250), bevel=0.009, seg=3), dark)
    body.add(lathe([(0.0165, 0.168), (0.0165, 0.1775), (0.0145, 0.1790), (0.0120, 0.1790)], n=28, axis='X',
                   origin=(0, 0, 0.250)), deck)
    body.add(lathe([(0.0120, 0.1790), (0.0100, 0.1800), (0.0, 0.1800)], n=28, axis='X', origin=(0, 0, 0.250)), glass)

    root = bpy.data.objects.new("robot", None)
    bpy.context.scene.collection.objects.link(root)
    body.finish(parent=root)

    # wheels: radius 0.07, axle along Y at y = +/-0.20, origin ON the axle
    for name, s in (("wheel_left", 1), ("wheel_right", -1)):
        w = Part(name)
        c = (0.0, s * 0.200, 0.070)
        w.add(lathe([(0.050, -0.012), (0.062, -0.012), (0.070, -0.007), (0.070, 0.007), (0.062, 0.012),
                     (0.050, 0.012)], n=40, axis='Y', origin=c), rubber)
        w.add(lathe([(0.0, -0.0100), (0.048, -0.0100), (0.053, -0.0115), (0.053, 0.0115), (0.048, 0.0100),
                     (0.0, 0.0100)], n=40, axis='Y', origin=c), rim)
        for k in range(5):  # five dark pockets, so a spinning wheel is seen to spin
            a = 2 * math.pi * k / 5
            for side in (1, -1):
                w.add(lathe([(0.0, 0.0), (0.0095, 0.0), (0.0095, 0.002), (0.0, 0.002)], n=12, axis='Y',
                            origin=(0.031 * math.cos(a), c[1] + side * 0.0100 - (0.002 if side < 0 else 0.0),
                                    c[2] + 0.031 * math.sin(a))), rubber)
        for side in (1, -1):
            w.add(lathe([(0.0, 0.0), (0.011, 0.0), (0.009, 0.004), (0.0, 0.004)] if side > 0 else
                        [(0.0, -0.004), (0.009, -0.004), (0.011, 0.0), (0.0, 0.0)], n=16, axis='Y',
                        origin=(0, c[1] + side * 0.0100, c[2])), deck)
        w.finish(origin=c, parent=root)

    # lidar head: origin on the vertical axis, in the scan plane at z = 0.30
    h = Part("lidar_head")
    h.add(lathe([(0.0, 0.2775), (0.035, 0.2775), (0.040, 0.2815), (0.040, 0.2900), (0.0370, 0.2900)], n=40), lidar_grey)
    h.add(lathe([(0.0370, 0.2900), (0.0370, 0.3100)], n=40), glass)
    h.add(lathe([(0.0370, 0.3100), (0.040, 0.3100), (0.040, 0.3190), (0.035, 0.3245), (0.0, 0.3245)], n=40), lidar_grey)
    h.add(box((0.022, 0.007, 0.002), (0.020, 0, 0.3250), bevel=0.0), glow)  # shows the spin
    h.finish(origin=(0, 0, 0.300), parent=root)

    export(os.path.join(out_dir, "robot.glb"))


# ----------------------------------------------------------------------------------------------
# goods: cartons, totes, shrink-wrapped bundles
# ----------------------------------------------------------------------------------------------


def carton(part, x0, x1, y0, y1, z0, h, rng):
    """A taped cardboard box with a label on both aisle faces."""
    w, d = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    card = m_cardboard(rng.randrange(len(CARDBOARD)))
    tape = mat("tape", (232, 234, 238), roughness=0.4) if rng.random() < 0.6 else mat("tape_blue", (58, 108, 186), roughness=0.4)
    label = mat("label_white", (240, 240, 238), roughness=0.6)
    ink = mat("label_ink", (26, 34, 60), roughness=0.6)
    part.add(box((w, d, h), (cx, cy, z0 + h / 2), bevel=0.005), card)
    part.add(box((w - 0.004, 0.05, 0.003), (cx, cy, z0 + h), bevel=0.0), tape)
    for sx in (-1, 1):
        part.add(box((0.003, 0.05, min(0.07, h * 0.4)), (cx + sx * w / 2, cy, z0 + h - min(0.07, h * 0.4) / 2), bevel=0.0), tape)
    if w > 0.22 and h > 0.16:
        lx = cx + rng.uniform(-0.25, 0.25) * (w - 0.16)
        lz = z0 + h * rng.uniform(0.45, 0.65)
        for sy in (-1, 1):
            part.add(box((0.11, 0.003, 0.075), (lx, cy + sy * d / 2, lz), bevel=0.0), label)
            part.add(box((0.085, 0.004, 0.016), (lx, cy + sy * d / 2, lz + 0.018), bevel=0.0), ink)
            part.add(box((0.05, 0.004, 0.008), (lx - 0.0175, cy + sy * d / 2, lz - 0.012), bevel=0.0), ink)


def tote(part, cx, cy, z0, w, d, h, colour):
    """A lidded plastic tote: tapered body, rim, lid, a label on both aisle faces."""
    body = m_tote(colour)
    lid = mat(f"tote_lid_{colour % len(TOTE)}", tuple(min(255, int(c * 1.18) + 14) for c in TOTE[colour % len(TOTE)]), roughness=0.5)
    label = mat("label_white", (240, 240, 238), roughness=0.6)
    bm = box((w, d, h), (0, 0, 0), bevel=0.012)
    for v in bm.verts:  # narrower at the bottom
        s = 0.92 + 0.08 * (v.co.z + h / 2) / h
        v.co.x *= s
        v.co.y *= s
    bmesh.ops.translate(bm, vec=Vector((cx, cy, z0 + h / 2)), verts=bm.verts)
    part.add(bm, body)
    part.add(box((w + 0.012, d + 0.012, 0.028), (cx, cy, z0 + h - 0.016), bevel=0.006), body)
    part.add(box((w - 0.03, d - 0.03, 0.012), (cx, cy, z0 + h + 0.002), bevel=0.004), lid)
    for sy in (-1, 1):
        part.add(box((min(0.12, w * 0.4), 0.010, 0.06), (cx, cy + sy * d / 2 * 0.955, z0 + h * 0.45), bevel=0.0), label)


def wrapped(part, x0, x1, y0, y1, z0, h, rng):
    """A shrink-wrapped bundle on a plastic skid."""
    w, d = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    skid = mat("skid_plastic", (44, 62, 96), roughness=0.5)
    wrap = mat("shrink_wrap", (214, 224, 232), roughness=0.18)
    band = mat("wrap_band", (188, 202, 214), roughness=0.22)
    part.add(box((w, d, 0.045), (cx, cy, z0 + 0.0225), bevel=0.006), skid)
    part.add(box((w - 0.03, d - 0.03, h - 0.045), (cx, cy, z0 + 0.045 + (h - 0.045) / 2), bevel=0.03, seg=3), wrap)
    for k in range(1, 4):  # the folds of the film
        zk = z0 + 0.045 + (h - 0.045) * k / 4 + rng.uniform(-0.015, 0.015)
        part.add(box((w - 0.026, d - 0.026, 0.012), (cx, cy, zk), bevel=0.005), band)


def fill_shelf(part, rng, z0, h_min, h_max, weights, allow_gaps, empty=0.15):
    """Fills one load level of a rack bay from end to end with a random mix of goods.
    `empty` is how likely a place is left free, where gaps are allowed."""
    x_lo, x_hi, gap = -1.585, 1.585, 0.024
    widths = []
    while sum(widths) < (x_hi - x_lo) - 0.3:
        widths.append(rng.uniform(0.38, 0.86))
    scale = ((x_hi - x_lo) - gap * (len(widths) - 1)) / sum(widths)
    widths = [w * scale for w in widths]
    kinds = [k for k, n in weights.items() for _ in range(n)]
    x = x_lo
    gaps_left = (1 if empty <= 0.15 else len(widths)) if allow_gaps else 0  # usually at most one empty place per level
    for w in widths:
        x0, x1 = x, x + w
        x = x1 + gap
        kind = rng.choice(kinds)
        d = rng.uniform(0.54, 0.64)
        cy = rng.uniform(-0.012, 0.012)
        if gaps_left and rng.random() < empty:
            gaps_left -= 1
            continue
        if kind == 'tote':
            n = max(1, int((w + 0.01) / 0.40))
            tw = (w - 0.012 * (n + 1)) / n
            layers = 2 if h_max > 0.3 and (h_min > 0.25 or rng.random() < 0.6) else 1
            th = min(0.22, (h_max - 0.02) / layers)
            col = rng.randrange(len(TOTE))
            for i in range(n):
                for k in range(layers):
                    tote(part, x0 + 0.012 + tw / 2 + i * (tw + 0.012), cy, z0 + k * (th + 0.004), tw, 0.60, th,
                         col if rng.random() < 0.7 else rng.randrange(len(TOTE)))
        elif kind == 'wrap':
            wrapped(part, x0, x1, cy - d / 2, cy + d / 2, z0, rng.uniform(max(h_min, 0.36), h_max), rng)
        elif kind == 'stack':
            h1 = rng.uniform(max(h_min, 0.22), max(h_min, 0.22) + 0.06)
            carton(part, x0, x1, cy - d / 2, cy + d / 2, z0, h1, rng)
            rest = h_max - h1
            if rest > 0.10:
                w2 = w * rng.uniform(0.55, 0.9)
                d2 = d * rng.uniform(0.6, 0.95)
                xo = x0 + rng.uniform(0, w - w2)
                carton(part, xo, xo + w2, cy - d2 / 2, cy + d2 / 2, z0 + h1, min(rest, rng.uniform(0.10, 0.2)), rng)
        else:  # one carton, or two side by side
            h = rng.uniform(max(h_min, 0.2), h_max)
            if w > 0.6 and rng.random() < 0.5:
                xm = x0 + w * rng.uniform(0.42, 0.58)
                carton(part, x0, xm - 0.008, cy - d / 2, cy + d / 2, z0, h, rng)
                carton(part, xm + 0.008, x1, cy - d / 2, cy + d / 2, z0, rng.uniform(max(h_min, 0.2), h_max), rng)
            else:
                carton(part, x0, x1, cy - d / 2, cy + d / 2, z0, h, rng)


# ----------------------------------------------------------------------------------------------
# shelving, one bay: 3.4 x 0.8 m footprint, 2 m high, four load levels
# ----------------------------------------------------------------------------------------------


RACK_H = 2.0
RACK_LEVELS = (0.12, 0.74, 1.36, 1.96)      # the top of each pair of beams


def build_rack(out_dir, name, seed, weights_low, weights_high):
    new_scene()
    rng = random.Random(seed)
    blue = mat("upright_blue", (28, 78, 160), roughness=0.45, metallic=0.1)
    slot = mat("upright_slot", (12, 22, 48), roughness=0.7)
    galv = mat("beam_galvanised", (168, 176, 184), roughness=0.42, metallic=0.35)
    plate = mat("base_plate", (120, 126, 134), roughness=0.5, metallic=0.3)
    deckm = mat("deck_panel", (196, 200, 204), roughness=0.6, metallic=0.1)

    p = Part(name)
    ux, uy = 1.655, 0.365  # upright centres; the base plates reach the footprint exactly
    for sx in (-1, 1):
        for sy in (-1, 1):
            p.add(box((0.08, 0.06, RACK_H - 0.005), (sx * ux, sy * uy, 0.005 + (RACK_H - 0.005) / 2), bevel=0.007), blue)
            p.add(box((0.09, 0.07, 0.008), (sx * 1.655, sy * 0.365, 0.004), bevel=0.0), plate)
            p.add(box((0.12, 0.10, 0.006), (sx * 1.64, sy * 0.35, 0.003), bevel=0.0), plate)
            k = 0
            z = 0.06
            while z < RACK_H - 0.05:  # the punched slots the beams hook into
                p.add(box((0.014, 0.003, 0.032), (sx * ux + (0.016 if k % 2 else -0.016), sy * (uy + 0.03), z), bevel=0.0), slot)
                z += 0.05
                k += 1
        # the end frame: horizontal and diagonal bracing between the two uprights
        ties = (0.15, 0.75, 1.35, RACK_H - 0.07)
        for z in ties:
            p.add(box((0.028, 0.67, 0.028), (sx * ux, 0, z), bevel=0.004), blue)
        for i, (z0, z1) in enumerate(zip(ties, ties[1:])):
            flip = 1 if i % 2 == 0 else -1
            dy, dz = 0.67 * flip, z1 - z0
            p.add(box((0.024, math.hypot(dy, dz), 0.024), (sx * ux, 0, (z0 + z1) / 2), bevel=0.004,
                      rot=Matrix.Rotation(math.atan2(dz, dy), 4, 'X')), blue)
    # four pairs of beams, each with a deck and goods
    for top in RACK_LEVELS:
        for sy in (-1, 1):
            p.add(box((3.23, 0.05, 0.10), (0, sy * uy, top - 0.05), bevel=0.008), galv)
            for sx in (-1, 1):
                p.add(box((0.012, 0.064, 0.125), (sx * 1.609, sy * uy, top - 0.05), bevel=0.003), plate)
        p.add(box((3.23, 0.70, 0.018), (0, 0, top - 0.004), bevel=0.003), deckm)
    # The lowest level is filled without a gap from 0.125 m to at least 0.485 m: that is
    # the height the lidar scans at, and it must see a rack, not look through one.
    fill_shelf(p, rng, RACK_LEVELS[0] + 0.005, 0.36, 0.46, weights_low, allow_gaps=False)
    fill_shelf(p, rng, RACK_LEVELS[1] + 0.005, 0.16, 0.46, weights_high, allow_gaps=True)
    fill_shelf(p, rng, RACK_LEVELS[2] + 0.005, 0.16, 0.44, weights_low, allow_gaps=True)
    # On top: a few low things, so that nothing reaches above 2.2 m.
    fill_shelf(p, rng, RACK_LEVELS[3] + 0.005, 0.12, 0.22, {'box': 2, 'tote': 2}, allow_gaps=True, empty=0.45)
    p.finish()
    export(os.path.join(out_dir, name + ".glb"))


# ----------------------------------------------------------------------------------------------
# pallets
# ----------------------------------------------------------------------------------------------


def pallet_base(p):
    """A 1200 x 800 x 144 mm pallet: three runners, nine blocks, three stringers, five deck boards."""
    k = 0
    for y in (-0.35, 0.0, 0.35):
        p.add(box((1.2, 0.10, 0.022), (0, y, 0.011), bevel=0.003), m_wood(k)); k += 1
        for x in (-0.5275, 0.0, 0.5275):
            p.add(box((0.145, 0.10, 0.078), (x, y, 0.022 + 0.039), bevel=0.004), m_wood(k)); k += 1
    for x in (-0.5275, 0.0, 0.5275):
        p.add(box((0.145, 0.8, 0.022), (x, 0, 0.100 + 0.011), bevel=0.003), m_wood(k)); k += 1
    for y, wd in ((-0.3275, 0.145), (-0.17, 0.10), (0.0, 0.145), (0.17, 0.10), (0.3275, 0.145)):
        p.add(box((1.2, wd, 0.022), (0, y, 0.122 + 0.011), bevel=0.003), m_wood(k)); k += 1


def build_pallet(out_dir, name):
    new_scene()
    rng = random.Random(7 if name.endswith("a") else 11)
    p = Part(name)
    pallet_base(p)
    z = 0.144
    if name.endswith("a"):  # cartons, three layers, strapped
        for i in range(3):
            carton(p, -0.59 + i * 0.395, -0.59 + i * 0.395 + 0.385, -0.385, 0.385, z, 0.27, rng)
        z += 0.27
        carton(p, -0.585, -0.005, -0.38, 0.38, z, 0.25, rng)
        carton(p, 0.005, 0.585, -0.38, 0.38, z, 0.25, rng)
        z += 0.25
        carton(p, -0.56, 0.02, -0.36, 0.04, z, 0.22, rng)
        carton(p, 0.06, 0.52, -0.30, 0.30, z, 0.16, rng)
        strap = mat("strap_blue", (30, 96, 190), roughness=0.4)
        for x in (-0.30, 0.30):
            p.add(box((0.018, 0.776, 0.003), (x, 0, 0.144 + 0.52 + 0.001), bevel=0.0), strap)
            for sy in (-1, 1):
                p.add(box((0.018, 0.003, 0.52), (x, sy * 0.3865, 0.144 + 0.26), bevel=0.0), strap)
    else:  # euro totes, 2 x 2, three high, the top layer incomplete
        cols = [0, 0, 1, 2]
        for k in range(3):
            for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
                if k == 2 and i == 3:
                    continue
                c = cols[(i + k) % 4]
                # totes are 0.6 long in Y in tote(); here they lie along X, so swap by building them rotated
                tote_x(p, sx * 0.30, sy * 0.20, z + k * 0.224, c)
    p.finish()
    export(os.path.join(out_dir, name + ".glb"))


def tote_x(part, cx, cy, z0, colour):
    """A 0.6 x 0.4 x 0.22 m tote with its long side along X."""
    tmp = Part("tmp")
    tote(tmp, 0, 0, 0, 0.385, 0.585, 0.22, colour)
    rot = Matrix.Translation(Vector((cx, cy, z0))) @ Matrix.Rotation(math.pi / 2, 4, 'Z')
    bmesh.ops.transform(tmp.bm, matrix=rot, verts=tmp.bm.verts)
    for f in tmp.bm.faces:  # carry the materials over to the target part
        m = tmp.mats[f.material_index]
        if m not in part.mats:
            part.mats.append(m)
    remap = [part.mats.index(m) for m in tmp.mats]
    for f in tmp.bm.faces:
        f.material_index = remap[f.material_index]
    me = bpy.data.meshes.new("tmp")
    tmp.bm.to_mesh(me)
    tmp.bm.free()
    part.bm.from_mesh(me)
    bpy.data.meshes.remove(me)


# ----------------------------------------------------------------------------------------------
# crate: a UNIT cube, scaled per axis by the simulator
# ----------------------------------------------------------------------------------------------


def build_crate(out_dir):
    new_scene()
    frame = mat("plywood_batten", PLYWOOD[1], roughness=0.75)
    panel = mat("plywood_panel", PLYWOOD[0], roughness=0.8)
    groove = mat("plywood_gap", PLYWOOD[2], roughness=0.85)
    steel = mat("crate_corner_steel", (150, 158, 166), roughness=0.4, metallic=0.4)
    p = Part("crate")
    t = 0.09  # frame board width
    p.add(box((0.94, 0.94, 0.94), (0, 0, 0.5), bevel=0.004), panel)
    for a in (-1, 1):       # twelve frame boards along the edges
        for b in (-1, 1):
            p.add(box((1.0, t, t), (0, a * (0.5 - t / 2), 0.5 + b * (0.5 - t / 2)), bevel=0.007), frame)
            p.add(box((t, 1.0, t), (a * (0.5 - t / 2), 0, 0.5 + b * (0.5 - t / 2)), bevel=0.007), frame)
            p.add(box((t, t, 1.0), (a * (0.5 - t / 2), b * (0.5 - t / 2), 0.5), bevel=0.007), frame)
    for s in (-1, 1):       # plank joints and a diagonal brace on each side, battens on the lid
        for u in (-0.205, 0.0, 0.205):
            p.add(box((0.012, 0.004, 0.82), (u, s * 0.471, 0.5), bevel=0.0), groove)
            p.add(box((0.004, 0.012, 0.82), (s * 0.471, u, 0.5), bevel=0.0), groove)
            p.add(box((0.012, 0.82, 0.004), (u, 0, 0.971), bevel=0.0), groove)
        p.add(box((1.16, 0.022, t), (0, s * 0.481, 0.5), bevel=0.005, rot=Matrix.Rotation(s * math.radians(45), 4, 'Y')), frame)
        p.add(box((0.022, 1.16, t), (s * 0.481, 0, 0.5), bevel=0.005, rot=Matrix.Rotation(s * math.radians(45), 4, 'X')), frame)
        p.add(box((t, 0.82, 0.022), (s * 0.20, 0, 0.981), bevel=0.005), frame)
    for a in (-1, 1):       # steel corner blocks
        for b in (-1, 1):
            for c in (-1, 1):
                p.add(box((0.11, 0.11, 0.11), (a * 0.445, b * 0.445, 0.5 + c * 0.445), bevel=0.01), steel)
    p.finish()
    export(os.path.join(out_dir, "crate.glb"))


# ----------------------------------------------------------------------------------------------
# signs: origin at the centre of the BACK face, readable from +X
# ----------------------------------------------------------------------------------------------


def find_font():
    for path in (os.path.join(os.environ.get("WINDIR", ""), "Fonts", "arialbd.ttf"),
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                 "/System/Library/Fonts/Supplemental/Arial Bold.ttf"):
        if os.path.isfile(path):
            return bpy.data.fonts.load(path)
    return None  # Blender's built-in font


def text_mesh(text, max_w, max_h, x_face, depth=0.003):
    """Real geometry for `text`, fitted into max_w x max_h, standing on the plane x = x_face."""
    cu = bpy.data.curves.new("txt", 'FONT')
    cu.body = text
    cu.align_x = 'CENTER'
    cu.extrude = depth / 2
    cu.resolution_u = 3
    font = find_font()
    if font:
        cu.font = font
    ob = bpy.data.objects.new("txt", cu)
    bpy.context.scene.collection.objects.link(ob)
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    bpy.data.objects.remove(ob)
    bm = bmesh.new()
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    xs = [v.co.x for v in bm.verts]
    ys = [v.co.y for v in bm.verts]
    s = min(max_w / (max(xs) - min(xs)), max_h / (max(ys) - min(ys)))
    centre = Vector(((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, -depth / 2))
    bmesh.ops.translate(bm, vec=-centre, verts=bm.verts)
    bmesh.ops.scale(bm, vec=Vector((s, s, 1.0)), verts=bm.verts)
    # text X -> +Y, text Y -> +Z, text Z -> +X: readable for someone looking along -X
    m = Matrix(((0, 0, 1, x_face), (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 0, 1)))
    bmesh.ops.transform(bm, matrix=m, verts=bm.verts)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


def build_sign(out_dir, name, text, w, h, field_rgb, text_h, emissive=False):
    new_scene()
    if emissive:
        field = mat("sign_field", field_rgb, roughness=0.5, emission=field_rgb, strength=0.45)
        white = mat("sign_white", (245, 247, 248), roughness=0.5, emission=(245, 247, 248), strength=0.45)
    else:
        field = mat("sign_field", field_rgb, roughness=0.5)
        white = mat("sign_white", (245, 247, 248), roughness=0.5)
    back = mat("sign_back", (150, 156, 162), roughness=0.5, metallic=0.3)
    p = Part(name)
    p.add(box((0.014, w, h), (0.007, 0, 0), bevel=0.003), back)
    p.add(box((0.006, w - 0.006, h - 0.006), (0.017, 0, 0), bevel=0.002), field)
    b, ins = 0.008, 0.016  # white border line
    for s in (-1, 1):
        p.add(box((0.002, w - 2 * ins, b), (0.0205, 0, s * (h / 2 - ins - b / 2)), bevel=0.0), white)
        p.add(box((0.002, b, h - 2 * ins), (0.0205, s * (w / 2 - ins - b / 2), 0), bevel=0.0), white)
    p.add(text_mesh(text, w - 2 * ins - 0.07, text_h, 0.0195), white)
    p.finish()
    export(os.path.join(out_dir, name + ".glb"))


# ----------------------------------------------------------------------------------------------

DEEP_BLUE = (22, 62, 140)
EXIT_GREEN = (18, 136, 72)

BUILDERS = {
    "robot": build_robot,
    "rack_a": lambda d: build_rack(d, "rack_a", 3, {'carton': 4, 'stack': 3, 'tote': 2, 'wrap': 1},
                                   {'carton': 4, 'stack': 2, 'tote': 2, 'wrap': 1}),
    "rack_b": lambda d: build_rack(d, "rack_b", 12, {'carton': 2, 'stack': 1, 'tote': 4, 'wrap': 3},
                                   {'carton': 2, 'stack': 2, 'tote': 3, 'wrap': 2}),
    "pallet_a": lambda d: build_pallet(d, "pallet_a"),
    "pallet_b": lambda d: build_pallet(d, "pallet_b"),
    "crate": build_crate,
    "sign_aisle_A": lambda d: build_sign(d, "sign_aisle_A", "A", 0.45, 0.45, DEEP_BLUE, 0.27),
    "sign_aisle_B": lambda d: build_sign(d, "sign_aisle_B", "B", 0.45, 0.45, DEEP_BLUE, 0.27),
    "sign_aisle_C": lambda d: build_sign(d, "sign_aisle_C", "C", 0.45, 0.45, DEEP_BLUE, 0.27),
    "sign_aisle_D": lambda d: build_sign(d, "sign_aisle_D", "D", 0.45, 0.45, DEEP_BLUE, 0.27),
    "sign_dock": lambda d: build_sign(d, "sign_dock", "LOADING DOCK", 1.2, 0.3, DEEP_BLUE, 0.13),
    "sign_storage": lambda d: build_sign(d, "sign_storage", "STORAGE", 1.2, 0.3, DEEP_BLUE, 0.13),
    "sign_office": lambda d: build_sign(d, "sign_office", "OFFICE", 1.2, 0.3, DEEP_BLUE, 0.13),
    "sign_exit": lambda d: build_sign(d, "sign_exit", "EXIT", 0.6, 0.25, EXIT_GREEN, 0.12, emissive=True),
}


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if not args:
        raise SystemExit("usage: blender --background --python make_models.py -- <output folder> [model names]")
    out_dir = os.path.abspath(args[0])
    os.makedirs(out_dir, exist_ok=True)
    for name in (args[1:] or list(BUILDERS)):
        BUILDERS[name](out_dir)


main()
