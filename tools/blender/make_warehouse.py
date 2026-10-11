"""Builds the warehouse's building in Blender, bakes its lighting with Cycles, and exports it.

Run it headless from the project root (no window opens):

    blender --background --factory-startup --python tools/blender/make_warehouse.py -- src/explorer_sim/assets

It writes, into <assets>/warehouse:

    building.glb        floor, walls, ceiling, beams, lamps, doors. Two UV sets per mesh:
                        the first repeats the surface texture, the second is the lightmap's
    props.glb           everything that stands in the building and never moves (racks,
                        pallets, desks ...), each at its place, merged into one mesh per room
    lm_<mesh>.jpg       one lightmap per mesh of the building: how much light falls on
                        every spot of it
    baked_<mesh>.jpg    one picture per mesh of the props: their colour AND the light on it
    materials.txt       what the simulator needs to know about the building's materials

Why bake. An interior looks real because of light that has bounced: off the floor onto the
ceiling, around a corner, under a rack. Computing that takes a path tracer seconds per
picture. But as long as nothing moves, it only has to be computed once. Cycles does it
here, and stores the result as a picture of light per surface (a lightmap). The simulator
then draws colour texture times lightmap, which costs less than one real-time lamp.

The building's measures must match src/explorer_sim/src/world.cpp: the simulator's
collisions and its lidar use the boxes there, this file only makes what is looked at.

Options after the folder:
    --preview <file.png>   also render one still from behind the robot's start (for a look)
    --samples N            path tracing samples per lightmap texel (default 256)
    --size N               lightmap size in pixels (default 2048)
    --no-bake              build and export only (the lightmaps on disk are kept)
"""
import math
import os
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Vector

# --------------------------------------------------------------------------------------
# The building. Measures in metres, ROS axes: x forward (east), y left (north), z up.
# --------------------------------------------------------------------------------------

MIN_X, MAX_X, MIN_Y, MAX_Y = -2.0, 24.0, -8.0, 8.0     # the outer walls stand on these lines
HALF_T = 0.1            # half the thickness of a wall
CEILING = 4.2           # underside of the roof deck
CUT = 2.2               # the overview shows the building cut open at this height (door height)
DADO = 1.1              # walls are painted darker up to here
IN_X0, IN_X1 = MIN_X + HALF_T, MAX_X - HALF_T           # the inner faces of the outer walls
IN_Y0, IN_Y1 = MIN_Y + HALF_T, MAX_Y - HALF_T

# Inner walls: (axis the wall runs along, its position on the other axis, [solid stretches]).
# The gaps between the stretches are the doorways.
INNER_WALLS = [
    ("y", 5.0, [(IN_Y0, -1.0), (1.0, 5.5), (7.0, IN_Y1)]),         # dock | hall
    ("y", 17.0, [(IN_Y0, -6.5), (-5.0, 2.0), (3.5, IN_Y1)]),       # hall | offices
    ("x", 0.0, [(17.0 + HALF_T, 20.0), (21.5, IN_X1)]),            # office | office
]
# How high each doorway is. Above it the wall goes on up to the ceiling.
DOOR_HEIGHT = {("y", 5.0, -1.0): 3.0}      # the wide door of the dock: (axis, position, where the gap starts)
DEFAULT_DOOR_HEIGHT = CUT

# Lamps: LED battens hanging from the roof, (x, y, runs along x?).
LAMP_Z = 3.45
LAMPS = [
    (1.5, 4.6, True), (1.5, 0.0, True), (1.5, -4.6, True),                      # loading dock
    (8.7, 0.0, True), (13.3, 0.0, True),                                        # main corridor
    (8.7, 3.7, True), (13.3, 3.7, True), (8.7, -3.7, True), (13.3, -3.7, True),  # aisles
    (8.7, 6.8, True), (13.3, 6.8, True), (8.7, -6.8, True), (13.3, -6.8, True),
    (19.0, 4.0, False), (22.0, 4.0, False), (19.0, -4.0, False), (22.0, -4.0, False),  # offices
]
# Roof beams, running north-south at these x.
BEAMS_X = [1.5, 9.0, 13.0, 20.5]      # (the inner walls at x = 5 and 17 carry the roof too)
# High windows in the north wall: (x0, x1). Daylight, cooler than the lamps.
WINDOWS = [(-0.5, 3.5), (6.5, 10.0), (12.0, 15.5), (18.5, 22.5)]
WINDOW_Z = (2.9, 3.7)

# What stands in the building: world.cpp's boxes that have a model. THIS LIST REPEATS
# world.cpp. A box added there collides, is seen by the lidar and is drawn at once, but
# with plain light. It gets baked light when it has been added here too and this script
# has been run.
# (x, y, size x, size y, height, model, quarter turns, stretched to its height?)
PROPS = [(x, y, 3.4, 0.8, 2.0, ("rack_b" if i % 3 == 1 else "rack_a"), (i % 2) * 2, False)
         for i, (y, x) in enumerate((y, x) for y in (5.0, 2.4, -2.4, -5.0) for x in (8.7, 13.3))]
PROPS += [
    (-0.8, 5.6, 1.0, 1.0, 0.9, "crate", 0, True), (0.5, 6.7, 1.0, 1.0, 0.6, "crate", 0, True),
    (1.9, -6.7, 0.8, 0.8, 0.5, "crate", 0, True), (3.2, -5.8, 1.2, 0.8, 0.9, "pallet_a", 0, False),
    (-1.0, -3.0, 0.8, 1.2, 0.9, "pallet_b", 1, False), (-1.55, -7.55, 0.5, 0.5, 0.88, "barrel", 0, False),
    (-0.95, -7.6, 0.5, 0.5, 0.88, "barrel", 0, False), (-1.55, -6.95, 0.5, 0.5, 0.88, "barrel", 1, False),
    (4.25, 3.3, 1.1, 1.6, 1.38, "cart", 0, False), (-1.78, -4.9, 0.2, 0.66, 2.1, "ladder", 0, False),
    (9.0, 0.9, 0.8, 0.8, 0.7, "crate", 0, True), (13.6, -0.8, 1.2, 0.8, 0.9, "pallet_a", 0, False),
    (22.3, 7.35, 2.0, 0.95, 0.79, "desk", 3, False), (19.3, 7.35, 2.0, 0.95, 0.79, "desk", 3, False),
    (23.37, 2.6, 0.95, 2.0, 0.79, "desk", 2, False), (18.4, -0.63, 2.0, 0.95, 0.79, "desk", 3, False),
    (22.6, -7.37, 2.0, 0.95, 0.79, "desk", 1, False), (22.3, 6.45, 0.47, 0.45, 0.46, "stool", 0, False),
    (22.45, 2.4, 0.47, 0.45, 0.46, "stool", 0, False), (22.6, -6.45, 0.47, 0.45, 0.46, "stool", 0, False),
    (21.0, -4.2, 1.0, 1.0, 0.8, "crate", 0, True), (17.65, 6.9, 0.8, 1.2, 0.9, "pallet_b", 1, False),
]
CONES = [(2.4, 5.4), (9.0, 6.6), (15.5, -3.7), (16.1, 0.9), (20.5, 5.0), (22.5, -3.0)]
FLOOR_LINES = [(5.4, 1.75, 16.6, 1.75, 0.08), (5.4, -1.75, 16.6, -1.75, 0.08)]
FLOOR_LINES += [(6.8, y, 15.2, y, 0.05) for y in (5.6, 4.4, 3.0, -3.0, -4.4, -5.6)]
FLOOR_LINES += [(-0.6, -0.6, 0.6, -0.6, 0.08), (0.6, -0.6, 0.6, 0.6, 0.08),
                (0.6, 0.6, -0.6, 0.6, 0.08), (-0.6, 0.6, -0.6, -0.6, 0.08)]

# Lightmap values are stored divided by this, so that light brighter than "1" (right under
# a lamp) survives in an 8-bit picture. The simulator multiplies by it again.
LIGHTMAP_SCALE = 4.0

# --------------------------------------------------------------------------------------
# Materials: (colour texture in the assets folder or None, tint, metres per repeat)
# --------------------------------------------------------------------------------------

MATERIALS = {
    "floor_concrete": ("warehouse/floor_concrete.jpg", (1.0, 1.0, 1.0), 4.0),
    "wall_paint": ("painted_plaster_wall_Diffuse.jpg", (1.30, 1.32, 1.36), 2.5),
    "wall_dado": ("painted_plaster_wall_Diffuse.jpg", (0.52, 0.66, 0.84), 2.5),
    "ceiling_deck": (None, (0.78, 0.80, 0.82), 1.0),
    "steel": (None, (0.42, 0.45, 0.50), 1.0),
    "door_metal": (None, (0.74, 0.77, 0.80), 1.0),
    "line_paint": (None, (0.82, 0.85, 0.88), 1.0),
    "cut_cap": (None, (0.012, 0.014, 0.018), 1.0),
    "lamp_glow": (None, (1.0, 0.97, 0.93), 1.0),       # neutral white LEDs: warm light would push cardboard
    "window_glow": (None, (0.78, 0.88, 1.0), 1.0),     # towards the orange the cone detector looks for
}
# Materials the simulator draws in one flat colour, without a lightmap: the things that
# shine by themselves (brighter than white, so that its bloom pass makes them glow), and
# the dark cut where the overview slices the walls open.
FLAT = {"lamp_glow": (2.6, 2.5, 2.35), "window_glow": (1.5, 1.75, 2.1), "cut_cap": (0.008, 0.009, 0.012)}
# The props' pictures are twice the size of the building's lightmaps: they hold colour,
# with labels and tape and wear, and the robot's camera gets close to them.
PROP_PICTURE_FACTOR = 2
EMISSION = {"lamp_glow": 170.0, "window_glow": 10.0}      # W/m2, set so the floor is well exposed


def make_floor_texture(assets):
    """The floor's colour: Poly Haven's worn concrete, made lighter and calmer.

    The photo is of an old, dark, blotchy floor (it averages 34 % grey). A warehouse
    floor is power-trowelled and sealed: light, and even. So the photo's variation is
    kept at a little over half its strength, around a much lighter grey."""
    source = bpy.data.images.load(os.path.join(assets, "concrete_floor_worn_001_Diffuse.jpg"))
    width, height = source.size
    pixels = np.empty(width * height * 4, np.float32)
    source.pixels.foreach_get(pixels)               # 0..1, as stored in the file (sRGB)
    pixels = pixels.reshape(-1, 4)
    mean = pixels[:, :3].mean(axis=0)
    pixels[:, :3] = np.clip(np.array([0.60, 0.60, 0.59]) + (pixels[:, :3] - mean) * 0.55, 0.0, 1.0)
    out = bpy.data.images.new("floor_concrete", width, height, alpha=False)
    out.pixels.foreach_set(pixels.ravel())
    out.filepath_raw = os.path.join(assets, "warehouse", "floor_concrete.jpg")
    out.file_format = 'JPEG'
    out.save(quality=92)
    bpy.data.images.remove(source)
    bpy.data.images.remove(out)


def make_materials(assets):
    made = {}
    for name, (texture, tint, _tile) in MATERIALS.items():
        material = bpy.data.materials.new(name)
        material.use_nodes = True
        tree = material.node_tree
        bsdf = next(n for n in tree.nodes if n.type == 'BSDF_PRINCIPLED')
        bsdf.inputs["Roughness"].default_value = 0.85
        bsdf.inputs["Base Color"].default_value = (*[min(c, 1.0) for c in tint], 1.0)
        if texture and os.path.isfile(os.path.join(assets, texture)):
            image = tree.nodes.new('ShaderNodeTexImage')
            image.image = bpy.data.images.load(os.path.join(assets, texture))
            uv = tree.nodes.new('ShaderNodeUVMap')
            uv.uv_map = "surface"
            tree.links.new(uv.outputs["UV"], image.inputs["Vector"])
            # colour of the photo times the tint (a vector multiply does it for colours too)
            mix = tree.nodes.new('ShaderNodeVectorMath')
            mix.operation = 'MULTIPLY'
            mix.inputs[1].default_value = tint
            tree.links.new(image.outputs["Color"], mix.inputs[0])
            tree.links.new(mix.outputs[0], bsdf.inputs["Base Color"])
        if name in EMISSION:
            bsdf.inputs["Emission Color"].default_value = (*tint, 1.0)
            bsdf.inputs["Emission Strength"].default_value = EMISSION[name]
        made[name] = material
    return made


# --------------------------------------------------------------------------------------
# Geometry: everything is made of rectangles.
# --------------------------------------------------------------------------------------

class Part:
    """One mesh of the building: a list of rectangles, each with a material."""

    def __init__(self, name):
        self.name = name
        self.quads = []         # (four corners counter-clockwise seen from the front, material name)

    def quad(self, a, b, c, d, material):
        self.quads.append(((Vector(a), Vector(b), Vector(c), Vector(d)), material))

    def wall_face(self, axis, position, lo, hi, z0, z1, facing, material):
        """An upright rectangle. axis 'y': it runs along y at x = position, and so on.
        facing is +1 or -1: the direction its front looks along the OTHER axis."""
        if z1 - z0 < 1e-6 or hi - lo < 1e-6:
            return
        # The corners go counter-clockwise as seen from the front. Which end comes first
        # therefore depends on which way the front looks.
        if axis == "y":
            a, b = ((position, lo), (position, hi)) if facing > 0 else ((position, hi), (position, lo))
        else:
            a, b = ((hi, position), (lo, position)) if facing > 0 else ((lo, position), (hi, position))
        self.quad((a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1), (a[0], a[1], z1), material)

    def box(self, x0, x1, y0, y1, z0, z1, material, skip=()):
        """A box seen from outside. skip: faces to leave out, of '+x', '-x', '+y', '-y', '+z', '-z'."""
        if "+x" not in skip:
            self.quad((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1), material)
        if "-x" not in skip:
            self.quad((x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1), material)
        if "+y" not in skip:
            self.quad((x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1), material)
        if "-y" not in skip:
            self.quad((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1), material)
        if "+z" not in skip:
            self.quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1), material)
        if "-z" not in skip:
            self.quad((x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0), material)

    def build(self, materials, lightmap_size):
        """Make the Blender object, with its two UV sets."""
        used = []
        for _corners, material in self.quads:
            if material not in used:
                used.append(material)
        mesh = bpy.data.meshes.new(self.name)
        bm = bmesh.new()
        surface = bm.loops.layers.uv.new("surface")
        lightmap = bm.loops.layers.uv.new("lightmap")
        rects = pack_rectangles([((c[1] - c[0]).length, (c[3] - c[0]).length) for c, _ in self.quads], lightmap_size)
        for (corners, material), (u0, v0, du, dv) in zip(self.quads, rects):
            face = bm.faces.new([bm.verts.new(p) for p in corners])
            face.material_index = used.index(material)
            normal = (corners[1] - corners[0]).cross(corners[3] - corners[0]).normalized()
            tile = MATERIALS[material][2]
            for loop, (lu, lv) in zip(face.loops, ((0, 0), (1, 0), (1, 1), (0, 1))):
                p = loop.vert.co
                # The surface texture is laid on like wallpaper: by the two world axes
                # the rectangle lies in, so it has the same scale everywhere.
                if abs(normal.z) > 0.5:
                    loop[surface].uv = (p.x / tile, p.y / tile)
                elif abs(normal.x) > 0.5:
                    loop[surface].uv = (p.y / tile, p.z / tile)
                else:
                    loop[surface].uv = (p.x / tile, p.z / tile)
                loop[lightmap].uv = (u0 + lu * du, v0 + lv * dv)
        bm.normal_update()
        bm.to_mesh(mesh)
        bm.free()
        for material in used:
            mesh.materials.append(materials[material])
        obj = bpy.data.objects.new(self.name, mesh)
        bpy.context.scene.collection.objects.link(obj)
        return obj


def pack_rectangles(sizes, atlas, padding=3):
    """Give every rectangle (w, h in metres) its own patch of a square atlas, all at the
    same number of pixels per metre, as many as will fit. Returns (u0, v0, du, dv) each.

    Shelf packing: sort by height, fill rows from left to right. Not the tightest packing
    there is, but these are a few hundred rectangles and it is fifteen lines."""
    order = sorted(range(len(sizes)), key=lambda i: -sizes[i][1])

    def place(density):
        x = y = row = 0
        out = {}
        for i in order:
            w = max(2, math.ceil(sizes[i][0] * density)) + 2 * padding
            h = max(2, math.ceil(sizes[i][1] * density)) + 2 * padding
            if w > atlas:
                return None
            if x + w > atlas:
                x, y, row = 0, y + row, 0
            if y + h > atlas:
                return None
            out[i] = (x + padding, y + padding, w - 2 * padding, h - 2 * padding)
            x, row = x + w, max(row, h)
        return out

    low, high = 1.0, 2000.0
    for _ in range(40):
        middle = (low + high) / 2
        if place(middle):
            low = middle
        else:
            high = middle
    placed = place(low)
    print(f"  lightmap: {len(sizes)} rectangles at {low:.0f} pixels per metre")
    return [(placed[i][0] / atlas, placed[i][1] / atlas, placed[i][2] / atlas, placed[i][3] / atlas)
            for i in range(len(sizes))]


def split_wall(low, high, axis, position, lo, hi, facing, top=CEILING, bottom=0.0):
    """One side of a wall from `bottom` to `top`, cut where the paint changes and where
    the overview cuts the building open."""
    for z0, z1, part, material in ((0.0, DADO, low, "wall_dado"), (DADO, CUT, low, "wall_paint"),
                                   (CUT, CEILING, high, "wall_paint")):
        z0, z1 = max(z0, bottom), min(z1, top)
        part.wall_face(axis, position, lo, hi, z0, z1, facing, material)


def build_building(materials, lightmap_size):
    floor = Part("floor")
    low = Part("walls_low")
    high = Part("walls_high")
    roof = Part("ceiling")

    # ---- floor, and the lines painted on it --------------------------------------------
    floor.quad((IN_X0, IN_Y0, 0), (IN_X1, IN_Y0, 0), (IN_X1, IN_Y1, 0), (IN_X0, IN_Y1, 0), "floor_concrete")
    for x0, y0, x1, y1, width in FLOOR_LINES:
        along = Vector((x1 - x0, y1 - y0, 0)).normalized()
        side = Vector((-along.y, along.x, 0)) * (width / 2)
        a = Vector((x0, y0, 0.003)) - along * (width / 2)
        b = Vector((x1, y1, 0.003)) + along * (width / 2)
        floor.quad(a - side, b - side, b + side, a + side, "line_paint")

    # ---- outer walls: only their inner faces. Seen from outside there is nothing, so
    #      the camera of the overview looks straight in. -----------------------------------
    split_wall(low, high, "x", IN_Y0, IN_X0, IN_X1, +1)     # south wall, looks north
    split_wall(low, high, "x", IN_Y1, IN_X0, IN_X1, -1)     # north wall
    split_wall(low, high, "y", IN_X0, IN_Y0, IN_Y1, +1)     # west wall, looks east
    split_wall(low, high, "y", IN_X1, IN_Y0, IN_Y1, -1)     # east wall
    # Where the overview cuts them open, a dark cut 0.2 m wide, like a section drawing.
    out = 2 * HALF_T
    low.box(IN_X0 - out, IN_X1 + out, IN_Y0 - out, IN_Y0, CUT - 0.01, CUT, "cut_cap", skip=("-z", "+x", "-x", "+y", "-y"))
    low.box(IN_X0 - out, IN_X1 + out, IN_Y1, IN_Y1 + out, CUT - 0.01, CUT, "cut_cap", skip=("-z", "+x", "-x", "+y", "-y"))
    low.box(IN_X0 - out, IN_X0, IN_Y0, IN_Y1, CUT - 0.01, CUT, "cut_cap", skip=("-z", "+x", "-x", "+y", "-y"))
    low.box(IN_X1, IN_X1 + out, IN_Y0, IN_Y1, CUT - 0.01, CUT, "cut_cap", skip=("-z", "+x", "-x", "+y", "-y"))

    # ---- inner walls: both faces, the sides of the doorways, and the wall above them ---
    for axis, position, stretches in INNER_WALLS:
        for lo, hi in stretches:
            split_wall(low, high, axis, position - HALF_T, lo, hi, -1)
            split_wall(low, high, axis, position + HALF_T, lo, hi, +1)
            # Where the wall was cut open, the overview shows a dark cut, like a section drawing.
            if axis == "y":
                low.quad((position - HALF_T, lo, CUT), (position + HALF_T, lo, CUT),
                         (position + HALF_T, hi, CUT), (position - HALF_T, hi, CUT), "cut_cap")
            else:
                low.quad((lo, position - HALF_T, CUT), (hi, position - HALF_T, CUT),
                         (hi, position + HALF_T, CUT), (lo, position + HALF_T, CUT), "cut_cap")
        for (_, end), (start, _) in zip(stretches, stretches[1:]):
            door = DOOR_HEIGHT.get((axis, position, end), DEFAULT_DOOR_HEIGHT)
            other = "x" if axis == "y" else "y"
            # the two sides of the doorway (jambs) ...
            split_wall(low, high, other, end, position - HALF_T, position + HALF_T, +1, top=door)
            split_wall(low, high, other, start, position - HALF_T, position + HALF_T, -1, top=door)
            # ... the wall above it, and its underside
            split_wall(low, high, axis, position - HALF_T, end, start, -1, bottom=door)
            split_wall(low, high, axis, position + HALF_T, end, start, +1, bottom=door)
            part = high if door >= CUT else low
            if axis == "y":
                part.quad((position - HALF_T, start, door), (position + HALF_T, start, door),
                          (position + HALF_T, end, door), (position - HALF_T, end, door), "wall_paint")
            else:
                part.quad((end, position - HALF_T, door), (end, position + HALF_T, door),
                          (start, position + HALF_T, door), (start, position - HALF_T, door), "wall_paint")

    # ---- doors in the west wall: a sectional dock door and a personnel door ---------------
    def door_panel(part_low, part_high, y0, y1, height, sections):
        x = IN_X0
        frame = 0.08
        for z0, z1 in ((0, min(height, CUT)), (CUT, height)):
            if z1 <= z0:
                continue
            part = part_low if z1 <= CUT else part_high
            part.box(x, x + 0.05, y0 - frame, y0, z0, z1, "steel", skip=("-x",))
            part.box(x, x + 0.05, y1, y1 + frame, z0, z1, "steel", skip=("-x",))
        (part_low if height + frame <= CUT else part_high).box(
            x, x + 0.05, y0 - frame, y1 + frame, height, height + frame, "steel", skip=("-x",))
        step = height / sections
        for i in range(sections):
            z0, z1 = i * step + 0.006, (i + 1) * step - 0.006
            for a, b in ((z0, min(z1, CUT)), (max(z0, CUT), z1)):
                if b - a > 1e-3:
                    (part_low if b <= CUT else part_high).box(x, x + 0.03, y0, y1, a, b, "door_metal", skip=("-x",))

    door_panel(low, high, 1.6, 4.6, 3.0, 6)         # the dock door: six panels
    door_panel(low, high, -0.45, 0.45, 2.05, 1)     # the door under the EXIT sign

    # ---- roof: deck, beams, purlins ----------------------------------------------------------
    roof.quad((IN_X0, IN_Y1, CEILING), (IN_X1, IN_Y1, CEILING), (IN_X1, IN_Y0, CEILING), (IN_X0, IN_Y0, CEILING),
              "ceiling_deck")
    for x in BEAMS_X:
        roof.box(x - 0.09, x + 0.09, IN_Y0, IN_Y1, CEILING - 0.38, CEILING, "steel", skip=("+z", "+y", "-y"))
    for y in (-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0):
        roof.box(IN_X0, IN_X1, y - 0.04, y + 0.04, CEILING - 0.14, CEILING, "steel", skip=("+z", "+x", "-x"))

    # ---- lamps: a housing on two rods, and the strip that shines ------------------------------
    for x, y, along_x in LAMPS:
        half_l, half_w = (0.75, 0.07) if along_x else (0.07, 0.75)
        roof.box(x - half_l, x + half_l, y - half_w, y + half_w, LAMP_Z, LAMP_Z + 0.06, "steel", skip=("-z",))
        roof.quad((x - half_l, y + half_w, LAMP_Z), (x + half_l, y + half_w, LAMP_Z),
                  (x + half_l, y - half_w, LAMP_Z), (x - half_l, y - half_w, LAMP_Z), "lamp_glow")
        for s in (-0.55, 0.55):
            rx, ry = (x + s, y) if along_x else (x, y + s)
            roof.box(rx - 0.006, rx + 0.006, ry - 0.006, ry + 0.006, LAMP_Z + 0.06, CEILING - 0.14, "steel",
                     skip=("+z", "-z"))

    # ---- high windows in the north wall: glass that glows with daylight, and its frame -----
    for x0, x1 in WINDOWS:
        z0, z1 = WINDOW_Z
        y = IN_Y1 - 0.012
        high.wall_face("x", y, x0, x1, z0, z1, -1, "window_glow")
        for a, b, c, d in ((x0 - 0.05, x1 + 0.05, z0 - 0.05, z0), (x0 - 0.05, x1 + 0.05, z1, z1 + 0.05),
                           (x0 - 0.05, x0, z0, z1), (x1, x1 + 0.05, z0, z1)):
            high.box(a, b, IN_Y1 - 0.04, IN_Y1, c, d, "steel", skip=("+y",))
        for i in range(1, 4):
            m = x0 + (x1 - x0) * i / 4
            high.box(m - 0.015, m + 0.015, IN_Y1 - 0.03, IN_Y1, z0, z1, "steel", skip=("+y",))

    return [part.build(materials, lightmap_size) for part in (floor, low, high, roof)]


# --------------------------------------------------------------------------------------
# What stands in the building: imported only so that it casts its shadow in the bake.
# --------------------------------------------------------------------------------------

def dress(material):
    """Give a prop's material the unevenness of a real surface.

    The models come with one flat colour per material (or a small photo). Nothing real
    is one flat colour. So before the bake every material gets: blotches a hand wide,
    a fine speckle, dust that darkens it towards the floor, and, on painted steel,
    paint worn off along the edges. All of it is computed from the place in the room,
    so two cartons of the same cardboard do not look alike.

    None of this reaches the simulator as a material. It is baked into the props'
    picture, together with the light."""
    tree = material.node_tree
    bsdf = next((n for n in tree.nodes if n.type == 'BSDF_PRINCIPLED'), None)
    if bsdf is None or tree.nodes.get("dressed"):
        return
    kind = material.name.split(".")[0]
    steel = kind.startswith(("upright", "beam", "base_plate", "deck", "crate_corner")) or "steel" in kind or "metal" in kind
    cardboard = kind.startswith(("cardboard", "plywood", "wood"))

    # Only the diffuse light is baked, and a metal has none of its own: treat paint and
    # zinc as what they look like from a few metres, a coloured matt surface.
    for name in ("Metallic",):
        socket = bsdf.inputs[name]
        for link in list(socket.links):
            tree.links.remove(link)
        socket.default_value = 0.0

    base = bsdf.inputs["Base Color"]
    if base.is_linked:
        colour = base.links[0].from_socket
        tree.links.remove(base.links[0])
    else:
        rgb = tree.nodes.new('ShaderNodeRGB')
        rgb.outputs[0].default_value = base.default_value[:]
        colour = rgb.outputs[0]

    where = tree.nodes.new('ShaderNodeNewGeometry')
    where.name = "dressed"

    def noise(scale, detail, low, high):
        """A number per place in the room, between low and high."""
        texture = tree.nodes.new('ShaderNodeTexNoise')
        texture.inputs["Scale"].default_value = scale
        texture.inputs["Detail"].default_value = detail
        tree.links.new(where.outputs["Position"], texture.inputs["Vector"])
        spread = tree.nodes.new('ShaderNodeMapRange')
        spread.inputs[1].default_value, spread.inputs[2].default_value = 0.3, 0.7
        spread.inputs[3].default_value, spread.inputs[4].default_value = low, high
        tree.links.new(texture.outputs[0], spread.inputs[0])
        return spread.outputs[0]

    def times(colour_socket, factor_socket):
        scale = tree.nodes.new('ShaderNodeVectorMath')
        scale.operation = 'SCALE'
        tree.links.new(colour_socket, scale.inputs[0])
        tree.links.new(factor_socket, scale.inputs[3])
        return scale.outputs[0]

    colour = times(colour, noise(2.5, 3.0, 0.86 if cardboard else 0.91, 1.07))    # blotches
    colour = times(colour, noise(70.0, 2.0, 0.95, 1.05))                          # speckle
    # dust: darker over the lowest 30 cm
    height = tree.nodes.new('ShaderNodeSeparateXYZ')
    tree.links.new(where.outputs["Position"], height.inputs[0])
    dust = tree.nodes.new('ShaderNodeMapRange')
    dust.inputs[1].default_value, dust.inputs[2].default_value = 0.0, 0.30
    dust.inputs[3].default_value, dust.inputs[4].default_value = 0.80, 1.0
    tree.links.new(height.outputs[2], dust.inputs[0])
    colour = times(colour, dust.outputs[0])

    if steel:
        # Paint wears off where things knock against it: along the outer edges. An
        # ambient occlusion node, asked to look INSIDE the surface, is dark exactly
        # there. Patches of noise decide where on the edge the paint is gone.
        edge = tree.nodes.new('ShaderNodeAmbientOcclusion')
        edge.inside = True
        edge.only_local = True
        edge.samples = 8
        edge.inputs["Distance"].default_value = 0.012
        worn = tree.nodes.new('ShaderNodeMapRange')         # 1 on a sharp edge, 0 on a flat face
        worn.inputs[1].default_value, worn.inputs[2].default_value = 0.92, 0.55
        worn.inputs[3].default_value, worn.inputs[4].default_value = 0.0, 1.0
        tree.links.new(edge.outputs["AO"], worn.inputs[0])
        patchy = tree.nodes.new('ShaderNodeMath')
        patchy.operation = 'MULTIPLY'
        tree.links.new(worn.outputs[0], patchy.inputs[0])
        tree.links.new(noise(18.0, 3.0, 0.0, 1.0), patchy.inputs[1])
        mix = tree.nodes.new('ShaderNodeMix')
        mix.data_type = 'RGBA'
        tree.links.new(patchy.outputs[0], mix.inputs[0])
        tree.links.new(colour, mix.inputs[6])
        mix.inputs[7].default_value = (0.42, 0.43, 0.45, 1.0)       # bare steel
        colour = mix.outputs[2]

    tree.links.new(colour, base)


def unwrap(obj, picture_size):
    """Give every spot of a prop mesh a place of its own in a picture: the texture
    coordinates called "lightmap". Unlike the building these meshes are not made of
    rectangles, so Blender cuts them into flat pieces and lays those out (Smart UV
    Project).

    The pieces are kept three pixels apart and no more. These meshes have thousands of
    pieces (every rounded edge is one), and the room between them is room lost: at
    twelve pixels apart, nine tenths of the picture were empty."""
    mesh = obj.data
    if "lightmap" in mesh.uv_layers:
        mesh.uv_layers.remove(mesh.uv_layers["lightmap"])
    if not mesh.uv_layers:
        mesh.uv_layers.new(name="UVMap")
    mesh.uv_layers.active = mesh.uv_layers.new(name="lightmap")
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.smart_project(angle_limit=math.radians(66), island_margin=3.2 / picture_size, area_weight=0.0)
    bpy.ops.object.mode_set(mode='OBJECT')
    mesh.uv_layers.active = mesh.uv_layers[0]


def cull_hidden(props, picture_size):
    """Throw away the faces of the props that no light reaches.

    A carton on a shelf has six faces. Its underside lies on the deck and one or two
    sides lie against its neighbours: nobody will ever see them, yet each would get its
    share of the props' picture. About half of all faces are like that. A quick, rough
    bake of the light finds them (where no light arrives, no eye looks either). Without
    them the faces that ARE seen get twice the room in the picture, and the file the
    simulator loads is smaller."""
    size = 1024
    for obj in props:
        light = bake_pass(obj, size, {'DIRECT', 'INDIRECT'}, 16, margin=0).max(axis=2)
        mesh = obj.data
        uv = np.empty(len(mesh.loops) * 2, np.float32)
        mesh.uv_layers["lightmap"].data.foreach_get("uv", uv)
        uv = uv.reshape(-1, 2)
        start = np.empty(len(mesh.polygons), np.int32)
        count = np.empty(len(mesh.polygons), np.int32)
        mesh.polygons.foreach_get("loop_start", start)
        mesh.polygons.foreach_get("loop_total", count)
        face_of = np.repeat(np.arange(len(start)), count)
        centre = np.add.reduceat(uv, start, axis=0) / count[:, None]

        def lit(points):
            x = np.clip((points[:, 0] * size).astype(np.int64), 0, size - 1)
            y = np.clip((points[:, 1] * size).astype(np.int64), 0, size - 1)
            return light[y, x]

        # The brightest of: the middle of the face, and a point halfway to each corner.
        seen = np.maximum(lit(centre), np.maximum.reduceat(lit(0.5 * (uv + centre[face_of])), start))
        # A face smaller than a few pixels of this rough bake may have missed every
        # pixel centre and looks dark for that reason alone. Those are kept: they are
        # the narrow bevels along edges, and take next to no room anyway.
        nxt = np.roll(uv, -1, axis=0)
        last = start + count - 1
        nxt[last] = uv[start]
        area = 0.5 * np.abs(np.add.reduceat(uv[:, 0] * nxt[:, 1] - nxt[:, 0] * uv[:, 1], start)) * size * size
        hidden = (seen < 0.003) & (area >= 6.0)

        bm = bmesh.new()
        bm.from_mesh(mesh)
        bm.faces.ensure_lookup_table()
        bmesh.ops.delete(bm, geom=[bm.faces[i] for i in np.nonzero(hidden)[0]], context='FACES')
        bm.to_mesh(mesh)
        bm.free()
        mesh.update()
        print(f"  {obj.name}: {int(hidden.sum())} of {len(hidden)} faces are never lit, {len(mesh.polygons)} left")
        unwrap(obj, picture_size)


def place_props(assets, picture_size):
    """Stand every prop where world.cpp has its box, and merge them into one mesh per room.
    Returns those meshes. (The cones are placed too, but only to cast their shadows: the
    simulator draws them itself, because the robot's camera has to find them.)"""
    models = os.path.join(assets, "models")
    rooms = {"props_dock": [], "props_hall_north": [], "props_hall_south": [], "props_offices": []}
    for x, y, sx, sy, height, model, turns, stretch in PROPS:
        path = os.path.join(models, model + ".glb")
        if not os.path.isfile(path):
            continue
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=path)
        new = [o for o in bpy.data.objects if o not in before]
        bpy.context.view_layer.update()
        corners = [o.matrix_world @ Vector(c) for o in new if o.type == 'MESH' for c in o.bound_box]
        lo = Vector((min(c.x for c in corners), min(c.y for c in corners), min(c.z for c in corners)))
        hi = Vector((max(c.x for c in corners), max(c.y for c in corners), max(c.z for c in corners)))
        size, centre = hi - lo, (hi + lo) / 2
        # The same fitting as the simulator's fitModelToBox().
        sideways = turns % 2 != 0
        scale_x = (sy if sideways else sx) / size.x
        scale_y = (sx if sideways else sy) / size.y
        scale_z = height / size.z if stretch else (scale_x + scale_y) / 2
        fit = bpy.data.objects.new(model + "_fit", None)
        holder = bpy.data.objects.new(model + "_at", None)
        for o in (fit, holder):
            bpy.context.scene.collection.objects.link(o)
        fit.parent = holder
        fit.scale = (scale_x, scale_y, scale_z)
        fit.location = (-centre.x * scale_x, -centre.y * scale_y, -lo.z * scale_z)
        holder.location = (x, y, 0)
        holder.rotation_euler = (0, 0, turns * math.pi / 2)
        for o in new:
            if o.parent is None:
                o.parent = fit
        room = "props_dock" if x < 5 else "props_offices" if x > 17 else "props_hall_north" if y > 0 else "props_hall_south"
        rooms[room] += [o for o in new if o.type == 'MESH']

    merged = []
    for name, meshes in rooms.items():
        if not meshes:
            continue
        bpy.context.view_layer.update()
        bpy.ops.object.select_all(action='DESELECT')
        for o in meshes:
            o.select_set(True)
        bpy.context.view_layer.objects.active = meshes[0]
        bpy.ops.object.join()
        obj = bpy.context.view_layer.objects.active
        # Out of the helper objects it hung on, with its place in the world baked into
        # its vertices: the simulator then loads it without having to move anything.
        world = obj.matrix_world.copy()
        obj.parent = None
        obj.matrix_world = world
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        obj.name = obj.data.name = name
        unwrap(obj, picture_size)
        print(f"  {name}: {len(meshes)} meshes merged, {len(obj.data.polygons)} faces")
        merged.append(obj)

    cone_material = bpy.data.materials.new("cone")
    cone_material.use_nodes = True
    next(n for n in cone_material.node_tree.nodes if n.type == 'BSDF_PRINCIPLED').inputs["Base Color"].default_value = (
        1.0, 0.15, 0.0, 1.0)
    for x, y in CONES:
        bpy.ops.mesh.primitive_cone_add(vertices=32, radius1=0.20, radius2=0.0, depth=0.70, location=(x, y, 0.35))
        bpy.context.object.data.materials.append(cone_material)
    return merged


# --------------------------------------------------------------------------------------
# Baking
# --------------------------------------------------------------------------------------

def setup_cycles(samples):
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    preferences = bpy.context.preferences.addons['cycles'].preferences
    for kind in ('OPTIX', 'CUDA', 'HIP', 'ONEAPI', 'METAL'):
        try:
            preferences.compute_device_type = kind
        except TypeError:
            continue
        preferences.get_devices()
        devices = [d for d in preferences.devices if d.type == kind]
        if devices:
            for d in preferences.devices:
                d.use = d.type == kind
            scene.cycles.device = 'GPU'
            print(f"Cycles on the GPU ({kind}: {devices[0].name})")
            break
    else:
        print("Cycles on the CPU")
    scene.cycles.samples = samples
    scene.cycles.max_bounces = 8
    scene.cycles.diffuse_bounces = 5
    scene.cycles.use_denoising = True
    scene.cycles.sample_clamp_indirect = 10.0
    # No light from the sky: a building is lit by its lamps and windows.
    world = bpy.data.worlds.new("dark")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.0
    scene.world = world


def srgb(linear):
    return np.where(linear <= 0.0031308, linear * 12.92, 1.055 * np.power(np.maximum(linear, 1e-9), 1 / 2.4) - 0.055)


def bake_target(material):
    """The picture node Cycles bakes into: it must be the ACTIVE node of the material."""
    tree = material.node_tree
    target = tree.nodes.get("bake_target")
    if target is None:
        target = tree.nodes.new('ShaderNodeTexImage')
        target.name = "bake_target"
        lightmap_uv = tree.nodes.new('ShaderNodeUVMap')
        lightmap_uv.uv_map = "lightmap"
        tree.links.new(lightmap_uv.outputs["UV"], target.inputs["Vector"])
    tree.nodes.active = target
    return target


def bake_pass(obj, size, passes, samples, margin):
    """Bake one object into a picture and return it: an array (size, size, 3) of linear
    values, first row at the bottom.

    passes: DIRECT and INDIRECT is "how much light arrives here", without the surface's
    own colour. COLOR is that colour alone, without any light."""
    bpy.context.scene.cycles.samples = samples
    image = bpy.data.images.new("bake", size, size, alpha=False, float_buffer=True)
    for material in obj.data.materials:
        bake_target(material).image = image
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    obj.data.uv_layers.active = obj.data.uv_layers["lightmap"]
    bpy.ops.object.bake(type='DIFFUSE', pass_filter=passes, margin=margin, use_clear=True)
    obj.data.uv_layers.active = obj.data.uv_layers[0]
    pixels = np.empty(size * size * 4, np.float32)
    image.pixels.foreach_get(pixels)
    bpy.data.images.remove(image)
    return pixels.reshape(size, size, 4)[:, :, :3].copy()


def save_picture(light, path, quality):
    """Store linear light as an 8-bit JPG: divided by LIGHTMAP_SCALE, then sRGB-encoded."""
    size = light.shape[0]
    encoded = np.ones((size, size, 4), np.float32)
    encoded[:, :, :3] = srgb(np.clip(light / LIGHTMAP_SCALE, 0.0, 1.0))
    picture = bpy.data.images.new("out", size, size, alpha=False)
    picture.pixels.foreach_set(encoded.ravel())
    picture.filepath_raw = path
    picture.file_format = 'JPEG'
    picture.save(quality=quality)
    bpy.data.images.remove(picture)


def report(name, light):
    lit = light[light.sum(axis=2) > 0]
    print(f"  {name}: mean {lit.mean():.3f}, 99th percentile {np.percentile(lit, 99):.3f}, max {lit.max():.2f}")


def bake_building(objects, out_dir, size, samples):
    """One lightmap per mesh of the building: lm_<name>.jpg."""
    for obj in objects:
        light = bake_pass(obj, size, {'DIRECT', 'INDIRECT'}, samples, margin=6)
        report(obj.name, light)
        save_picture(light, os.path.join(out_dir, f"lm_{obj.name}.jpg"), 93)


def box_blur(a, radius):
    """The mean over a square of 2 * radius + 1 pixels, with nothing beyond the border."""
    for axis in (0, 1):
        total = np.cumsum(a, axis=axis, dtype=np.float64)
        total = np.concatenate([np.zeros_like(np.take(total, [0], axis=axis)), total], axis=axis)
        n = a.shape[axis]
        high = np.clip(np.arange(n) + radius + 1, 0, n)
        low = np.clip(np.arange(n) - radius, 0, n)
        a = ((np.take(total, high, axis=axis) - np.take(total, low, axis=axis)) / (2 * radius + 1)).astype(np.float32)
    return a


def smooth_within(light, covered):
    """Take the path tracer's grain out of a lightmap.

    Light changes slowly over a surface, so a small blur costs little. But the picture
    is a patchwork: next to the patch of one surface lies the patch of another, or
    nothing. So only covered pixels are averaged, and the blur reaches two pixels:
    less than the three the patches are apart (see unwrap)."""
    weight = covered.astype(np.float32)
    out = np.empty_like(light)
    for channel in range(3):
        value = light[:, :, channel] * weight
        blurred, share = value, weight
        for _ in range(2):
            blurred, share = box_blur(blurred, 1), box_blur(share, 1)
        out[:, :, channel] = np.where(covered, blurred / np.maximum(share, 1e-6), 0.0)
    return out


def grow(picture, covered, pixels):
    """Spread every patch a few pixels beyond its edge. When the simulator draws a
    surface small and far away it reads a blend of neighbouring pixels: without this
    the black between the patches would bleed into them as dark seams."""
    picture, covered = picture.copy(), covered.copy()
    for _ in range(pixels):
        total = np.zeros_like(picture)
        count = np.zeros(covered.shape, np.float32)
        for shift in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            total += np.roll(picture * covered[:, :, None], shift, axis=(0, 1))
            count += np.roll(covered, shift, axis=(0, 1))
        new = ~covered & (count > 0)
        picture[new] = total[new] / count[new][:, None]
        covered |= new
    return picture


def bake_prop_colours(props, size):
    """Bake every prop's own colour, with the wear dress() gave it, into a picture. Then
    replace its forty materials by one that just shows that picture.

    That is not only tidying up. When light is baked next, every ray that bounces off a
    prop asks for the prop's colour, and the dressed materials are slow to ask (the edge
    wear alone costs eight rays). The picture answers at once, with the same colour."""
    colours = {}
    for obj in props:
        colour = bake_pass(obj, size, {'COLOR'}, 16, margin=0)
        colours[obj.name] = colour
        shown = bpy.data.images.new(obj.name + "_colour", size, size, alpha=False, float_buffer=True)
        rgba = np.ones((size, size, 4), np.float32)
        rgba[:, :, :3] = grow(colour, colour.sum(axis=2) > 1e-6, 4)
        shown.pixels.foreach_set(rgba.ravel())
        plain = bpy.data.materials.new(obj.name + "_colour")
        plain.use_nodes = True
        tree = plain.node_tree
        bsdf = next(n for n in tree.nodes if n.type == 'BSDF_PRINCIPLED')
        bsdf.inputs["Roughness"].default_value = 0.8
        picture = tree.nodes.new('ShaderNodeTexImage')
        picture.image = shown
        where = tree.nodes.new('ShaderNodeUVMap')
        where.uv_map = "lightmap"
        tree.links.new(where.outputs["UV"], picture.inputs["Vector"])
        tree.links.new(picture.outputs["Color"], bsdf.inputs["Base Color"])
        obj.data.materials.clear()
        obj.data.materials.append(plain)
        for polygon in obj.data.polygons:
            polygon.material_index = 0
        print(f"  {obj.name}: colour baked, {100 * (colour.sum(axis=2) > 1e-6).mean():.0f} % of the picture is used")
    return colours


def bake_props(props, colours, out_dir, size, samples):
    """One picture per mesh of the props, baked_<name>.jpg: its colour times its light.
    (The building's meshes have a colour texture that repeats, so the simulator
    multiplies. A prop's colour is different in every spot, so it is multiplied here.)"""
    for obj in props:
        colour = colours[obj.name]
        covered = colour.sum(axis=2) > 1e-6
        light = smooth_within(bake_pass(obj, size, {'DIRECT', 'INDIRECT'}, samples, margin=0), covered)
        report(obj.name, light)
        save_picture(grow(colour * light, covered, 8), os.path.join(out_dir, f"baked_{obj.name}.jpg"), 88)


def strip_for_export(props):
    """After the bake a prop mesh needs one thing only: where each spot of it is in its
    baked picture. Its other texture coordinates go, and it gets one empty material.
    That makes one draw call per room, and a much smaller file."""
    plain = bpy.data.materials.new("baked")
    for obj in props:
        mesh = obj.data
        for layer in [l for l in mesh.uv_layers if l.name != "lightmap"]:
            mesh.uv_layers.remove(layer)
        mesh.materials.clear()
        mesh.materials.append(plain)
        for polygon in mesh.polygons:
            polygon.material_index = 0


def write_materials(path):
    """The building's file carries no pictures and no tints (glTF cannot say "2.5 times this
    photo"). This list does. One material per line: how it is drawn (lit = colour times
    lightmap, flat = one colour), its name, its colour texture or "-", and its tint.

    After the materials: where each prop that was baked stands ("baked x y"). A box of
    world.cpp that is not in this list is new since the last bake, and the simulator
    draws it the plain way until this script has been run again."""
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"lightmap_scale {LIGHTMAP_SCALE}\n")
        for name, (texture, tint, _tile) in MATERIALS.items():
            kind, colour = ("flat", FLAT[name]) if name in FLAT else ("lit", tint)
            f.write(f"{kind} {name} {texture or '-'} {colour[0]} {colour[1]} {colour[2]}\n")
        for prop in PROPS:
            f.write(f"baked {prop[0]} {prop[1]}\n")


def export(objects, path):
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    # Without pictures: the simulator has the building's colour textures already, and
    # finds the baked pictures by the name of the mesh. Materials keep their names.
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=True, export_yup=True,
                              export_apply=True, export_texcoords=True, export_normals=True,
                              export_materials='EXPORT', export_image_format='NONE')


def preview(path):
    """One still from behind the robot's start, looking into the building."""
    scene = bpy.context.scene
    camera = bpy.data.objects.new("camera", bpy.data.cameras.new("camera"))
    scene.collection.objects.link(camera)
    camera.location = (-1.5, -0.9, 1.3)
    camera.data.lens = 20
    target = Vector((6.0, 0.6, 0.9))
    camera.rotation_euler = (target - camera.location).to_track_quat('-Z', 'Y').to_euler()
    scene.camera = camera
    scene.render.resolution_x, scene.render.resolution_y = 1280, 720
    scene.cycles.samples = 96
    scene.view_settings.view_transform = 'Standard'
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    assets = os.path.abspath(args[0])
    option = lambda name, default: type(default)(args[args.index(name) + 1]) if name in args else default
    samples, size = option("--samples", 256), option("--size", 2048)
    out_dir = os.path.join(assets, "warehouse")
    os.makedirs(out_dir, exist_ok=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    setup_cycles(samples)
    make_floor_texture(assets)
    materials = make_materials(assets)
    print("Building")
    objects = build_building(materials, size)
    prop_size = size * PROP_PICTURE_FACTOR
    props = place_props(assets, prop_size)
    cull_hidden(props, prop_size)
    for obj in props:
        for material in obj.data.materials:
            dress(material)
    if "--preview" in args:
        preview(os.path.abspath(args[args.index("--preview") + 1]))
    if "--no-bake" not in args:
        print("Baking")
        colours = bake_prop_colours(props, prop_size)
        if "--colour-only" in args:
            return
        bake_building(objects, out_dir, size, samples)
        bake_props(props, colours, out_dir, prop_size, samples)
    strip_for_export(props)
    export(objects, os.path.join(out_dir, "building.glb"))
    export(props, os.path.join(out_dir, "props.glb"))
    write_materials(os.path.join(out_dir, "materials.txt"))
    print("Done:", out_dir)


main()
