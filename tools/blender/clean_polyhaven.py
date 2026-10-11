"""Step 2 of 2 for the Poly Haven props: bring them to the demo's conventions and export .glb files.

Run step 1 first (fetch_polyhaven.py, it downloads the props and prepares their textures), then:

    "C:\\Program Files\\Blender Foundation\\Blender 5.0\\blender.exe" --background --python tools/blender/clean_polyhaven.py -- <work folder> src/explorer_sim/assets/models

For each prop this script
  * joins its meshes into one object named after the output file
  * turns it so that the front of the Poly Haven model (Blender -Y) looks along +X (ROS forward)
  * moves the origin to the centre of the footprint, on the floor
  * thins the mesh if it has more than 30 000 triangles
  * recolours a prop whose material colour is orange (the tote)
  * exports a self-contained .glb (+Y up, textures embedded)
"""
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

# model file name -> Poly Haven asset id; keep in step with fetch_polyhaven.py
ASSETS = {
    "desk": "metal_office_desk",
    "barrel": "Barrel_02",
    "tote": "plastic_crate_02",
    "cart": "industrial_storage_cart",
    "ladder": "ladder_sectioned_01",
    "stool": "metal_stool_02",
}
MAX_TRIANGLES = 30000
# plastic_crate_02 has a grey texture and gets its colour from the material: orange as it comes,
# which is exactly what the cone detector looks for. It becomes blue here (linear RGB).
TINT = {"tote": (0.030, 0.130, 0.420)}


def set_tint(ob, rgb):
    """Replaces the constant colour that the glTF importer multiplies onto the colour texture."""
    for m in ob.data.materials:
        for n in m.node_tree.nodes:
            if n.type in ('MIX', 'MIX_RGB'):
                for inp in n.inputs:
                    if inp.type == 'RGBA' and not inp.is_linked:
                        inp.default_value = (rgb[0], rgb[1], rgb[2], 1.0)


def clean(name, asset, work, out_dir):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    folder = os.path.join(work, "_clean", asset)
    gltf = next(f for f in os.listdir(folder) if f.endswith(".gltf"))
    bpy.ops.import_scene.gltf(filepath=os.path.join(folder, gltf))

    meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    turn = Matrix.Rotation(math.pi / 2, 4, 'Z')
    for o in meshes:  # bake every transform into the mesh, so the objects can be joined at the origin
        world = turn @ o.matrix_world
        o.parent = None
        o.data = o.data.copy()
        o.data.transform(world)
        o.matrix_world = Matrix.Identity(4)
    bpy.ops.object.select_all(action='DESELECT')
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if len(meshes) > 1:
        bpy.ops.object.join()
    ob = bpy.context.view_layer.objects.active
    for o in list(bpy.context.scene.objects):
        if o != ob:
            bpy.data.objects.remove(o)
    ob.name = name
    ob.data.name = name
    if name in TINT:
        set_tint(ob, TINT[name])

    xs = [v.co.x for v in ob.data.vertices]
    ys = [v.co.y for v in ob.data.vertices]
    zs = [v.co.z for v in ob.data.vertices]
    ob.data.transform(Matrix.Translation(Vector((-(min(xs) + max(xs)) / 2, -(min(ys) + max(ys)) / 2, -min(zs)))))

    tris = sum(len(p.vertices) - 2 for p in ob.data.polygons)
    if tris > MAX_TRIANGLES:
        dec = ob.modifiers.new("decimate", 'DECIMATE')
        dec.ratio = 0.95 * MAX_TRIANGLES / tris
    path = os.path.join(out_dir, name + ".glb")
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', export_yup=True, export_apply=True,
                              export_cameras=False, export_lights=False, export_animations=False,
                              use_selection=False)
    print(f"[clean_polyhaven] {name}: {asset}, {tris} triangles in, "
          f"size {max(xs) - min(xs):.3f} x {max(ys) - min(ys):.3f} x {max(zs) - min(zs):.3f} m, {os.path.getsize(path)} bytes")


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if len(args) < 2:
        raise SystemExit("usage: blender --background --python clean_polyhaven.py -- <work folder> <output folder> [names]")
    work, out_dir = os.path.abspath(args[0]), os.path.abspath(args[1])
    os.makedirs(out_dir, exist_ok=True)
    for name in (args[2:] or list(ASSETS)):
        clean(name, ASSETS[name], work, out_dir)


main()
