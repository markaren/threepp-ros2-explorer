# Assets

Almost everything in this folder comes from [Poly Haven](https://polyhaven.com) and is licensed
[CC0](https://polyhaven.com/license): free to use, change and redistribute, no attribution
required. Credit is given anyway.

| Files | Poly Haven asset | Used for |
|---|---|---|
| `concrete_floor_worn_001_*.jpg` | [concrete_floor_worn_001](https://polyhaven.com/a/concrete_floor_worn_001) | the floor (its colour map is the source of `warehouse/floor_concrete.jpg`) |
| `painted_plaster_wall_*.jpg` | [painted_plaster_wall](https://polyhaven.com/a/painted_plaster_wall) | the walls |
| `blue_metal_plate_*.jpg` | [blue_metal_plate](https://polyhaven.com/a/blue_metal_plate) | the boxes of the plain model |
| `weathered_planks_*.jpg` | [weathered_planks](https://polyhaven.com/a/weathered_planks) | the crates of the plain model |
| `empty_warehouse_01_1k.hdr` | [empty_warehouse_01](https://polyhaven.com/a/empty_warehouse_01) | lights the robot, the cones and dropped crates |

One file is not from Poly Haven: `cone.png`, the texture of the traffic cones (orange
plastic, a white reflective band, scuffs). It is painted by
[`tools/make_cone_texture.py`](../../../tools/make_cone_texture.py), which also explains
why it looks the way it does.

The Poly Haven textures are the 1k versions. Each texture set has three maps:

* `_Diffuse`: the colour of the surface
* `_nor_gl`: the normal map, fine relief that catches the light (OpenGL convention)
* `_Rough`: the roughness map, where the surface is matt and where it shines

One file was changed: `weathered_planks_grey.jpg` is the planks' colour map converted to
greyscale and brightened. It is only used by the plain model (see below), whose crates
get a teal tint.

## The building: `warehouse/`

What the simulator shows is not put together from the files above at run time. It is
made beforehand by [`tools/blender/make_warehouse.py`](../../../tools/blender/make_warehouse.py),
in Blender, and stored here:

| File | What it is |
|---|---|
| `building.glb` | floor, walls, ceiling, beams, doors, lamps. Geometry and material names only, 0.1 MB |
| `lm_floor.jpg`, `lm_walls_low.jpg`, `lm_walls_high.jpg`, `lm_ceiling.jpg` | one lightmap per mesh of the building: how much light falls on every spot of it |
| `floor_concrete.jpg` | the floor's colour: the Poly Haven concrete, made lighter and calmer |
| `props.glb` | everything that stands in the building and never moves, at its place: one mesh per room, 11 MB |
| `baked_props_*.jpg` | one picture per mesh of the props: their colour times the light on it |
| `materials.txt` | what the simulator needs to know that the files cannot say: tints, which material shines, and which boxes of `world.cpp` are in the bake |

```
blender --background --factory-startup --python tools/blender/make_warehouse.py -- src/explorer_sim/assets
```

run from the project root, makes all of it again: about five and a half minutes with a
desktop graphics card, Blender 5. Add `--size 1024 --samples 64` for a rough version in
under a minute, and `--preview picture.png` for a path-traced still from behind the
robot's start. The script's first lines say what it does and why.

The lightmaps are stored at a quarter of their brightness (`lightmap_scale` in
`materials.txt`), so that the light right under a lamp, several times brighter than
"white", fits into an 8-bit JPG.

Two things the script must agree on with [`world.cpp`](../src/world.cpp), and repeats
from it: where the walls and doorways are, and where the props stand. Change one, change
the other, and bake again.

If this folder is missing, the simulator builds a **plain model** from the boxes of
`world.cpp`: walls 1.4 m high, textured from the files at the top of this page, lit by
real-time lamps and one shadow-casting light. It was the demo's look before the building
was made in Blender.

The simulator runs without this folder too. It then falls back to plain colours.
