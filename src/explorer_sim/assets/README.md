# Assets

Everything in this folder comes from [Poly Haven](https://polyhaven.com) and is licensed
[CC0](https://polyhaven.com/license): free to use, change and redistribute, no attribution
required. Credit is given anyway.

| Files | Poly Haven asset | Used for |
|---|---|---|
| `concrete_floor_worn_001_*.jpg` | [concrete_floor_worn_001](https://polyhaven.com/a/concrete_floor_worn_001) | the floor |
| `painted_plaster_wall_*.jpg` | [painted_plaster_wall](https://polyhaven.com/a/painted_plaster_wall) | the walls |
| `blue_metal_plate_*.jpg` | [blue_metal_plate](https://polyhaven.com/a/blue_metal_plate) | the shelves |
| `weathered_planks_*.jpg` | [weathered_planks](https://polyhaven.com/a/weathered_planks) | the crates |
| `empty_warehouse_01_1k.hdr` | [empty_warehouse_01](https://polyhaven.com/a/empty_warehouse_01) | lighting and backdrop |

All are the 1k versions. Each texture set has three maps:

* `_Diffuse`: the colour of the surface
* `_nor_gl`: the normal map, fine relief that catches the light (OpenGL convention)
* `_Rough`: the roughness map, where the surface is matt and where it shines

One file was changed: `weathered_planks_grey.jpg` is the planks' colour map converted to
greyscale and brightened. The crates get their teal from the material's tint instead.
That is not only taste. The cone detector (`explorer_vision`) looks for saturated orange,
and brown wood is a dark orange: with the original colours, crates could be mistaken
for cones.

The simulator runs without this folder too. It then falls back to plain colours.
