# Models

The 3D models of the warehouse: the robot, the racking and what stands around it.

## Conventions

* Units are metres, at real-world size. One self-contained `.glb` per model, textures embedded.
* Authored with ROS axes (+X forward, +Y left, +Z up) and exported with the standard glTF +Y-up
  conversion. A loader therefore sees +X forward, +Y up, and ROS +Y as its -Z.
* The origin is the centre of the footprint, on the floor. The signs are the exception: their
  origin is the centre of the back face, and the readable face looks along +X.

## The models

Sizes are X x Y x Z in ROS axes, measured from the exported files (bounding box, in threepp).

| File | What it is | Size (m) | Triangles | File size | Made from | Named nodes |
|---|---|---|---|---|---|---|
| `robot.glb` | the robot: differential drive, lidar on a mast, forward camera | 0.440 x 0.428 x 0.326 | 8800 | 215 kB | `make_models.py` | `robot` (root), `body`, `wheel_left`, `wheel_right`, `lidar_head` |
| `rack_a.glb` | one bay of pallet racking, mostly cartons | 3.400 x 0.800 x 1.350 | 8708 | 240 kB | `make_models.py` | `rack_a` |
| `rack_b.glb` | the same bay, mostly totes and wrapped goods | 3.400 x 0.800 x 1.350 | 10260 | 256 kB | `make_models.py` | `rack_b` |
| `pallet_a.glb` | 1200 x 800 pallet with strapped cartons | 1.200 x 0.800 x 0.885 | 3672 | 98 kB | `make_models.py` | `pallet_a` |
| `pallet_b.glb` | 1200 x 800 pallet with stacked totes | 1.200 x 0.800 x 0.820 | 5988 | 119 kB | `make_models.py` | `pallet_b` |
| `crate.glb` | shipping crate, a UNIT cube that the simulator scales per axis | 1.000 x 1.000 x 1.000 | 3132 | 67 kB | `make_models.py` | `crate` |
| `sign_aisle_A.glb` | aisle sign "A" | 0.0225 x 0.450 x 0.450 | 308 | 13 kB | `make_models.py` | `sign_aisle_A` |
| `sign_aisle_B.glb` | aisle sign "B" | 0.0225 x 0.450 x 0.450 | 592 | 23 kB | `make_models.py` | `sign_aisle_B` |
| `sign_aisle_C.glb` | aisle sign "C" | 0.0225 x 0.450 x 0.450 | 460 | 18 kB | `make_models.py` | `sign_aisle_C` |
| `sign_aisle_D.glb` | aisle sign "D" | 0.0225 x 0.450 x 0.450 | 480 | 19 kB | `make_models.py` | `sign_aisle_D` |
| `sign_dock.glb` | sign "LOADING DOCK" | 0.0225 x 1.200 x 0.300 | 1716 | 69 kB | `make_models.py` | `sign_dock` |
| `sign_storage.glb` | sign "STORAGE" | 0.0225 x 1.200 x 0.300 | 1408 | 57 kB | `make_models.py` | `sign_storage` |
| `sign_office.glb` | sign "OFFICE" | 0.0225 x 1.200 x 0.300 | 804 | 34 kB | `make_models.py` | `sign_office` |
| `sign_exit.glb` | sign "EXIT", green, slightly emissive | 0.0225 x 0.600 x 0.250 | 392 | 18 kB | `make_models.py` | `sign_exit` |
| `desk.glb` | office desk, the drawer side faces +X | 0.947 x 2.000 x 0.787 | 6898 | 324 kB | [metal_office_desk](https://polyhaven.com/a/metal_office_desk), CC0 | `desk` |
| `barrel.glb` | blue plastic drum | 0.485 x 0.488 x 0.880 | 2688 | 174 kB | [Barrel_02](https://polyhaven.com/a/Barrel_02), CC0 | `barrel` |
| `tote.glb` | open plastic crate, recoloured blue | 0.406 x 0.505 x 0.254 | 5840 | 538 kB | [plastic_crate_02](https://polyhaven.com/a/plastic_crate_02), CC0 | `tote` |
| `cart.glb` | storage cart on castors | 1.103 x 1.603 x 1.377 | 18902 | 742 kB | [industrial_storage_cart](https://polyhaven.com/a/industrial_storage_cart), CC0 | `cart` |
| `ladder.glb` | sectioned ladder, standing upright | 0.178 x 0.661 x 2.135 | 29140 | 1020 kB | [ladder_sectioned_01](https://polyhaven.com/a/ladder_sectioned_01), CC0 | `ladder` |
| `stool.glb` | round stool | 0.473 x 0.448 x 0.458 | 6532 | 307 kB | [metal_stool_02](https://polyhaven.com/a/metal_stool_02), CC0 | `stool` |

The whole folder is about 4.5 MB.

## The robot's moving parts

`wheel_left`, `wheel_right` and `lidar_head` are separate nodes, children of the root `robot`.

| Node | Origin (ROS, m) | Spins about | In a +Y-up loader |
|---|---|---|---|
| `wheel_left` | (0, +0.20, 0.07), on the axle | its Y axis | its Z axis, with the sign flipped: `rotation.z = -angle` |
| `wheel_right` | (0, -0.20, 0.07), on the axle | its Y axis | its Z axis, with the sign flipped: `rotation.z = -angle` |
| `lidar_head` | (0, 0, 0.30), in the scan plane | its Z axis | its Y axis: `rotation.y = angle` |

The wheels have a radius of 0.07 m. The camera lens is at x = 0.18, z = 0.25, and nothing above
the chassis reaches further forward. The highest point is the lidar cap at z = 0.326.

## How they were made

The scripts are in `tools/blender/`. Run them from the project root; no window opens.

```
"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe" --background --python tools/blender/make_models.py -- src/explorer_sim/assets/models

pixi run -e jazzy python tools/blender/fetch_polyhaven.py <work folder>
"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe" --background --python tools/blender/clean_polyhaven.py -- <work folder> src/explorer_sim/assets/models
```

* `make_models.py` builds the robot, racks, pallets, crate and signs from bevelled boxes, lathed
  profiles and lofted outlines, with plain materials (colour, roughness, emission), no textures.
* `fetch_polyhaven.py` downloads the six [Poly Haven](https://polyhaven.com) props (CC0) and
  prepares their textures; `clean_polyhaven.py` puts each prop on the floor at its footprint
  centre, turns its front to +X and exports it.

## Colours and the cone detector

`explorer_vision` finds the traffic cones by colour: OpenCV hue 3 to 24 (red-orange to amber and
brown), saturation 110 or more, value 40 or more. Nothing in this folder may land in that band,
so the racks are blue and grey instead of the usual orange, and "wood" and "cardboard" are pale,
desaturated tones.

Two things are less obvious, and both come from the warm warehouse photo that lights the scene:

* A black or mirror-like surface shows little but its reflections, and those are dark amber:
  inside the band. So the robot's dark parts are a matt charcoal blue, not gloss black.
* The Poly Haven props were changed for the same reason: warm hues turned into cool greys,
  blacks lifted, metal switched off, everything matt, textures shrunk to 512 px. The tote is
  orange as it comes (a material colour, not its texture) and is blue here.

Each model was rendered with threepp from four sides under the simulator's own lighting (the
warehouse HDRI at 0.75, the hemisphere light, the white sun, Neutral tone mapping) and the frames
were run through `cv2.inRange(hsv, (3, 110, 40), (24, 255, 255))`: no pixel of any model matched.
The closest are the pale pallet wood and cardboard, at a saturation of 78 of 255 where the
detector needs 110.
