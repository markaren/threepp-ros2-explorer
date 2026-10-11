"""Render the test image `data/rendered_cones.png` with threepp, off-screen.

The painted images of `synthetic_world.py` are flat shapes. This one comes
from a real renderer, with a sun and ambient light on a 3D cone, which is what
the simulator's camera delivers. `test_rendered_image.py` runs the detector
on the result.

The image is committed, so the tests do not need a graphics card. Run this
script only to regenerate it, from a shell where `explorer_vision` can be
imported (after `colcon build`, or with the package directory on PYTHONPATH):

    python render_test_image.py
"""

from pathlib import Path

import cv2
import threepp as tp

import synthetic_world as world

# Where the cones stand, in `base_link` (x forward, y left).
CONES = [(3.0, 0.4), (2.0, -0.8)]
WALL_X = 4.5
OUTPUT = Path(__file__).resolve().parent / 'data' / 'rendered_cones.png'


def ros_to_three(x: float, y: float, z: float):
    """ROS has z up, threepp has y up: three = (ros.x, ros.z, -ros.y)."""
    return x, z, -y


def main() -> None:
    canvas = tp.Canvas('render_test_image', width=world.IMAGE_WIDTH,
                       height=world.IMAGE_HEIGHT, headless=True)
    renderer = tp.GLRenderer(canvas)
    renderer.set_clear_color(0x9aa4b0)

    scene = tp.Scene()
    scene.add(tp.HemisphereLight(0xffffff, 0x555555, 0.9))
    sun = tp.DirectionalLight(0xffffff, 2.0)
    sun.position.set(*ros_to_three(-2.0, 3.0, 6.0))
    scene.add(sun)

    floor_material = tp.MeshStandardMaterial()
    floor_material.color = 0x808080
    floor = tp.Mesh(tp.PlaneGeometry(30, 30), floor_material)
    floor.rotate_x(-3.14159265 / 2)
    scene.add(floor)

    wall_material = tp.MeshStandardMaterial()
    wall_material.color = 0x607088
    wall = tp.Mesh(tp.BoxGeometry(0.2, 3.0, 12.0), wall_material)
    wall.position.set(*ros_to_three(WALL_X + 0.1, 0.0, 1.5))
    scene.add(wall)

    cone_material = tp.MeshStandardMaterial()
    cone_material.color = 0xff6a00
    for cone_x, cone_y in CONES:
        cone = tp.Mesh(tp.ConeGeometry(world.CONE.base_radius, world.CONE.height, 32),
                       cone_material)
        # The geometry's origin is at half its height.
        cone.position.set(*ros_to_three(cone_x, cone_y, world.CONE.height / 2.0))
        scene.add(cone)

    camera = tp.PerspectiveCamera(world.VERTICAL_FOV_DEG,
                                  world.IMAGE_WIDTH / world.IMAGE_HEIGHT, 0.05, 50.0)
    camera.position.set(*ros_to_three(world.CAMERA_X, 0.0, world.CAMERA_Z))
    camera.look_at(*ros_to_three(world.CAMERA_X + 10.0, 0.0, world.CAMERA_Z))

    renderer.render(scene, camera)
    image_rgb = renderer.read_pixels()

    OUTPUT.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(OUTPUT), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    print(f'wrote {OUTPUT.name}: {image_rgb.shape[1]} x {image_rgb.shape[0]}')


if __name__ == '__main__':
    main()
