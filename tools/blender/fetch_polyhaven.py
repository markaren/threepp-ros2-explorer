"""Step 1 of 2 for the Poly Haven props: download them and prepare their textures.

Poly Haven (https://polyhaven.com) assets are CC0. This script fetches the 1k glTF version of each
prop into a work folder, then writes a cleaned copy next to it (`_clean/<asset>/`):

  * every texture is shrunk to 512 px (the props are small on screen, and the files stay light)
  * the colour texture is checked against the cone detector: warm hues (red, orange, amber, brown)
    are turned into cool greys, because `explorer_vision` finds the traffic cones by their
    saturated orange; blacks are lifted and metal is switched off for the same reason (a black or
    metallic surface mirrors the warm warehouse photo that lights the scene)

Run it with the project's Python (needs cv2 and numpy), then run step 2 in Blender:

    pixi run -e jazzy python tools/blender/fetch_polyhaven.py <work folder>
    "C:\\Program Files\\Blender Foundation\\Blender 5.0\\blender.exe" --background --python tools/blender/clean_polyhaven.py -- <work folder> src/explorer_sim/assets/models
"""
import json
import os
import shutil
import sys
import urllib.request

import cv2
import numpy as np

# model file name -> Poly Haven asset id (https://polyhaven.com/a/<id>)
ASSETS = {
    "desk": "metal_office_desk",
    "barrel": "Barrel_02",
    "tote": "plastic_crate_02",
    "cart": "industrial_storage_cart",
    "ladder": "ladder_sectioned_01",
    "stool": "metal_stool_02",
}
TEXTURE_SIZE = 512
SATURATION_CAP = 36  # of 255, for warm hues; the detector needs 110 or more


def get(url):
    req = urllib.request.Request(url, headers={"User-Agent": "threepp-ros2-demo-asset-fetch/1.0"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read()


def download(asset, folder):
    """The glTF file, its .bin and its textures, as listed by the Poly Haven API."""
    if os.path.isdir(folder) and any(f.endswith(".gltf") for f in os.listdir(folder)):
        return
    gltf = json.loads(get(f"https://api.polyhaven.com/files/{asset}"))["gltf"]["1k"]["gltf"]
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, os.path.basename(gltf["url"])), "wb") as f:
        f.write(get(gltf["url"]))
    for rel, inc in gltf.get("include", {}).items():
        path = os.path.join(folder, *rel.split("/"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as f:
            f.write(get(inc["url"]))


def tame_warm_hues(bgr, cap):
    """Desaturates red, orange, amber and brown. OpenCV hue is 0..180; the detector's band is 3..24."""
    h, s, v = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    warm = (h <= 45) | (h >= 168)  # a wide margin around the band: lighting shifts hues
    s[warm] = np.minimum(s[warm], cap)
    h[warm] = 105                  # what is left of the colour becomes a cool grey, not a warm one
    dark = v < 100                 # dark pixels of any hue become cool greys too: a little light
    s[dark] = np.minimum(s[dark], cap)  # or JPEG noise is a lot of saturation down there
    h[dark & (h < 90)] = 105
    # Lift the blacks. A black surface shows almost nothing but its reflections, and the warehouse
    # photo that lights the scene is warm: black parts came out dark amber, inside the cone band.
    v = (88.0 + v.astype(np.float32) * (167.0 / 255.0)).astype(np.uint8)
    return cv2.cvtColor(cv2.merge([h, s, v]), cv2.COLOR_HSV2BGR)


def no_mirrors(arm_bgr):
    """The occlusion/roughness/metal texture (R, G, B): no metal, and matt all over.
    Metal and gloss mirror the warm warehouse photo, and those reflections fall in the cone band."""
    out = arm_bgr.copy()
    out[..., 0] = 0                                # blue channel: metalness
    out[..., 1] = np.maximum(out[..., 1], 215)     # green channel: roughness
    out[..., 2] = np.maximum(out[..., 2], 200)     # red channel: occlusion; deep creases would otherwise
    #                                                show nothing but the (warm) reflection
    return out


def in_cone_band(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return 100.0 * np.count_nonzero(cv2.inRange(hsv, (3, 110, 40), (24, 255, 255))) / (bgr.shape[0] * bgr.shape[1])


def clean(src, dst):
    os.makedirs(os.path.join(dst, "textures"), exist_ok=True)
    for f in os.listdir(src):
        if f.endswith((".gltf", ".bin")):
            shutil.copyfile(os.path.join(src, f), os.path.join(dst, f))
    for f in os.listdir(os.path.join(src, "textures")):
        img = cv2.imread(os.path.join(src, "textures", f), cv2.IMREAD_COLOR)
        img = cv2.resize(img, (TEXTURE_SIZE, TEXTURE_SIZE), interpolation=cv2.INTER_AREA)
        if "_diff_" in f:
            before = in_cone_band(img)
            for cap in (SATURATION_CAP, 36, 24, 12):  # JPEG can push dark pixels back; tighten until none is left
                cv2.imwrite(os.path.join(dst, "textures", f), tame_warm_hues(img, cap), [cv2.IMWRITE_JPEG_QUALITY, 92])
                after = in_cone_band(cv2.imread(os.path.join(dst, "textures", f)))
                if after == 0.0:
                    break
            print(f"{os.path.basename(src):28s} colour texture in the cone band: {before:6.2f} % -> {after:.2f} %")
        elif "_arm_" in f:
            cv2.imwrite(os.path.join(dst, "textures", f), no_mirrors(img), [cv2.IMWRITE_JPEG_QUALITY, 88])
        else:
            cv2.imwrite(os.path.join(dst, "textures", f), img, [cv2.IMWRITE_JPEG_QUALITY, 88])


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: python fetch_polyhaven.py <work folder>")
    work = os.path.abspath(sys.argv[1])
    for asset in ASSETS.values():
        download(asset, os.path.join(work, asset))
        clean(os.path.join(work, asset), os.path.join(work, "_clean", asset))


if __name__ == "__main__":
    main()
