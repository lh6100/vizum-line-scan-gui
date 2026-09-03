#!/usr/bin/env python3
"""Create a Blender scene showing the calibrated fields of view of both cameras.

Run with Blender, not regular Python:

    blender --background --python tools/create_stereo_fov_blender.py -- \
      --output data/stereo/optical_simulation/stereo_fov.blend \
      --render data/stereo/optical_simulation/stereo_fov_preview.png

The scene uses millimetres. Camera geometry follows the OpenCV convention:
x right, y down, z forward. T_flange_camera maps camera coordinates to the
common FR5 flange frame.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from calibration_profile_input import load_camera_profile


DISTANCES_MM = (300.0, 500.0, 800.0, 1000.0)

CAMERAS = (
    {
        **load_camera_profile(PROJECT_ROOT, "scanner_650", include_handeye=True),
        "name": "scanner_650",
        "label": "650 nm / MV-CS016-10GM / 1440x1080",
        "color": (1.0, 0.035, 0.02, 1.0),
    },
    {
        **load_camera_profile(PROJECT_ROOT, "scanner_450", include_handeye=True),
        "name": "scanner_450",
        "label": "450 nm / MV-CS013-60GN / 1224x1024",
        "color": (0.01, 0.12, 1.0, 1.0),
    },
)


def parse_args() -> argparse.Namespace:
    user_args = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render", type=Path)
    return parser.parse_args(user_args)


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for data_collection in (
        bpy.data.curves,
        bpy.data.meshes,
        bpy.data.materials,
        bpy.data.cameras,
        bpy.data.lights,
    ):
        # Only remove orphaned datablocks left by the default scene.
        for block in list(data_collection):
            if block.users == 0:
                data_collection.remove(block)


def make_material(name: str, color, emission_strength=1.0):
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    emission = nodes.new("ShaderNodeEmission")
    emission.inputs["Color"].default_value = color
    emission.inputs["Strength"].default_value = emission_strength
    links.new(emission.outputs["Emission"], output.inputs["Surface"])
    return material


def add_polyline(name: str, points, material, thickness=1.5, cyclic=False):
    curve = bpy.data.curves.new(name, type="CURVE")
    curve.dimensions = "3D"
    curve.resolution_u = 1
    curve.bevel_depth = thickness
    curve.bevel_resolution = 2
    spline = curve.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for point, coordinate in zip(spline.points, points):
        point.co = (*coordinate, 1.0)
    spline.use_cyclic_u = cyclic
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def add_camera_body(camera, transform: Matrix, material):
    center = transform.translation
    rotation = transform.to_3x3()

    bpy.ops.mesh.primitive_cube_add(location=center)
    body = bpy.context.object
    body.name = f"{camera['name']}_body"
    body.dimensions = (55.0, 45.0, 55.0)
    body.rotation_mode = "QUATERNION"
    # The cube's local +Z follows the OpenCV optical axis.
    body.rotation_quaternion = rotation.to_quaternion()
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    body.data.materials.append(material)

    bpy.ops.mesh.primitive_cylinder_add(
        vertices=48, radius=18.0, depth=38.0, location=center
    )
    lens = bpy.context.object
    lens.name = f"{camera['name']}_lens"
    lens.rotation_mode = "QUATERNION"
    lens.rotation_quaternion = rotation.to_quaternion()
    # Move the lens along the camera's +Z optical direction.
    lens.location = center + rotation @ Vector((0.0, 0.0, 38.0))
    lens.data.materials.append(material)

    # Add a Blender camera object for optional later rendering. Blender looks
    # along local -Z and has local +Y up; the custom frustum remains the source
    # of truth for the calibrated view.
    data = bpy.data.cameras.new(f"{camera['name']}_camera_data")
    data.sensor_fit = "HORIZONTAL"
    data.sensor_width = 36.0
    data.lens = camera["fx"] / camera["width"] * data.sensor_width
    data.clip_start = 10.0
    data.clip_end = 3000.0
    obj = bpy.data.objects.new(f"{camera['name']}_camera", data)
    bpy.context.collection.objects.link(obj)
    cv_to_blender = Matrix(
        ((1.0, 0.0, 0.0, 0.0),
         (0.0, -1.0, 0.0, 0.0),
         (0.0, 0.0, -1.0, 0.0),
         (0.0, 0.0, 0.0, 1.0))
    )
    obj.matrix_world = transform @ cv_to_blender
    obj["image_width"] = camera["width"]
    obj["image_height"] = camera["height"]
    obj["fx_px"] = camera["fx"]
    obj["fy_px"] = camera["fy"]
    obj["cx_px"] = camera["cx"]
    obj["cy_px"] = camera["cy"]
    obj["note"] = "Custom FOV curves are authoritative; render resolution is per camera."


def camera_point(camera, u: float, v: float, depth: float) -> Vector:
    return Vector(
        (
            (u - camera["cx"]) / camera["fx"] * depth,
            (v - camera["cy"]) / camera["fy"] * depth,
            depth,
        )
    )


def add_frustum(camera, material):
    transform = Matrix(camera["transform"])
    center = transform.translation
    width = float(camera["width"])
    height = float(camera["height"])
    far_depth = DISTANCES_MM[-1]
    far_corners_camera = (
        camera_point(camera, 0.0, 0.0, far_depth),
        camera_point(camera, width, 0.0, far_depth),
        camera_point(camera, width, height, far_depth),
        camera_point(camera, 0.0, height, far_depth),
    )
    far_corners = [transform @ point for point in far_corners_camera]

    for index, corner in enumerate(far_corners, start=1):
        add_polyline(
            f"{camera['name']}_frustum_edge_{index}",
            (center, corner),
            material,
            thickness=1.6,
        )

    optical_axis_end = transform @ Vector((0.0, 0.0, far_depth + 100.0))
    add_polyline(
        f"{camera['name']}_optical_axis",
        (center, optical_axis_end),
        material,
        thickness=2.4,
    )

    for depth in DISTANCES_MM:
        corners_camera = (
            camera_point(camera, 0.0, 0.0, depth),
            camera_point(camera, width, 0.0, depth),
            camera_point(camera, width, height, depth),
            camera_point(camera, 0.0, height, depth),
        )
        corners = [transform @ point for point in corners_camera]
        field_width = depth * width / camera["fx"]
        field_height = depth * height / camera["fy"]
        object_name = (
            f"{camera['name']}_FOV_{int(depth)}mm_"
            f"{field_width:.1f}x{field_height:.1f}mm"
        )
        rectangle = add_polyline(
            object_name, corners, material, thickness=2.0, cyclic=True
        )
        rectangle["distance_mm"] = depth
        rectangle["field_width_mm"] = field_width
        rectangle["field_height_mm"] = field_height

    add_camera_body(camera, transform, material)


def add_axes():
    origin = Vector((0.0, 0.0, 0.0))
    axes = (
        ("flange_X", Vector((220.0, 0.0, 0.0)), (1.0, 0.0, 0.0, 1.0)),
        ("flange_Y", Vector((0.0, 220.0, 0.0)), (0.0, 1.0, 0.0, 1.0)),
        ("flange_Z", Vector((0.0, 0.0, 220.0)), (0.0, 0.35, 1.0, 1.0)),
    )
    for name, end, color in axes:
        material = make_material(f"{name}_material", color, 1.5)
        add_polyline(name, (origin, end), material, thickness=3.0)


def look_at(obj, target: Vector) -> None:
    direction = target - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def add_overview_camera():
    data = bpy.data.cameras.new("Overview_camera_data")
    data.lens = 35.0
    data.sensor_width = 36.0
    data.clip_start = 10.0
    data.clip_end = 10000.0
    obj = bpy.data.objects.new("Overview_camera", data)
    bpy.context.collection.objects.link(obj)
    obj.location = Vector((1900.0, -2400.0, 1550.0))
    look_at(obj, Vector((80.0, 85.0, 520.0)))
    bpy.context.scene.camera = obj


def add_floor():
    material = bpy.data.materials.new("Flange_plane_material")
    material.diffuse_color = (0.035, 0.045, 0.06, 1.0)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    principled = nodes.new("ShaderNodeBsdfPrincipled")
    principled.inputs["Base Color"].default_value = (0.025, 0.035, 0.05, 1.0)
    principled.inputs["Roughness"].default_value = 0.85
    links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    bpy.ops.mesh.primitive_plane_add(size=1800.0, location=(80.0, 80.0, -15.0))
    plane = bpy.context.object
    plane.name = "FR5_flange_reference_plane"
    plane.data.materials.append(material)


def configure_scene():
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "MILLIMETERS"
    scene.unit_settings.scale_length = 0.001
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.008, 0.012, 0.022)
    scene["coordinate_frame"] = "FR5 flange"
    scene["length_unit"] = "millimetre"
    scene["layout_warning"] = (
        "Relative pose is derived from two independent hand-eye calibrations. "
        "Replace transforms with dedicated stereo extrinsics for metric simulation."
    )

    # Use Eevee when available; emission lines remain visible in either engine.
    available = [item.identifier for item in scene.bl_rna.properties["render"].fixed_type.properties["engine"].enum_items]
    if "BLENDER_EEVEE_NEXT" in available:
        scene.render.engine = "BLENDER_EEVEE_NEXT"

    bpy.ops.object.light_add(type="AREA", location=(300.0, -400.0, 1000.0))
    key = bpy.context.object
    key.name = "Key_light"
    key.data.energy = 1800.0
    key.data.shape = "DISK"
    key.data.size = 1000.0
    look_at(key, Vector((80.0, 80.0, 450.0)))

    bpy.ops.object.light_add(type="AREA", location=(-900.0, 800.0, 700.0))
    fill = bpy.context.object
    fill.name = "Fill_light"
    fill.data.energy = 900.0
    fill.data.size = 800.0
    look_at(fill, Vector((80.0, 80.0, 450.0)))


def main() -> None:
    args = parse_args()
    clear_scene()
    configure_scene()
    add_floor()
    add_axes()
    for camera in CAMERAS:
        material = make_material(
            f"{camera['name']}_FOV_material", camera["color"], 2.5
        )
        add_frustum(camera, material)
    add_overview_camera()

    # Store the mechanical separation and optical-axis angle for quick checks.
    first = Matrix(CAMERAS[0]["transform"])
    second = Matrix(CAMERAS[1]["transform"])
    separation = (first.translation - second.translation).length
    axis_1 = first.to_3x3() @ Vector((0.0, 0.0, 1.0))
    axis_2 = second.to_3x3() @ Vector((0.0, 0.0, 1.0))
    angle = math.degrees(axis_1.angle(axis_2))
    bpy.context.scene["camera_center_separation_mm"] = separation
    bpy.context.scene["optical_axis_angle_deg"] = angle

    args.output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output.resolve()))
    if args.render:
        args.render.parent.mkdir(parents=True, exist_ok=True)
        bpy.context.scene.render.filepath = str(args.render.resolve())
        bpy.ops.render.render(write_still=True)

    print(f"Saved Blender scene: {args.output}")
    if args.render:
        print(f"Saved preview: {args.render}")
    print(f"Camera separation: {separation:.3f} mm")
    print(f"Optical-axis angle: {angle:.3f} deg")


if __name__ == "__main__":
    main()
