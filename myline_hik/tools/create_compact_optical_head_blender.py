#!/usr/bin/env python3
"""Build the V1 compact dual camera/line-laser head in Blender.

Run with Blender:

    blender --background --python tools/create_compact_optical_head_blender.py

The design coordinate system is:
    X: horizontal / stereo baseline / camera image X
    Y: up
    Z: forward, toward the workpiece

All numeric dimensions are millimetres.  The welding-gun fixture interface is
provisional until its final mounting-hole drawing is available.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from calibration_profile_input import load_camera_profile


MODEL_DIR = ROOT / "models" / "new_model"
STL_DIR = MODEL_DIR / "stl"
OUTPUT_DIR = MODEL_DIR / "design_v1"

BLEND_PATH = OUTPUT_DIR / "compact_optical_head_v1.blend"
ASSEMBLY_RENDER = OUTPUT_DIR / "compact_optical_head_v1_preview.png"
FRONT_RENDER = OUTPUT_DIR / "compact_optical_head_v1_front.png"
OPTICAL_RENDER = OUTPUT_DIR / "compact_optical_head_v1_optical.png"

CAMERA_Y = 16.0
LASER_Y = -18.0
CAMERA_LASER_BASELINE = CAMERA_Y - LASER_Y
CAMERA_BASELINE = 116.0
LEFT_X = -CAMERA_BASELINE / 2.0
RIGHT_X = CAMERA_BASELINE / 2.0

POD_WIDTH = 42.0
POD_Y_MIN = -41.0
POD_Y_MAX = 39.0
POD_Z_BACK = 106.0
POD_Z_FRONT = 224.0
WALL = 2.5

CAMERA_OPTICAL_Z = 227.0
LASER_EXIT_Z = 227.0
NOMINAL_WORKING_DISTANCE = 600.0
LASER_PITCH_DEG = math.degrees(
    math.atan2(CAMERA_LASER_BASELINE, NOMINAL_WORKING_DISTANCE)
)
LASER_PITCH_MIN_DEG = math.degrees(
    math.atan2(CAMERA_LASER_BASELINE, 1000.0)
)
LASER_PITCH_MAX_DEG = math.degrees(
    math.atan2(CAMERA_LASER_BASELINE, 300.0)
)

CAMERAS = (
    {
        **load_camera_profile(ROOT, "scanner_450"),
        "profile": "scanner_450",
        "side": "left",
        "camera": "130万相机",
        "laser": "450 nm线激光",
        "laser_nm": 450,
        "x": LEFT_X,
        "camera_color": (0.28, 0.31, 0.36, 1.0),
        "laser_color": (0.015, 0.08, 1.0, 1.0),
        "accent_color": (0.018, 0.12, 0.78, 1.0),
    },
    {
        **load_camera_profile(ROOT, "scanner_650"),
        "profile": "scanner_650",
        "side": "right",
        "camera": "160万相机",
        "laser": "650 nm线激光",
        "laser_nm": 650,
        "x": RIGHT_X,
        "camera_color": (0.28, 0.31, 0.36, 1.0),
        "laser_color": (1.0, 0.018, 0.008, 1.0),
        "accent_color": (0.72, 0.025, 0.018, 1.0),
    },
)


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for datablocks in (
        bpy.data.meshes,
        bpy.data.curves,
        bpy.data.materials,
        bpy.data.cameras,
        bpy.data.lights,
    ):
        for block in list(datablocks):
            if block.users == 0:
                datablocks.remove(block)


def make_principled_material(
    name: str,
    color,
    *,
    metallic: float = 0.0,
    roughness: float = 0.45,
):
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])
    return material


def make_emission_material(name: str, color, strength: float = 4.0):
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
    emission.inputs["Strength"].default_value = strength
    links.new(emission.outputs["Emission"], output.inputs["Surface"])
    return material


def make_glass_material(name: str, color):
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    glass = nodes.new("ShaderNodeBsdfGlass")
    glass.inputs["Color"].default_value = color
    glass.inputs["Roughness"].default_value = 0.08
    glass.inputs["IOR"].default_value = 1.46
    links.new(glass.outputs["BSDF"], output.inputs["Surface"])
    return material


def make_translucent_emission_material(
    name: str, color, strength: float = 1.5, opacity: float = 0.18
):
    material = bpy.data.materials.new(name)
    material.diffuse_color = (*color[:3], opacity)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    transparent = nodes.new("ShaderNodeBsdfTransparent")
    emission = nodes.new("ShaderNodeEmission")
    emission.inputs["Color"].default_value = color
    emission.inputs["Strength"].default_value = strength
    mix = nodes.new("ShaderNodeMixShader")
    mix.inputs[0].default_value = opacity
    links.new(transparent.outputs["BSDF"], mix.inputs[1])
    links.new(emission.outputs["Emission"], mix.inputs[2])
    links.new(mix.outputs["Shader"], output.inputs["Surface"])
    if hasattr(material, "surface_render_method"):
        material.surface_render_method = "DITHERED"
    return material


def add_box(name: str, dimensions, location, material=None, bevel: float = 0.0):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel > 0.0:
        modifier = obj.modifiers.new("edge_radius", "BEVEL")
        modifier.width = bevel
        modifier.segments = 3
    if material is not None:
        obj.data.materials.append(material)
    return obj


def add_cylinder(
    name: str,
    radius: float,
    depth: float,
    location,
    material=None,
    *,
    vertices: int = 64,
    rotation=(0.0, 0.0, 0.0),
):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        location=location,
        rotation=rotation,
    )
    obj = bpy.context.object
    obj.name = name
    if material is not None:
        obj.data.materials.append(material)
    return obj


def add_polyline(name: str, points, material, thickness=0.8, cyclic=False):
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


def add_triangle(name: str, vertices, material):
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    mesh.from_pydata(vertices, [], [(0, 1, 2)])
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def add_text(name: str, body: str, location, material, size=5.0):
    curve = bpy.data.curves.new(f"{name}_curve", type="FONT")
    curve.body = body
    curve.align_x = "CENTER"
    curve.align_y = "CENTER"
    curve.size = size
    curve.extrude = 0.15
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    obj.location = location
    obj.data.materials.append(material)
    return obj


def join_objects(name: str, objects):
    bpy.ops.object.select_all(action="DESELECT")
    valid = [obj for obj in objects if obj and obj.name in bpy.data.objects]
    for obj in valid:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = valid[0]
    bpy.ops.object.join()
    joined = bpy.context.object
    joined.name = name
    return joined


def subtract_slot(target, x: float, y: float, length: float = 14.0):
    """Cut one provisional 5 x 14 mm mounting slot through a bridge rail."""
    cutters = [
        add_box(
            "slot_mid",
            (5.0, length - 5.0, 12.0),
            (x, y, target.location.z),
        ),
        add_cylinder(
            "slot_end_a",
            2.5,
            12.0,
            (x, y - (length - 5.0) / 2.0, target.location.z),
            vertices=32,
        ),
        add_cylinder(
            "slot_end_b",
            2.5,
            12.0,
            (x, y + (length - 5.0) / 2.0, target.location.z),
            vertices=32,
        ),
    ]
    cutter = join_objects("temporary_slot_cutter", cutters)
    modifier = target.modifiers.new("provisional_M4_slot", "BOOLEAN")
    modifier.operation = "DIFFERENCE"
    modifier.solver = "EXACT"
    modifier.object = cutter
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=modifier.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def import_fixture(material):
    imported = []
    for side in ("左", "右"):
        source = STL_DIR / f"新型焊枪夹具{side}.STL"
        before = set(bpy.data.objects)
        bpy.ops.wm.stl_import(filepath=str(source))
        new_objects = list(set(bpy.data.objects) - before)
        for obj in new_objects:
            obj.name = f"现有新型焊枪夹具_{side}"
            obj.location += Vector((-39.604, -23.7285, 0.0))
            obj.data.materials.append(material)
            imported.append(obj)
    return imported


def build_bridge(material):
    z = 109.0
    thickness = 6.0
    outer_w = 160.0
    outer_h = 90.0
    opening_w = 84.0
    opening_h = 48.0
    top_h = (outer_h - opening_h) / 2.0
    side_w = (outer_w - opening_w) / 2.0

    top = add_box(
        "bridge_top",
        (outer_w, top_h, thickness),
        (0.0, (opening_h + top_h) / 2.0, z),
        material,
        1.5,
    )
    bottom = add_box(
        "bridge_bottom",
        (outer_w, top_h, thickness),
        (0.0, -(opening_h + top_h) / 2.0, z),
        material,
        1.5,
    )
    left = add_box(
        "bridge_left",
        (side_w, opening_h, thickness),
        (-(opening_w + side_w) / 2.0, 0.0, z),
        material,
        1.5,
    )
    right = add_box(
        "bridge_right",
        (side_w, opening_h, thickness),
        ((opening_w + side_w) / 2.0, 0.0, z),
        material,
        1.5,
    )
    for rail, x in ((left, -61.0), (right, 61.0)):
        subtract_slot(rail, x, -12.0)
        subtract_slot(rail, x, 12.0)

    bridge = join_objects("桥式转接框_160x90x6_内孔84x48", [top, bottom, left, right])
    bridge["outer_size_mm"] = "160 x 90 x 6"
    bridge["fixture_clearance_mm"] = "84 x 48"
    bridge["mounting_slots"] = "4 x provisional M4, 5 x 14 mm"
    return bridge


def build_pod_shell(side: str, x: float, shell_material):
    y_center = (POD_Y_MIN + POD_Y_MAX) / 2.0
    y_size = POD_Y_MAX - POD_Y_MIN
    z_center = (POD_Z_BACK + POD_Z_FRONT) / 2.0
    z_size = POD_Z_FRONT - POD_Z_BACK
    inner_width = POD_WIDTH - 2.0 * WALL
    parts = [
        add_box(
            f"{side}_shell_outer",
            (WALL, y_size, z_size),
            (x - POD_WIDTH / 2.0 + WALL / 2.0, y_center, z_center),
            shell_material,
            1.2,
        ),
        add_box(
            f"{side}_shell_inner",
            (WALL, y_size, z_size),
            (x + POD_WIDTH / 2.0 - WALL / 2.0, y_center, z_center),
            shell_material,
            1.2,
        ),
        add_box(
            f"{side}_shell_top",
            (inner_width, WALL, z_size),
            (x, POD_Y_MAX - WALL / 2.0, z_center),
            shell_material,
            1.2,
        ),
        add_box(
            f"{side}_shell_bottom",
            (inner_width, WALL, z_size),
            (x, POD_Y_MIN + WALL / 2.0, z_center),
            shell_material,
            1.2,
        ),
        add_box(
            f"{side}_shell_rear",
            (inner_width, y_size - 2.0 * WALL, 3.0),
            (x, y_center, POD_Z_BACK + 1.5),
            shell_material,
            1.0,
        ),
        add_box(
            f"{side}_optical_partition",
            (inner_width, 2.5, z_size - 6.0),
            (x, -4.5, z_center - 1.0),
            shell_material,
            0.8,
        ),
    ]
    shell = join_objects(f"{side}_一体式防护外壳_42x80x118", parts)
    shell["outer_size_mm"] = "42 x 80 x 118"
    shell["wall_mm"] = WALL
    shell["note"] = "Camera upper cell; laser lower cell; independent optical partition."
    return shell


def frame_around_opening(
    prefix: str,
    x: float,
    y: float,
    opening_w: float,
    opening_h: float,
    outer_w: float,
    outer_h: float,
    z: float,
    depth: float,
    material,
):
    side_w = (outer_w - opening_w) / 2.0
    top_h = (outer_h - opening_h) / 2.0
    return [
        add_box(
            f"{prefix}_left",
            (side_w, outer_h, depth),
            (x - (opening_w + side_w) / 2.0, y, z),
            material,
            0.7,
        ),
        add_box(
            f"{prefix}_right",
            (side_w, outer_h, depth),
            (x + (opening_w + side_w) / 2.0, y, z),
            material,
            0.7,
        ),
        add_box(
            f"{prefix}_top",
            (opening_w, top_h, depth),
            (x, y + (opening_h + top_h) / 2.0, z),
            material,
            0.7,
        ),
        add_box(
            f"{prefix}_bottom",
            (opening_w, top_h, depth),
            (x, y - (opening_h + top_h) / 2.0, z),
            material,
            0.7,
        ),
    ]


def build_front_bezel(side: str, x: float, material, accent_material):
    parts = []
    parts += frame_around_opening(
        f"{side}_camera_bezel",
        x,
        CAMERA_Y,
        34.0,
        34.0,
        42.0,
        42.0,
        225.5,
        3.0,
        material,
    )
    parts += frame_around_opening(
        f"{side}_laser_bezel",
        x,
        LASER_Y,
        26.0,
        24.0,
        34.0,
        32.0,
        225.5,
        3.0,
        accent_material,
    )
    bezel = join_objects(f"{side}_可更换双窗口前框", parts)
    bezel["camera_clear_aperture_mm"] = "34 x 34"
    bezel["laser_clear_aperture_mm"] = "26 x 24"
    return bezel


def build_hoods(side: str, x: float, material):
    parts = []
    for prefix, y, width, height, extension in (
        ("camera", CAMERA_Y, 42.0, 42.0, 8.0),
        ("laser", LASER_Y, 34.0, 32.0, 10.0),
    ):
        z = 229.0 + extension / 2.0
        lip = 1.6
        parts.extend(
            (
                add_box(
                    f"{side}_{prefix}_hood_left",
                    (lip, height, extension),
                    (x - width / 2.0 + lip / 2.0, y, z),
                    material,
                ),
                add_box(
                    f"{side}_{prefix}_hood_right",
                    (lip, height, extension),
                    (x + width / 2.0 - lip / 2.0, y, z),
                    material,
                ),
                add_box(
                    f"{side}_{prefix}_hood_top",
                    (width - 2.0 * lip, lip, extension),
                    (x, y + height / 2.0 - lip / 2.0, z),
                    material,
                ),
                add_box(
                    f"{side}_{prefix}_hood_bottom",
                    (width - 2.0 * lip, lip, extension),
                    (x, y - height / 2.0 + lip / 2.0, z),
                    material,
                ),
            )
        )
    return join_objects(f"{side}_防飞溅遮光罩", parts)


def build_camera_hardware(camera, body_material, lens_material):
    side = camera["side"]
    x = camera["x"]
    body = add_box(
        f"{side}_{camera['camera']}_机身29x29x42",
        (29.0, 29.0, 42.0),
        (x, CAMERA_Y, 139.0),
        body_material,
        1.4,
    )
    lens = add_cylinder(
        f"{side}_{camera['camera']}_镜头",
        13.5,
        59.0,
        (x, CAMERA_Y, 189.5),
        body_material,
    )
    front = add_cylinder(
        f"{side}_{camera['camera']}_镜头前片",
        11.5,
        1.5,
        (x, CAMERA_Y, 219.75),
        lens_material,
    )
    for obj in (body, lens, front):
        obj["pairing"] = f"{camera['camera']} + {camera['laser']}"
    return [body, lens, front]


def build_laser_hardware(camera, body_material, aperture_material, metal_material):
    side = camera["side"]
    x = camera["x"]
    pitch = math.radians(LASER_PITCH_DEG)
    direction = Vector((0.0, math.sin(pitch), math.cos(pitch)))
    exit_point = Vector((x, LASER_Y, 221.5))
    center = exit_point - direction * 49.0

    body = add_box(
        f"{side}_{camera['laser']}_18x18x98",
        (18.0, 18.0, 98.0),
        center,
        body_material,
        1.0,
    )
    body.rotation_euler.x = -pitch
    nozzle_center = exit_point - direction * 5.0
    nozzle = add_cylinder(
        f"{side}_{camera['laser']}_出光筒",
        7.0,
        10.0,
        nozzle_center,
        body_material,
        rotation=(-pitch, 0.0, 0.0),
    )
    aperture = add_cylinder(
        f"{side}_{camera['laser']}_出光口",
        4.2,
        1.0,
        exit_point + direction * 0.2,
        aperture_material,
        rotation=(-pitch, 0.0, 0.0),
    )

    pivot_center = exit_point - direction * 47.0
    pivots = []
    for offset in (-11.0, 11.0):
        pivots.append(
            add_cylinder(
                f"{side}_激光俯仰枢轴_{offset:+.0f}",
                7.0,
                3.0,
                (x + offset, pivot_center.y, pivot_center.z),
                metal_material,
                rotation=(0.0, math.pi / 2.0, 0.0),
            )
        )
    lock_screw = add_cylinder(
        f"{side}_M3俯仰锁紧螺钉",
        2.8,
        4.5,
        (x + 13.0, pivot_center.y, pivot_center.z),
        metal_material,
        rotation=(0.0, math.pi / 2.0, 0.0),
    )
    body["nominal_pitch_deg"] = LASER_PITCH_DEG
    body["pitch_range_deg"] = (
        f"{LASER_PITCH_MIN_DEG:.2f} to {LASER_PITCH_MAX_DEG:.2f}"
    )
    body["roll_setting_deg"] = 0.0
    body["line_direction"] = "Parallel to camera image X"
    return [body, nozzle, aperture, *pivots, lock_screw]


def build_windows(camera, clear_glass, laser_glass, gasket_material):
    side = camera["side"]
    x = camera["x"]
    parts = [
        add_box(
            f"{side}_相机防护玻璃_34x34x2",
            (34.0, 34.0, 2.0),
            (x, CAMERA_Y, 226.2),
            clear_glass,
            0.5,
        ),
        add_box(
            f"{side}_激光防护玻璃_26x24x2",
            (26.0, 24.0, 2.0),
            (x, LASER_Y, 226.2),
            laser_glass,
            0.5,
        ),
    ]
    camera_gasket = frame_around_opening(
        f"{side}_camera_gasket",
        x,
        CAMERA_Y,
        34.0,
        34.0,
        37.0,
        37.0,
        224.4,
        1.2,
        gasket_material,
    )
    laser_gasket = frame_around_opening(
        f"{side}_laser_gasket",
        x,
        LASER_Y,
        26.0,
        24.0,
        29.0,
        27.0,
        224.4,
        1.2,
        gasket_material,
    )
    return parts + camera_gasket + laser_gasket


def camera_corner(camera, u: float, v: float, depth: float):
    return Vector(
        (
            camera["x"] + (u - camera["cx"]) / camera["fx"] * depth,
            CAMERA_Y - (v - camera["cy"]) / camera["fy"] * depth,
            CAMERA_OPTICAL_Z + depth,
        )
    )


def add_optical_layout(camera):
    side = camera["side"]
    source = Vector((camera["x"], CAMERA_Y, CAMERA_OPTICAL_Z))
    line_material = make_emission_material(
        f"{side}_FOV_line_material", camera["laser_color"], 2.2
    )
    far = 1000.0
    far_corners = (
        camera_corner(camera, 0.0, 0.0, far),
        camera_corner(camera, camera["width"], 0.0, far),
        camera_corner(camera, camera["width"], camera["height"], far),
        camera_corner(camera, 0.0, camera["height"], far),
    )
    objects = []
    for index, corner in enumerate(far_corners):
        objects.append(
            add_polyline(
                f"{side}_相机视锥边_{index + 1}",
                (source, corner),
                line_material,
                thickness=0.9,
            )
        )
    for depth in (300.0, 600.0, 1000.0):
        corners = (
            camera_corner(camera, 0.0, 0.0, depth),
            camera_corner(camera, camera["width"], 0.0, depth),
            camera_corner(camera, camera["width"], camera["height"], depth),
            camera_corner(camera, 0.0, camera["height"], depth),
        )
        rectangle = add_polyline(
            f"{side}_FOV_{int(depth)}mm",
            corners,
            line_material,
            thickness=1.2,
            cyclic=True,
        )
        rectangle["field_width_mm"] = depth * camera["width"] / camera["fx"]
        rectangle["field_height_mm"] = depth * camera["height"] / camera["fy"]
        objects.append(rectangle)

    axis_end = Vector((camera["x"], CAMERA_Y, CAMERA_OPTICAL_Z + 1050.0))
    objects.append(
        add_polyline(
            f"{side}_相机光轴",
            (source, axis_end),
            line_material,
            thickness=1.2,
        )
    )

    pitch = math.radians(LASER_PITCH_DEG)
    laser_source = Vector((camera["x"], LASER_Y, LASER_EXIT_Z))
    far_z = LASER_EXIT_Z + 1000.0
    far_y = LASER_Y + math.tan(pitch) * 1000.0
    half_width = math.tan(math.radians(30.0)) * 1000.0
    far_left = Vector((camera["x"] - half_width, far_y, far_z))
    far_right = Vector((camera["x"] + half_width, far_y, far_z))
    plane = add_polyline(
        f"{side}_{camera['laser']}_30deg扇面边界",
        (
            laser_source,
            far_left,
            far_right,
        ),
        line_material,
        thickness=1.3,
        cyclic=True,
    )
    plane["fan_half_angle_deg"] = 30.0
    plane["nominal_intersection_mm"] = NOMINAL_WORKING_DISTANCE
    objects.append(plane)
    objects.append(
        add_polyline(
            f"{side}_{camera['laser']}_扇面中心线",
            (
                laser_source,
                Vector((camera["x"], far_y, far_z)),
            ),
            line_material,
            thickness=1.0,
        )
    )

    nominal_y = LASER_Y + math.tan(pitch) * NOMINAL_WORKING_DISTANCE
    nominal_z = LASER_EXIT_Z + NOMINAL_WORKING_DISTANCE
    nominal_half_width = math.tan(math.radians(30.0)) * NOMINAL_WORKING_DISTANCE
    objects.append(
        add_polyline(
            f"{side}_600mm激光线",
            (
                Vector((camera["x"] - nominal_half_width, nominal_y, nominal_z)),
                Vector((camera["x"] + nominal_half_width, nominal_y, nominal_z)),
            ),
            line_material,
            thickness=2.0,
        )
    )
    return objects


def add_reference_floor(material):
    floor = add_box(
        "assembly_reference_floor",
        (500.0, 2.0, 500.0),
        (0.0, -65.0, 280.0),
        material,
    )
    return floor


def look_at(obj, target: Vector) -> None:
    direction = (target - obj.location).normalized()
    # Build the camera/light basis explicitly so world +Y always remains image-up.
    local_z = -direction
    world_up = Vector((0.0, 1.0, 0.0))
    local_x = world_up.cross(local_z).normalized()
    local_y = local_z.cross(local_x).normalized()
    rotation = Matrix((local_x, local_y, local_z)).transposed()
    obj.rotation_euler = rotation.to_euler()


def add_render_camera(name: str, location, target, lens=55.0):
    data = bpy.data.cameras.new(f"{name}_data")
    data.lens = lens
    data.sensor_width = 36.0
    data.clip_start = 1.0
    data.clip_end = 5000.0
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.location = location
    look_at(obj, Vector(target))
    return obj


def add_light(name: str, location, target, energy: float, size: float):
    bpy.ops.object.light_add(type="AREA", location=location)
    light = bpy.context.object
    light.name = name
    light.data.energy = energy
    light.data.shape = "DISK"
    light.data.size = size
    look_at(light, Vector(target))
    return light


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

    engines = [
        item.identifier
        for item in scene.bl_rna.properties["render"]
        .fixed_type.properties["engine"]
        .enum_items
    ]
    if "BLENDER_EEVEE_NEXT" in engines:
        scene.render.engine = "BLENDER_EEVEE_NEXT"

    scene.world.use_nodes = True
    background = scene.world.node_tree.nodes.get("Background")
    background.inputs["Color"].default_value = (0.018, 0.024, 0.038, 1.0)
    background.inputs["Strength"].default_value = 0.55

    scene["design_revision"] = "V1 concept"
    scene["coordinate_system"] = "X horizontal, Y up, Z optical forward"
    scene["pairing_left"] = "scanner_450：130万相机 + 450 nm线激光"
    scene["pairing_right"] = "scanner_650：160万相机 + 650 nm线激光"
    scene["camera_baseline_mm"] = CAMERA_BASELINE
    scene["camera_laser_vertical_baseline_mm"] = CAMERA_LASER_BASELINE
    scene["nominal_working_distance_mm"] = NOMINAL_WORKING_DISTANCE
    scene["laser_pitch_nominal_deg"] = LASER_PITCH_DEG
    scene["laser_pitch_range_deg"] = (
        f"{LASER_PITCH_MIN_DEG:.2f} to {LASER_PITCH_MAX_DEG:.2f}"
    )
    scene["laser_roll_deg"] = 0.0
    scene["overall_head_envelope_mm"] = "160 W x 90 H x 133 D including hoods"
    scene["manufacturing_warning"] = (
        "Adapter slots are provisional. Confirm the final welding-gun fixture "
        "hole pattern before machining."
    )


def export_stl(path: Path, objects):
    bpy.ops.object.select_all(action="DESELECT")
    valid = [obj for obj in objects if obj and obj.name in bpy.data.objects]
    for obj in valid:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = valid[0]
    bpy.ops.wm.stl_export(
        filepath=str(path.resolve()),
        export_selected_objects=True,
        apply_modifiers=True,
        global_scale=1.0,
        use_scene_unit=False,
    )


def render_scene(path: Path, camera, optical_objects, show_optics: bool):
    for obj in optical_objects:
        obj.hide_render = not show_optics
    bpy.context.scene.camera = camera
    bpy.context.scene.render.filepath = str(path.resolve())
    bpy.ops.render.render(write_still=True)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    clear_scene()
    configure_scene()
    bpy.context.preferences.filepaths.save_version = 0

    aluminum = make_principled_material(
        "6061_T6_阳极氧化铝", (0.19, 0.215, 0.245, 1.0), metallic=0.82, roughness=0.28
    )
    shell_material = make_principled_material(
        "外壳_黑色硬质阳极", (0.045, 0.055, 0.072, 1.0), metallic=0.65, roughness=0.26
    )
    camera_material = make_principled_material(
        "相机与激光机身", (0.055, 0.064, 0.078, 1.0), metallic=0.55, roughness=0.31
    )
    fixture_material = make_principled_material(
        "现有焊枪夹具", (0.085, 0.10, 0.125, 1.0), metallic=0.82, roughness=0.37
    )
    gasket_material = make_principled_material(
        "密封圈", (0.006, 0.008, 0.011, 1.0), metallic=0.0, roughness=0.72
    )
    lens_material = make_glass_material(
        "相机镜头玻璃", (0.03, 0.13, 0.20, 1.0)
    )
    clear_glass = make_principled_material(
        "相机防护玻璃_AR400_700", (0.18, 0.46, 0.64, 1.0), roughness=0.12
    )
    red_glass = make_principled_material(
        "650nm激光防护玻璃", (0.72, 0.035, 0.018, 1.0), roughness=0.12
    )
    blue_glass = make_principled_material(
        "450nm激光防护玻璃", (0.018, 0.16, 0.82, 1.0), roughness=0.12
    )
    floor_material = make_principled_material(
        "地面", (0.012, 0.017, 0.027, 1.0), roughness=0.86
    )
    white_emission = make_emission_material(
        "标注文字", (0.75, 0.82, 0.92, 1.0), 1.6
    )

    import_fixture(fixture_material)
    bridge = build_bridge(aluminum)
    add_reference_floor(floor_material)

    shell_exports = {}
    bezel_exports = {}
    for camera in CAMERAS:
        side = camera["side"]
        x = camera["x"]
        accent = make_principled_material(
            f"{side}_波长识别色",
            camera["accent_color"],
            metallic=0.48,
            roughness=0.32,
        )
        aperture = make_emission_material(
            f"{side}_激光出光", camera["laser_color"], 7.0
        )
        shell = build_pod_shell(side, x, shell_material)
        bezel = build_front_bezel(side, x, aluminum, accent)
        build_hoods(side, x, shell_material)
        build_camera_hardware(camera, camera_material, lens_material)
        build_laser_hardware(camera, camera_material, aperture, aluminum)
        build_windows(
            camera,
            clear_glass,
            red_glass if camera["laser_nm"] == 650 else blue_glass,
            gasket_material,
        )
        side_label = "左" if side == "left" else "右"
        label = f"{side_label}：{camera['profile']} / {camera['laser_nm']}nm"
        add_text(
            f"{side}_pair_label",
            label,
            (x, 44.0, 231.5),
            white_emission,
            size=4.2,
        )
        shell_exports[side] = shell
        bezel_exports[side] = bezel

    optical_objects = []
    for camera in CAMERAS:
        optical_objects.extend(add_optical_layout(camera))

    assembly_camera = add_render_camera(
        "Assembly_camera",
        (250.0, 105.0, 510.0),
        (0.0, -3.0, 178.0),
        lens=62.0,
    )
    optical_camera = add_render_camera(
        "Optical_camera",
        (1800.0, 1300.0, 1550.0),
        (0.0, 10.0, 700.0),
        lens=55.0,
    )
    front_camera = add_render_camera(
        "Front_camera",
        (0.0, 18.0, 610.0),
        (0.0, -3.0, 178.0),
        lens=72.0,
    )
    add_light(
        "Key_light",
        (350.0, 300.0, 500.0),
        (0.0, 0.0, 160.0),
        4500000.0,
        320.0,
    )
    add_light(
        "Fill_light",
        (-330.0, 120.0, 330.0),
        (0.0, 0.0, 155.0),
        2600000.0,
        260.0,
    )
    add_light(
        "Rim_light",
        (60.0, -260.0, 360.0),
        (0.0, 0.0, 150.0),
        3200000.0,
        240.0,
    )

    export_stl(OUTPUT_DIR / "bridge_adapter_v1.stl", [bridge])
    export_stl(OUTPUT_DIR / "pod_shell_left_v1.stl", [shell_exports["left"]])
    export_stl(OUTPUT_DIR / "pod_shell_right_v1.stl", [shell_exports["right"]])
    export_stl(OUTPUT_DIR / "front_bezel_left_v1.stl", [bezel_exports["left"]])
    export_stl(OUTPUT_DIR / "front_bezel_right_v1.stl", [bezel_exports["right"]])

    render_scene(ASSEMBLY_RENDER, assembly_camera, optical_objects, False)
    render_scene(FRONT_RENDER, front_camera, optical_objects, False)
    render_scene(OPTICAL_RENDER, optical_camera, optical_objects, True)
    for obj in optical_objects:
        obj.hide_render = False
    bpy.context.scene.camera = assembly_camera
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH.resolve()))

    print(f"Saved: {BLEND_PATH}")
    print(f"Saved: {ASSEMBLY_RENDER}")
    print(f"Saved: {FRONT_RENDER}")
    print(f"Saved: {OPTICAL_RENDER}")
    print(
        "Layout: left scanner_450 / 130万 + 450 nm; "
        "right scanner_650 / 160万 + 650 nm; "
        f"camera-laser baseline {CAMERA_LASER_BASELINE:.1f} mm"
    )
    print(
        f"Laser pitch: {LASER_PITCH_DEG:.2f} deg nominal; "
        f"{LASER_PITCH_MIN_DEG:.2f} to {LASER_PITCH_MAX_DEG:.2f} deg adjustment"
    )


if __name__ == "__main__":
    main()
