#!/usr/bin/env python3
"""Add or replace ASCII PLY RGB values according to point height."""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from pathlib import Path


COLOR_STOPS = {
    "turbo": (
        (0.00, (48, 18, 59)),
        (0.13, (67, 97, 209)),
        (0.25, (32, 183, 233)),
        (0.38, (47, 238, 174)),
        (0.50, (164, 252, 60)),
        (0.63, (238, 208, 35)),
        (0.75, (251, 126, 32)),
        (0.88, (204, 45, 12)),
        (1.00, (122, 4, 3)),
    ),
    "viridis": (
        (0.00, (68, 1, 84)),
        (0.25, (59, 82, 139)),
        (0.50, (33, 145, 140)),
        (0.75, (94, 201, 98)),
        (1.00, (253, 231, 37)),
    ),
    "gray": ((0.00, (0, 0, 0)), (1.00, (255, 255, 255))),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replace a PLY point cloud's RGB values with a height colormap."
    )
    parser.add_argument("input", type=Path, help="input ASCII PLY")
    parser.add_argument(
        "-o", "--output", type=Path, help="output PLY (default: INPUT_height.ply)"
    )
    parser.add_argument(
        "--axis", choices=("x", "y", "z"), default="z", help="height axis (default: z)"
    )
    parser.add_argument(
        "--colormap",
        choices=tuple(COLOR_STOPS),
        default="turbo",
        help="color palette (default: turbo)",
    )
    parser.add_argument(
        "--percentile",
        nargs=2,
        type=float,
        metavar=("LOW", "HIGH"),
        default=(1.0, 99.0),
        help="color limits as percentiles (default: 1 99; use 0 100 for full range)",
    )
    parser.add_argument(
        "--min-height", type=float, help="fixed lower color limit; overrides LOW percentile"
    )
    parser.add_argument(
        "--max-height", type=float, help="fixed upper color limit; overrides HIGH percentile"
    )
    parser.add_argument(
        "--invert", action="store_true", help="map low points to high palette colors"
    )
    parser.add_argument(
        "--rgb-only",
        action="store_true",
        help=(
            "write only x/y/z and RGB properties; this prevents viewers such as "
            "CloudCompare from displaying intensity/confidence instead of RGB"
        ),
    )
    parser.add_argument(
        "--frame-id",
        help="set or replace the PLY 'comment frame_id' metadata",
    )
    return parser.parse_args()


def read_header(stream) -> tuple[list[bytes], list[str], int]:
    header: list[bytes] = []
    vertex_properties: list[str] = []
    vertex_count = 0
    current_element = None

    while True:
        line = stream.readline()
        if not line:
            raise ValueError("unexpected EOF before end_header")
        header.append(line)
        text = line.decode("ascii").strip()
        parts = text.split()
        if parts[:1] == ["format"] and parts[1:3] != ["ascii", "1.0"]:
            raise ValueError("only 'format ascii 1.0' PLY files are supported")
        if parts[:1] == ["element"] and len(parts) == 3:
            current_element = parts[1]
            if current_element == "vertex":
                vertex_count = int(parts[2])
        elif parts[:1] == ["property"] and current_element == "vertex":
            if len(parts) < 3 or parts[1] == "list":
                raise ValueError("list properties are not supported on vertices")
            vertex_properties.append(parts[-1])
        if text == "end_header":
            break

    if not vertex_count:
        raise ValueError("PLY contains no vertices")
    return header, vertex_properties, vertex_count


def header_with_rgb(
    header: list[bytes], properties: list[str], rgb_only: bool
) -> tuple[list[bytes], list[str]]:
    """Return a vertex header with RGB, optionally dropping non-display scalars."""
    colors = ("red", "green", "blue")
    if rgb_only:
        required_coordinates = ("x", "y", "z")
        missing_coordinates = [
            name for name in required_coordinates if name not in properties
        ]
        if missing_coordinates:
            raise ValueError(
                "--rgb-only requires vertex properties: "
                + ", ".join(missing_coordinates)
            )
        output_properties = [
            name for name in properties if name in required_coordinates or name in colors
        ]
    else:
        output_properties = list(properties)

    missing_colors = [name for name in colors if name not in output_properties]
    output_properties.extend(missing_colors)

    output_header: list[bytes] = []
    in_vertex_element = False
    inserted = False
    for line in header:
        parts = line.decode("ascii").strip().split()
        if parts[:2] == ["element", "vertex"]:
            in_vertex_element = True
        elif (
            rgb_only
            and in_vertex_element
            and parts[:1] == ["property"]
            and parts[-1] not in output_properties
        ):
            continue
        elif parts[:1] == ["element"] and in_vertex_element:
            for name in missing_colors:
                output_header.append(f"property uchar {name}\n".encode("ascii"))
            inserted = True
            in_vertex_element = False
        elif parts[:1] == ["end_header"] and in_vertex_element and not inserted:
            for name in missing_colors:
                output_header.append(f"property uchar {name}\n".encode("ascii"))
            inserted = True
            in_vertex_element = False
        output_header.append(line)

    if not inserted:
        raise ValueError("could not locate the vertex property block in PLY header")
    return output_header, output_properties


def header_with_frame_id(header: list[bytes], frame_id: str | None) -> list[bytes]:
    if not frame_id:
        return header
    output: list[bytes] = []
    replaced = False
    for line in header:
        parts = line.decode("ascii").strip().split()
        if parts[:2] == ["comment", "frame_id"]:
            if not replaced:
                output.append(f"comment frame_id {frame_id}\n".encode("ascii"))
                replaced = True
            continue
        if parts[:1] == ["end_header"] and not replaced:
            output.append(f"comment frame_id {frame_id}\n".encode("ascii"))
            replaced = True
        output.append(line)
    return output


def percentile(sorted_values: list[float], percent: float) -> float:
    position = (len(sorted_values) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[lower]
    fraction = position - lower
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def color_at(value: float, stops) -> tuple[int, int, int]:
    value = min(1.0, max(0.0, value))
    for (p0, c0), (p1, c1) in zip(stops, stops[1:]):
        if value <= p1:
            fraction = (value - p0) / (p1 - p0)
            return tuple(round(a + (b - a) * fraction) for a, b in zip(c0, c1))
    return stops[-1][1]


def main() -> int:
    args = parse_args()
    low_percent, high_percent = args.percentile
    if not 0 <= low_percent < high_percent <= 100:
        raise ValueError("--percentile must satisfy 0 <= LOW < HIGH <= 100")
    output = args.output or args.input.with_name(f"{args.input.stem}_height.ply")
    if output.resolve() == args.input.resolve():
        raise ValueError("input and output paths must be different")

    with args.input.open("rb") as source:
        header, properties, vertex_count = read_header(source)
        required = {args.axis}
        missing = required.difference(properties)
        if missing:
            raise ValueError(f"missing vertex properties: {', '.join(sorted(missing))}")
        height_index = properties.index(args.axis)
        heights = []
        for point_number in range(vertex_count):
            fields = source.readline().split()
            if len(fields) != len(properties):
                raise ValueError(f"invalid vertex data at point {point_number + 1}")
            heights.append(float(fields[height_index]))

    heights.sort()
    lower = args.min_height
    if lower is None:
        lower = percentile(heights, low_percent)
    upper = args.max_height
    if upper is None:
        upper = percentile(heights, high_percent)
    if not lower < upper:
        raise ValueError(f"color limits must satisfy min < max (got {lower} and {upper})")

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = None
    try:
        with args.input.open("rb") as source:
            header, properties, vertex_count = read_header(source)
            output_header, output_properties = header_with_rgb(
                header, properties, args.rgb_only
            )
            output_header = header_with_frame_id(output_header, args.frame_id)
            red_index = output_properties.index("red")
            green_index = output_properties.index("green")
            blue_index = output_properties.index("blue")
            height_index = properties.index(args.axis)
            with tempfile.NamedTemporaryFile(
                mode="wb", dir=output.parent, prefix=f".{output.name}.", delete=False
            ) as target:
                temporary_name = target.name
                target.writelines(output_header)
                for point_number in range(vertex_count):
                    fields = source.readline().split()
                    if len(fields) != len(properties):
                        raise ValueError(f"invalid vertex data at point {point_number + 1}")
                    output_fields = [
                        fields[properties.index(name)] if name in properties else b"0"
                        for name in output_properties
                    ]
                    normalized = (float(fields[height_index]) - lower) / (upper - lower)
                    if args.invert:
                        normalized = 1.0 - normalized
                    red, green, blue = color_at(normalized, COLOR_STOPS[args.colormap])
                    output_fields[red_index] = str(red).encode()
                    output_fields[green_index] = str(green).encode()
                    output_fields[blue_index] = str(blue).encode()
                    target.write(b" ".join(output_fields) + b"\n")
        os.chmod(temporary_name, args.input.stat().st_mode & 0o777)
        os.replace(temporary_name, output)
    except Exception:
        if temporary_name:
            Path(temporary_name).unlink(missing_ok=True)
        raise

    print(f"Wrote {vertex_count:,} points to {output}")
    print(f"{args.axis.upper()} color range: {lower:.6g} .. {upper:.6g}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
