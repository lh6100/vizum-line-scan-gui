#!/usr/bin/env python3
"""Create a hash-locked manifest from the current two scanner profiles."""

import argparse
import datetime
import hashlib
import pathlib
import sys


ROLES = {
    "camera_650_intrinsics": "config/devices/scanner_650/hik_intrinsics.yaml",
    "camera_450_intrinsics": "config/devices/scanner_450/hik_intrinsics.yaml",
    "camera_650_handeye": "config/devices/scanner_650/hik_handeye.yaml",
    "camera_450_handeye": "config/devices/scanner_450/hik_handeye.yaml",
    "stereo_extrinsics": "config/hik_stereo.yaml",
    "laser_650_plane": "config/devices/scanner_650/hik_laser_plane.yaml",
    "laser_450_plane": "config/devices/scanner_450/hik_laser_plane.yaml",
}


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def quoted(value):
    return '"{}"'.format(str(value).replace("\\", "\\\\").replace('"', '\\"'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--package-id", required=True)
    parser.add_argument("--validated", action="store_true",
                        help="mark validated only after independent validation has passed")
    args = parser.parse_args()

    root = args.project_root.resolve()
    missing = [relative for relative in ROLES.values() if not (root / relative).is_file()]
    if missing:
        print("Missing required calibration files:\n  " + "\n  ".join(missing), file=sys.stderr)
        return 2
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "schema_version: 2",
        "package_id: {}".format(quoted(args.package_id)),
        "status: {}".format("validated" if args.validated else "candidate"),
        "created_at: {}".format(quoted(datetime.datetime.now(datetime.timezone.utc).isoformat())),
        "files:",
    ]
    for role, relative in ROLES.items():
        path = (root / relative).resolve()
        lines.append("  {}:".format(role))
        lines.append("    path: {}".format(quoted(path)))
        lines.append("    sha256: {}".format(digest(path)))
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    temporary.replace(output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
