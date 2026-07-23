#!/usr/bin/env python3
"""Open the generated FR5 twin explicitly in the Isaac Sim editor."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def _parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Launch Isaac Sim 6 and open the generated FR5 digital-twin stage."
    )
    parser.add_argument("--usd", required=True, type=Path, help="USD/USDA stage to open")
    parser.add_argument(
        "--experience",
        default="isaacsim.exp.full",
        help="Isaac Sim .kit experience path or name (default: isaacsim.exp.full)",
    )
    parser.add_argument(
        "--twin-headless",
        action="store_true",
        help="Run without a window; intended for launcher smoke tests",
    )
    parser.add_argument(
        "--twin-max-updates",
        type=int,
        default=0,
        metavar="N",
        help="Exit after N update frames; 0 keeps the editor open",
    )
    return parser.parse_known_args()


def _resolve_experience(value: str) -> Path:
    supplied = Path(value).expanduser()
    if supplied.is_file():
        return supplied.resolve()

    name = value if value.endswith(".kit") else f"{value}.kit"
    search_roots = [os.environ.get("EXP_PATH"), os.environ.get("ISAAC_PATH")]
    for root in search_roots:
        if not root:
            continue
        for candidate in (Path(root) / name, Path(root) / "apps" / name):
            if candidate.is_file():
                return candidate.resolve()
    raise FileNotFoundError(
        f"Isaac Sim experience not found: {value!r}; set ISAAC_SIM_EXPERIENCE to a valid .kit file"
    )


def main() -> int:
    args, kit_args = _parse_args()
    usd_path = args.usd.expanduser().resolve()
    if not usd_path.is_file():
        print(f"[FR5 Twin] USD does not exist: {usd_path}", file=sys.stderr)
        return 2
    if args.twin_max_updates < 0:
        print("[FR5 Twin] --twin-max-updates must be non-negative", file=sys.stderr)
        return 2

    # Importing isaacsim initializes its package paths, including EXP_PATH.
    # Omniverse/pxr modules must only be imported after SimulationApp starts.
    from isaacsim import SimulationApp

    try:
        experience = _resolve_experience(args.experience)
    except FileNotFoundError as exc:
        print(f"[FR5 Twin] {exc}", file=sys.stderr)
        return 2

    print(f"[FR5 Twin] Experience: {experience}", flush=True)
    print(f"[FR5 Twin] Stage to open after application startup: {usd_path}", flush=True)
    simulation_app = SimulationApp(
        {
            "headless": args.twin_headless,
            # Let the Full experience finish creating its initial empty stage.
            # The twin replaces that temporary stage below, after startup.
            "create_new_stage": True,
            "fast_shutdown": True,
            "display_options": 3286,
            "renderer": "RaytracedLighting",
            "extra_args": kit_args,
        },
        experience=str(experience),
    )

    try:
        import omni.usd

        usd_context = omni.usd.get_context()
        print(f"[FR5 Twin] Opening stage now: {usd_path}", flush=True)
        if not usd_context.open_stage(str(usd_path)):
            print(f"[FR5 Twin] Isaac Sim could not open stage: {usd_path}", file=sys.stderr)
            return 3
        simulation_app.update()
        stage = usd_context.get_stage()
        if stage is None:
            print("[FR5 Twin] Isaac Sim started, but no USD stage was opened.", file=sys.stderr)
            return 3

        root_layer_path = Path(stage.GetRootLayer().realPath).resolve()
        if root_layer_path != usd_path:
            print(
                "[FR5 Twin] Wrong stage is open: "
                f"expected {usd_path}, got {root_layer_path}",
                file=sys.stderr,
            )
            return 3

        robot_path = "/World/FR5"
        if not stage.GetPrimAtPath(robot_path).IsValid():
            print(f"[FR5 Twin] Required robot prim is missing: {robot_path}", file=sys.stderr)
            return 3

        print(f"[FR5 Twin] Stage opened successfully; robot found at {robot_path}", flush=True)

        # Let the editor and Hydra finish creating the active viewport before
        # framing the referenced robot. Framing also avoids a valid model being
        # off-screen because of a stale per-user viewport camera.
        if not args.twin_headless:
            from omni.kit.viewport.utility import frame_viewport_prims, get_active_viewport

            for _ in range(12):
                simulation_app.update()
            viewport = get_active_viewport()
            if viewport is None:
                print(
                    "[FR5 Twin] Warning: stage is open, but no active viewport was found; "
                    "select /World/FR5 and press F.",
                    file=sys.stderr,
                )
            elif frame_viewport_prims(viewport, prims=[robot_path]):
                print("[FR5 Twin] Viewport framed on /World/FR5", flush=True)
            else:
                print(
                    "[FR5 Twin] Warning: automatic framing failed; "
                    "select /World/FR5 and press F.",
                    file=sys.stderr,
                )

        update_count = 0
        while simulation_app.is_running():
            if args.twin_max_updates and update_count >= args.twin_max_updates:
                break
            simulation_app.update()
            update_count += 1
        return 0
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
