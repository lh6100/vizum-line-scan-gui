#!/usr/bin/env python3
"""Estimate the fixed camera/robot time offset from bidirectional scans."""

import argparse
import math
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compute camera.fixed_time_offset_us = "
            "(x_forward_mm - x_reverse_mm) / (2 * speed_mm_s)."
        )
    )
    parser.add_argument("--forward-mm", type=float, required=True,
                        help="step position measured during the forward scan (mm)")
    parser.add_argument("--reverse-mm", type=float, required=True,
                        help="step position measured during the reverse scan (mm)")
    parser.add_argument("--speed-mm-s", type=float, required=True,
                        help="positive scan-speed magnitude (mm/s)")
    args = parser.parse_args()

    values = (args.forward_mm, args.reverse_mm, args.speed_mm_s)
    if not all(math.isfinite(value) for value in values):
        parser.error("all inputs must be finite")
    if not 10.0 <= args.speed_mm_s <= 50.0:
        parser.error("--speed-mm-s must be in [10, 50] mm/s")

    delta_mm = args.forward_mm - args.reverse_mm
    offset_seconds = delta_mm / (2.0 * args.speed_mm_s)
    offset_us = offset_seconds * 1_000_000.0
    print(f"forward_reverse_difference_mm={delta_mm:.9f}")
    print(f"estimated_time_offset_s={offset_seconds:.12f}")
    print(f"suggested_fixed_time_offset_us={offset_us:.3f}")
    print("Set camera.fixed_time_offset_us to the signed value above, then validate with a new bidirectional scan.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
