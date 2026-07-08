# Robot Offset LR Capture Design

## Goal

Implement a robot-control subpage workflow in `VizumScanGUI` that keeps the Vizum line laser fixed while the Fairino robot moves the camera along a selected axis of the current flange coordinate system. At every 2 mm offset sample, the app saves one left-eye image and one right-eye image.

## Existing Baseline

The command-line demo:

```bash
./build/VizumDynamicProfileOnceDemo --ip 192.168.1.105 --out data --gain 1 --wait-ms 5000
```

already proves the desired camera-side capture sequence:

- open the dust cover once
- begin laser detection/runtime
- set master trigger and LR-image capture state
- configure exposure, gain, frame rate, ROI/image mode
- enable the line laser
- call `VzNL_GetEyeImage()` to obtain left/right images

The GUI already has the same boundary available through `ScanWorker::saveLeftRightEyeImages()`. The robot page should reuse that worker path instead of directly calling Vizum SDK APIs.

## Architecture

### RobotControlWidget

`RobotControlWidget` owns the high-level workflow:

1. Read the current flange pose and current TCP pose at workflow start.
2. Let the user choose flange `X`, `Y`, or `Z` direction.
3. Let the user set total offset distance and sample step. The default step is 2 mm.
4. Convert the selected flange-axis direction into base coordinates using the start flange pose.
5. Generate offset samples: `0, 2, 4, ... final`, preserving the exact final endpoint even when the distance is not divisible by the step.
6. Move by `MoveL` to each target TCP pose. Only `x/y/z` change; `rx/ry/rz` stay equal to the start TCP pose.
7. After each point is reached, read the current flange pose for file naming.
8. Request left/right image saving from `ScanWorker`.
9. Stop cleanly if the user presses StopMotion, if robot motion fails, or if image saving fails.

The widget must not directly call Vizum SDK camera functions.

### RobotOffsetCapturePlanner

This small helper module owns deterministic math and naming:

- build offset samples
- compute selected flange-axis direction in base coordinates
- build target TCP poses
- format capture filename stems

This module is testable without robot or camera hardware.

### ScanWorker

`ScanWorker` owns camera capture:

- ensure the dust cover is open before capture
- configure the same capture-state bundle as the working demo
- save left/right grayscale images
- emit completion back to `RobotControlWidget`

The worker should not know about robot offsets or robot poses.

## Capture Behavior

The dust cover should not be closed and reopened at every 2 mm point. The workflow should ensure it is open once before capture and keep it open during the sequence.

Line laser behavior should be configurable:

- default: keep the line laser on for the whole sequence
- optional: turn the line laser on before each capture and off after each capture

The default is faster and more stable for continuous 2 mm sampling.

## File Naming

Each capture writes:

```text
{index}_X{x}_Y{y}_Z{z}_{timestamp}_left.png
{index}_X{x}_Y{y}_Z{z}_{timestamp}_right.png
```

Example:

```text
000001_X-129.052_Y-524.495_Z545.469_20260708_183839_836_left.png
000001_X-129.052_Y-524.495_Z545.469_20260708_183839_836_right.png
```

`X/Y/Z` are the current flange coordinates read after the robot reaches that sample point.

## Safety And Error Handling

Before real robot motion, the GUI must require:

- robot connected
- current flange/TCP pose read
- dry-run disabled only when the user explicitly allows real MoveL
- robot state suitable for MoveL: auto mode, no main/sub fault, no emergency stop, no safety stop, no collision

During motion:

- UI remains responsive
- StopMotion remains available
- MoveL runs in a background thread
- failures log both SDK error code and current robot state
- image-save failures stop the sequence and leave already saved files intact

## Tests

Hardware-independent tests should cover:

- offset sample generation for positive and negative distance
- final endpoint inclusion
- flange-axis conversion for simple poses
- target TCP preserves orientation
- filename stem formatting

Manual hardware verification should cover:

- dry-run prints expected target list and file paths
- real run moves to every 2 mm sample
- left/right images are saved at each reached point
- no repeated cover open/close happens during the sequence
- StopMotion interrupts the sequence
