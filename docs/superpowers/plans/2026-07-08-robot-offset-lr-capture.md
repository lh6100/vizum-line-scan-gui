# Robot Offset LR Capture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the `VizumScanGUI` robot-control workflow that moves the robot along a selected current-flange axis and saves left/right Vizum eye images every 2 mm.

**Architecture:** Keep robot motion orchestration in `RobotControlWidget`, deterministic path math and filename formatting in `RobotOffsetCapturePlanner`, and Vizum SDK capture in `ScanWorker`. The camera path reuses the working `VizumDynamicProfileOnceDemo` capture-state bundle through GUI worker slots instead of calling the camera SDK from the robot widget.

**Tech Stack:** C++17, Qt Widgets/signals/slots, CMake, Fairino C++ SDK, Vizum VzNL SDK.

## Global Constraints

- Do not directly call Vizum SDK camera APIs from `RobotControlWidget`.
- Only TCP `x/y/z` change during offset motion; `rx/ry/rz` remain equal to the start TCP pose.
- Use the current flange pose at workflow start to define flange `X/Y/Z` direction in base coordinates.
- Default sample step is 2 mm and the exact final endpoint must be included.
- Filename stem format is `{index}_X{x}_Y{y}_Z{z}_{timestamp}` with current flange `X/Y/Z` read after reaching the sample point.
- The dust cover is opened/ensured once; do not close and reopen it every 2 mm.
- Default line-laser behavior is keep-on for the whole sequence, with an option to turn on/off around each capture.
- Keep the UI responsive and StopMotion available during real motion.
- Real motion must be guarded by dry-run/allow-motion checks and robot state checks.

---

### Task 1: Offset Planner Pure Logic

**Files:**
- Create/modify: `src/robot_capture/RobotOffsetCapturePlanner.h`
- Create/modify: `src/robot_capture/RobotOffsetCapturePlanner.cpp`
- Test: `tests/test_robot_offset_capture_planner.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `std::vector<double> buildOffsetSamples(double totalOffsetMm, double stepMm)`
- Produces: `weld_geometry::Vec3 flangeAxisDirectionInBase(const weld_geometry::Pose6D& flangePose, CaptureAxis axis)`
- Produces: `weld_geometry::Pose6D offsetTcpPose(const weld_geometry::Pose6D& startTcp, const weld_geometry::Vec3& directionInBase, double offsetMm)`
- Produces: `std::string formatCaptureStem(int index, const weld_geometry::Pose6D& flangePose, const std::string& timestamp)`

- [ ] **Step 1: Write failing planner tests**

```cpp
ok = checkSamples(robot_capture::buildOffsetSamples(5.0, 2.0),
                  std::vector<double>{0.0, 2.0, 4.0, 5.0}) && ok;
ok = checkSamples(robot_capture::buildOffsetSamples(-5.0, 2.0),
                  std::vector<double>{0.0, -2.0, -4.0, -5.0}) && ok;
ok = near(target.x, 100.0) && near(target.y, 200.0) && near(target.z, 312.5) && ok;
ok = near(target.rx, tcp.rx) && near(target.ry, tcp.ry) && near(target.rz, tcp.rz) && ok;
ok = stem == "000007_X1.250_Y-2.500_Z3.750_20260708_170000_123" && ok;
```

- [ ] **Step 2: Verify planner tests fail before implementation**

Run: `cmake --build build --target RobotOffsetCapturePlannerTest -j2 && ./build/RobotOffsetCapturePlannerTest`

Expected before implementation: build fails because `RobotOffsetCapturePlanner` symbols or target are missing.

- [ ] **Step 3: Implement planner**

Implement the four functions listed in Interfaces. Use `poseToMatrix()` and `transformPoint()` from `TransformUtils` to compute flange-axis direction.

- [ ] **Step 4: Verify planner tests pass**

Run: `cmake --build build --target RobotOffsetCapturePlannerTest -j2 && ./build/RobotOffsetCapturePlannerTest`

Expected: `RobotOffsetCapturePlannerTest passed.`

---

### Task 2: Camera Worker Left/Right Capture Reuse

**Files:**
- Modify: `src/ScanWorker.h`
- Modify: `src/ScanWorker.cpp`
- Modify: `src/MainWindow.cpp`

**Interfaces:**
- Produces slot: `void saveLeftRightEyeImages(int requestId, QString leftPath, QString rightPath, int frameRate, int exposure, int gain, bool keepLaserOn)`
- Produces signal: `void leftRightEyeImagesSaved(int requestId, bool ok, QString leftPath, QString rightPath, QString desc)`
- Consumes SDK state and settings from the working demo: master trigger, ignored external trigger, hardware external enable off, LR image capture, full eye ROI for calibration image, exposure, gain, frame rate, line laser.

- [ ] **Step 1: Ensure ScanWorker exposes save-left-right slot and completion signal**

The slot must capture both images with `VzNL_GetEyeImage(m_hDevice, &leftFrame, &rightFrame, timeoutMs)`, save both paths, and emit completion with the same `requestId`.

- [ ] **Step 2: Configure capture like the demo**

Before `GetEyeImage`, call the existing runtime helpers:

```cpp
ensureCoverOpenForScan();
configureFullEyeRoiForCalibration(true);
configureEyeCalibrationRuntime();
configureLeftEyeImaging(frameRate, exposure, gain);
VzNL_SetOutputImageFormat(keVzNLImageType_GRAY);
```

- [ ] **Step 3: Preserve line-laser behavior**

If `keepLaserOn` is true, leave the laser enabled after capture. If false, turn the laser on before capture when needed and off after capture.

- [ ] **Step 4: Connect worker to robot widget**

In `MainWindow.cpp`, connect:

```cpp
connect(robotWidget, &RobotControlWidget::requestLeftRightEyeCapture,
        m_worker, &ScanWorker::saveLeftRightEyeImages, Qt::QueuedConnection);
connect(m_worker, &ScanWorker::leftRightEyeImagesSaved,
        robotWidget, &RobotControlWidget::onLeftRightEyeCaptureSaved, Qt::QueuedConnection);
```

---

### Task 3: Robot Widget Offset Capture Workflow

**Files:**
- Modify: `src/RobotControlWidget.h`
- Modify: `src/RobotControlWidget.cpp`
- Modify: `src/robot_client/FairinoRobotClient.h`
- Modify: `src/robot_client/FairinoRobotClient.cpp`

**Interfaces:**
- Consumes `RobotOffsetCapturePlanner` functions from Task 1.
- Consumes `requestLeftRightEyeCapture` signal and `onLeftRightEyeCaptureSaved` slot from Task 2.
- Produces GUI controls for axis, total distance, step, capture speed, FPS, exposure, gain, keep-laser-on.

- [ ] **Step 1: Add GUI controls**

Add a group named `线激光固定：机械臂偏移拍照` with axis combo (`法兰 X/Y/Z`), total distance spin, step spin default `2.0 mm`, speed spin, FPS spin, exposure spin default `260 us`, gain spin default `1`, keep-laser-on checkbox default checked, and an execute button.

- [ ] **Step 2: Add blocking capture request handshake**

Use a request id, mutex, condition variable, and completion slot so the robot motion thread can request an image pair and wait until `ScanWorker` emits the result.

- [ ] **Step 3: Add robot-state preflight**

Before true motion, read `GetRobotRealTimeState()` through `FairinoRobotClient`; block motion if mode is not auto, main/sub fault is nonzero, emergency stop/safety stop/collision is active, or robot is already running/dragging.

- [ ] **Step 4: Execute offset sequence in background thread**

For each offset sample: build the target TCP, log it, `MoveL` for `i > 0`, wait for motion done with polling, read current flange pose, format filenames, request left/right image saving, stop on any error.

- [ ] **Step 5: Keep StopMotion available**

Poll `m_stopRequested` during `waitRobotMotionDone`; send `StopMotion` once when requested and terminate the sequence with `-20 用户停止`.

---

### Task 4: Build And Verification

**Files:**
- No source ownership changes; verification only.

**Interfaces:**
- Consumes all previous tasks.

- [ ] **Step 1: Run planner test**

Run: `cmake --build build --target RobotOffsetCapturePlannerTest -j2 && ./build/RobotOffsetCapturePlannerTest`

Expected: `RobotOffsetCapturePlannerTest passed.`

- [ ] **Step 2: Build GUI**

Run: `cmake --build build --target VizumScanGUI -j2`

Expected: target reaches `[100%] Built target VizumScanGUI`.

- [ ] **Step 3: Check whitespace**

Run:

```bash
git diff --check -- CMakeLists.txt src/robot_capture src/ScanWorker.h src/ScanWorker.cpp src/MainWindow.cpp src/RobotControlWidget.h src/RobotControlWidget.cpp src/robot_client/FairinoRobotClient.h src/robot_client/FairinoRobotClient.cpp tests/test_robot_offset_capture_planner.cpp
```

Expected: no output.

- [ ] **Step 4: Manual hardware dry-run**

Run GUI and execute offset capture with dry-run enabled. Expected: log lists all target TCP poses and planned left/right output paths; no camera save request is sent.

- [ ] **Step 5: Manual hardware real-run**

Run GUI with robot connected, servo on, auto mode, dry-run disabled, and real MoveL allowed. Expected: every 2 mm sample moves, saves left/right images, and filenames include current flange coordinates.
