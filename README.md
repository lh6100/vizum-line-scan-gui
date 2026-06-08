# Vizum Line Scan GUI

Qt-based desktop GUI for Vizum VzNL SDK line-laser 3D reconstruction cameras. It wraps the SDK flow for device connection, dust-cover control, swing-motor line scan, and PLY point-cloud export.

## Features

- Connect, disconnect, and reboot a Vizum Ethernet laser robot camera.
- Read SDK, device, firmware, algorithm, hardware, and swing-motor version information.
- Open and close the camera dust cover.
- Run one swing-motor line scan and save the reconstructed point cloud as a `.ply` file.
- Repeat scans in the same session without reconnecting the camera.
- Keep SDK callbacks lightweight by deep-copying laser lines into a queue and writing PLY data from the worker thread.
- Load a PLY point cloud, click near a weld seam, fit a 3D line segment, and drag the segment endpoints for manual correction.

## SDK Layout

This repository includes the Vizum headers and Linux x64 SDK libraries needed to build the GUI:

```text
VizumScanGUI/
  SDK/
    VzNLSDK/
      Inc/
      Linux/x64/
  src/
```

The CMake file uses the bundled SDK first:

- headers: `SDK/VzNLSDK/Inc`
- libraries: `SDK/VzNLSDK/Linux/x64`

It also keeps compatibility with the original sibling layout at `../SDK/VzNLSDK` for local development.

## Build

Install Qt 5, VTK Qt support, PCL, Eigen, and CMake, then build:

```bash
sudo apt install cmake qtbase5-dev libvtk9-dev libvtk9-qt-dev libpcl-dev libeigen3-dev
```

```bash
cd VizumScanGUI
cmake -S . -B build
cmake --build build -j$(nproc)
```

Run:

```bash
./build/VizumScanGUI
```

## Usage

1. Connect the camera and network adapter.
2. Start the GUI.
3. Click `连接设备`.
4. Click `开盖` if the device supports dust-cover control.
5. Click `线扫建图并保存 PLY`, choose an output path, and wait for the scan to finish.
6. Click the scan button again to perform another scan and save another PLY file.
7. Click `关盖` and `关闭设备` when finished.

## Weld Seam Fitting

Open the `焊缝拟合` tab:

1. Click `加载 PLY` and select a scanned point cloud.
2. Click a point near the weld seam.
3. The tool uses a KDTree radius search around the picked point and fits a 3D line with RANSAC.
4. The red line is the fitted seam segment. The green sphere is the start point and the red sphere is the end point.
5. Drag either sphere to manually adjust the segment endpoint.
6. Copy the result to get the start/end coordinates in camera coordinates.

The viewer automatically downsamples the displayed points for interaction when a cloud is large, but fitting still uses the full-resolution point cloud.

## Implementation Notes

The scan path follows the SDK sequence:

```text
VzNL_Init
VzNL_ResearchDevice
VzNL_BindEthernetEye if needed
VzNL_OpenDevice
VzNL_BeginDetectLaser
VzNL_SetTriggerMode
VzNL_EnableSwingMotor
VzNL_SetSwingScanMode(Once)
VzNL_StartAutoDetectEx
laser callback: deep-copy line data only
worker thread: VzNL_WriteLaserFile
VzNL_StopAutoDetect
VzNL_CloseLaserFile
VzNL_EndDetectLaser
VzNL_CloseDevice
VzNL_Destroy
```

The callback deliberately avoids blocking SDK calls, file I/O, and UI work. A bounded queue protects the process if disk writing cannot keep up with camera output.
