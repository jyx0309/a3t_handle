# A3-T Handle Project

Qt/C++ desktop application that uses an A3-T robot arm as a 6-DOF physical handle. The project
keeps the vendor SDK separate and places all SDK access in `src/arm/arm_worker.cpp`.

## Repository scope

This repository contains only project-owned source code, configuration, and documentation. It
does **not** contain the vendor SDK, firmware packages, CMake build output, or runtime logs.

Tested dependency version: `arm_control_sdk-main.dev-1.0.260819` for Linux amd64.

Place the unpacked vendor SDK beside this repository:

```text
A3-T/
├── a3t_handle/        # this repository
└── arm_control_sdk/   # vendor dependency; not committed
```

Or configure a different SDK path when building with
`-DARM_CONTROL_SDK_DIR=/absolute/path/to/arm_control_sdk`.

## Current capabilities

- Connection/status display, Ready, servo disable, emergency stop, speed and collision settings
- Vendor Drag mode, drag parameter tuning, trajectory recording and replay
- Session logging for events, commands, state polls, timing, and Handle data
- Handle V0: center capture, 6-DOF relative pose, deadband/filter/scale, and virtual wrench display
- MIT Handle control path: Cartesian virtual spring-damper, SDK kinematics/dynamics/Jacobian,
  and torque mapping. It is disabled by default through `mit.allow_real_mit=false` until it has
  been validated with the real arm.

## Source layout

```text
src/
  arm/       Sole vendor-SDK access layer and robot worker thread
  control/   Application mode FSM and MIT torque-command mapping
  logging/   Per-session CSV logger
  math/      Pose/quaternion and virtual spring-damper calculations
  safety/    Safety state and transition checks
  ui/        Qt desktop interface
config/      Connection and controller parameters
```

## Build

Requirements: Linux amd64, CMake >= 3.16, C++17 compiler, Qt5 Widgets, and the vendor SDK above.

```bash
cmake -S . -B build
cmake --build build -j
./build/a3t_handle
```

Robot commands affect real hardware. Clear the workspace, keep the physical emergency stop
reachable, and avoid simultaneously commanding the arm from the vendor web UI.
