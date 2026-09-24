# A3-T Handle Project — Codex Handoff Snapshot

> Update this file whenever the system architecture, safety boundary, or development priority
> changes. A new Codex session should read this file, `README.md`, `config/handle.json`, and
> `git log --oneline -10` before editing code.

## 1. Objective

Use an A3-T robot arm as a physical 6-DOF handle. An operator moves the robot end-effector by
hand around a captured center pose. The arm should eventually feel like a Cartesian virtual
spring-damper and provide the measured relative pose as a teleoperation/teaching command.

The primary task is the **MIT-based Handle mode**. Trajectory teaching and later replay/tracking
are secondary features.

## 2. Repository and dependency boundary

```text
A3-T/
├── a3t_handle/        # this repository: project-owned code
└── arm_control_sdk/   # vendor SDK: local dependency, NEVER commit here
```

- Required vendor package: Linux amd64 `arm_control_sdk-main.dev-1.0.260819`.
- Default controller IP: `10.42.0.101`.
- The vendor web UI must not command the same robot while this app is active.
- Build: `cmake -S . -B build && cmake --build build -j`.
- Build products and `runtime_logs/` are intentionally ignored by Git.

## 3. Vendor SDK facts used by this project

The only code allowed to invoke the vendor SDK is `src/arm/arm_worker.cpp`, on its dedicated Qt
worker thread. Do not call the SDK directly from Qt UI code.

Important APIs from `arm_control_sdk/carm_cobot.h`:

- High-level: `connect`, `disconnect`, `is_connected`, `set_ready`, `set_servo_enable`,
  `set_control_mode`, `get_status`, `get_joint_pos`, `get_joint_vel`, `get_cart_pose`,
  `set_drag_params`, `trajectory_teach`, `trajectory_recorder`, `check_teach`.
- Documented high-level modes: `0=Idle`, `1=Position`, `2=MIT`, `3=Drag`, `4=PF`.
- Low-level/MIT: `set_low_mode`, `low_refresh`, `low_get_forward_kine`,
  `low_get_jacobian`, `low_get_dynamics`, `low_mit_command`.

Do not invent duplicate SDK mode/state enums. `data_type_def.h` and comments elsewhere contain
some inconsistent mode labels; treat the explicit `carm_cobot.h` control-mode documentation as
the working reference until real-hardware verification.

## 4. Current implemented functions

### Desktop UI

- **Connection & Safety**: connect/disconnect, Ready, disable servo, emergency stop, speed,
  collision configuration, status/joint/pose display.
- **Drag & Teach**: enter/exit vendor drag, set drag factors, record/list/replay vendor
  trajectories.
- **Handle V0**: capture/reset center; configure translational/rotational stiffness, damping,
  scaling; display 6-DOF command and virtual wrench.
- **Tracking**: UI placeholder only; no tracking implementation.

### Logging

`SessionLogger` writes a new `runtime_logs/<timestamp>/` folder with `events.csv`,
`commands.csv`, `states.csv`, `performance.csv`, and, when active, `handle.csv`.

### Handle math

`src/math/handle_controller.cpp` implements:

1. Capture a center pose `[x,y,z,qx,qy,qz,qw]`.
2. Translation error and shortest axis-angle quaternion error.
3. Position/rotation deadband and first-order filter.
4. Finite-difference velocity.
5. A virtual Cartesian wrench:

   `W = -K * filtered_error - D * filtered_velocity`

Handle V0 only computes/displays/logs this wrench; its physical mode is vendor Drag, so it does
**not** yet apply the wrench as motor torque.

### MIT Handle path (implemented but hardware locked)

`src/control/mit_handle_controller.cpp` constructs, per joint:

`tau_cmd = gravity(q) + J(q)^T * W_virtual`

It clamps absolute torque and torque slew rate, and sends current `q`/`dq` as MIT reference with
configurable joint `kp`/`kd` (currently zero). `ArmWorker::runMitCycle()` uses SDK calls in this
order:

1. `low_refresh`
2. forward kinematics
3. Jacobian
4. dynamics
5. Handle math
6. torque mapping
7. `low_mit_command`

This is disabled by default: `config/handle.json` has `mit.allow_real_mit=false`. Do not change
it to `true` or test MIT on a real robot without the safety/test procedure below.

## 5. Current safety and verification status

The application builds successfully on the development desktop. **No MIT operation has been
verified on physical hardware.**

Known incomplete or unverified items:

- Exact SDK MIT enable/exit sequencing and accepted `kp=kd=0` behavior.
- Communication rate/jitter of all low-level SDK calls.
- Jacobian layout and coordinate frame; virtual wrench frame must match it.
- Gravity sign, robot inertial parameters, tool mass, and tool center of mass.
- Joint position/velocity/acceleration limits, Cartesian workspace limits, singularity handling,
  physical deadman switch, and robust watchdog.
- `mit.max_failures` exists in config but is not yet used; current failed low-level call stops
  MIT immediately.
- Qt `QTimer` is not hard real time. It is acceptable for initial low-risk tests, not proof of
  a 1 kHz controller.

## 6. Planned hardware-test order

1. Push/clone the repository and build it on the test PC with the matching SDK.
2. Confirm connection, Ready, servo disable, emergency stop, state polling, and vendor Drag.
3. Record timing distributions for low-level refresh, kinematics, Jacobian, dynamics, and MIT
   send; target a stable 100 Hz loop (`period_ms=10`) with timing margin.
4. With the arm secured, no payload or known payload, low torque limits, physical E-stop and a
   separate observer: validate low mode entry/exit and zero/near-zero torque behavior.
5. Validate gravity compensation direction and tool payload calibration.
6. Enable small Cartesian stiffness/damping and tune one axis at a time.
7. Add hard limits/deadman/watchdog before any normal hand-guided use.
8. Only then implement external teleoperation, trajectory tracking, adaptive impedance, or
   high-dynamic motions such as cloth/towel shaking.

## 7. Development rules for the next Codex session

- Preserve the one-way dependency: UI -> mode manager -> ArmWorker -> vendor SDK.
- Keep SDK calls out of UI and do not modify or commit `../arm_control_sdk`.
- Do not claim MIT safety/performance without physical measurement logs.
- Prefer configuration parameters and clear logging over hidden magic constants.
- Before modifying control math, add or update unit tests under `tests/` where practical.
- Before testing physical motion, keep `mit.allow_real_mit=false` unless the operator explicitly
  authorizes a controlled test.
