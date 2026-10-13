---
name: moveit-teleop-new-arm
description: "Bring up VR teleoperation on a robot arm that this stack has never driven, using the moveit_teleop_* packages. Use when asked to add, port, integrate or configure teleoperation for a new arm, a new robot, or a second/another arm — e.g. 'get teleop working on our UR5e', 'add the Rizon to the teleop stack', 'make a teleop bringup for this MoveIt config'. Covers writing the robot's teleop.yaml, the scene, the RViz view, validating against a scripted device, and the hardware questions the stack cannot answer for itself. Not for tuning an arm that already runs (see doc/tuning.md) or diagnosing a broken session (doc/diagnostics.md)."
---

# Adding an arm to the teleop stack

**The deliverable is one config file in the arm's own MoveIt config package, not
a patch to these packages.** If you find yourself editing anything under
`moveit_teleop_*` to make a particular arm work, stop: that is a bug in the
stack, and the right move is to report it rather than special-case the arm.

Read [doc/new-arm.md](../../../moveit_teleop_bringup/doc/new-arm.md) for the
prose version and [doc/config-reference.md](../../../moveit_teleop_bringup/doc/config-reference.md)
for every setting. This file is the working procedure.

## 0. Check the prerequisites before writing anything

The arm needs a MoveIt config package that already plans: URDF/xacro, SRDF,
`joint_limits.yaml`, `kinematics.yaml`, a controller manager config, and a
`move_group` that comes up. If `ros2 launch <arm>_moveit_config demo.launch.py`
does not give you a planning arm in RViz, fix that first — none of this works
around a broken MoveIt config.

Confirm, and write down, four things from the model rather than assuming them:

```bash
ros2 param get /move_group robot_description_semantic | grep -i "group name"
ros2 control list_controllers            # which controller takes the arm's joints
ros2 topic list | grep -i joint_states   # candidates for MEASURED state
```

## 1. The SRDF needs a named posture

`ready_state` and `nominal_state` are SRDF `<group_state>` names. If the SRDF
has none, add one — and **do not use all-zeros without checking it.** On many
arms zero is a singularity; on some it also sits exactly on a joint limit, which
leaves the solver with no feasible direction to start from.

The stack checks this for you at startup and WARNs above 500, so you can simply
read the number off the first launch (step 4). For scale: FR3 `ready_pose` 8.2,
Rizon 10s `ready_pose` 8.4, Piper `ready_pose` 121.8, Piper mid-range 34111.

## 2. Copy the template

```bash
cp <ws>/src/moveit-teleop/moveit_teleop_bringup/config/samples/template_teleop.yaml \
   <arm>_moveit_config/config/teleop.yaml
```

Make sure the package installs its config directory, or the launch will not find
it:

```cmake
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
```

## 3. Fill in only what cannot be derived

Six values genuinely need a human: `move_group`, `ready_state`,
`nominal_state`, `controllers.arm`, `ik.rate_limit_scale`, and
`retargeting.per_solver.ik.translation_scale`.

**Never put any of these in the config** — they are read at runtime from the
robot model, and typing them creates a second source of truth that will drift:

- joint names, joint order, joint limits
- the base link, the tip link, the planning frame
- the gripper's open and closed joint values (SRDF group states matching
  `*open*` / `*close*`)
- the number of DOF

If you are typing a link name into `teleop.yaml`, you have taken a wrong turn.

Two values that depend on the arm's kinematics rather than taste:

| | |
|---|---|
| `ik.weight_regularization` | **6-DOF: 0.005. 7-DOF: 0.02.** On 6 joints a pose fully determines the configuration, so this term can only pull you off target (0.02 costs ~2.3 mm of steady-state error, 0.005 costs ~0.6 mm). On 7 joints a real nullspace exists and the posture term shapes the elbow for free. |
| `ik.rate_limit_scale` | Start at **0.2** on an arm you have never driven. This is the main safety ceiling. Raise it until `/ik_node/status` stops reporting `rate_limited` during normal motion, then stop — going past the point where the solver's own demand is met buys nothing and widens what a fault can command. |

## 4. Launch against a scripted device and read four lines

No headset, no hardware:

```bash
ros2 launch <arm>_moveit_config demo.launch.py            # or your sim bringup
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=<arm>_moveit_config device:=fake pattern:=circle
```

`ik_node` prints four lines. **Read all four before believing anything else:**

```
nominal posture from SRDF named state 'ready_pose', condition number N
speed from MoveIt joint_limits.yaml x rate_limit_scale S -> [...] rad/s
group='G' chain 'base' -> 'tip' (N joints) controller='C'
guards: joint limits from the model, jump limit N deg (retry xN), ...
```

1. Condition number — over 500 and the nominal posture is near-singular; pick a
   different named state.
2. The speed ceiling actually in force. If this is not what you intended, the
   config is not being read.
3. The chain resolved to the right tip link and controller.
4. Every guard, with anything **disabled printed at WARN**. A guard that reports
   off while it is in fact firing invalidates every measurement you take next.

A key the resolver does not know is an **error**, not a silent skip. If the
launch refuses a setting, that setting does not exist — look it up rather than
assuming it took effect.

## 5. RViz view

Copy the stack's `config/teleop.rviz` next to your `teleop.yaml`, name it in
`files.rviz`, and change exactly two things: `Fixed Frame` to your URDF root
link, and the TF display's `Filter (whitelist)` to `<your tip link>|teleop_target`.
Without the whitelist, every frame on the robot is drawn and the view is unusable.

## 6. The scene, and the one subtlety in it

`files.scene` names a YAML declaring boxes. A box that declares a `wall` face
gets a virtual wall **derived from that box's own bounds**, so the surface the
tool slides along is the surface the collision checker stops the arm at — they
cannot drift apart.

Two things routinely get this wrong:

- A table whose top reaches `z = 0` **intersects the robot base**, so every
  state is in collision and the arm never moves at all. Keep the surface below
  the mounting plane.
- `wall_clearance` must cover the **tracking error**, not just the gripper
  geometry. The wall is enforced on the *command*, and an open-loop arm trails
  it — measured on one 6-DOF arm: median 46 mm, p95 115 mm. Measure yours
  (compare the commanded pose against the driver's measured TCP pose) before
  trusting a small clearance.

## 7. The model files, for replay and datasets

```yaml
files:
  preview_urdf: <arm>.urdf.xacro
  preview_srdf: <arm>.srdf.xacro
  preview_xacro_args: '<exactly what the robot bringup passes>'
```

Required for episode replay and for dataset export, which runs **offline** with
no `move_group` to ask. They must describe the *same* robot the bringup built,
tool offset included, or the replay ghost is subtly the wrong shape and the
exported dataset's joint layout is wrong.

## 8. Then hardware — the questions the stack cannot answer

- **`joint_states_topic` must be MEASURED state.** Check whether the driver's
  topic is feedback or a command bus it subscribes to. On one arm
  `/control/joint_states` is the *command* bus, so recording it captures what
  the arm was told, not what it did, and TF tracks a pose the arm does not
  occupy.
- **`safety.hardware_gate`** — enable it if the driver advertises one, and
  decide **what kind of gate it is**, because the two kinds want opposite things
  while a planned trajectory executes:

  | Kind | `open_for_planning` | How to tell |
  |---|---|---|
  | motion enable | `true` (default) | nothing moves while it is shut |
  | streaming-mode select | **`false`** | the driver's trajectory goal callback rejects goals while streaming mode is on |

  **Read the driver's `FollowJointTrajectory` goal callback.** If it refuses
  goals while its servo/streaming mode is active, you need `false`. Getting it
  wrong is silent: the goal is rejected, the rejection reads as a planning
  failure, that is a fallback verdict, and every planned home quietly becomes an
  unchecked straight interpolation while still reporting itself as planned.
- **`safety.home_via_moveit`** — leave `true` unless the arm's MoveIt stack
  cannot execute trajectories.
- Re-measure `wall_clearance` against the tracking error you now have.

## 9. Verify before claiming it works

```bash
# the stack's own suites still pass
colcon test --packages-select moveit_teleop_core moveit_teleop_ik moveit_teleop_bringup
colcon test-result --verbose

# the config resolves, and to the values you meant
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=<arm>_moveit_config --show-args

# with the stack running
ros2 topic echo /mode_manager/status --once      # IDLE -> ALIGNED once input is healthy
ros2 topic echo /ik_node/status --once           # solve_time_p99 well under 1/solve_rate
```

Then drive it with `device:=fake` and watch for `rate_limited`, non-empty
`limited_joints`, or `refused_jumps` climbing during ordinary motion — each
means a limit is being hit rather than respected.

Check homing specifically, because it is the one motion the operator does not
drive: you want `planning a collision-checked path to '<ready_state>'`. If you
see `homing by direct interpolation (...)` the planner refused, and the named
error code says why — `START_STATE_IN_COLLISION` usually means the arm is parked
touching modelled geometry.

## What never needs doing

- Editing any `moveit_teleop_*` package.
- Writing a kinematics solver, a retargeter, or a device driver.
- Declaring joint names, limits, frames, controllers or gripper values.
- Converting the device's coordinate convention anywhere but
  `retargeting.device_convention`.
