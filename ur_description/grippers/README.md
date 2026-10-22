# Grippers

`urdf/ur_gripper.xacro` attaches any gripper in this folder to any UR arm's
`tool0`. Each gripper is a submodule `grippers/<name>_description/` holding a
`gripper.xacro` and its `meshes/`. Force-torque sensors live here too, as
`grippers/<name>_description/` with a `sensor.xacro`: with `sensor:=<name>`
the sensor is the first flange on `tool0` and the gripper mounts on it. Each
gripper's `gripper.xacro` offers the sensor itself (so it also works without
`ur_gripper.xacro`); `ur_gripper.xacro` adds it only for `gripper:=none`.

Clone with `git clone --recurse-submodules`, or run `git submodule update
--init` in an existing clone. Meshes are stored with Git LFS, so run
`git lfs install` once beforehand, or they check out as pointer files.

## Usage

From `ur_description/`:

```python
from xacrodoc import XacroDoc, packages
packages.look_in(['grippers'])  # only for grippers with package:// meshes (inspire)
XacroDoc.from_file('urdf/ur_gripper.xacro', subargs={
    'robot': 'ur3e', 'gripper': 'inspire', 'side': 'left',
    'offset_z': '0.012', 'offset_yaw': '${pi/2}',
}).to_urdf_string()
```

| Argument | Default | |
|---|---|---|
| `robot` | `ur10e` | any `urdf/inc/<robot>_macro.xacro` |
| `collision` | `cylinder` | arm collision type |
| `gripper` | `robotiq_hande` | `robotiq_hande`, `inspire`, or `none` for the bare arm |
| `gripper_collision` | `collision` | gripper collision type |
| `gripper_prefix` | empty | prepended to the gripper's link and joint names |
| `side` | `right` | `left` / `right`, for hands; other grippers ignore it |
| `adapter` | `true` | Inspire flange adapter between `tool0` and the hand (hand base 52 mm above `tool0`), the offsets then place the adapter; `false` mounts the hand directly |
| `offset_{x,y,z,roll,pitch,yaw}` | 0 | tool0 (or the sensor's `ft_frame`) → gripper base, i.e. the flange adapter |
| `sensor` | `none` | force-torque sensor as the first flange on `tool0`, before the gripper: `bota_sensone`; the gripper then mounts on its `ft_frame` (a gripper-specific arg, read by each `gripper.xacro`) |
| `sensor_yaw` | 0 | rotation of the sensor about the flange axis (where its cable points) |
| `gripper_file`, `gripper_macro`, `gripper_meshes_path` | from `gripper` | for a gripper outside this folder |
| `ur_path` | `.` | the `ur_description` folder, when not running from it |

`urdf/ur3_hande.xacro`, `ur5e_hande.xacro` and `ur3e_inspire.xacro` are the
same compositions with a fixed robot and gripper. `tests/test_ur_gripper.py`
checks `ur_gripper.xacro` with every gripper and sensor, and that `ur3e_inspire.xacro`
matches it. The sensor is described in `bota_sensone_description/README.md`.

## Interface

`gripper.xacro` defines

```xml
<xacro:macro name="<name>_gripper" params="prefix parent *origin meshes_path collision_type">
```

- **`prefix`**: prepended to every link and joint name. It is empty by
  default, so the gripper's own names must not clash with the arm's
  (`base_link`, `tool0`, ...): namespace them (`robotiq_hande_base_link`,
  `hand_base_link`).
- **`parent`**: the link to attach to, through one fixed joint that uses the
  `origin` block.
- **`meshes_path`**: the `meshes/` folder, for `file://${meshes_path}/...`
  URIs. Grippers with `package://` URIs ignore it.
- **`collision_type`**: `mesh`, `box`, `cylinder`, `sphere` or `capsule`;
  fall back to the closest one supported.

Gripper-specific options (like `side`) are `xacro:arg`s used as macro
defaults, so they pass through `subargs`.

## Adding a gripper

```bash
git submodule add <url> ur_description/grippers/<name>_description
```

The repository needs a `gripper.xacro` at its root. Use
`robotiq_hande_description` as a template for a new model, or
`inspire_description/gripper.xacro` as an adapter around a package that
already has its own macro.

## Adding a sensor

A folder (or submodule) `grippers/<name>_description/` with a `sensor.xacro`
defining

```xml
<xacro:macro name="<name>_sensor" params="prefix parent *origin meshes_path:=''">
```

which provides `${prefix}ft_sensor_link`, its body on `parent` (with the
`origin` block), and `${prefix}ft_frame`, the F/T frame on its tool face where
the gripper is mounted, with `package://<name>_description/...` meshes. Use
`bota_sensone_description` as a template. xacro cannot build a package name
from an arg, so each `gripper.xacro` (and `ur_gripper.xacro`, for
`gripper:=none`) includes the sensors it supports by name:

```xml
<xacro:if value="${'$(arg sensor)' == '<name>'}">
  <xacro:include filename="$(find <name>_description)/sensor.xacro"/>
</xacro:if>
```

A gripper supports sensors with the `sensor:=$(arg sensor) sensor_yaw:=$(arg sensor_yaw)`
macro params: it calls `${sensor}_sensor` on `parent` and mounts itself on
`${prefix}ft_frame` (see `robotiq_hande_description/gripper.xacro`).
