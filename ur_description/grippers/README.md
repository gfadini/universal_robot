# Grippers

`urdf/ur_gripper.xacro` attaches any gripper in this folder to any UR arm's
`tool0`. Each gripper is a submodule `grippers/<name>_description/` holding a
`gripper.xacro` and its `meshes/`.

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
| `offset_{x,y,z,roll,pitch,yaw}` | 0 | tool0 → gripper base, i.e. the flange adapter |
| `gripper_file`, `gripper_macro`, `gripper_meshes_path` | from `gripper` | for a gripper outside this folder |
| `ur_path` | `.` | the `ur_description` folder, when not running from it |

`urdf/ur3_hande.xacro`, `ur5e_hande.xacro` and `ur3e_inspire.xacro` are the
same compositions with a fixed robot and gripper. `tests/test_ur_gripper.py`
checks `ur_gripper.xacro` with every gripper, and that `ur3e_inspire.xacro`
matches it.

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
