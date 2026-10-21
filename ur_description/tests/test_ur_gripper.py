"""
Check that urdf/ur_gripper.xacro composes UR arms with the grippers in grippers/.

    python tests/test_ur_gripper.py      (or: pytest tests/test_ur_gripper.py)

- every gripper (and gripper:=none) builds and its meshes exist on disk;
- the Inspire hand switches between left and right with side:=, and rejects
  anything else;
- the gripper base sits at tool0 composed with the offset_* mount transform;
- the specialized urdf/ur3e_inspire.xacro gives the same model as ur_gripper.xacro.
Needs: numpy, pinocchio, xacrodoc. Each URDF is built in a fresh process,
since a failed xacro run leaves xacro's global state dirty.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

UR_DESCRIPTION = Path(__file__).resolve().parents[1]

BUILD = """
import json, sys
from xacrodoc import XacroDoc, packages
packages.look_in([sys.argv[1]])
print(XacroDoc.from_file(sys.argv[2], subargs=json.loads(sys.argv[3])).to_urdf_string())
"""

# gripper name -> base link of the gripper (with an empty gripper_prefix)
GRIPPERS = {"robotiq_hande": "robotiq_hande_base_link", "inspire": "hand_base_link"}


def build(xacro: str = "urdf/ur_gripper.xacro", cwd: Path = UR_DESCRIPTION, **subargs: str) -> str:
    """Build a xacro relative to cwd (includes in this package resolve against it)."""
    result = subprocess.run([sys.executable, "-c", BUILD, str(UR_DESCRIPTION / "grippers"), xacro, json.dumps(subargs)],
                            cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(result.stderr)
    return result.stdout


def tool0_to(urdf: str, frame: str) -> pin.SE3:
    model = pin.buildModelFromXML(urdf)
    data = model.createData()
    pin.framesForwardKinematics(model, data, pin.neutral(model))
    return data.oMf[model.getFrameId("tool0")].actInv(data.oMf[model.getFrameId(frame)])


def mesh_files(urdf: str) -> list[Path]:
    paths = []
    for mesh in ET.fromstring(urdf).iter("mesh"):
        uri = urlparse(mesh.get("filename"))
        path = Path(uri.netloc + uri.path) if uri.scheme == "file" else None
        assert path is not None, f"unresolved mesh URI {mesh.get('filename')}"
        paths.append(path if path.is_absolute() else UR_DESCRIPTION / path)
    return paths


def test_grippers_build_with_meshes() -> None:
    for gripper, base_link in GRIPPERS.items():
        urdf = build(robot="ur5e", gripper=gripper)
        links = {link.get("name") for link in ET.fromstring(urdf).iter("link")}
        assert base_link in links, f"{gripper}: no {base_link}"
        missing = [str(p) for p in mesh_files(urdf) if not p.is_file()]
        assert not missing, f"{gripper}: missing meshes {missing[:3]}"
        # An LFS pointer file instead of the mesh means `git lfs pull` is needed.
        assert all(p.stat().st_size > 200 for p in mesh_files(urdf)), f"{gripper}: meshes are LFS pointers"
    links = {link.get("name") for link in ET.fromstring(build(robot="ur5e", gripper="none")).iter("link")}
    assert "tool0" in links and not links & set(GRIPPERS.values())


def test_inspire_side() -> None:
    for side, other in (("left", "right"), ("right", "left")):
        urdf = build(robot="ur3e", gripper="inspire", side=side)
        joints = {joint.get("name") for joint in ET.fromstring(urdf).iter("joint")}
        assert f"hand_{side}_thumb_1_joint" in joints and not any(f"_{other}_" in j for j in joints), side
        assert "hand_pinch_tcp" in {link.get("name") for link in ET.fromstring(urdf).iter("link")}
        assert not any(other in p.name for p in mesh_files(urdf)), f"{side} hand uses {other} meshes"
    try:
        build(robot="ur3e", gripper="inspire", side="middle")
    except RuntimeError as error:
        assert "side must be left or right, got middle" in str(error), error
    else:
        raise AssertionError("side:=middle should fail")


def test_mount_offset() -> None:
    offset = {"offset_x": 0.01, "offset_y": -0.02, "offset_z": 0.035,
              "offset_roll": 0.1, "offset_pitch": -0.2, "offset_yaw": np.pi / 2}
    expected = pin.SE3(pin.rpy.rpyToMatrix(offset["offset_roll"], offset["offset_pitch"], offset["offset_yaw"]),
                       np.array([offset["offset_x"], offset["offset_y"], offset["offset_z"]]))
    for gripper, base_link in GRIPPERS.items():
        identity = tool0_to(build(robot="ur3e", gripper=gripper), base_link)
        assert np.allclose(identity.homogeneous, np.eye(4)), f"{gripper}: base not at tool0 with zero offset"
        mounted = tool0_to(build(robot="ur3e", gripper=gripper, **{k: repr(v) for k, v in offset.items()}), base_link)
        error = np.linalg.norm(pin.log6(expected.actInv(mounted)).vector)
        assert error < 1e-9, f"{gripper}: mount transform off by {error}"


def test_ur3e_inspire_matches_ur_gripper() -> None:
    def body(urdf: str) -> str:  # without the robot name and the header comment
        return re.sub(r'<robot name="[^"]+">', "", re.sub(r"<!--.*?-->", "", urdf, flags=re.S))

    mount = {"offset_z": "0.012", "offset_yaw": "1.5707963267948966"}
    for side in ("left", "right"):
        specialized = build("ur3e_inspire.xacro", UR_DESCRIPTION / "urdf", side=side, **mount)
        generic = build(robot="ur3e", gripper="inspire", side=side, **mount)
        assert body(specialized) == body(generic), f"ur3e_inspire.xacro ({side}) differs from ur_gripper.xacro"


if __name__ == "__main__":
    for test in (test_grippers_build_with_meshes, test_inspire_side, test_mount_offset,
                 test_ur3e_inspire_matches_ur_gripper):
        test()
        print(f"{test.__name__}: ok")
    print("All checks passed.")
