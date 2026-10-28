"""Validate the standalone bundle, including after copying it elsewhere."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise ValueError(message)


def resolve_resource(filename, parent):
    for prefix in ("model://go2_piper_d435_assets/", "package://go2_piper_d435_assets/"):
        if filename.startswith(prefix):
            return ROOT / filename.removeprefix(prefix)
    require(not Path(filename).is_absolute(), f"Absolute runtime dependency: {filename}")
    return parent / filename


def static_checks():
    count = 0
    for line in (ROOT / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ", 1)
        path = ROOT / name
        require(path.is_file(), f"Missing file: {name}")
        require(hashlib.sha256(path.read_bytes()).hexdigest() == digest, f"Changed file: {name}")
        require(not path.is_symlink(), f"Symlink: {name}")
        count += 1
    dependencies = 0
    for path in ROOT.rglob("*"):
        if path.suffix not in (".xml", ".urdf", ".sdf", ".config"):
            continue
        root = ET.parse(path).getroot()
        for element in root.iter():
            filenames = []
            if element.tag == "mesh":
                filenames += [element.get(k) for k in ("file", "filename") if element.get(k)]
            if element.tag == "include" and element.get("file"):
                filenames.append(element.get("file"))
            if element.tag == "uri" and element.text:
                if element.text == "model://go2_piper_d435_assets":
                    require((ROOT / "model.config").is_file(), "Missing model.config")
                    continue
                filenames.append(element.text)
            for filename in filenames:
                target = resolve_resource(filename, path.parent).resolve()
                require(target.is_relative_to(ROOT), f"Dependency escapes bundle: {filename}")
                require(target.is_file(), f"Missing dependency: {target}")
                dependencies += 1
    for path in ROOT.rglob("*.obj"):
        require(
            not any(line.lstrip().startswith(("mtllib ", "usemtl ")) for line in path.read_text().splitlines()),
            f"External material reference: {path}",
        )
    urdf = ET.parse(ROOT / "urdf/go2_piper_d435.urdf").getroot()
    moving = [j for j in urdf.findall("joint") if j.get("type") != "fixed"]
    require(len(moving) == 20, "Expected 20 movable URDF joints")
    require(len([j for j in moving if j.get("type") == "prismatic"]) == 2, "Expected two sliding fingers")
    mimic = urdf.find("joint[@name='joint8']/mimic")
    require(mimic.get("joint") == "joint7" and float(mimic.get("multiplier")) == -1, "Wrong gripper coupling")
    sdf = ET.parse(ROOT / "gazebo/model.sdf").getroot().find("model")
    require(len(sdf.findall("link")) == 25, "Expected 25 SDF rigid bodies")
    require(len([j for j in sdf.findall("joint") if j.get("type") != "fixed"]) == 20, "Wrong SDF joints")
    require(len(sdf.findall(".//sensor[@type='rgbd_camera']")) == 2, "Expected two RGB-D sensors")
    for xml in (urdf, sdf):
        joints = xml.findall("joint")
        children = [j.find("child").get("link") if xml.tag == "robot" else j.findtext("child") for j in joints]
        require(len(set(children)) == len(children), "A body has multiple parents")
        names = {link.get("name") for link in xml.findall("link")}
        require(names - set(children) == {"base"}, "Expected a floating base link as sole root")
    print(
        json.dumps(
            {
                "integrity": "PASS",
                "files": count,
                "local_resource_references": dependencies,
                "urdf_and_sdf_moving_joints": 20,
                "sdf_cameras": 2,
            }
        )
    )


def physics_checks():
    import mujoco
    import numpy as np
    from scipy.spatial.transform import Rotation

    def vector(text):
        return np.fromstring(text or "0 0 0", sep=" ")

    def transform(xyz, rpy):
        value = np.eye(4)
        value[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        value[:3, 3] = xyz
        return value

    def origin(element):
        if element is None:
            return np.eye(4)
        return transform(vector(element.get("xyz")), vector(element.get("rpy")))

    def sdf_pose(element):
        xyzrpy = vector(element.text)
        return transform(xyzrpy[:3], xyzrpy[3:])

    model = mujoco.MjModel.from_xml_path(str(ROOT / "mujoco/go2_piper_d435.xml"))
    scene = mujoco.MjModel.from_xml_path(str(ROOT / "mujoco/scene.xml"))
    require((model.nq, model.nv, model.nu, model.ncam) == (27, 26, 19, 2), "Unexpected MJCF dimensions")
    descriptions = []
    for filename in ("go2_piper_d435.urdf", "go2_piper_d435.ros.urdf"):
        xml = ET.parse(ROOT / "urdf" / filename).getroot()
        joints = {j.find("child").get("link"): j for j in xml.findall("joint")}
        descriptions.append((filename, xml, joints))
    sdf = ET.parse(ROOT / "gazebo/model.sdf").getroot().find("model")
    sjoints = {j.findtext("child"): j for j in sdf.findall("joint")}
    slinks = {link.get("name"): link for link in sdf.findall("link")}
    rng = np.random.default_rng(435)
    max_error = 0.0
    for sample in range(9):
        data = mujoco.MjData(model)
        data.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        positions = {}
        for j in range(1, model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            lo, hi = model.jnt_range[j]
            positions[name] = float(rng.uniform(lo, hi)) if sample else 0.0
        positions["joint8"] = -positions["joint7"]
        for name, q in positions.items():
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[j]] = q
        mujoco.mj_forward(model, data)
        for label, _, joints in descriptions:
            actual = {"base": np.eye(4)}
            pending = dict(joints)
            while pending:
                progress = False
                for child, joint in list(pending.items()):
                    parent = joint.find("parent").get("link")
                    if parent not in actual:
                        continue
                    motion = np.eye(4)
                    if joint.get("type") != "fixed":
                        axis = vector(joint.find("axis").get("xyz"))
                        q = positions[joint.get("name")]
                        if joint.get("type") == "prismatic":
                            motion[:3, 3] = axis * q
                        else:
                            motion[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
                    actual[child] = actual[parent] @ origin(joint.find("origin")) @ motion
                    del pending[child]
                    progress = True
                require(progress, f"Cyclic or unresolved topology in {label}")
            for b in range(1, model.nbody):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
                expected = np.eye(4)
                expected[:3, 3] = data.xpos[b]
                expected[:3, :3] = data.xmat[b].reshape(3, 3)
                error = float(np.abs(actual[name] - expected).max())
                max_error = max(max_error, error)
                require(error < 2e-9, f"{label} FK differs for {name}: {error}")

        actual = {"base": np.eye(4)}
        for b in range(2, model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            joint, link = sjoints[name], slinks[name]
            parent = joint.findtext("parent")
            require(link.find("pose").get("relative_to") == parent, f"Wrong SDF pose reference for {name}")
            require(joint.find("pose").get("relative_to") == name, f"Wrong joint pose frame for {name}")
            require(np.allclose(sdf_pose(joint.find("pose")), np.eye(4)), "Joint offset unsupported")
            motion = np.eye(4)
            if joint.get("type") != "fixed":
                q = positions[joint.get("name")]
                axis = vector(joint.findtext("axis/xyz"))
                if joint.get("type") == "prismatic":
                    motion[:3, 3] = axis * q
                else:
                    motion[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
            actual[name] = actual[parent] @ sdf_pose(link.find("pose")) @ motion
            error = max(
                float(np.abs(actual[name][:3, 3] - data.xpos[b]).max()),
                float(np.abs(actual[name][:3, :3] - data.xmat[b].reshape(3, 3)).max()),
            )
            max_error = max(max_error, error)
            require(error < 2e-9, f"SDF FK differs for {name}: {error}")

    for b in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        for _, xml, _ in descriptions:
            inertial = xml.find(f"link[@name='{name}']/inertial")
            require(
                abs(float(inertial.find("mass").get("value")) - model.body_mass[b]) < 1e-12, f"Mass mismatch: {name}"
            )
            require(np.allclose(origin(inertial.find("origin"))[:3, 3], model.body_ipos[b], atol=1e-12), "COM mismatch")
            r = origin(inertial.find("origin"))[:3, :3]
            values = [float(inertial.find("inertia").get(key)) for key in ("ixx", "iyy", "izz")]
            np.testing.assert_allclose(values, model.body_inertia[b], atol=1e-14)
            expected_r = Rotation.from_quat(model.body_iquat[b][[1, 2, 3, 0]]).as_matrix()
            np.testing.assert_allclose(
                r @ np.diag(values) @ r.T, expected_r @ np.diag(model.body_inertia[b]) @ expected_r.T, atol=1e-12
            )
        inertial = slinks[name].find("inertial")
        require(abs(float(inertial.findtext("mass")) - model.body_mass[b]) < 1e-12, f"SDF mass mismatch: {name}")
        values = vector(" ".join(inertial.findtext(f"inertia/{key}") for key in ("ixx", "iyy", "izz")))
        np.testing.assert_allclose(values, model.body_inertia[b], atol=1e-14)
        np.testing.assert_allclose(sdf_pose(inertial.find("pose"))[:3, 3], model.body_ipos[b], atol=1e-12)
        r = sdf_pose(inertial.find("pose"))[:3, :3]
        expected_r = Rotation.from_quat(model.body_iquat[b][[1, 2, 3, 0]]).as_matrix()
        np.testing.assert_allclose(
            r @ np.diag(values) @ r.T, expected_r @ np.diag(model.body_inertia[b]) @ expected_r.T, atol=1e-12
        )
    data = mujoco.MjData(scene)
    mujoco.mj_resetDataKeyframe(scene, data, 0)
    for _ in range(50):
        mujoco.mj_step(scene, data)
    require(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all(), "Non-finite state during short smoke run")
    print(
        json.dumps(
            {
                "physics": "PASS",
                "mujoco": mujoco.__version__,
                "fk_poses": 9,
                "max_fk_matrix_error": max_error,
                "rigid_bodies": model.nbody - 1,
                "nq": model.nq,
                "actuators": model.nu,
                "smoke_steps": 50,
            }
        )
    )


def usd_checks():
    from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils

    path = ROOT / "isaac_sim/go2_piper_d435.usd"
    stage = Usd.Stage.Open(str(path))
    require(bool(stage), "USD could not be opened")
    _, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
    require(not unresolved and not assets, f"USD external dependencies: {assets}, unresolved: {unresolved}")
    require(str(stage.GetDefaultPrim().GetPath()) == "/go2_piper_d435", "Wrong USD default prim")
    require(UsdGeom.GetStageMetersPerUnit(stage) == 1, "USD must use metres")
    require(UsdGeom.GetStageUpAxis(stage) == "Z", "USD must be Z up")
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    moving = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)]
    collisions = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    require(len(bodies) == 25 and len(moving) == 20, "Unexpected USD topology")
    for body, frame in (("base", "HeadCameraOpticalFrame"), ("link6", "WristCameraOpticalFrame")):
        require(bool(stage.GetPrimAtPath(f"/go2_piper_d435/{body}/{frame}")), f"Missing optical frame: {frame}")
    print(
        json.dumps(
            {
                "usd": "PASS",
                "rigid_bodies": len(bodies),
                "moving_joints": len(moving),
                "colliders": len(collisions),
                "external_dependencies": len(assets),
            }
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--physics", action="store_true")
    parser.add_argument("--usd", action="store_true")
    args = parser.parse_args()
    static_checks()
    if args.physics:
        physics_checks()
    if args.usd:
        usd_checks()


if __name__ == "__main__":
    main()
