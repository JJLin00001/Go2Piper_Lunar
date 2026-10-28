#!/usr/bin/env python3
"""Check exported scene structure, camera axes, drives, and relocation."""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

from configuration import DEFAULT_SCENE, ROOT, ROBOT_BUNDLE


def check(scene_path):
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils, UsdShade, PhysxSchema
    from robot import ARTICULATION_PATH, ROBOT_PATH, SENSOR_PATHS, FOOT_PATHS
    stage = Usd.Stage.Open(str(scene_path))
    assert stage and stage.GetDefaultPrim(), "Missing stage/default prim"
    cfg = json.loads(stage.GetDefaultPrim().GetCustomDataByKey("configuration_json"))
    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1
    physics = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene"))
    assert np.isclose(physics.GetGravityMagnitudeAttr().Get(), cfg["physics"]["gravity_m_s2"])
    assert tuple(physics.GetGravityDirectionAttr().Get()) == (0, 0, -1)
    robot = stage.GetPrimAtPath(ROBOT_PATH)
    bodies = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p for p in Usd.PrimRange(robot)
              if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)]
    roots = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert len(bodies) == 25 and len(joints) == 20 and len(roots) == 1
    assert str(roots[0].GetPath()) == ARTICULATION_PATH
    assert roots[0].GetAttribute("physxArticulation:sleepThreshold").Get() == 0
    limits = {j["name"]: j for j in json.loads((ROBOT_BUNDLE / "joint_metadata.json").read_text())["joints"]}
    for prim in joints:
        linear = prim.IsA(UsdPhysics.PrismaticJoint)
        drive = UsdPhysics.DriveAPI(prim, "linear" if linear else "angular")
        assert np.isclose(drive.GetMaxForceAttr().Get(), limits[prim.GetName()]["effort"])
    for prim in Usd.PrimRange(robot):
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            for relation in (joint.GetBody0Rel(), joint.GetBody1Rel()):
                targets = relation.GetTargets()
                assert len(targets) == 1 and stage.GetPrimAtPath(targets[0]), "World-fixed or missing joint body"
    for name, path in SENSOR_PATHS.items():
        camera = stage.GetPrimAtPath(path)
        assert camera.IsA(UsdGeom.Camera)
        cam_mat = UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        opt_mat = UsdGeom.Xformable(camera.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        usd_forward = cam_mat.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
        ros_forward = opt_mat.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        assert Gf.Dot(usd_forward, ros_forward) > 0.999999, f"{name}: optical axis mismatch"
        assert (cam_mat.ExtractTranslation() - opt_mat.ExtractTranslation()).GetLength() < 1e-9
    ore_mass = UsdPhysics.MassAPI(stage.GetPrimAtPath("/World/Ores/Target"))
    assert np.isclose(ore_mass.GetMassAttr().Get(), cfg["ore"]["mass_kg"])
    assert min(ore_mass.GetDiagonalInertiaAttr().Get()) > 0
    ground = stage.GetPrimAtPath("/World/Terrain/Ground")
    assert ground.HasAPI(UsdPhysics.CollisionAPI)
    assert UsdPhysics.MeshCollisionAPI(ground).GetApproximationAttr().Get() == "none"
    assert stage.GetDefaultPrim().GetCustomDataByKey("environment") == cfg["environment"]
    ground_material, _ = UsdShade.MaterialBindingAPI(ground).ComputeBoundMaterial("physics")
    ground_mu = UsdPhysics.MaterialAPI(ground_material.GetPrim())
    assert ground_mu.GetStaticFrictionAttr().Get() == 1.0
    assert ground_mu.GetDynamicFrictionAttr().Get() == 1.0
    assert PhysxSchema.PhysxMaterialAPI(ground_material.GetPrim()).GetFrictionCombineModeAttr().Get() == "multiply"
    for foot in FOOT_PATHS:
        colliders = [p for p in Usd.PrimRange(stage.GetPrimAtPath(foot)) if p.HasAPI(UsdPhysics.CollisionAPI)]
        assert colliders
        for collider in colliders:
            material, _ = UsdShade.MaterialBindingAPI(collider).ComputeBoundMaterial("physics")
            mu = UsdPhysics.MaterialAPI(material.GetPrim())
            assert np.isclose(mu.GetStaticFrictionAttr().Get(), cfg["robot"]["foot_static_friction"])
            assert np.isclose(mu.GetDynamicFrictionAttr().Get(), cfg["robot"]["foot_dynamic_friction"])
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(scene_path))
    assert not unresolved, f"Unresolved assets: {unresolved}"
    # Copy every runtime dependency, then reopen after relocation.
    with tempfile.TemporaryDirectory(prefix="go2_lunar_portability_") as directory:
        copied_root = Path(directory)
        for source in [Path(layer.realPath) for layer in layers] + [Path(asset) for asset in assets]:
            assert source.is_relative_to(ROOT), f"Dependency outside repository: {source}"
            destination = copied_root / source.relative_to(ROOT)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
        copied_scene = copied_root / scene_path.relative_to(ROOT)
        copied_stage = Usd.Stage.Open(str(copied_scene))
        assert copied_stage.GetPrimAtPath("/World/Robot/link6").IsValid()
        copied_layers, copied_assets, missing = UsdUtils.ComputeAllDependencies(str(copied_scene))
        assert not missing
        assert all(Path(layer.realPath).is_relative_to(copied_root) for layer in copied_layers)
        assert all(Path(asset).is_relative_to(copied_root) for asset in copied_assets)
        copied_stage = None
        copied_layers = None
    return {"scene": str(scene_path.relative_to(ROOT)), "passed": True,
            "rigid_bodies": len(bodies), "moving_joints": len(joints), "floating_base": True,
            "environment": cfg["environment"],
            "mass_kg": cfg["ore"]["mass_kg"], "runtime_layers": len(layers), "runtime_textures": len(assets),
            "effective_foot_friction": [cfg["robot"]["foot_static_friction"], cfg["robot"]["foot_dynamic_friction"]],
            "camera_optical_axes": "PASS", "bounded_drives": "PASS", "relocation": "PASS",
            "unresolved_assets": []}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", nargs="+", type=Path, default=[DEFAULT_SCENE])
    args = parser.parse_args()
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    from isaacsim import SimulationApp
    sys.argv = [sys.argv[0]]
    app = SimulationApp({"headless": True})
    try:
        results = [check(path.resolve()) for path in args.scene]
        (ROOT / "env/reports/structure.json").write_text(json.dumps(results, indent=2) + "\n")
        print("SCENE_VALIDATION", json.dumps(results, indent=2), flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        app.app.post_quit(1)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
