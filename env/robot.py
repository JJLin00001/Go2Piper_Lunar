"""Reference, pose, and configure the supplied floating-base robot."""
import json

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema
from scipy.spatial.transform import Rotation

from configuration import ROBOT_BUNDLE, initial_joints, foot_material_coefficients
from usd_helpers import bind_physics, bind_surface, physics_material, relative_asset, rigid_transform, surface, transform


ROBOT_PATH = "/World/Robot"
ARTICULATION_PATH = ROBOT_PATH + "/base"
FOOT_PATHS = [ROBOT_PATH + "/" + leg + "_foot" for leg in ("FL", "FR", "RL", "RR")]
SENSOR_PATHS = {
    "head": ROBOT_PATH + "/base/HeadCameraOpticalFrame/Camera",
    "wrist": ROBOT_PATH + "/link6/WristCameraOpticalFrame/Camera",
}


def joint_frames(stage, q):
    """FK from the actual USD joint frames, including both prismatic fingers."""
    poses = {ARTICULATION_PATH: np.eye(4)}
    pending = [UsdPhysics.Joint(p) for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH))
               if p.IsA(UsdPhysics.Joint)]
    while pending:
        advanced = False
        for joint in pending[:]:
            parent = joint.GetBody0Rel().GetTargets()
            child = joint.GetBody1Rel().GetTargets()
            if len(parent) != 1 or len(child) != 1:
                raise ValueError("Expected a floating articulation without world-fixed joints")
            parent, child = str(parent[0]), str(child[0])
            if parent not in poses:
                continue
            frame0 = rigid_transform(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get())
            frame1 = rigid_transform(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())
            motion = np.eye(4)
            prim = joint.GetPrim()
            if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
                value = q[prim.GetName()]
                axis = np.eye(3)["XYZ".index(prim.GetAttribute("physics:axis").Get())]
                if prim.IsA(UsdPhysics.RevoluteJoint):
                    motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
                else:
                    motion[:3, 3] = axis * value
            poses[child] = poses[parent] @ frame0 @ motion @ np.linalg.inv(frame1)
            pending.remove(joint)
            advanced = True
        if not advanced:
            raise ValueError("USD joint graph is disconnected or contains a cycle")
    return poses


def add_robot(stage, scene_path, cfg, terrain):
    root = UsdGeom.Xform.Define(stage, ROBOT_PATH).GetPrim()
    root.GetReferences().AddReference(relative_asset(
        ROBOT_BUNDLE / "isaac_sim/go2_piper_d435.usd", scene_path))
    q = initial_joints(cfg)
    poses = joint_frames(stage, q)
    rigid_paths = {str(p.GetPath()) for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    if set(poses) != rigid_paths:
        raise ValueError("FK does not cover every rigid body in the supplied robot")
    r = cfg["robot"]
    yaw = Rotation.from_euler("z", r["yaw_deg"], degrees=True).as_matrix()
    xy = np.asarray(r["xy_m"])
    base_z = -np.inf
    for foot in FOOT_PATHS:
        foot_prim = stage.GetPrimAtPath(foot)
        spheres = [UsdGeom.Sphere(p) for p in Usd.PrimRange(foot_prim)
                   if p.IsA(UsdGeom.Sphere) and p.HasAPI(UsdPhysics.CollisionAPI)]
        if len(spheres) != 1:
            raise ValueError("Expected one spherical foot collider per leg")
        radius = float(spheres[0].GetRadiusAttr().Get())
        p = yaw @ poses[foot][:3, 3]
        base_z = max(base_z, float(terrain.height(*(xy + p[:2]))) - p[2] + radius)
    base_z += r["foot_clearance_m"]
    transform(root, (*xy, base_z), yaw)
    for path, mat in poses.items():
        transform(stage.GetPrimAtPath(path), mat[:3, 3], mat[:3, :3])

    base = stage.GetPrimAtPath(ARTICULATION_PATH)
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(base)
    articulation.CreateEnabledSelfCollisionsAttr(r["self_collisions"])
    articulation.CreateSleepThresholdAttr(0.0)
    articulation.CreateStabilizationThresholdAttr(0.0)
    articulation.CreateSolverPositionIterationCountAttr(cfg["physics"]["solver_position_iterations"])
    articulation.CreateSolverVelocityIterationCountAttr(cfg["physics"]["solver_velocity_iterations"])
    foot_static, foot_dynamic = foot_material_coefficients(cfg)
    foot_mat = physics_material(stage, "/World/Materials/FootContact",
                                foot_static, foot_dynamic)
    body_mat = physics_material(stage, "/World/Materials/BodyContact",
                                r["body_static_friction"], r["body_dynamic_friction"])
    pad_mat = physics_material(stage, "/World/Materials/PadContact",
                               r["pad_static_friction"], r["pad_dynamic_friction"])
    visual_material = surface(stage, "/World/Materials/RobotVisual", vertex_colors=True, roughness=0.55)
    metadata = json.loads((ROBOT_BUNDLE / "joint_metadata.json").read_text())
    limits = {j["name"]: j for j in metadata["joints"]}
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Gprim) and UsdGeom.Gprim(prim).GetDisplayColorAttr().Get():
            # The supplied USD has displayColor but no render materials. Preserve
            # those colors explicitly in RTX instead of using its white fallback.
            bind_surface(prim, visual_material)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateDisableGravityAttr(False)
            body.CreateLinearDampingAttr(0.0)
            body.CreateAngularDampingAttr(0.0)
            body.CreateMaxDepenetrationVelocityAttr(1.0)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            path_string = str(prim.GetPath())
            material = (pad_mat if "/link7/" in path_string or "/link8/" in path_string else
                        foot_mat if any(path_string.startswith(p + "/") or path_string == p for p in FOOT_PATHS)
                        else body_mat)
            bind_physics(prim, material)
            contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            contact.CreateContactOffsetAttr(0.002)
            contact.CreateRestOffsetAttr(0.0)
        if prim.GetName() in ("FL_foot", "FR_foot", "RL_foot", "RR_foot", "link7", "link8"):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        name = prim.GetName()
        linear = prim.IsA(UsdPhysics.PrismaticJoint)
        group = ("gripper" if linear else "legs" if "_joint" in name
                 else "arm_load_bearing" if name in ("joint2", "joint3") else "arm")
        kp, kd = r["pd"][group]
        # USD angular drives use degrees; the user-facing gains are Nm/rad.
        gain_scale = 1.0 if linear else np.pi / 180
        drive = UsdPhysics.DriveAPI.Apply(prim, "linear" if linear else "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(kp * gain_scale)
        drive.CreateDampingAttr(kd * gain_scale)
        drive.CreateMaxForceAttr(limits[name]["effort"])
        drive.CreateTargetPositionAttr(q[name] if linear else np.rad2deg(q[name]))
        drive.CreateTargetVelocityAttr(0.0)

    for path in SENSOR_PATHS.values():
        camera = UsdGeom.Camera.Define(stage, path)
        # USD cameras look along -Z with +Y up. Parent is a ROS optical frame.
        transform(camera.GetPrim(), rotation=np.diag([1.0, -1.0, -1.0]))
        c = cfg["camera"]
        aperture = 20.955
        focal = aperture / (2 * np.tan(np.deg2rad(c["sensor_horizontal_fov_deg"]) / 2))
        camera.CreateFocalLengthAttr(float(focal))
        camera.CreateHorizontalApertureAttr(aperture)
        camera.CreateVerticalApertureAttr(aperture * c["sensor_height"] / c["sensor_width"])
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 30))
        camera.GetPrim().SetCustomDataByKey("sensor_model", "Ideal pinhole RGB + depth; nominal mount; no D435 noise model")
    root.SetCustomDataByKey("initial_joint_positions_json", json.dumps(q, sort_keys=True))
    root.SetCustomDataByKey("control_mode", "Model-limited joint PD preview; replace drives for WBC")
    return {
        "prim_path": ROBOT_PATH,
        "articulation_root": ARTICULATION_PATH,
        "rigid_bodies": len(poses),
        "moving_joints": len(q),
        "initial_base_position_m": [float(xy[0]), float(xy[1]), float(base_z)],
        "initial_joint_positions": q,
        "mass_kg": sum(float(UsdPhysics.MassAPI(stage.GetPrimAtPath(p)).GetMassAttr().Get()) for p in poses),
        "self_collisions": r["self_collisions"],
        "drive_effort_limits": {j["name"]: j["effort"] for j in metadata["joints"]},
        "camera_paths": SENSOR_PATHS,
    }
