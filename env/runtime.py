"""Isaac Sim runtime checks and camera capture."""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
from scipy.spatial.transform import Rotation

from configuration import ROOT, initial_joints, experiment_parameters
from terrain import make_terrain
from lunar_scene import ORE_PATH
from robot import ARTICULATION_PATH, SENSOR_PATHS
from usd_helpers import transform


def capture_images(stage, output: Path, cfg, include_sensors):
    import omni.replicator.core as rep
    from PIL import Image
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    cameras = {
        "close": ("/World/Cameras/Close", (cfg["camera"]["width"], cfg["camera"]["height"])),
        "overview": ("/World/Cameras/Overview", (cfg["camera"]["width"], cfg["camera"]["height"])),
    }
    if include_sensors:
        cameras.update({key: (path, (cfg["camera"]["sensor_width"], cfg["camera"]["sensor_height"]))
                        for key, path in SENSOR_PATHS.items()})
    products, annotations = [], {}
    for name, (path, resolution) in cameras.items():
        product = rep.create.render_product(path, resolution)
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        annotations[name] = {"rgb": rgb}
        if name in SENSOR_PATHS:
            depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
            depth.attach(product)
            annotations[name]["depth"] = depth
        products.append(product)
    # delta_time=0 preserves the measured pose during capture.
    for _ in range(4):
        rep.orchestrator.step(rt_subframes=8, pause_timeline=True, delta_time=0.0)
    report = {}
    for name, entries in annotations.items():
        rgba = np.asarray(entries["rgb"].get_data())
        if rgba.size == 0 or rgba.ndim != 3:
            raise RuntimeError(f"No RGB image received from {name}")
        Image.fromarray(rgba[:, :, :3].astype(np.uint8)).save(output / f"{name}_rgb.png")
        report[name] = {"prim_path": cameras[name][0], "resolution": list(cameras[name][1]),
                        "rgb_std": float(rgba[:, :, :3].std())}
        if report[name]["rgb_std"] < 1:
            raise RuntimeError(f"{name} render is unexpectedly blank")
        if "depth" in entries:
            depth = np.asarray(entries["depth"].get_data(), dtype=np.float32)
            valid = np.isfinite(depth) & (depth > 0)
            if not valid.any():
                raise RuntimeError(f"No valid depth received from {name}")
            np.save(output / f"{name}_depth_m.npy", depth)
            preview = np.where(valid, np.clip(depth / 3.0, 0, 1), 0)
            Image.fromarray((preview * 255).astype(np.uint8)).save(output / f"{name}_depth_preview.png")
            report[name]["valid_depth_fraction"] = float(valid.mean())
            report[name]["depth_range_m"] = [float(depth[valid].min()), float(depth[valid].max())]
            cam = UsdGeom.Camera(stage.GetPrimAtPath(cameras[name][0]))
            w, h = cameras[name][1]
            fx = cam.GetFocalLengthAttr().Get() / cam.GetHorizontalApertureAttr().Get() * w
            fy = cam.GetFocalLengthAttr().Get() / cam.GetVerticalApertureAttr().Get() * h
            report[name]["intrinsics"] = {"fx": fx, "fy": fy, "cx": w / 2, "cy": h / 2}
            report[name]["depth_convention"] = "Optical Z distance in metres; nonfinite/<=0 values invalid"
            report[name]["model"] = "Ideal pinhole; CAD nominal pose; no active IR/stereo/noise simulation"
    for entries in annotations.values():
        for annotator in entries.values():
            annotator.detach()
    for product in products:
        product.destroy()
    (output / "capture.json").write_text(json.dumps(report, indent=2) + "\n")
    print("CAPTURE", output, json.dumps(report), flush=True)
    return report


def run_scene(app, args, cfg):
    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    stage = omni.usd.get_context().get_stage()
    cameras = {"close": "/World/Cameras/Close", "overview": "/World/Cameras/Overview", **SENSOR_PATHS}
    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = cameras[args.camera]
    if not args.simulate:
        frames = 0
        while app.is_running() and (not args.steps or frames < args.steps):
            app.update()
            frames += 1
        if args.capture and app.is_running():
            capture_images(stage, args.capture, cfg, args.sensors)
        return

    import carb
    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    from isaacsim.core.api import World
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim, RigidPrim

    dt = cfg["physics"]["dt_s"]
    # set_defaults=False is essential: World otherwise resets gravity to Earth's value.
    world = World(physics_dt=dt, rendering_dt=1 / 60, stage_units_in_meters=1.0,
                  physics_prim_path="/World/PhysicsScene", set_defaults=False,
                  backend="numpy", device="cpu")
    world.get_physics_context().set_gravity(-cfg["physics"]["gravity_m_s2"])
    robot = world.scene.add(SingleArticulation(ARTICULATION_PATH, name="robot", reset_xform_properties=False))
    feet = world.scene.add(RigidPrim("/World/Robot/.*_foot", name="feet", reset_xform_properties=False,
                                     track_contact_forces=True))
    ore = world.scene.add(SingleRigidPrim(ORE_PATH, name="ore", reset_xform_properties=False))
    probe = None
    if args.validate:
        p = UsdGeom.Sphere.Define(stage, "/World/ValidationGravityProbe")
        p.CreateRadiusAttr(0.01)
        p.CreateVisibilityAttr("invisible")
        transform(p.GetPrim(), (-3.0, -3.0, 5.0))
        UsdPhysics.RigidBodyAPI.Apply(p.GetPrim())
        UsdPhysics.MassAPI.Apply(p.GetPrim()).CreateMassAttr(1.0)
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(p.GetPrim())
        px.CreateLinearDampingAttr(0)
        px.CreateDisableGravityAttr(False)
        probe = world.scene.add(SingleRigidPrim(str(p.GetPath()), name="gravity_probe", reset_xform_properties=False))
    world.reset()
    q_map = initial_joints(cfg)
    target = np.asarray([q_map[name] for name in robot.dof_names])
    # Also set state and targets by name after reset, rather than assuming joint ordering.
    robot.set_joint_positions(target)
    robot.set_joint_velocities(np.zeros_like(target))
    robot.apply_action(ArticulationAction(joint_positions=target))
    probe_v0 = probe.get_linear_velocity().copy() if probe is not None else None
    gravity_measured = None
    samples = []
    peak_tilt = 0.0
    max_joint_speed = 0.0
    max_joint_error = 0.0
    step = 0
    start = time.monotonic()
    render_every = max(1, round((1 / 60) / dt))
    reset_pending = False
    while app.is_running() and (not args.steps or step < args.steps):
        if world.is_stopped():
            if args.validate:
                raise RuntimeError("Validation interrupted by timeline Stop")
            reset_pending = True
            app.update()
            continue
        if not world.is_playing():
            app.update()
            continue
        if reset_pending:
            world.reset()
            robot.set_joint_positions(target)
            robot.set_joint_velocities(np.zeros_like(target))
            robot.apply_action(ArticulationAction(joint_positions=target))
            reset_pending = False
        world.step(render=False)
        step += 1
        if probe is not None and step == 24:
            gravity_measured = -(probe.get_linear_velocity()[2] - probe_v0[2]) / (24 * dt)
        position, quaternion = robot.get_world_pose()
        positions = robot.get_joint_positions()
        velocities = robot.get_joint_velocities()
        if not all(np.isfinite(v).all() for v in (position, quaternion, positions, velocities)):
            raise RuntimeError(f"Nonfinite robot state at step {step}")
        z_axis = Rotation.from_quat([*quaternion[1:], quaternion[0]]).as_matrix()[:, 2]
        tilt = float(np.rad2deg(np.arccos(np.clip(z_axis[2], -1, 1))))
        peak_tilt = max(peak_tilt, tilt)
        max_joint_speed = max(max_joint_speed, float(np.max(np.abs(velocities))))
        max_joint_error = max(max_joint_error, float(np.max(np.abs(positions - target))))
        if step % 12 == 0:
            forces = feet.get_net_contact_forces(dt=dt)
            if not np.isfinite(forces).all():
                raise RuntimeError("Nonfinite foot contact forces")
            samples.append({"step": step, "time_s": step * dt,
                            "base_position_m": position.tolist(), "base_tilt_deg": tilt,
                            "foot_force_world_N": forces.tolist()})
        if step % render_every == 0:
            world.render()
    if args.validate and step < args.steps:
        raise RuntimeError("Simulation closed before completing the requested validation")
    if not app.is_running():
        return
    ore_pos, ore_quat = ore.get_world_pose()
    ore_vel = ore.get_linear_velocity()
    if not all(np.isfinite(v).all() for v in (ore_pos, ore_quat, ore_vel)):
        raise RuntimeError("Nonfinite ore state")
    terrain = make_terrain(cfg)
    base_ground_clearance = float(position[2] - terrain.height(*position[:2]))
    ore_vertices = np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(ORE_PATH + "/Geometry")).GetPointsAttr().Get())
    ore_rotation = Rotation.from_quat([*ore_quat[1:], ore_quat[0]]).as_matrix()
    ore_points = ore_vertices @ ore_rotation.T + ore_pos
    ore_ground_gap = float(np.min(ore_points[:, 2] - terrain.height(ore_points[:, 0], ore_points[:, 1])))
    tail = samples[-min(10, len(samples)):]
    mean_forces = np.mean([s["foot_force_world_N"] for s in tail], axis=0) if tail else np.zeros((4, 3))
    report = {
        "scene": str(args.scene.resolve()), "environment": cfg["environment"],
        "experiment_parameters": experiment_parameters(cfg),
        "physics_steps": step, "simulated_time_s": step * dt,
        "elapsed_wall_s": time.monotonic() - start,
        "gravity_configured_m_s2": cfg["physics"]["gravity_m_s2"],
        "gravity_measured_m_s2": float(gravity_measured) if gravity_measured is not None else None,
        "joint_count": len(robot.dof_names), "joint_names": list(robot.dof_names),
        "base_final_position_m": position.tolist(), "base_peak_tilt_deg": peak_tilt,
        "max_joint_speed_rad_or_m_s": max_joint_speed, "max_joint_error_rad_or_m": max_joint_error,
        "foot_paths": list(feet.prim_paths), "mean_final_foot_force_world_N": mean_forces.tolist(),
        "base_ground_clearance_m": base_ground_clearance,
        "ore_lowest_ground_gap_m": ore_ground_gap,
        "ore_final_position_m": ore_pos.tolist(), "ore_final_speed_m_s": float(np.linalg.norm(ore_vel)),
        "control": "Bounded joint PD preview; no WBC or grasp execution",
    }
    failures = []
    if args.validate:
        if not np.isclose(gravity_measured, cfg["physics"]["gravity_m_s2"], rtol=0.02, atol=0.01):
            failures.append("free-fall acceleration does not match lunar gravity")
        if len(robot.dof_names) != 20:
            failures.append("expected 20 moving joints")
        if peak_tilt > 15 or base_ground_clearance < 0.20:
            failures.append("robot did not remain standing")
        if np.any(mean_forces[:, 2] < 0.2):
            failures.append("not all four feet support the robot")
        if not -0.01 <= ore_ground_gap <= 0.025 or np.linalg.norm(ore_vel) > 0.1:
            failures.append("ore failed to settle on the terrain")
    report["validation_passed"] = not failures if args.validate else None
    report["validation_failures"] = failures
    out = args.capture.resolve() if args.capture else ROOT / "env/reports"
    out.mkdir(parents=True, exist_ok=True)
    (out / "simulation.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "contacts.json").write_text(json.dumps(samples, indent=2) + "\n")
    print("SIMULATION", json.dumps(report, indent=2), flush=True)
    world.pause()
    if args.capture:
        capture_images(stage, args.capture, cfg, args.sensors)
    if failures:
        raise RuntimeError("; ".join(failures))
