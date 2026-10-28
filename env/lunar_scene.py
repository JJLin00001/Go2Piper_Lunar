"""Build portable lunar assets and a composed Isaac Sim scene."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils, PhysxSchema
from scipy.spatial.transform import Rotation

from configuration import ROOT, ROBOT_BUNDLE, validate_config, experiment_parameters
from geometry import rock_mesh, mass_properties, vertex_normals
from terrain import make_terrain, file_sha256
from regolith import create_textures
from robot import add_robot
from usd_helpers import (bind_surface, collision, mesh, new_stage, physics_material,
                         relative_asset, surface, transform, textured_regolith, texture_coordinates)

ORE_PATH = "/World/Ores/Target"


def config_digest(cfg):
    return hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest()


def build_digest(cfg):
    h = hashlib.sha256(config_digest(cfg).encode())
    for name in ("configuration.py", "geometry.py", "terrain.py", "regolith.py",
                 "usd_helpers.py", "robot.py", "lunar_scene.py"):
        h.update((ROOT / "env" / name).read_bytes())
    if cfg["terrain"]["type"] == "dem":
        for name in ("provenance.json", "dem.npz", "ortho_reference.png"):
            h.update(file_sha256(ROOT / cfg["terrain"]["data_dir"] / name).encode())
    return h.hexdigest()


def build_terrain(path, cfg, terrain):
    stage = new_stage(path, "/LunarTerrain")
    measured = cfg["terrain"]["type"] == "dem"
    texture_files = None
    if measured:
        texture_files = create_textures(path.parent / "textures", cfg["seed"],
                                       cfg["terrain"]["detail_texture_period_m"])
        look = textured_regolith(stage, "/LunarTerrain/Materials/Regolith",
                                 {k: relative_asset(v, path) for k, v in texture_files.items()})
    else:
        look = surface(stage, "/LunarTerrain/Materials/Regolith", vertex_colors=True)
    rock_look = surface(stage, "/LunarTerrain/Materials/Rock", color=(0.13, 0.13, 0.125))
    t = cfg["terrain"]
    # Unit ground reference: multiply wins over each body's average combine.
    # Therefore effective foot/ore-ground friction equals that body's requested coefficient.
    contact = physics_material(stage, "/LunarTerrain/Materials/Contact",
                               1.0, 1.0, t["restitution"], combine="multiply")
    vertices, faces, normals, colors = terrain.mesh()
    ground = mesh(stage, "/LunarTerrain/Ground", vertices, faces, normals, colors)
    bind_surface(ground.GetPrim(), look)
    if measured:
        texture_coordinates(ground, vertices[:, :2] / t["detail_texture_period_m"], colors)
    # Concave triangle collision preserves the crater basins. Never use a convex hull here.
    collision(ground.GetPrim(), contact, "none")
    rng = np.random.default_rng(cfg["seed"] + 1)
    UsdGeom.Scope.Define(stage, "/LunarTerrain/Rocks")
    exclusion = 1.4 if measured else t["flat_radius_m"] + t["flat_blend_m"] + 0.3
    for i in range(t["rock_count"]):
        angle = rng.uniform(0, 2 * np.pi)
        radius = (np.sqrt(rng.uniform(exclusion**2, (t["size_m"] * 0.46)**2)) if measured
                  else rng.uniform(exclusion, t["size_m"] * 0.46))
        xy = radius * np.array([np.cos(angle), np.sin(angle)])
        size = rng.uniform(0.07, 0.38) * (1.5 if i % 9 == 0 else 1)
        dims = size * rng.uniform(0.6, 1.3, 3)
        points, triangles = rock_mesh(cfg["seed"] + 100 + i, dims, count=192 if measured else 46)
        # Embed the bottom slightly into the ground as a static obstacle.
        z = float(terrain.height(*xy)) + dims[2] * 0.38
        shape = mesh(stage, f"/LunarTerrain/Rocks/Rock_{i:03d}", points, triangles,
                     normals=vertex_normals(points, triangles) if measured else None)
        transform(shape.GetPrim(), (*xy, z), Rotation.from_euler("z", angle).as_matrix())
        if measured:
            tint = np.tile(rng.uniform(0.12, 0.22), (len(points), 3))
            uv = np.column_stack((points[:, 0] + 0.3 * points[:, 2],
                                  points[:, 1] + 0.5 * points[:, 2])) / 0.4
            texture_coordinates(shape, uv, tint)
            bind_surface(shape.GetPrim(), look)
        else:
            bind_surface(shape.GetPrim(), rock_look)
        collision(shape.GetPrim(), contact, "convexHull")
    if measured and t["pebble_count"]:
        # Merge closed pebble meshes into one static triangle collider, avoiding
        # thousands of independent actors while retaining actual contact geometry.
        all_points, all_faces, all_colors = [], [], []
        offset = 0
        for i in range(t["pebble_count"]):
            angle = rng.uniform(0, 2 * np.pi)
            radius = np.sqrt(rng.uniform(1.0, 15.0**2))
            xy = radius * np.array([np.cos(angle), np.sin(angle)])
            size = rng.uniform(0.008, 0.045)
            dims = size * rng.uniform(0.6, 1.3, 3)
            points, triangles = rock_mesh(cfg["seed"] + 7000 + i, dims)
            points = points @ Rotation.from_euler("z", angle).as_matrix().T
            points += [*xy, float(terrain.height(*xy)) + dims[2] * 0.3]
            all_points.append(points)
            all_faces.append(triangles + offset)
            all_colors.append(np.tile(rng.uniform(0.09, 0.20), (len(points), 3)))
            offset += len(points)
        pebbles = mesh(stage, "/LunarTerrain/Pebbles", np.vstack(all_points), np.vstack(all_faces),
                       colors=np.vstack(all_colors))
        pebble_look = surface(stage, "/LunarTerrain/Materials/Pebbles", vertex_colors=True)
        bind_surface(pebbles.GetPrim(), pebble_look)
        collision(pebbles.GetPrim(), contact, "none", contact_offset=0.001)
        pebbles.GetPrim().SetCustomDataByKey("provenance", "Synthetic deterministic collidable pebble scatter")
    stage.GetDefaultPrim().SetCustomDataByKey("environment", cfg["environment"])
    stage.GetRootLayer().Save()
    report = {"vertices": len(vertices), "triangles": len(faces),
              "craters": len(terrain.craters), "static_rocks": t["rock_count"],
              "collidable_pebbles": t.get("pebble_count", 0),
              "initialization_area_radius_m": t["flat_radius_m"],
              "central_area_flattened": not measured,
              "near_field_size_m": t["size_m"],
              "total_extent_m": 2 * t["horizon_extent_m"],
              "height_range_m": [float(vertices[:, 2].min()), float(vertices[:, 2].max())],
              "ground_material": {"static_friction": 1.0, "dynamic_friction": 1.0, "combine": "multiply"}}
    report.update(terrain.report() if measured else {"type": "procedural", "source": "Seeded analytic terrain, no measured lunar data"})
    return report


def build_ore(path, cfg):
    stage = new_stage(path, "/Ore")
    root = stage.GetDefaultPrim()
    o = cfg["ore"]
    vertices, faces = rock_mesh(cfg["seed"] + 2, o["dimensions_m"])
    shape = mesh(stage, "/Ore/Geometry", vertices, faces)
    look = surface(stage, "/Ore/Materials/Basalt", color=(0.24, 0.20, 0.15), roughness=0.8)
    bind_surface(shape.GetPrim(), look)
    contact = physics_material(stage, "/Ore/Materials/Contact", o["static_friction"], o["dynamic_friction"])
    collision(shape.GetPrim(), contact, "convexHull", contact_offset=0.001)
    UsdPhysics.RigidBodyAPI.Apply(root)
    api = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
    api.CreateEnableCCDAttr(True)
    api.CreateLinearDampingAttr(0.0)
    api.CreateAngularDampingAttr(0.0)
    api.CreateMaxDepenetrationVelocityAttr(1.0)
    api.CreateSolverPositionIterationCountAttr(cfg["physics"]["solver_position_iterations"])
    api.CreateSolverVelocityIterationCountAttr(cfg["physics"]["solver_velocity_iterations"])
    volume, center, moments, axes = mass_properties(vertices, faces, o["mass_kg"])
    mass_api = UsdPhysics.MassAPI.Apply(root)
    mass_api.CreateMassAttr(o["mass_kg"])
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, center)))
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, moments)))
    q = Rotation.from_matrix(axes).as_quat()
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))
    PhysxSchema.PhysxContactReportAPI.Apply(root).CreateThresholdAttr(0.0)
    root.SetCustomDataByKey("role", "Dynamic grasp target; no attachment or fixed joint")
    stage.GetRootLayer().Save()
    return vertices, {"mass_kg": o["mass_kg"], "volume_m3": volume,
                      "effective_density_kg_m3": o["mass_kg"] / volume,
                      "dimensions_m": o["dimensions_m"], "center_of_mass_m": center.tolist(),
                      "principal_inertia_kg_m2": moments.tolist(), "prim_path": ORE_PATH}


def camera(stage, path, eye, target, far=500):
    cam = UsdGeom.Camera.Define(stage, path)
    mat = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(mat)
    cam.CreateFocalLengthAttr(25.0)
    cam.CreateHorizontalApertureAttr(36.0)
    cam.CreateVerticalApertureAttr(20.25)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, far))
    return cam


def add_lighting_and_cameras(stage, cfg):
    UsdGeom.Scope.Define(stage, "/World/Lights")
    l = cfg["lighting"]
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(l["sun_intensity"])
    sun.CreateAngleAttr(0.53)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    elevation, azimuth = np.deg2rad([l["sun_elevation_deg"], l["sun_azimuth_deg"]])
    direction = Gf.Vec3d(np.cos(elevation) * np.cos(azimuth),
                        np.cos(elevation) * np.sin(azimuth), -np.sin(elevation))
    rotation = Gf.Rotation(Gf.Vec3d(0, 0, -1), direction).GetQuat()
    UsdGeom.Xformable(sun).AddOrientOp().Set(Gf.Quatf(rotation))
    # Small optional fill is a viewing aid, not an atmospheric scattering model.
    fill = UsdLux.DistantLight.Define(stage, "/World/Lights/ViewingFill")
    fill.CreateIntensityAttr(l["fill_intensity"])
    fill.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))
    fill.CreateAngleAttr(30)
    UsdGeom.Xformable(fill).AddRotateXYZOp().Set(Gf.Vec3f(-20, -25, 0))
    UsdGeom.Scope.Define(stage, "/World/Cameras")
    c = cfg["camera"]
    camera(stage, "/World/Cameras/Overview", c["overview_eye_m"], c["overview_target_m"], max(500, cfg["terrain"]["horizon_extent_m"] * 3))
    camera(stage, "/World/Cameras/Close", c["close_eye_m"], c["close_target_m"], max(500, cfg["terrain"]["horizon_extent_m"] * 3))
    layer = stage.GetRootLayer()
    data = layer.customLayerData
    data["cameraSettings"] = {"boundCamera": "/World/Cameras/Close"}
    layer.customLayerData = data


def build_scene(cfg, scene_path):
    validate_config(cfg)
    scene_path = Path(scene_path).resolve()
    # Alternate scene names get their own generated geometry.
    asset_dir = ROOT / "assets/lunar" / scene_path.stem
    terrain_path, ore_path = asset_dir / "terrain.usdc", asset_dir / "ore.usdc"
    terrain = make_terrain(cfg)
    terrain_report = build_terrain(terrain_path, cfg, terrain)
    ore_vertices, ore_report = build_ore(ore_path, cfg)
    stage = new_stage(scene_path, "/World")
    stage.SetTimeCodesPerSecond(round(1 / cfg["physics"]["dt_s"]))
    stage.SetFramesPerSecond(60)
    world = stage.GetDefaultPrim()
    world.SetCustomDataByKey("configuration_json", json.dumps(cfg, sort_keys=True))
    world.SetCustomDataByKey("configuration_sha256", config_digest(cfg))
    world.SetCustomDataByKey("build_sha256", build_digest(cfg))
    world.SetCustomDataByKey("environment", cfg["environment"])
    world.SetCustomDataByKey("purpose", "Lunar Go2 + Piper heavy-rock grasp / future WBC experiments")
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(cfg["physics"]["gravity_m_s2"])
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateTimeStepsPerSecondAttr(round(1 / cfg["physics"]["dt_s"]))
    px.CreateEnableGPUDynamicsAttr(False)
    px.CreateBroadphaseTypeAttr("MBP")
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(False)
    # Disable sleeping so low lunar contact forces remain observable.
    terrain_root = UsdGeom.Xform.Define(stage, "/World/Terrain").GetPrim()
    terrain_root.GetReferences().AddReference(relative_asset(terrain_path, scene_path))
    UsdGeom.Scope.Define(stage, "/World/Materials")
    UsdGeom.Scope.Define(stage, "/World/Ores")
    ore = UsdGeom.Xform.Define(stage, ORE_PATH).GetPrim()
    ore.GetReferences().AddReference(relative_asset(ore_path, scene_path))
    xy = np.asarray(cfg["ore"]["xy_m"])
    z = np.max(terrain.height(xy[0] + ore_vertices[:, 0], xy[1] + ore_vertices[:, 1]) - ore_vertices[:, 2])
    transform(ore, (*xy, float(z) + cfg["ore"]["clearance_m"]))
    robot_report = add_robot(stage, scene_path, cfg, terrain)
    # Sleeping disabled for contact and small-force measurements.
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateSleepThresholdAttr(0.0)
    add_lighting_and_cameras(stage, cfg)
    stage.GetRootLayer().Save()
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(scene_path))
    if unresolved:
        raise RuntimeError(f"Unresolved scene assets: {unresolved}")
    report = {
        "scene": str(scene_path.relative_to(ROOT)) if scene_path.is_relative_to(ROOT) else str(scene_path),
        "environment": cfg["environment"],
        "experiment_parameters": experiment_parameters(cfg),
        "config_sha256": config_digest(cfg), "build_sha256": build_digest(cfg),
        "robot_usd_sha256": hashlib.sha256((ROBOT_BUNDLE / "isaac_sim/go2_piper_d435.usd").read_bytes()).hexdigest(),
        "gravity_m_s2": cfg["physics"]["gravity_m_s2"],
        "physics_dt_s": cfg["physics"]["dt_s"],
        "terrain": terrain_report, "robot": robot_report, "ore": ore_report,
        "dependency_layer_count": len(layers), "unresolved_assets": list(unresolved),
        "limitations": [
            ("DEM is measured at 1.5 m grid spacing; interpolated/synthetic small-scale detail is not a measurement."
             if cfg["terrain"]["type"] == "dem" else "Procedural terrain, not a measured lunar DEM."),
            "Rigid Coulomb contact, not deformable or granular regolith.",
            "PD standing preview only; no WBC or grasp success claim.",
            "Ideal pinhole RGB/depth cameras, not a calibrated D435 hardware model.",
            "Effort limits inherited from supplied model; hardware qualification is separate."
        ]
    }
    scene_path.with_suffix(".manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    return stage, report
