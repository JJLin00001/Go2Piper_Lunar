"""Build a cached training crop from the existing lunar scene, without editing it."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
from .config import REPO_ROOT, metadata

def prepare_assets(settings, lunar_cfg):
    # These imports must follow SimulationApp startup (PhysxSchema is an extension).
    sys.path.insert(0, str(REPO_ROOT / "env"))
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Vt, Gf
    from lunar_scene import build_scene, build_digest
    from configuration import initial_joints
    from terrain import make_terrain
    from robot import add_robot, joint_frames
    from usd_helpers import new_stage, relative_asset, transform

    source_digest = build_digest(lunar_cfg)
    original = REPO_ROOT / "env/generated" / (lunar_cfg["environment"] + ".usda")
    source = original
    if not original.exists() or json.loads(original.with_suffix(".manifest.json").read_text())["build_sha256"] != source_digest:
        source = REPO_ROOT / "env/generated" / (lunar_cfg["environment"] + "_training_" + source_digest[:12] + ".usda")
        if not source.exists():
            stage, _ = build_scene(lunar_cfg, source)
            del stage
    robot_usd = REPO_ROOT / "assets/go2_piper_d435_assets/isaac_sim/go2_piper_d435.usd"
    h = hashlib.sha256()
    for data in (source_digest.encode(), robot_usd.read_bytes(), json.dumps(metadata(),sort_keys=True).encode(),
                 Path(__file__).read_bytes(), json.dumps({k:settings[k] for k in
                 ("patch_size_m","tcp_offset_link6_m","nominal_base_height_m")},sort_keys=True).encode()):
        h.update(data)
    directory = REPO_ROOT / "assets/lunar_training" / h.hexdigest()[:16]
    manifest_path = directory / "manifest.json"
    if manifest_path.exists():
        result = json.loads(manifest_path.read_text())
        if all((directory / k).exists() and hashlib.sha256((directory / k).read_bytes()).hexdigest() == v
               for k,v in result["files_sha256"].items()):
            return directory, result
    directory.mkdir(parents=True, exist_ok=True)
    source_stage = Usd.Stage.Open(str(source))
    flat = source_stage.Flatten()
    crop = new_stage(directory / "terrain.usdc", "/Terrain")
    Sdf.CopySpec(flat, "/World/Terrain", crop.GetRootLayer(), "/Terrain")
    # Copy the actual scene mesh, preserving triangle diagonals and all in-bounds scatter.
    half = settings["patch_size_m"]/2
    ground_info = None
    for prim in list(crop.Traverse()):
        if not prim.IsA(UsdGeom.Mesh): continue
        shape = UsdGeom.Mesh(prim)
        vertices = np.asarray(shape.GetPointsAttr().Get())
        faces = np.asarray(shape.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
        world_points = np.array([UsdGeom.XformCache().GetLocalToWorldTransform(prim).Transform(Gf.Vec3d(*map(float,p)))
                                 for p in vertices]) if "/Rocks/" in str(prim.GetPath()) else vertices
        inside = np.all(np.abs(world_points[:,:2]) <= half,axis=1)
        keep_faces = np.all(inside[faces],axis=1)
        if not keep_faces.any():
            crop.RemovePrim(prim.GetPath()); continue
        if keep_faces.all(): continue
        used = np.unique(faces[keep_faces])
        mapping = np.full(len(vertices),-1,dtype=np.int32); mapping[used]=np.arange(len(used))
        indices = mapping[faces[keep_faces]].astype(np.int32)
        shape.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(vertices[used].astype(np.float32)))
        shape.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(indices.ravel()))
        shape.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(indices),3,dtype=np.int32)))
        # Vertex-interpolated normals/color/UV must be cropped with the vertices.
        attrs = [shape.GetNormalsAttr()] + [p.GetAttr() for p in UsdGeom.PrimvarsAPI(prim).GetPrimvars()]
        for attr in attrs:
            value = attr.Get()
            if value is not None and hasattr(value,"__len__") and len(value)==len(vertices):
                attr.Set(type(value)(np.asarray(value)[used].tolist()))
        shape.GetExtentAttr().Set([Gf.Vec3f(*map(float,vertices[used].min(0))),
                                   Gf.Vec3f(*map(float,vertices[used].max(0)))])
    ground = UsdGeom.Mesh(crop.GetPrimAtPath("/Terrain/Ground"))
    points = np.asarray(ground.GetPointsAttr().Get())
    axis = np.unique(points[:,0])
    heights = points[:,2].reshape(len(axis),len(axis))
    if not np.allclose(points[:,1].reshape(len(axis),len(axis))[:,0],axis):
        raise ValueError("Training ground must be a square height grid")
    # Make any flattened external texture paths portable relative to this crop.
    for prim in crop.Traverse():
        for attr in prim.GetAttributes():
            v = attr.Get()
            if isinstance(v,Sdf.AssetPath) and v.path and Path(v.path).is_absolute():
                attr.Set(Sdf.AssetPath(relative_asset(Path(v.path),directory/"terrain.usdc")))
    crop.GetRootLayer().Save()
    np.savez_compressed(directory/"heightfield.npz",axis=axis,heights=heights)

    scene = new_stage(directory/"scene.usda","/World")
    terrain = make_terrain(lunar_cfg)
    report = add_robot(scene,directory/"scene.usda",lunar_cfg,terrain)
    # Explicitly activate body contacts before PhysX instantiation.
    for prim in scene.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    scene.GetDefaultPrim().SetCustomDataByKey("training_task",settings["task"])
    t = UsdGeom.Xform.Define(scene,"/World/Terrain").GetPrim()
    t.GetReferences().AddReference("terrain.usdc")
    # Keep the configured dynamic ore present; tracking does not attach it to the gripper.
    ore = UsdGeom.Xform.Define(scene,"/World/Ore").GetPrim()
    source_ore = source_stage.GetPrimAtPath("/World/Ores/Target")
    Sdf.CopySpec(flat,source_ore.GetPath(),scene.GetRootLayer(),"/World/Ore")
    scene.GetRootLayer().Save()

    # Reachable Cartesian goal bank from this robot's actual joint frames.
    q = initial_joints(lunar_cfg)
    home_frames = joint_frames(scene,q)
    tcp = np.array(settings["tcp_offset_link6_m"])
    home = home_frames["/World/Robot/link6"][:3,:3]@tcp + home_frames["/World/Robot/link6"][:3,3]
    rng = np.random.default_rng(1234)
    limits = {j["name"]:j["range"] for j in metadata()["joints"]}
    bank=[home]; joint_bank=[[q[f"joint{i}"] for i in range(1,7)]]
    for _ in range(1536):
        candidate=dict(q)
        for i,width in enumerate((.55,.3,.4,.35,.3,.35),1):
            candidate[f"joint{i}"]=float(np.clip(q[f"joint{i}"]+rng.uniform(-width,width),*limits[f"joint{i}"]))
        m=joint_frames(scene,candidate)["/World/Robot/link6"]
        p=m[:3,:3]@tcp+m[:3,3]
        if p[0]>.32 and abs(p[1])<.4 and -.06<p[2]<.6:
            bank.append(p); joint_bank.append([candidate[f"joint{i}"] for i in range(1,7)])
    if len(bank)<64: raise ValueError("Too few valid Piper trajectory goals")
    np.savez_compressed(directory/"goals.npz",positions=np.asarray(bank),joints=np.asarray(joint_bank),home=home)
    result={
        "task":settings["task"],"source_scene":str(source.relative_to(REPO_ROOT)),
        "source_build_sha256":source_digest,"environment":lunar_cfg["environment"],
        "crop_bounds_m":[float(axis[0]),float(axis[-1])],"ground_vertices":len(points),
        "ground_triangles":len(ground.GetFaceVertexIndicesAttr().Get())//3,
        "goal_bank_size":len(bank),"tcp_offset_link6_m":tcp.tolist(),
        "robot":report,"physics":lunar_cfg["physics"],
        "lunar_config":lunar_cfg,
        "terrain_note":"Exact central crop of source mesh and scatter. Repeated patch, no new elevation measurements.",
        "tcp_note":"Virtual tool point in link6; configure/calibrate for the physical gripper.",
    }
    result["files_sha256"]={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (directory/"scene.usda",directory/"terrain.usdc",directory/"heightfield.npz",directory/"goals.npz")}
    manifest_path.write_text(json.dumps(result,indent=2)+"\n")
    return directory,result
