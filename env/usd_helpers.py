"""USD authoring helpers. Import only after SimulationApp has started."""
from pathlib import Path
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Vt
from scipy.spatial.transform import Rotation


def relative_asset(source: Path, layer: Path) -> str:
    return Path(os.path.relpath(source, layer.parent)).as_posix()


def transform(prim, position=(0, 0, 0), rotation=None):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
    if rotation is not None:
        q = Rotation.from_matrix(np.asarray(rotation)).as_quat()
        xf.AddOrientOp().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))


def rigid_transform(position, quat):
    mat = np.eye(4)
    q = [*quat.GetImaginary(), quat.GetReal()]
    mat[:3, :3] = Rotation.from_quat(q).as_matrix()
    mat[:3, 3] = position
    return mat


def surface(stage, path, color=(0.2, 0.2, 0.2), vertex_colors=False, roughness=0.95):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    if vertex_colors:
        reader = UsdShade.Shader.Define(stage, path + "/DisplayColor")
        reader.CreateIdAttr("UsdPrimvarReader_float3")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float3)
        shader.GetInput("diffuseColor").ConnectToSource(reader.ConnectableAPI(), "result")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def physics_material(stage, path, static, dynamic, restitution=0, combine="average"):
    mat = UsdShade.Material.Define(stage, path)
    physics = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    physics.CreateStaticFrictionAttr(float(static))
    physics.CreateDynamicFrictionAttr(float(dynamic))
    physics.CreateRestitutionAttr(float(restitution))
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px.CreateFrictionCombineModeAttr(combine)
    px.CreateRestitutionCombineModeAttr("min")
    return mat


def bind_surface(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def bind_physics(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics")


def collision(prim, material, approximation=None, contact_offset=0.002):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    if approximation is not None:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(contact_offset)
    api.CreateRestOffsetAttr(0.0)
    bind_physics(prim, material)


def mesh(stage, path, vertices, faces, normals=None, colors=None):
    shape = UsdGeom.Mesh.Define(stage, path)
    vertices, faces = np.asarray(vertices, dtype=np.float32), np.asarray(faces, dtype=np.int32)
    shape.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices))
    shape.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    shape.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.ravel()))
    shape.CreateSubdivisionSchemeAttr("none")
    shape.CreateExtentAttr([Gf.Vec3f(*map(float, vertices.min(0))), Gf.Vec3f(*map(float, vertices.max(0)))])
    if normals is not None:
        shape.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals, dtype=np.float32)))
        shape.SetNormalsInterpolation("vertex")
    if colors is not None:
        shape.CreateDisplayColorPrimvar("vertex").Set(Vt.Vec3fArray.FromNumpy(colors))
    return shape


def new_stage(path, root_path):
    path.parent.mkdir(parents=True, exist_ok=True)
    # Rebuild only files in the explicitly chosen generated destination.
    stage = Usd.Stage.CreateNew(str(path)) if not path.exists() else Usd.Stage.Open(str(path))
    stage.GetRootLayer().Clear()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, root_path).GetPrim()
    stage.SetDefaultPrim(root)
    return stage


def textured_regolith(stage, path, files):
    """Standard USD Preview Surface with linear textures and tangent-space normals."""
    material = surface(stage, path)
    shader = UsdShade.Shader(stage.GetPrimAtPath(path + "/PreviewSurface"))
    uv = UsdShade.Shader.Define(stage, path + "/UV")
    uv.CreateIdAttr("UsdPrimvarReader_float2")
    uv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tint = UsdShade.Shader.Define(stage, path + "/Tint")
    tint.CreateIdAttr("UsdPrimvarReader_float4")
    tint.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("regolithTint")
    tint.CreateOutput("result", Sdf.ValueTypeNames.Float4)
    for name, filename in files.items():
        tex = UsdShade.Shader.Define(stage, path + "/" + name)
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(filename))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), "result")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        tex.CreateOutput("r", Sdf.ValueTypeNames.Float)
        if name == "normal":
            tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
            tex.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
            shader.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        elif name == "color":
            tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).ConnectToSource(tint.ConnectableAPI(), "result")
            shader.GetInput("diffuseColor").ConnectToSource(tex.ConnectableAPI(), "rgb")
        elif name == "roughness":
            shader.GetInput("roughness").ConnectToSource(tex.ConnectableAPI(), "r")
    return material


def texture_coordinates(shape, uv, colors=None):
    UsdGeom.PrimvarsAPI(shape).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex").Set(
        Vt.Vec2fArray.FromNumpy(np.asarray(uv, dtype=np.float32)))
    if colors is not None:
        rgba = np.column_stack((colors, np.ones(len(colors)))).astype(np.float32)
        UsdGeom.PrimvarsAPI(shape).CreatePrimvar("regolithTint", Sdf.ValueTypeNames.Float4Array, "vertex").Set(
            Vt.Vec4fArray.FromNumpy(rgba))
