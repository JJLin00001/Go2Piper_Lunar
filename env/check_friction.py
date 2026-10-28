#!/usr/bin/env python3
"""Verify effective foot-ground dynamic friction against a = mu*g in Isaac Sim."""
import json
import os
from pathlib import Path
import sys

from configuration import ROOT, load_config, foot_material_coefficients

def main():
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    from isaacsim import SimulationApp
    sys.argv = [sys.argv[0]]
    app = SimulationApp({"headless": True, "fast_shutdown": True})
    try:
        import numpy as np
        import omni.usd
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleRigidPrim
        from usd_helpers import physics_material, collision, transform
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Z")
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        physics.CreateGravityMagnitudeAttr(1.62)
        px = PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim())
        px.CreateSolverTypeAttr("TGS")
        px.CreateTimeStepsPerSecondAttr(240)
        ground_material = physics_material(stage, "/World/GroundMaterial", 1, 1, combine="multiply")
        floor = UsdGeom.Cube.Define(stage, "/World/Floor")
        floor.CreateSizeAttr(1)
        transform(floor.GetPrim(), (0, 0, -0.05))
        UsdGeom.Xformable(floor).AddScaleOp().Set(Gf.Vec3f(12, 8, 0.1))
        collision(floor.GetPrim(), ground_material)
        dt = 1 / 240
        world = World(physics_dt=dt, rendering_dt=1/60, stage_units_in_meters=1.0,
                      physics_prim_path="/World/PhysicsScene", set_defaults=False,
                      backend="numpy", device="cpu")
        world.get_physics_context().set_gravity(-1.62)
        specs = [(0.2, 0.1, 1.0), (0.6, 0.5, 1.0), (0.6, 0.5, 5.0)]
        blocks = []
        for i, (static, dynamic, mass) in enumerate(specs):
            cfg = load_config()
            cfg["robot"]["foot_static_friction"] = static
            cfg["robot"]["foot_dynamic_friction"] = dynamic
            mat = physics_material(stage, f"/World/FootMaterial{i}", *foot_material_coefficients(cfg))
            cube = UsdGeom.Cube.Define(stage, f"/World/Block{i}")
            cube.CreateSizeAttr(1)
            transform(cube.GetPrim(), (-2, (i - 1) * 2, 0.026))
            UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.05))
            collision(cube.GetPrim(), mat, contact_offset=0.001)
            UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(mass)
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim())
            body.CreateLinearDampingAttr(0)
            body.CreateAngularDampingAttr(0)
            body.CreateSleepThresholdAttr(0)
            body.CreateSolverPositionIterationCountAttr(16)
            body.CreateSolverVelocityIterationCountAttr(4)
            blocks.append(world.scene.add(SingleRigidPrim(str(cube.GetPath()), name=f"block{i}", reset_xform_properties=False)))
        world.reset()
        for _ in range(240):
            world.step(render=False)
        for body in blocks:
            body.set_linear_velocity(np.array([2.0, 0, 0]))
        for _ in range(24):
            world.step(render=False)
        initial = [float(body.get_linear_velocity()[0]) for body in blocks]
        for _ in range(120):
            world.step(render=False)
        report = []
        for body, v0, (static, dynamic, mass) in zip(blocks, initial, specs):
            v1 = float(body.get_linear_velocity()[0])
            measured = (v0 - v1) / (120 * dt) / 1.62
            passed = bool(abs(measured - dynamic) < 0.01)
            report.append({"requested_static_mu": static, "requested_dynamic_mu": dynamic,
                           "mass_kg": mass, "measured_dynamic_mu": measured,
                           "initial_velocity_x_m_s": v0, "final_velocity_x_m_s": v1,
                           "measurement_duration_s": 120 * dt, "passed": passed})
        destination = ROOT / "env/reports/friction.json"
        destination.write_text(json.dumps(report, indent=2) + "\n")
        print("FRICTION_CHECK", json.dumps(report), flush=True)
        if not all(row["passed"] for row in report):
            raise RuntimeError("Effective contact friction differs from the requested coefficient")
    except Exception:
        import traceback
        traceback.print_exc()
        app.app.post_quit(1)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
