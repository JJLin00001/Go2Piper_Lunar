#!/usr/bin/env python3
"""Build, view, simulate, and capture Lunar or Lunar_baseline with Isaac Sim 5.1."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

from configuration import (ENVIRONMENTS, ROOT, load_config, apply_experiment_overrides)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--env", choices=ENVIRONMENTS, default=None, help="Default: Lunar (measured DEM)")
    p.add_argument("--config", type=Path, help="JSON configuration; --env must agree if also provided")
    p.add_argument("--scene", type=Path, help="Explicit output USD path")
    p.add_argument("--rebuild", action="store_true")
    p.add_argument("--build-only", action="store_true")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--simulate", action="store_true", help="Floating robot with model-limited PD drives")
    p.add_argument("--steps", type=int, default=0, help="Physics steps or preview frames; 0 keeps GUI open")
    p.add_argument("--slope-deg", type=float, help="X slope; DEM: additive tilt, zero keeps native relief")
    p.add_argument("--foot-static-friction", type=float, help="Effective static foot-ground coefficient")
    p.add_argument("--foot-dynamic-friction", type=float, help="Effective dynamic foot-ground coefficient")
    p.add_argument("--mass-kg", type=float, help="Target mass; recompute inertia without changing geometry")
    p.add_argument("--camera", choices=("close", "overview", "head", "wrist"), default="close")
    p.add_argument("--capture", type=Path, help="Directory for RGB and optional sensor RGB/depth")
    p.add_argument("--sensors", action="store_true")
    p.add_argument("--validate", action="store_true", help="Check standing, contacts and lunar gravity")
    args = p.parse_args(argv)
    if args.steps < 0:
        p.error("--steps must be nonnegative")
    if args.validate and (not args.simulate or args.steps < 240):
        p.error("--validate requires --simulate and --steps >= 240")
    if args.sensors and not args.capture:
        p.error("--sensors requires --capture")
    config_path = args.config or ROOT / "env/config" / ((args.env or "Lunar") + ".json")
    cfg = load_config(config_path)
    if args.env is not None and args.env != cfg["environment"]:
        p.error("--env and config environment disagree")
    cfg = apply_experiment_overrides(cfg, slope_deg=args.slope_deg,
                                    foot_static_friction=args.foot_static_friction,
                                    foot_dynamic_friction=args.foot_dynamic_friction,
                                    mass_kg=args.mass_kg)
    if args.scene is None:
        canonical = load_config(ROOT / "env/config" / (cfg["environment"] + ".json"))
        suffix = ""
        if cfg != canonical:
            digest = hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:12]
            suffix = "_variant_" + digest
        args.scene = ROOT / "env/generated" / (cfg["environment"] + suffix + ".usda")
    if args.headless and not args.build_only and args.steps == 0:
        args.steps = 600 if args.simulate else 30
    args.config = config_path
    return args, cfg


def main():
    args, cfg = parse_args()
    print("ENVIRONMENT", cfg["environment"], "SCENE", args.scene, flush=True)
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    from isaacsim import SimulationApp
    sys.argv = [sys.argv[0]]
    app = SimulationApp({
        "headless": args.headless or args.build_only,
        "width": cfg["camera"]["width"], "height": cfg["camera"]["height"],
        "renderer": "RaytracedLighting", "fast_shutdown": True,
        "extra_args": ["--/app/settings/persistent=false", "--/app/asyncRendering=false",
                       "--/rtx/ecoMode/enabled=false",
                       "--/app/window/title=Go2 Piper | " + cfg["environment"] + " | Isaac Sim 5.1"],
    })
    try:
        import omni.usd
        from pxr import Usd
        from lunar_scene import build_scene, build_digest
        scene_path = args.scene.resolve()
        rebuild = args.rebuild or args.build_only or not scene_path.exists()
        if not rebuild:
            existing = Usd.Stage.Open(str(scene_path))
            rebuild = existing.GetDefaultPrim().GetCustomDataByKey("build_sha256") != build_digest(cfg)
            existing = None
        if rebuild:
            stage, manifest = build_scene(cfg, scene_path)
            print("SCENE_BUILT", scene_path, flush=True)
            print(json.dumps(manifest, indent=2), flush=True)
            stage = None
        if args.build_only:
            return
        if not omni.usd.get_context().open_stage(str(scene_path)):
            raise RuntimeError(f"Could not open {scene_path}")
        for _ in range(8):
            app.update()
        from runtime import run_scene
        run_scene(app, args, cfg)
    except Exception:
        import traceback
        traceback.print_exc()
        app.app.post_quit(1)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
