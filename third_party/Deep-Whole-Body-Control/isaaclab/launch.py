#!/usr/bin/env python3
"""Local train/play/validate entry point. Start SimulationApp before simulator imports."""
import argparse
import copy
from datetime import datetime
import importlib.metadata
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback

HERE=Path(__file__).resolve().parent
UPSTREAM=HERE.parent
ROOT=UPSTREAM.parent.parent
# Select THIS fork's PPO, never the globally installed rsl-rl-lib 3.x API.
sys.path.insert(0,str(UPSTREAM/"rsl_rl"))
sys.path.insert(0,str(HERE))
sys.path.insert(0,str(ROOT/"env"))

def main():
    from isaaclab.app import AppLauncher
    from dwbc_isaaclab.config import load_settings,validate_settings,DEFAULT_CONFIG
    from dwbc_isaaclab import TASK_ID
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode",choices=("train","play","validate","prepare"),default="train")
    parser.add_argument("--task",default=TASK_ID,choices=(TASK_ID,))
    parser.add_argument("--config",type=Path)
    parser.add_argument("--env",choices=("Lunar","Lunar_baseline"))
    parser.add_argument("--num-envs","--num_envs",type=int)
    parser.add_argument("--iterations","--max_iterations",type=int,default=10000)
    parser.add_argument("--steps",type=int,default=300)
    parser.add_argument("--steps-per-env",type=int)
    parser.add_argument("--epochs",type=int)
    parser.add_argument("--episode-seconds",type=float)
    parser.add_argument("--seed",type=int)
    parser.add_argument("--slope-deg",type=float)
    parser.add_argument("--foot-static-friction",type=float)
    parser.add_argument("--foot-dynamic-friction",type=float)
    parser.add_argument("--mass-kg",type=float)
    parser.add_argument("--checkpoint",type=Path)
    parser.add_argument("--log-dir",type=Path)
    parser.add_argument("--save-interval",type=int,default=100)
    parser.add_argument("--teacher",action="store_true",help="Playback with privileged latent (history is default)")
    parser.add_argument("--full-curriculum",action="store_true",help="Evaluate at the final command range")
    parser.add_argument("--capture",action="store_true",help="Save a viewport RGB image; enable cameras too")
    AppLauncher.add_app_launcher_args(parser)
    args=parser.parse_args()
    if min(args.iterations,args.steps,args.save_interval)<=0: parser.error("iterations, steps, save interval must be positive")
    if args.mode=="play" and args.checkpoint is None: parser.error("--mode play requires --checkpoint")
    if args.capture and not args.enable_cameras: parser.error("--capture requires --enable_cameras")
    import torch
    torch.set_num_threads(4)
    saved=None
    if args.checkpoint:
        saved=torch.load(args.checkpoint,map_location="cpu",weights_only=True)
    settings=load_settings(args.config or DEFAULT_CONFIG) if args.config or saved is None else copy.deepcopy(saved["settings"])
    for arg,key in (("env","environment"),("num_envs","num_envs"),("seed","seed"),("episode_seconds","episode_length_s")):
        if getattr(args,arg) is not None: settings[key]=getattr(args,arg)
    if args.steps_per_env is not None: settings["ppo"]["num_steps_per_env"]=args.steps_per_env
    if args.epochs is not None: settings["ppo"]["num_learning_epochs"]=args.epochs
    validate_settings(settings)
    from configuration import load_config,apply_experiment_overrides
    if saved is not None and args.env is None:
        lunar=copy.deepcopy(saved["contract"]["lunar_config"])
    else:
        lunar=load_config(ROOT/"env/config"/(settings["environment"]+".json"))
    lunar=apply_experiment_overrides(lunar,slope_deg=args.slope_deg,
        foot_static_friction=args.foot_static_friction,foot_dynamic_friction=args.foot_dynamic_friction,
        mass_kg=args.mass_kg)
    log_dir=args.log_dir or ROOT/"outputs"/TASK_ID/(datetime.now().strftime("%Y%m%d_%H%M%S")+f"_{os.getpid()}")
    log_dir=log_dir.resolve()
    log_dir.mkdir(parents=True,exist_ok=True)
    print("[INFO] Outputs: "+str(log_dir),flush=True)
    launcher=AppLauncher(args)
    app=launcher.app
    env=runner=None
    code=0
    try:
        from dwbc_isaaclab.assets import prepare_assets
        directory,manifest=prepare_assets(settings,lunar)
        import rsl_rl
        run={"settings":settings,"asset_manifest":manifest,"asset_directory":str(directory.relative_to(ROOT)),
            "versions":{n:importlib.metadata.version(n) for n in ("isaacsim","isaaclab","torch","gymnasium")},
            "rsl_rl_source":str(Path(rsl_rl.__file__).resolve()),
            "upstream_commit":"8159e4ed8695b2d3f62a40d2ab8d88205ac5021a",
            "code_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in list(HERE.rglob("*.py"))+list((UPSTREAM/"rsl_rl/rsl_rl").rglob("*.py"))},
            "note":"DWBC learned joint policy; not a QP WBC controller. Tracking only, no heavy-grasp qualification."}
        if not run["versions"]["isaacsim"].startswith("5.1."):
            raise RuntimeError("This backend is validated against Isaac Sim 5.1.x")
        (log_dir/"run.json").write_text(json.dumps(run,indent=2)+"\n")
        if args.mode!="prepare":
            import gymnasium as gym
            from dwbc_isaaclab.env_cfg import make_env_cfg
            cfg=make_env_cfg(settings,directory,manifest,args.device)
            run["simulation_backend"]={"device":args.device,"ccd":cfg.sim.physx.enable_ccd,
                "physics_dt":cfg.sim.dt,"decimation":cfg.decimation}
            (log_dir/"run.json").write_text(json.dumps(run,indent=2)+"\n")
            env=gym.make(TASK_ID,cfg=cfg,render_mode="rgb_array" if args.capture else None).unwrapped
            if args.mode=="validate":
                from dwbc_isaaclab.validation import validate
                validate(env,args.steps,log_dir/"validation.json")
            else:
                from dwbc_isaaclab.runner import Runner
                runner=Runner(env,settings,log_dir)
                if args.checkpoint: runner.load(args.checkpoint,optimizer=args.mode=="train")
                if args.mode=="train":
                    runner.learn(args.iterations,args.save_interval)
                else:
                    if args.full_curriculum: env.set_iteration(settings["curriculum"]["ramp_iterations"])
                    from dwbc_isaaclab.validation import play
                    play(runner,args.steps,log_dir/"play.json",args.teacher)
            if args.capture:
                from PIL import Image
                import numpy as np
                env.render()  # Allocate the render product before warming it up.
                for _ in range(60):
                    env.sim.render()  # Warm up materials/textures as well as the annotator.
                rgb=env.render()
                if rgb.size==0 or np.ptp(rgb)<10:
                    raise RuntimeError("Viewport capture is blank")
                Image.fromarray(rgb).save(log_dir/"scene.png")
        print("[DONE] "+str(log_dir),flush=True)
    except Exception:
        traceback.print_exc()
        code=1
    finally:
        if runner is not None: runner.close()
        if env is not None: env.close()
        if code: app.app.post_quit(code)
        app.close()
    return code

if __name__=="__main__":
    sys.exit(main())
