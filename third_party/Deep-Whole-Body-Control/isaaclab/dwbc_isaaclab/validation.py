"""Real simulator smoke checks; no policy-quality claims from short runs."""
import json
from pathlib import Path
import torch
from .config import JOINT_NAMES
from .math_utils import smooth_trajectory

def validate(env,steps,output):
    obs,_=env.reset()
    n=env.num_envs
    assert obs["policy"].shape==(n,env.cfg.observation_space)
    assert env.robot.num_joints==20 and env.joint_ids.__len__()==18
    initial=env.robot.data.root_pos_w.clone()
    samples=[]
    for _ in range(steps):
        obs,reward,done,timeout,info=env.step(torch.zeros(n,18,device=env.device))
        if not torch.isfinite(obs["policy"]).all() or not torch.isfinite(reward).all():
            raise AssertionError("Nonfinite simulation output")
        samples.append({k:float(v) for k,v in info["metrics"].items()})
    # Exercise partial automatic reset and terminal observation, not only env.reset().
    env.episode_length_buf[0]=env.max_episode_length-1
    obs,reward,done,timeout,info=env.step(torch.zeros(n,18,device=env.device))
    assert timeout[0] and env.episode_length_buf[0]==0
    assert torch.isfinite(info["terminal_observation"]).all()
    assert torch.allclose(env.actions[0],torch.zeros_like(env.actions[0]))
    assert torch.allclose(env.history[0,0],env.history[0,-1])
    if n>1: assert env.episode_length_buf[1]>0
    # Check reward response using the identical physical state.
    env._kinematics()
    before=env.target.clone()
    env.target=env.tcp_yaw.clone(); env._kinematics()
    good=torch.exp(-env.ee_error.square().sum(1)/.015)
    env.target[:,0]+=.3; env._kinematics()
    bad=torch.exp(-env.ee_error.square().sum(1)/.015)
    assert torch.all(good>bad)
    env.target=before
    # Actual gains/limits and contact materials queried from PhysX, not configuration alone.
    materials=env.robot.root_physx_view.get_material_properties().cpu()
    efforts=env.robot.data.joint_effort_limits[:,env.joint_ids].cpu()
    expected=torch.tensor([env.cfg.asset_manifest["robot"]["drive_effort_limits"][k] for k in JOINT_NAMES])
    assert torch.allclose(efforts[0],expected)
    c=env.cfg.asset_manifest["lunar_config"]
    foot=torch.tensor([c["robot"]["foot_static_friction"],c["robot"]["foot_dynamic_friction"],0.])
    assert torch.isclose(materials.reshape(-1,3),foot).all(-1).any()
    ore_mass=env.ore.root_physx_view.get_masses().cpu()
    assert torch.allclose(ore_mass,torch.full_like(ore_mass,c["ore"]["mass_kg"]))


    # Check actual lunar acceleration with a freely falling dynamic ore, above all terrain.
    env.reset()
    ore_state=env.ore.data.root_state_w.clone()
    ore_state[:,2]+=2.0
    ore_state[:,7:]=0.0
    env.ore.write_root_state_to_sim(ore_state)
    for _ in range(12):
        env.step(torch.zeros(n,18,device=env.device))
    measured_g=-env.ore.data.root_lin_vel_w[:,2].mean()/(12*env.step_dt)
    expected_g=-env.cfg.sim.gravity[2]
    assert abs(float(measured_g)-expected_g)<.02, (measured_g,expected_g)
    # Scene composition must preserve material bindings and local asset dependencies.
    from pxr import Usd,UsdUtils,UsdGeom
    asset_path=Path(env.cfg.asset_dir)/"scene.usda"
    stage=Usd.Stage.Open(str(asset_path))
    deps=UsdUtils.ComputeAllDependencies(str(asset_path))
    assert not deps[2], deps[2]
    missing=[str(t) for prim in stage.Traverse() for rel in prim.GetRelationships()
             for t in rel.GetTargets() if not stage.GetPrimAtPath(t.GetPrimPath())]
    assert not missing, missing
    original=Usd.Stage.Open(str(Path(__file__).resolve().parents[4]/env.cfg.asset_manifest["source_scene"]))
    import numpy as np
    source_points=np.asarray(UsdGeom.Mesh(original.GetPrimAtPath("/World/Terrain/Ground")).GetPointsAttr().Get())
    crop_points=np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath("/World/Terrain/Ground")).GetPointsAttr().Get())
    bound=env.settings["patch_size_m"]/2
    selected=source_points[np.all(np.abs(source_points[:,:2])<=bound,axis=1)]
    assert np.array_equal(crop_points,selected), "Training crop changed source elevation vertices"
    env.reset()
    result={"passed":True,"num_envs":n,"steps":steps,"joint_names":JOINT_NAMES,
        "observation_shape":list(obs["policy"].shape),"gravity":list(env.cfg.sim.gravity),
        "measured_gravity_m_s2":float(measured_g),"source_mesh_crop_exact":True,
        "missing_usd_dependencies":list(deps[2]),"physics_dt":env.physics_dt,"policy_dt":env.step_dt,
        "source_scene":env.cfg.asset_manifest["source_scene"],
        "terrain_bounds_m":env.cfg.asset_manifest["crop_bounds_m"],
        "effort_limits":efforts[0].tolist(),
        "ore_mass_kg":float(ore_mass[0].flatten()[0]),
        "material_coefficients_unique":torch.unique(materials.reshape(-1,3),dim=0).tolist(),
        "partial_timeout_reset":True,"tcp_reward_sensitivity":True,
        "last_metrics":samples[-1],"initial_base_positions":initial.cpu().tolist()}
    Path(output).write_text(json.dumps(result,indent=2)+"\n")
    print("[VALIDATION] "+json.dumps(result),flush=True)
    return result

def play(runner,steps,output,use_privileged=False):
    env=runner.env
    runner.policy.eval()
    obs=env.reset()[0]["policy"]
    records=[]
    with torch.no_grad():
        for i in range(steps):
            actions=runner.policy.act_inference(obs,hist_encoding=not use_privileged)
            observations,_,terminated,truncated,extras=env.step(actions)
            obs=observations["policy"]
            if not torch.isfinite(obs).all(): raise FloatingPointError("Nonfinite playback observation")
            records.append({"step":i,"resets":int((terminated|truncated).sum()),
                **{k:float(v) for k,v in extras["metrics"].items()}})
    result={"checkpoint_iteration":runner.iteration,"steps":steps,"finite":True,
        "policy_encoding":"privileged_teacher" if use_privileged else "history",
        "metrics_mean":{k:sum(r[k] for r in records)/len(records) for k in records[0] if k!="step"}}
    Path(output).write_text(json.dumps(result,indent=2)+"\n")
    print("[PLAY] "+json.dumps(result),flush=True)
    return result
