"""Isaac Lab 2.3.x configuration for the local Go2/Piper USD."""
from pathlib import Path
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from .config import NUM_PROPRIO,NUM_PRIV,metadata

@configclass
class Go2PiperLunarTrackingCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 20.0
    action_space = 18
    observation_space = NUM_PROPRIO*11+NUM_PRIV
    state_space = 0
    sim = sim_utils.SimulationCfg(dt=1/240,render_interval=4,gravity=(0,0,-1.62),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1,dynamic_friction=1,
            friction_combine_mode="multiply",restitution=0),
        physx=sim_utils.PhysxCfg(enable_ccd=True,enable_stabilization=False,
            gpu_max_rigid_contact_count=2**20,gpu_max_rigid_patch_count=2**17,
            gpu_found_lost_pairs_capacity=2**21))
    scene = InteractiveSceneCfg(num_envs=64,env_spacing=18,replicate_physics=True)
    robot = ArticulationCfg(prim_path="/World/envs/env_.*/Scene/Robot",spawn=None,actuators={})
    ore = RigidObjectCfg(prim_path="/World/envs/env_.*/Scene/Ore",spawn=None)
    contacts = ContactSensorCfg(prim_path="/World/envs/env_.*/Scene/Robot/.*",
        update_period=0.0,history_length=3,track_air_time=True)
    asset_dir: str = ""
    settings: dict = {}
    asset_manifest: dict = {}

def make_env_cfg(settings, directory, manifest, device):
    cfg=Go2PiperLunarTrackingCfg()
    cfg.settings=settings
    cfg.asset_dir=str(directory)
    cfg.asset_manifest=manifest
    cfg.seed=settings["seed"]
    cfg.scene.num_envs=settings["num_envs"]
    cfg.scene.env_spacing=settings["patch_size_m"]+2
    cfg.decimation=settings["decimation"]
    cfg.episode_length_s=settings["episode_length_s"]
    cfg.observation_space=NUM_PROPRIO*(settings["history_len"]+1)+NUM_PRIV
    cfg.sim.device=device
    cfg.sim.physx.enable_ccd=(device == "cpu")
    physics=manifest["physics"]
    cfg.sim.dt=physics["dt_s"]; cfg.sim.render_interval=cfg.decimation
    cfg.sim.gravity=(0,0,-physics["gravity_m_s2"])
    cfg.viewer.origin_type="asset_root"
    cfg.viewer.asset_name="robot"
    cfg.viewer.env_index=0
    cfg.viewer.eye=(2,-2.3,1.35)
    cfg.viewer.lookat=(.3,0,.3)
    cfg.robot.init_state=ArticulationCfg.InitialStateCfg(
        pos=tuple(manifest["robot"]["initial_base_position_m"]),
        joint_pos=manifest["robot"]["initial_joint_positions"],joint_vel={".*":0.0})
    # Match the existing scene's model limits and gains, by name, never USD order.
    actuators={}
    gains=manifest["lunar_config"]["robot"]["pd"]
    for j in metadata()["joints"]:
        name=j["name"]
        group="gripper" if name in ("joint7","joint8") else "legs" if "_joint" in name else (
              "arm_load_bearing" if name in ("joint2","joint3") else "arm")
        actuators[name]=ImplicitActuatorCfg(joint_names_expr=[name],
            stiffness=gains[group][0],damping=gains[group][1],
            effort_limit_sim=j["effort"],velocity_limit_sim=j["velocity"])
    cfg.robot.actuators=actuators
    return cfg

def default_env_cfg():
    """Registry factory, called after SimulationApp has started."""
    import sys
    from .config import load_settings, REPO_ROOT
    from .assets import prepare_assets
    sys.path.insert(0,str(REPO_ROOT/"env"))
    from configuration import load_config
    settings=load_settings()
    lunar=load_config(REPO_ROOT/"env/config"/(settings["environment"]+".json"))
    directory,manifest=prepare_assets(settings,lunar)
    return make_env_cfg(settings,directory,manifest,"cuda:0")
