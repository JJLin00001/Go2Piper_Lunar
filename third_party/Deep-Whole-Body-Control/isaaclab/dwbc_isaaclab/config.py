"""Simulator-independent configuration and the public joint/observation contract."""
import copy
import json
import math
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
UPSTREAM_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "configs/training/Go2_Piper_Lunar_Tracking.json"
JOINT_NAMES = [f"{leg}_{joint}_joint" for leg in ("FL", "FR", "RL", "RR")
               for joint in ("hip", "thigh", "calf")] + [f"joint{i}" for i in range(1, 7)]
FINGER_NAMES = ["joint7", "joint8"]
NUM_PROPRIO = 83
NUM_PRIV = 6
OBSERVATION_FIELDS = {
    "base_linear_velocity_b": 3, "base_angular_velocity_b": 3, "projected_gravity_b": 3,
    "velocity_command_yaw": 3, "joint_position_offset": 18, "joint_velocity_scaled": 18,
    "previous_action": 18, "feet_contact_FL_FR_RL_RR": 4, "tcp_position_yaw": 3,
    "tcp_target_yaw": 3, "tcp_error_yaw": 3, "tcp_target_velocity_yaw": 3,
    "base_height": 1,
}

def load_settings(path=DEFAULT_CONFIG):
    result = json.loads(Path(path).read_text())
    validate_settings(result)
    return result

def validate_settings(cfg):
    def finite(x):
        if isinstance(x, dict):
            for v in x.values(): finite(v)
        elif isinstance(x, list):
            for v in x: finite(v)
        elif isinstance(x, (float, int)) and not math.isfinite(x):
            raise ValueError("All numeric settings must be finite")
    finite(cfg)
    if cfg["task"] != "Go2_Piper_Lunar_Tracking": raise ValueError("Unknown task")
    if cfg["environment"] not in ("Lunar", "Lunar_baseline"): raise ValueError("Unknown environment")
    for k in ("num_envs", "decimation", "history_len"):
        if not isinstance(cfg[k], int) or cfg[k] < 1: raise ValueError(k)
    if cfg["history_len"] not in (10, 20, 50): raise ValueError("history_len must be 10,20,50")
    if len(cfg["action_scale"]) != 18 or min(cfg["action_scale"]) <= 0: raise ValueError("action_scale")
    if len(cfg["tcp_offset_link6_m"]) != 3: raise ValueError("TCP offset")
    if not 8 <= cfg["patch_size_m"] <= 40: raise ValueError("patch_size_m must be in [8,40]")
    if not 1 <= cfg["boundary_margin_m"] < cfg["patch_size_m"]/2: raise ValueError("boundary margin")
    if cfg["episode_length_s"] <= 0 or cfg["nominal_base_height_m"] <= 0: raise ValueError("time/height")
    for key in ("velocity_x_m_s", "velocity_y_m_s", "yaw_rate_rad_s", "trajectory_time_s", "hold_time_s"):
        v = cfg["command"][key]
        if len(v) != 2 or v[1] < v[0]: raise ValueError(key)
    if cfg["command"]["trajectory_time_s"][0] <= 0 or cfg["command"]["resample_time_s"] <= 0:
        raise ValueError("Command durations must be positive")
    if cfg["command"]["hold_time_s"][0] < 0 or not 0 <= cfg["command"]["standing_fraction"] <= 1:
        raise ValueError("hold time / standing fraction")
    if not 0 < cfg["curriculum"]["initial_scale"] <= 1 or cfg["curriculum"]["ramp_iterations"] <= 0:
        raise ValueError("Invalid curriculum")
    p = cfg["ppo"]
    for k in ("num_steps_per_env", "num_learning_epochs", "num_mini_batches", "dagger_update_freq"):
        if not isinstance(p[k], int) or p[k] < 1: raise ValueError(k)
    if cfg["num_envs"] * p["num_steps_per_env"] % p["num_mini_batches"]:
        raise ValueError("rollout size must be divisible by num_mini_batches")
    if p["mixing_schedule"][2] <= 0 or p["priv_reg_coef_schedual"][3] <= 0:
        raise ValueError("Schedule durations must be positive")

def metadata():
    return json.loads((REPO_ROOT / "assets/go2_piper_d435_assets/joint_metadata.json").read_text())

def policy_kwargs(cfg):
    p = copy.deepcopy(cfg["policy"])
    p.pop("min_std")
    p.update(num_leg_actions=12, num_arm_actions=6, adaptive_arm_gains=False,
             adaptive_arm_gains_scale=1.0, num_priv=NUM_PRIV,
             num_hist=cfg["history_len"], num_prop=NUM_PROPRIO, activation="elu")
    return p
