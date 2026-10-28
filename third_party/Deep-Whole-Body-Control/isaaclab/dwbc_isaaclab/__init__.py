"""Isaac Sim 5.1 / Isaac Lab backend for Deep Whole-Body Control."""
import gymnasium as gym
TASK_ID = "Go2_Piper_Lunar_Tracking"
if TASK_ID not in gym.registry:
    gym.register(id=TASK_ID, entry_point="dwbc_isaaclab.tracking_env:Go2PiperLunarTrackingEnv",
        disable_env_checker=True,
        kwargs={"env_cfg_entry_point": "dwbc_isaaclab.env_cfg:default_env_cfg"})
