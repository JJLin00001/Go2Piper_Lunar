# Modified: USD path made relative to this standalone bundle.
"""Active Go2PiperD435 articulation (CAD nominal, awaiting requalification).

The historical LeggedManip_Lab USD is retained separately for rollback.
"""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg
from isaaclab.assets import ArticulationCfg

ASSET_DIR = Path(__file__).resolve().parent
GO2_PIPER_USD = ASSET_DIR / "go2_piper_d435.usd"

# Historical canonical Piper poses, retained as reset poses only. The D435
# asset, changed joint geometry and camera mount are NOT qualified by the
# former qstow015 lineage; old trajectories/policies need requalification.
PIPER_STOW_JOINT_POS = (0.0, 0.15, -0.15, 0.0, 0.0, 0.0)
PIPER_STOW_GRIPPER_JOINT_POS = (0.0, 0.0)
PIPER_STOW_ALL_JOINT_POS = PIPER_STOW_JOINT_POS + PIPER_STOW_GRIPPER_JOINT_POS
# Frozen M1/M4 carry-transition pose. Keep this synchronized with the accepted
# Hybrid training snapshot; M5 owns its separate final wrist pose.
PIPER_CARRY_JOINT_POS = (0.0, 0.80, -1.25, 0.0, 0.45, 0.0)

GO2_PIPER_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(GO2_PIPER_USD),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=100.0,
            max_angular_velocity=100.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=2,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.42),
        joint_pos={
            "FL_hip_joint": 0.1,
            "FR_hip_joint": -0.1,
            "RL_hip_joint": 0.1,
            "RR_hip_joint": -0.1,
            ".*_thigh_joint": 0.8,
            ".*_calf_joint": -1.5,
            **{f"joint{index}": value for index, value in enumerate(PIPER_STOW_JOINT_POS, start=1)},
            "joint7": PIPER_STOW_GRIPPER_JOINT_POS[0],
            "joint8": PIPER_STOW_GRIPPER_JOINT_POS[1],
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": DelayedPDActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
            effort_limit=45.0,
            velocity_limit=30.0,
            # Route-A requalification swept the deployed Piper trajectory at
            # the 1.20 kg design payload. 35/1.0 was the first shared plant
            # setting to complete all four routes without a fall, collision,
            # or undesired contact while reducing peak tilt. This actuator is
            # shared by MOVE and STANCE, so every downstream frozen policy is
            # revalidated after this change; the external 86 -> 12 contract
            # remains unchanged.
            stiffness=35.0,
            damping=1.0,
            armature=0.01,
            min_delay=0,
            max_delay=2,
            friction=0.01,
        ),
        "arm": DelayedPDActuatorCfg(
            joint_names_expr=["joint[1456]"],
            effort_limit=20.0,
            velocity_limit=5.0,
            # Matches the position-loop gains in the local Piper ROS model.
            # The MJCF effort limit remains the torque authority boundary.
            stiffness=60.0,
            damping=3.0,
            armature=0.01,
            min_delay=0,
            max_delay=1,
            friction=0.01,
        ),
        # Joints 2/3 carry most of the arm's gravity moment.  The route-A
        # ground-grasp envelope exposed a narrow carrier clearance that the
        # previous 120 gain crossed under steady gravity sag.  A 360 position
        # gain keeps both empty and 1.20 kg design-load tracking inside that
        # clearance while retaining the MJCF-derived 20 Nm torque cap.  Route-A
        # dynamic replay additionally measured an 11.5 N link3/base contact
        # with damping 8 during the 0 -> 40 mm insertion; damping 20 removed
        # the transient contact without changing the position gain or effort
        # authority.
        "arm_load_bearing": DelayedPDActuatorCfg(
            joint_names_expr=["joint[23]"],
            effort_limit=20.0,
            velocity_limit=5.0,
            stiffness=360.0,
            damping=20.0,
            armature=0.01,
            min_delay=0,
            max_delay=1,
            friction=0.01,
        ),
        "gripper": DelayedPDActuatorCfg(
            joint_names_expr=["joint[78]"],
            effort_limit=10.0,
            velocity_limit=1.0,
            # Seed-442 closed-loop grasping showed that 180 N/m allowed the
            # 0.60 kg handle to lag the TCP and slip during lift. At 800 N/m,
            # both pads remained in contact, the object lifted 87.4 mm, and
            # measured finger effort stayed below the existing 10 N cap.
            stiffness=800.0,
            damping=5.0,
            armature=0.001,
            min_delay=0,
            max_delay=1,
            friction=0.02,
        ),
    },
)
